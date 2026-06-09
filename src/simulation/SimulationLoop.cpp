// SimulationLoop.cpp - Simulation loop implementation
// Extracted from engine_sim_cli.cpp for SOLID SRP compliance
// Phase E: Uses ISimulator* instead of EngineSimHandle/EngineSimAPI&
// Phase F: Moved to engine-sim-bridge for reusability (GUI, iOS, headless)

#include "simulation/SimulationLoop.h"
#include "simulation/CrankingController.h"
#include "session/ISimulatorSession.h"

#include "simulator/ISimulator.h"
#include "simulator/ICombustionEngine.h"
#include "simulator/BridgeSimulator.h"
#include "simulator/EngineSimTypes.h"
#include "simulator/SimulatorFactory.h"

#include "hardware/IAudioHardwareProvider.h"
#include "strategy/IAudioBuffer.h"
#include "io/IInputProvider.h"
#include "io/IPresentation.h"
#include "common/ILogging.h"
#include "telemetry/ITelemetryProvider.h"
#include "common/Verification.h"

#include <cstring>
#include <stdexcept>
#include <thread>
#include <chrono>

#define CRANKING_DEBUG false  // Enable detailed logging for cranking state transitions
#if CRANKING_DEBUG
#define IF_CRANKING_DEBUG(x) x
#else
#define IF_CRANKING_DEBUG(x)
#endif

// SimulationConfig — value type, compiler-generated special members

// ============================================================================
// PresentationContext - Bundles parameters for updatePresentation()
// ============================================================================

struct PresentationContext {
    presentation::IPresentation* presentation;
    const SimulationConfig* config;
    double currentTime;
    const EngineSimStats* stats;
    double throttle;
    bool ignition;
    bool starterEngaged;
    EnginePhase phase;
    int underrunCount;
    IAudioBuffer* audioBuffer;
    telemetry::ITelemetryReader* telemetryReader;
    const char* presetShortName;
    int actualSimFrequency;
    double brakeLevel;
};

// ============================================================================
// Private Helper Functions - SRP Compliance
// ============================================================================

namespace {

// Timed input simulation constants
static constexpr double THROTTLE_RAMP_DURATION_SECONDS = 0.5;  // Time to ramp from 0 to 1
static constexpr double FULL_THROTTLE = 1.0;                     // Maximum throttle value
static constexpr double SECONDS_TO_MICROSECONDS = 1000000.0;
static constexpr double SECONDS_TO_MILLISECONDS = 1000.0;

input::EngineInput pollInput(input::IInputProvider* inputProvider, double currentTime, double updateInterval, bool isFirstTick) {
    if (inputProvider) {
        return inputProvider->OnUpdateSimulation(updateInterval);
    }
    input::EngineInput timed;
    timed.throttle = currentTime < THROTTLE_RAMP_DURATION_SECONDS
        ? currentTime / THROTTLE_RAMP_DURATION_SECONDS : FULL_THROTTLE;

    // Send starter button on first tick for non-interactive mode to auto-start engine
    if (isFirstTick) {
        timed.starterButton = true;
    }
    return timed;
}

int readUnderrunCount(telemetry::ITelemetryReader* reader) {
    return reader->getAudioDiagnostics().underrunCount;
}

// Timing control for 60Hz loop pacing using sleep_until for accuracy
struct LoopTimer {
    std::chrono::steady_clock::time_point nextWakeTime;
    std::chrono::microseconds intervalUs;

    explicit LoopTimer(double intervalSeconds)
        : nextWakeTime(std::chrono::steady_clock::now())
        , intervalUs(static_cast<long long>(intervalSeconds * SECONDS_TO_MICROSECONDS))
    {}

    void waitUntilNextTick() {
        nextWakeTime += intervalUs;
        std::this_thread::sleep_until(nextWakeTime);
    }
};

// Named audio render callback -- bridges AudioBufferView to strategy->render()
int audioRenderCallback(IAudioBuffer* strategy, AudioBufferView& buffer) {
    if (!strategy->isPlaying()) {
        float* dst = buffer.asFloat();
        if (dst) {
            size_t totalSamples = static_cast<size_t>(buffer.frameCount) * buffer.channelCount;
            std::memset(dst, 0, totalSamples * sizeof(float));
        }
        return 0;
    }

    strategy->render(buffer);
    return 0;
}

// Create and initialize the audio hardware provider. Throws on failure.
std::unique_ptr<IAudioHardwareProvider> createHardwareProvider(
    int sampleRate,
    const IAudioHardwareProvider::AudioCallback& callback,
    ILogging* logger)
{
    auto provider = AudioHardwareProviderFactory::createProvider(logger);
    provider->registerAudioCallback(callback);

    // Use AudioStreamFormat defaults (stereo float32 interleaved), only override sampleRate
    AudioStreamFormat format;
    format.sampleRate = sampleRate;

    if (!provider->initialize(format)) {
        throw std::runtime_error("Failed to initialize audio hardware");
    }

    return provider;
}

// Apply gear changes from keyboard input ([/] keys). Clamps at gear 0 (neutral).
void applyGearChange(BridgeSimulator* engineSim, int gearDelta, ILogging* logger) {

    if (engineSim->changeGear(gearDelta)) {
        logger->info(LogMask::BRIDGE, "New gear: %+d", engineSim->getGear());
    }
}

void applyDecision(ICombustionEngine* engine, const TransitionDecision& decision) {
    if (!engine) return;
    engine->applyTransition(decision);
}

void applyDynoControl(BridgeSimulator* bridgeSim, double scale, double& lastScale) {
    if (scale == lastScale) return;  // OK: no-op when unchanged
    if (scale < 0.0) return;  // OK: negative values are invalid, ignore silently
    bridgeSim->setDynoTorqueScale(scale);
    lastScale = scale;
}

void applyVehicleControls(
    ISimulator& simulator, ICombustionEngine* combustionEngine,
    const input::EngineInput& input, const CrankingController::State& crankingState,
    double& lastDynoTorqueScale, ILogging* logger)
{
    auto* bridgeSim = dynamic_cast<BridgeSimulator*>(&simulator);

    simulator.setThrottle(crankingState.startingThrottle);

    if (combustionEngine) {
        combustionEngine->setIgnition(input.ignition);
    }

    // Apply gear changes: twin gearAbsolute takes priority over keyboard gearDelta
    if (input.gearAbsolute >= 0) {
        simulator.setGear(input.gearAbsolute);
    } else {
        applyGearChange(bridgeSim, input.gearDelta, logger);
    }

    // Vehicle controls (gear, dyno) — dyno only when engine running
    if (!crankingState.starterEngaged) {
        // HACK: Should put the clutch in here really
        applyDynoControl(bridgeSim, input.dynoTorqueScale, lastDynoTorqueScale);
    } else {
        logger->info(LogMask::BRIDGE, "Cranking: starter engaged, dyno disabled - consider using the clutch instead");
    }

    // Twin clutch control (direct pressure, overrides applyGearChange's hardwired clutch)
    if (input.clutchPressure >= 0.0) {
        simulator.setClutchPressure(input.clutchPressure);
    }

    // Brake
    simulator.setBrakePressure(input.brakeLevel);

    // QUESTION: This is superceded by CrankingController now, right?
    // // Twin starter motor control — only when twin explicitly sets it
    // // Default input.starterMotor is false; don't override warmup starter logic
    // if (input.gearAbsolute >= 0) {
    //     simulator.setStarterMotor(input.starterMotor);
    // }

}

void updatePresentation(const PresentationContext& ctx) {
    if (!ctx.presentation) return;

    // Read audio timing diagnostics from telemetry (strategies push to telemetry after each render)
    telemetry::AudioTimingTelemetry timing;
    if (ctx.telemetryReader) {
        timing = ctx.telemetryReader->getAudioTiming();
    }

    presentation::EngineState state;
    state.timestamp = ctx.currentTime;
    state.rpm = ctx.stats->currentRPM;
    state.throttle = ctx.throttle;
    state.load = ctx.stats->currentLoad;
    state.speedMph = ctx.stats->speedMph();
    state.underrunCount = ctx.underrunCount;
    state.audioMode = ctx.audioBuffer->getModeString();
    state.ignition = ctx.ignition;
    state.starterMotorEngaged = ctx.starterEngaged;
    state.enginePhase = ctx.phase;
    state.exhaustFlow = ctx.stats->exhaustFlow;
    state.gear = ctx.stats->gear;
    state.gearSelector = ctx.stats->gearSelector;
    state.gearAutoMode = ctx.stats->gearAutoMode;
    state.dynoTorque = ctx.stats->dynoTorque;
    state.dynoTargetRPM = ctx.stats->dynoTargetRPM;
    state.renderMs = timing.renderMs;
    state.headroomMs = timing.headroomMs;
    state.budgetPct = timing.budgetPct;
    state.framesRequested = timing.framesRequested;
    state.framesRendered = timing.framesRendered;
    state.callbackRateHz = timing.callbackRateHz;
    state.generatingRateFps = timing.generatingRateFps;
    state.trendPct = timing.trendPct;
    state.sampleRate = ctx.config->sampleRate();
    state.vehicleSpeedKmh = ctx.stats->vehicleSpeedKmh;
    state.engineTorqueNm = ctx.stats->engineTorqueNm;
    state.drivetrainTorqueNm = ctx.stats->drivetrainTorqueNm;
    state.simulationFrequency = ctx.actualSimFrequency;
    state.presetShortName = ctx.presetShortName ? ctx.presetShortName : "";
    state.brakeLevel = ctx.brakeLevel;

    ctx.presentation->ShowEngineState(state);
}

void writeTelemetry(telemetry::ITelemetryWriter* telemetryWriter,
                    double currentTime,
                    double throttle,
                    bool ignition,
                    bool starterEngaged) {
    // Push vehicle inputs (loop owns throttle/ignition, simulator doesn't)
    telemetry::VehicleInputsTelemetry inputs;
    inputs.throttlePosition = throttle;
    inputs.ignitionOn = ignition;
    inputs.starterMotorEngaged = starterEngaged;
    telemetryWriter->writeVehicleInputs(inputs);

    // Push simulator metrics
    telemetry::SimulatorMetricsTelemetry metrics;
    metrics.timestamp = currentTime;
    telemetryWriter->writeSimulatorMetrics(metrics);

    // Note: EngineStateTelemetry and FramePerformanceTelemetry are pushed
    // by BridgeSimulator::update() -- SRP/ISP compliance
    // Note: AudioDiagnostics and AudioTiming are pushed by strategies
}

// Initialize the simulator: create with audio config.
// Script loading is handled by SimulatorFactory before this is called.
// Throws std::runtime_error on failure.
void initializeSimulator(
    ISimulator& simulator,
    const SimulationConfig& config,
    ILogging* logger,
    telemetry::ITelemetryWriter* telemetryWriter,
    const ISimulatorConfig* engineConfig)
{
    // Use provided label directly, no internal logic about simulator type
    const std::string& label = config.simulatorLabel;
    logger->info(LogMask::BRIDGE, "Loading simulator: %s", label.c_str());

    if (!simulator.create(*engineConfig, logger, telemetryWriter)) {
        throw std::runtime_error("Failed to create simulator: " + simulator.getLastError());
    }
}

void cleanupSimulation(IAudioHardwareProvider* hardwareProvider, ISimulator& simulator) {
    if (hardwareProvider) {
        hardwareProvider->stopPlayback();
        hardwareProvider->cleanup();
    }
    simulator.destroy();
}

// ============================================================================
// SimulatorSession - Concrete session managing audio hardware + simulator lifecycle
// Owns audio hardware for session lifetime. Reuses runSimulationLoop() for the main tick loop.
// Hot-swap is triggered by initSimulation(existingSession) — the session swaps its internal simulator pointer.
// ============================================================================

class SimulatorSession : public ISimulatorSession {
public:
    SimulatorSession(
        const SimulationConfig& config,
        std::unique_ptr<ISimulator> simulator,
        IAudioBuffer* audioBuffer,
        std::unique_ptr<IAudioHardwareProvider> hardwareProvider,
        input::IInputProvider* inputProvider,
        presentation::IPresentation* presentation,
        telemetry::ITelemetryWriter* telemetryWriter,
        telemetry::ITelemetryReader* telemetryReader,
        ILogging* logger)
        : config_(config)
        , simulator_(std::move(simulator))
        , audioBuffer_(audioBuffer)
        , hardwareProvider_(std::move(hardwareProvider))
        , inputProvider_(inputProvider)
        , presentation_(presentation)
        , telemetryWriter_(telemetryWriter)
        , telemetryReader_(telemetryReader)
        , logger_(logger)
    {}

    ~SimulatorSession() override {
        if (!closed_) {
            close();
        }
    }

    int run() override {
        if (closed_) throw std::runtime_error("Session is closed");

        // Start audio only if not already playing (hot-swap keeps audio running)
        if (!audioBuffer_->isPlaying()) {
            if (!audioBuffer_->startPlayback(simulator_.get())) {
                throw std::runtime_error("Failed to start audio playback");
            }
            hardwareProvider_->setVolume(config_.volume);
            audioBuffer_->prepareBuffer();

            if (!hardwareProvider_->startPlayback()) {
                logger_->error(LogMask::AUDIO, "Failed to start hardware playback");
            }
        }

        // Delegate to existing loop — returns EXIT_BUT_CONTINUE_NEXT on 'P' press
        int exitCode = runSimulationLoop(
            *simulator_, config_, *audioBuffer_, crankingController_,
            stopRequested_,
            inputProvider_, presentation_,
            telemetryWriter_, telemetryReader_, logger_);

        // Stop audio only on final exit, not on preset cycle
        if (exitCode != EXIT_BUT_CONTINUE_NEXT) {
            audioBuffer_->stopPlayback(simulator_.get());
        }

        return exitCode;
    }

    void stop() override {
        stopRequested_.store(true, std::memory_order_release);
    }

    bool hasDrivetrainMomentum(const BridgeSimulator::DrivetrainSnapshot& snapshot) const {
        return snapshot.gear >= 0 && std::abs(snapshot.vehicleMassVtheta) > 1.0;
    }

    void transferDrivetrainState(ISimulator& newSimulator, ISimulator& oldSimulator, ILogging* logger) {
        // Transfer drivetrain state from old simulator to new
        auto* oldBridge = dynamic_cast<BridgeSimulator*>(&oldSimulator);
        auto* newBridge = dynamic_cast<BridgeSimulator*>(&newSimulator);

        if (oldBridge && newBridge) {
            auto snapshot = oldBridge->captureDrivetrainState();
            newBridge->restoreDrivetrainState(snapshot);

            if (snapshot.enginePhase == EnginePhase::Stopped) {
                logger->info(LogMask::BRIDGE, "Old engine was Stopped — no state to transfer");
                return;
            }

            crankingController_.reset();
            auto* combustion = dynamic_cast<ICombustionEngine*>(&newSimulator);
            if (!combustion) return;

            if (hasDrivetrainMomentum(snapshot)) {
                auto rolloverDecision = TransitionDecision{EnginePhase::Rollover, false, 0.0, true};
                applyDecision(combustion, rolloverDecision);
                logger->info(LogMask::BRIDGE, "Hot-swap → Rollover (gear=%d, vtheta=%.1f)", snapshot.gear, snapshot.vehicleMassVtheta);
            } else {
                auto decision = crankingController_.engageStarter(*combustion, true);
                applyDecision(combustion, decision);
                logger->info(LogMask::BRIDGE, "Hot-swap → Cranking (neutral, starter engaged)");
            }
        } else {
            logger->error(LogMask::BRIDGE, "Drivetrain transfer skipped — one or both simulators are not BridgeSimulators");
        }
    }

    bool handoverSession(const std::string& presetFilePath, std::unique_ptr<ISimulator> newSimulator) {
        ASSERT(!closed_, "handoverSession: session is closed");
        ASSERT(newSimulator, "handoverSession: null simulator provided");
        ASSERT(simulator_, "handoverSession: current simulator is null");

        logger_->info(LogMask::BRIDGE, "handoverSession: loading %s", presetFilePath.c_str());

        // Initialize the new simulator (audio config only, not pipeline)
        initializeSimulator(*newSimulator, config_, logger_, telemetryWriter_, &config_.engineConfig);

        // pre-init the new simulator with the engine state, road speed, gears etc
        //as if we swapped the engine in a running vehicle. no effect on a non combustion sim
        transferDrivetrainState(*newSimulator, *simulator_, logger_);

        // Keep old simulator alive in previousSimulator_ to prevent
        // use-after-free in the audio callback (SyncPullStrategy holds a raw pointer)
        previousSimulator_ = std::move(simulator_);

        // Swap to the new simulator
        simulator_ = std::move(newSimulator);

        // Update the audio buffer's pointer to the new simulator
        audioBuffer_->swapSimulator(simulator_.get());

        // Update config path
        config_.configPath = presetFilePath;

        // Reset stopped flag so the loop runs again when session->run() is called
        stopRequested_.store(false, std::memory_order_release);

        logger_->info(LogMask::BRIDGE, "handoverSession: complete");
        return true;
    }

    void close() override {
        if (closed_) return;
        cleanupSimulation(hardwareProvider_.get(), *simulator_);
        closed_ = true;
    }

    ISimulator* getSimulator() const override {
        return simulator_.get();
    }

private:
    SimulationConfig config_;
    std::unique_ptr<ISimulator> simulator_;
    std::unique_ptr<ISimulator> previousSimulator_;
    IAudioBuffer* audioBuffer_;
    std::unique_ptr<IAudioHardwareProvider> hardwareProvider_;
    input::IInputProvider* inputProvider_;
    presentation::IPresentation* presentation_;
    telemetry::ITelemetryWriter* telemetryWriter_;
    telemetry::ITelemetryReader* telemetryReader_;
    ILogging* logger_;
    CrankingController crankingController_;
    std::atomic<bool> stopRequested_{false};
    bool closed_{false};
};

} // anonymous namespace

// ============================================================================
// Unified Main Loop Implementation
// ============================================================================

int runSimulationLoop(
    ISimulator& simulator,
    const SimulationConfig& config,
    IAudioBuffer& audioBuffer,
    CrankingController& crankingController,
    const std::atomic<bool>& stopRequested,
    input::IInputProvider* inputProvider,
    presentation::IPresentation* presentation,
    telemetry::ITelemetryWriter* telemetryWriter,
    telemetry::ITelemetryReader* telemetryReader,
    ILogging* logger)
{
    double currentTime = 0.0;
    LoopTimer timer(config.updateInterval());

    logger->info(LogMask::BRIDGE, "runSimulationLoop starting simulation loop with %s", config.simulatorLabel.c_str());

    // Track first tick to trigger auto-start in non-interactive mode
    bool isFirstTick = true;
    input::EngineInput input;

    auto* combustion = dynamic_cast<ICombustionEngine*>(&simulator);

    // Initialise and track these mutating values for the loop lifetime
    double lastDynoTorqueScale = -1.0;
    EngineSimStats previousStats = {};
    if (inputProvider) inputProvider->provideFeedback(previousStats); // NOTE: input is presently optional for direct control, eg. testing or non-interactive CLI mode. I'm not entirely happy with this and wonder if we should assert there's always an input provider but I can see the argument for allowing one to be optional
 
    // MAIN LOOP
    do {
        // Non-interactive timeout: stop the loop when duration expires
        if (config.duration > 0.0 && currentTime >= config.duration) break;

        if (combustion) {
            auto starterDecision = crankingController.engageStarter(*combustion, input.starterButton);
            applyDecision(combustion, starterDecision);
        }
        auto crankingDecision = combustion
            ? crankingController.step(*combustion, input.throttle, input.ignition)
            : TransitionDecision{EnginePhase::Running, false, input.throttle, false};
        if (combustion) {
            applyDecision(combustion, crankingDecision);
        }
        auto crankingState = CrankingController::State{crankingDecision.effectiveThrottle, combustion && crankingDecision.starterMotor, crankingDecision.targetPhase};

        applyVehicleControls(simulator, combustion, input, crankingState, lastDynoTorqueScale, logger);
        audioBuffer.updateSimulation(&simulator, config.updateInterval() * SECONDS_TO_MILLISECONDS);

        // Get standard engine sim data and overlay input-provider gear state onto stats for display
        EngineSimStats stats = simulator.getStats();
        stats.gearSelector = input.gearSelector;
        stats.gearAutoMode = input.gearAutoMode;

        audioBuffer.fillBufferFromEngine(&simulator, config.framesPerUpdate());

        writeTelemetry(telemetryWriter, currentTime, crankingState.startingThrottle, input.ignition, crankingState.starterEngaged);

        currentTime += config.updateInterval();
        PresentationContext presCtx{
            presentation,
            &config,
            currentTime,
            &stats,
            crankingState.startingThrottle,
            input.ignition,
            crankingState.starterEngaged,
            crankingState.phase,
            readUnderrunCount(telemetryReader),
            &audioBuffer,
            telemetryReader,
            simulator.getName(),
            simulator.getSimulationFrequency(),
            input.brakeLevel
        };
        updatePresentation(presCtx);

        // Timing control - QUESTION: should/can pollInput go before waitUntilNextTick or can waitUntilNextTick go at the bottom before the loop while, o rjust before input.preseCycle
        timer.waitUntilNextTick();
        input = pollInput(inputProvider, currentTime, config.updateInterval(), isFirstTick);
        isFirstTick = false;
        previousStats = stats;

        if (input.presetCycle) {
            return EXIT_BUT_CONTINUE_NEXT;
        }
    } while (!stopRequested.load(std::memory_order_acquire));

    return 0;
}

// ============================================================================
// Main Simulation Entry Point - Factory that creates a session
// ============================================================================

std::unique_ptr<ISimulatorSession> createSession(
    const SimulationConfig& config,
    const std::string& scriptPath,
    std::unique_ptr<ISimulator> simulator,
    IAudioBuffer* audioBuffer,
    std::unique_ptr<ISimulatorSession> existingSession,
    input::IInputProvider* inputProvider,
    presentation::IPresentation* presentation,
    telemetry::ITelemetryWriter* telemetryWriter,
    telemetry::ITelemetryReader* telemetryReader,
    ILogging* logger)
{
    // Hot-swap path: caller passed an existing session — swap the simulator within it
    if (existingSession) {
        ASSERT(simulator, "simulator must be provided for hot-swap");
        auto* session = static_cast<SimulatorSession*>(existingSession.get());
        if (!session || !session->handoverSession(scriptPath, std::move(simulator))) {
            throw std::runtime_error("Failed to swap preset within existing session: " + scriptPath);
        }
        return existingSession;
    }

    // First-run path: create a new session with audio hardware
    ASSERT(simulator, "simulator must be provided");
    ASSERT(audioBuffer, "audioBuffer must be provided");
    ASSERT(config.engineConfig.sampleRate > 0, "config.sampleRate must be set");
    ASSERT(config.updateInterval() > 0.0, "config.updateInterval must be set");
    ASSERT(config.framesPerUpdate() > 0, "config.framesPerUpdate must be set");

    // Initialize the simulator
    initializeSimulator(*simulator, config, logger, telemetryWriter, &config.engineConfig);
    SimulationConfig sessionConfig = config;
    sessionConfig.configPath = scriptPath;

    // Initialize audio buffer and create hardware provider (first run only)
    AudioBufferConfig strategyConfig;
    strategyConfig.channels = EngineSimAudio::STEREO;
    strategyConfig.synthLatency = config.engineConfig.targetSynthesizerLatency;
    audioBuffer->initialize(strategyConfig, config.sampleRate());

    auto callback = [audioBuffer](AudioBufferView& buffer) -> int {
        return audioRenderCallback(audioBuffer, buffer);
    };
    auto hardwareProvider = createHardwareProvider(config.sampleRate(), callback, logger);

    logger->info(LogMask::AUDIO, "Audio initialized: strategy=%s, sr=%d", audioBuffer->getName(), config.sampleRate());

    return std::make_unique<SimulatorSession>(
        sessionConfig,
        std::move(simulator),
        audioBuffer,
        std::move(hardwareProvider),
        inputProvider, presentation,
        telemetryWriter, telemetryReader,
        logger);
}
