// SimulationLoop.cpp - Simulation loop implementation
// Extracted from engine_sim_cli.cpp for SOLID SRP compliance
// Phase E: Uses ISimulator* instead of EngineSimHandle/EngineSimAPI&
// Phase F: Moved to engine-sim-bridge for reusability (GUI, iOS, headless)
// Phase G: Refactored from free function to class with injected dependencies

#include "simulation/SimulationLoop.h"
#include "simulation/CrankingController.h"
#include "simulation/PresentationStateBuilders.h"
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
#include "common/PresetExceptions.h"
#include "telemetry/ITelemetryProvider.h"
#include "common/Verification.h"

#include <cstring>


namespace {

// Timed input simulation constants
constexpr double THROTTLE_RAMP_DURATION_SECONDS = 0.5;  // Time to ramp from 0 to 1
constexpr double FULL_THROTTLE = 1.0;                     // Maximum throttle value
constexpr double SECONDS_TO_MILLISECONDS = 1000.0;

} // anonymous namespace — constants only

// ============================================================================
// SimulationLoop - Private methods (file scope, access members directly)
// ============================================================================

input::EngineInput SimulationLoop::pollInput(double currentTime, double updateInterval, bool isFirstTick) {
    if (inputProvider_) {
        return inputProvider_->OnUpdateSimulation(updateInterval);
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

void SimulationLoop::updatePresentation(
                        const EngineSimStats& stats,
                        const CrankingController::State& crankingState,
                        const input::EngineInput& input,
                        double tickTime) {

    if (!presentation_) return;

    telemetry::AudioTimingTelemetry timing;
    if (telemetryReader_) {
        timing = telemetryReader_->getAudioTiming();
    }

    presentation::EngineState state;
    state.engine = presentation::builders::buildEngineState(stats, crankingState);
    state.drivetrain = presentation::builders::buildDrivetrainState(stats, input);
    state.controls = presentation::builders::buildControlState(input, crankingState);
    state.audio = presentation::builders::buildAudioState(timing, telemetryReader_, audioBuffer_, config_, tickTime, simulator_);
    state.presetShortName = simulator_.getName() ? simulator_.getName() : "";
    presentation_->ShowSimulatorStates(state);
}

// ============================================================================
// File-local helpers — pure functions and internal types
// ============================================================================
namespace {

// Named audio render callback -- bridges AudioBufferView to strategy->render()
int audioRenderCallback(IAudioBuffer* strategy, AudioBufferView& buffer) {
    if (!strategy->isPlaying()) {
        if (float* dst = buffer.asFloat(); dst) {
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
        throw SimulatorException("Failed to initialize audio hardware");
    }

    return provider;
}

// Apply gear changes from keyboard input ([/] keys). Clamps at gear 0 (neutral).
void applyGearChange(ISimulator& simulator, int gearDelta, ILogging* logger) {

    if (simulator.changeGear(gearDelta)) {
        logger->info(LogMask::BRIDGE, __ilog_format("New gear: %+d", simulator.getGear()));
    }
}

void applyDecision(ICombustionEngine* combustionEngine, const TransitionDecision& decision) {
    if (!combustionEngine) return;
    combustionEngine->applyTransition(decision);
}

} // anonymous namespace — file-local helpers

// ============================================================================
// SimulationLoop - Private methods (file scope, access members directly)
// ============================================================================

CrankingController::State SimulationLoop::applyCrankingDecision(
                                                ICombustionEngine* combustionEngine,
                                                const input::EngineInput& engineInput) {

    auto crankingDecision = TransitionDecision{EnginePhase::Running, false, engineInput.throttle, false};

    if (combustionEngine) {
        auto starterDecision = crankingController_.engageStarter(*combustionEngine, engineInput.starterButton, engineInput.ignition);
        applyDecision(combustionEngine, starterDecision);

        crankingDecision = crankingController_.step(*combustionEngine, engineInput.throttle, engineInput.ignition);
        applyDecision(combustionEngine, crankingDecision);
    }

    return CrankingController::State{crankingDecision.effectiveThrottle, combustionEngine && crankingDecision.starterMotor, crankingDecision.targetPhase};
}

void applyDynoControl(ISimulator& simulator, double scale, double& lastScale) {
    if (scale == lastScale) return;  // OK: no-op when unchanged
    if (scale < 0.0) return;  // OK: negative values are invalid, ignore silently
    simulator.setDynoTorqueScale(scale);
    lastScale = scale;
}

void SimulationLoop::applyVehicleControls(
    ICombustionEngine* combustionEngine,
    const input::EngineInput& input, const CrankingController::State& crankingState,
    double& lastDynoTorqueScale) {

    simulator_.setThrottle(crankingState.startingThrottle);

    if (combustionEngine) {
        combustionEngine->setIgnition(input.ignition);
    }

    // Apply gear changes: twin gearAbsolute takes priority over keyboard gearDelta
    if (input.gearAbsolute >= 0) {
        simulator_.setGear(input.gearAbsolute);
    } else {
        applyGearChange(simulator_, input.gearDelta, logger_);
    }

    // Vehicle controls (gear, dyno) — dyno only when engine running
    if (!crankingState.starterEngaged) {
        // HACK: Should put the clutch in here really
        applyDynoControl(simulator_, input.dynoTorqueScale, lastDynoTorqueScale);

        // Speed tracking. Spike-A: prefer the vehicle-speed constraint (drives the
        // wheels, clutch couples to engine, dyno OFF) when commanded — this is the
        // --auto DRIVE path that fixes the dragged-engine sound. Fall back to the
        // dyno path (setSpeedTrackingTarget) for the legacy road-speed input.
        // Both are BridgeSimulator-specific APIs, so downcast the engine.
        if (auto* bridgeSim = dynamic_cast<BridgeSimulator*>(combustionEngine)) {
            if (input.vehicleSpeedTargetKmh >= 0.0) {
                // Spike-A inverse path: drive wheels, clutch couples, dyno OFF.
                bridgeSim->setVehicleSpeedTarget(input.vehicleSpeedTargetKmh);
            } else if (!input.gearAutoMode && input.roadSpeedKmh >= 0.0) {
                // Legacy dyno fallback — manual road-speed input only. In --auto
                // mode the gearbox provider owns speed tracking via the constraint;
                // PARK/NEUTRAL frames must NOT fall through to the dyno (free-rev).
                bridgeSim->setSpeedTrackingTarget(input.roadSpeedKmh, input.engineRpmFloor);
            }
        }
    } else {
        logger_->info(LogMask::BRIDGE, "Cranking: starter engaged, dyno disabled - consider using the clutch instead");
    }

    // Twin clutch control (direct pressure, overrides applyGearChange's hardwired clutch)
    if (input.clutchPressure >= 0.0) {
        simulator_.setClutchPressure(input.clutchPressure);
    }

    // Brake
    simulator_.setBrakePressure(input.brakeLevel);
}

void SimulationLoop::writeTelemetry(double currentTime, double throttle, bool ignition, bool starterEngaged) {
    if (!telemetryWriter_) return;

    telemetry::VehicleInputsTelemetry inputs;
    inputs.throttlePosition = throttle;
    inputs.ignitionOn = ignition;
    inputs.starterMotorEngaged = starterEngaged;
    telemetryWriter_->writeVehicleInputs(inputs);

    // Push simulator metrics
    telemetry::SimulatorMetricsTelemetry metrics;
    metrics.timestamp = currentTime;
    telemetryWriter_->writeSimulatorMetrics(metrics);
}

// ============================================================================
// File-local helpers (continued) + SimulatorSession
// ============================================================================
namespace {

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
    logger->info(LogMask::BRIDGE, __ilog_format("Loading simulator: %s", label.c_str()));

    if (!simulator.create(*engineConfig, logger, telemetryWriter)) {
        throw SimulatorException("Failed to create simulator: " + simulator.getLastError());
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
// Composes SimulationLoop for the main tick execution.
// ============================================================================

class SimulatorSession : public ISimulatorSession {
public:
    SimulatorSession(
        const SimulationConfig& config,
        std::unique_ptr<ISimulator> simulator,
        const SessionDependencies& deps,
        std::unique_ptr<IAudioHardwareProvider> hardwareProvider)
        : config_(config)
        , simulator_(std::move(simulator))
        , audioBuffer_(deps.audioBuffer)
        , hardwareProvider_(std::move(hardwareProvider))
        , inputProvider_(deps.inputProvider)
        , presentation_(deps.presentation)
        , telemetryWriter_(deps.telemetryWriter)
        , telemetryReader_(deps.telemetryReader)
        , logger_(deps.logger)
    {}

    ~SimulatorSession() override {
        if (!closed_) {
            doClose();
        }
    }

    int run() override {
        if (closed_) throw SimulatorException("Session is closed");

        // Start audio only if not already playing (hot-swap keeps audio running)
        if (!audioBuffer_->isPlaying()) {
            if (!audioBuffer_->startPlayback(simulator_.get())) {
                throw SimulatorException("Failed to start audio playback");
            }
            hardwareProvider_->setVolume(config_.volume);
            audioBuffer_->prepareBuffer();

            if (!hardwareProvider_->startPlayback()) {
                logger_->error(LogMask::AUDIO, "Failed to start hardware playback");
            }
        }

        // Create loop with injected dependencies — no parameter plumbing
        SessionDependencies loopDeps{
            audioBuffer_,
            &crankingController_,
            &stopRequested_,
            inputProvider_,
            presentation_,
            telemetryWriter_,
            telemetryReader_,
            logger_
        };
        SimulationLoop loop(*simulator_, config_, loopDeps);

        int exitCode = loop.run();

        // Stop audio only on final exit, not on preset cycle
        if (exitCode != EXIT_BUT_CONTINUE_NEXT) {
            audioBuffer_->stopPlayback(simulator_.get());
        }

        return exitCode;
    }

    void stop() override {
        stopRequested_.store(true, std::memory_order_seq_cst);
    }

    bool hasDrivetrainMomentum(const BridgeSimulator::DrivetrainSnapshot& snapshot) const {
        return snapshot.gear >= 0 && std::abs(snapshot.vehicleMassVtheta) > 1.0;
    }

    void transferDrivetrainState(ISimulator& newSimulator, const ISimulator& oldSimulator, ILogging* logger) {
        // Transfer drivetrain state from old simulator to new
        const auto* oldBridge = dynamic_cast<const BridgeSimulator*>(&oldSimulator);
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
                logger->info(LogMask::BRIDGE, __ilog_format("Hot-swap → Rollover (gear=%d, vtheta=%.1f)", snapshot.gear, snapshot.vehicleMassVtheta));
            } else {
                auto decision = crankingController_.engageStarter(*combustion, true, true);
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

        logger_->info(LogMask::BRIDGE, __ilog_format("handoverSession: loading %s", presetFilePath.c_str()));

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
        stopRequested_.store(false, std::memory_order_seq_cst);

        logger_->info(LogMask::BRIDGE, "handoverSession: complete");
        return true;
    }

    void close() override {
        if (closed_) return;
        doClose();
    }

    void doClose() {
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
// SimulationLoop - Implementation
// ============================================================================

SimulationLoop::SimulationLoop(
    ISimulator& simulator,
    const SimulationConfig& config,
    const SessionDependencies& deps)
        : simulator_(simulator)
        , config_(config)
        , audioBuffer_(*deps.audioBuffer)
        , crankingController_(*deps.crankingController)
        , stopRequested_(deps.stopRequested)
        , inputProvider_(deps.inputProvider)
        , presentation_(deps.presentation)
        , telemetryWriter_(deps.telemetryWriter)
        , telemetryReader_(deps.telemetryReader)
        , logger_(deps.logger)
        , clock_(deps.clock ? deps.clock : nullptr)
        , steadyClock_(deps.clock ? nullptr : std::make_unique<SteadyClockLoopClock>(config.updateInterval()))
    {
        // Point clock_ to steadyClock_ if no clock was injected
        if (!clock_) {
            clock_ = steadyClock_.get();
        }
    }

int SimulationLoop::run() {
    logger_->info(LogMask::BRIDGE, __ilog_format("SimulationLoop starting with %s", config_.simulatorLabel.c_str()));

    // Initialize loop state
    LoopState state;
    state.currentTime = 0.0;
    state.isFirstTick = true;
    state.lastDynoTorqueScale = -1.0;
    state.combustionEngine = dynamic_cast<ICombustionEngine*>(&simulator_);

    // Pre-loop: provide initial feedback to input provider
    if (inputProvider_) inputProvider_->provideFeedback(state.previousStats);

    // Main loop: thin wrapper calling step()
    for (;;) {
        // Execute one simulation tick
        StepResult result = step(state);

        // Loop control: pacing, input polling, feedback
        clock_->waitUntilNextTick();
        state.engineInput = pollInput(state.currentTime, config_.updateInterval(), state.isFirstTick);
        state.isFirstTick = false;

        // Feed stats back to input provider for next tick's decisions
        if (inputProvider_) inputProvider_->provideFeedback(state.previousStats);

        // Handle step results (after feedback, matching original behavior)
        if (result == StepResult::PresetCycle) {
            return EXIT_BUT_CONTINUE_NEXT;
        }
        if (result == StepResult::Stop) {
            return 0;
        }

        // Check stop flag
        if (stopRequested_->load(std::memory_order_seq_cst)) {
            return 0;
        }
    }
}

// ============================================================================
// step() - Per-tick simulation logic (sleep-free, deterministic)
// Extracted from run() for unit testing
// ============================================================================

StepResult SimulationLoop::step(LoopState& state) {
    // Duration check: stop when currentTime reaches or exceeds duration
    if (config_.duration > 0.0 && state.currentTime >= config_.duration) {
        return StepResult::Stop;
    }

    // Per-tick simulation logic
    CrankingController::State crankingState = applyCrankingDecision(state.combustionEngine, state.engineInput);

    applyVehicleControls(state.combustionEngine, state.engineInput, crankingState, state.lastDynoTorqueScale);

    audioBuffer_.updateSimulation(&simulator_, config_.updateInterval() * SECONDS_TO_MILLISECONDS);
    audioBuffer_.fillBufferFromEngine(&simulator_, config_.framesPerUpdate());

    writeTelemetry(state.currentTime, crankingState.startingThrottle, state.engineInput.ignition, crankingState.starterEngaged);

    EngineSimStats stats = simulator_.getStats();
    state.currentTime += config_.updateInterval();
    updatePresentation(stats, crankingState, state.engineInput, state.currentTime);

    // Update cross-tick state
    state.previousStats = stats;

    // Check for preset cycle request
    if (state.engineInput.presetCycle) {
        return StepResult::PresetCycle;
    }

    return StepResult::Continue;
}

// ============================================================================
// Session factory - GLOBAL scope
// ============================================================================

std::unique_ptr<ISimulatorSession> createSession(
    const SimulationConfig& config,
    const std::string& scriptPath,
    std::unique_ptr<ISimulator> simulator,
    const SessionDependencies& deps,
    std::unique_ptr<ISimulatorSession> existingSession)
{
    auto* audioBuffer = deps.audioBuffer;
    auto* inputProvider = deps.inputProvider;
    auto* presentation = deps.presentation;
    auto* telemetryWriter = deps.telemetryWriter;
    auto* telemetryReader = deps.telemetryReader;
    auto* logger = deps.logger;

    // Hot-swap path: caller passed an existing session — swap the simulator within it
    if (existingSession) {
        ASSERT(simulator, "simulator must be provided for hot-swap");
        if (auto* session = static_cast<SimulatorSession*>(existingSession.get());
            !session || !session->handoverSession(scriptPath, std::move(simulator))) {
            throw SimulatorException("Failed to swap preset within existing session: " + scriptPath);
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

    auto callback = [audioBuffer](AudioBufferView& buffer) {
        return audioRenderCallback(audioBuffer, buffer);
    };
    auto hardwareProvider = createHardwareProvider(config.sampleRate(), callback, logger);

    logger->info(LogMask::AUDIO, __ilog_format("Audio initialized: strategy=%s, sr=%d", audioBuffer->getName(), config.sampleRate()));

    SessionDependencies sessionDeps{
        audioBuffer,
        nullptr,  // crankingController not needed at session construction
        nullptr,  // stopRequested set by session internally
        inputProvider,
        presentation,
        telemetryWriter,
        telemetryReader,
        logger
    };
    return std::make_unique<SimulatorSession>(
        sessionConfig,
        std::move(simulator),
        sessionDeps,
        std::move(hardwareProvider));
}// GLOBAL scope — session factory, no access to SimulationLoop members
