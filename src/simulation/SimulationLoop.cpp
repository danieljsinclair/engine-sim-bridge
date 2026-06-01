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
#include <atomic>
#include <mutex>

#define CRANKING_DEBUG false  // Enable detailed logging for cranking state transitions
#if CRANKING_DEBUG
#define IF_CRANKING_DEBUG(x) x
#else
#define IF_CRANKING_DEBUG(x)
#endif

// SimulationConfig — value type, compiler-generated special members

// ============================================================================
// Private Helper Functions - SRP Compliance
// ============================================================================

namespace {

// Timed input simulation constants
static constexpr double THROTTLE_RAMP_DURATION_SECONDS = 0.5;  // Time to ramp from 0 to 1
static constexpr double FULL_THROTTLE = 1.0;                     // Maximum throttle value
static constexpr double SECONDS_TO_MICROSECONDS = 1000000.0;
static constexpr double SECONDS_TO_MILLISECONDS = 1000.0;

input::EngineInput pollInput(input::IInputProvider* inputProvider, double currentTime, double duration, double updateInterval) {
    if (inputProvider) {
        return inputProvider->OnUpdateSimulation(updateInterval);
    }
    if (currentTime >= duration) {
        input::EngineInput stop;
        stop.throttle = 0.1;
        stop.shouldContinue = false;
        return stop;
    }
    input::EngineInput timed;
    timed.throttle = currentTime < THROTTLE_RAMP_DURATION_SECONDS
        ? currentTime / THROTTLE_RAMP_DURATION_SECONDS : FULL_THROTTLE;

    // Send starter button on first tick for non-interactive mode to auto-start engine
    static bool firstTick = true;
    if (firstTick) {
        timed.starterButton = true;
        firstTick = false;
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

void applyGearChange(BridgeSimulator* bridgeSim, int gearDelta, ILogging* logger) {
    if (gearDelta == 0) return;  // OK: no-op when no change requested
    if (bridgeSim->changeGear(gearDelta)) {
        logger->info(LogMask::BRIDGE, "Gear change: %+d", gearDelta);
    }
}

void applyDynoControl(BridgeSimulator* bridgeSim, double scale, double& lastScale) {
    if (scale == lastScale) return;  // OK: no-op when unchanged
    if (scale < 0.0) return;  // OK: negative values are invalid, ignore silently
    bridgeSim->setDynoTorqueScale(scale);
    lastScale = scale;
}

void applyVehicleControls(
    ISimulator& simulator, ICombustionEngine* combustion, BridgeSimulator* bridgeSim,
    const input::EngineInput& input, const CrankingController::State& crankingState,
    double& lastDynoTorqueScale, ILogging* logger)
{
    simulator.setThrottle(crankingState.startingThrottle);

    if (combustion) {
        combustion->setIgnition(input.ignition);
    }

    // Vehicle controls (gear, dyno) — dyno only when engine running
    if (bridgeSim){
        applyGearChange(bridgeSim, input.gearDelta, logger);
        if (!crankingState.starterEngaged) {
            applyDynoControl(bridgeSim, input.dynoTorqueScale, lastDynoTorqueScale);
        }
    }
}

void updatePresentation(presentation::IPresentation* presentation, const SimulationConfig& config,
                        double currentTime,
                        const EngineSimStats& stats, double throttle, bool ignition, bool starterEngaged,
                        EnginePhase phase,
                        int underrunCount, IAudioBuffer& audioBuffer,
                        telemetry::ITelemetryReader* telemetryReader,
                        const std::string& presetShortName,
                        int actualSimFrequency) {
    if (!presentation) return;

    // Read audio timing diagnostics from telemetry (strategies push to telemetry after each render)
    telemetry::AudioTimingTelemetry timing;
    if (telemetryReader) {
        timing = telemetryReader->getAudioTiming();
    }

    presentation::EngineState state;
    state.timestamp = currentTime;
    state.rpm = stats.currentRPM;
    state.throttle = throttle;
    state.load = stats.currentLoad;
    state.speed = 0;
    state.underrunCount = underrunCount;
    state.audioMode = audioBuffer.getModeString();
    state.ignition = ignition;
    state.starterMotorEngaged = starterEngaged;
    state.enginePhase = phase;
    state.exhaustFlow = stats.exhaustFlow;
    state.gear = stats.gear;
    state.dynoTorque = stats.dynoTorque;
    state.dynoTargetRPM = stats.dynoTargetRPM;
    state.renderMs = timing.renderMs;
    state.headroomMs = timing.headroomMs;
    state.budgetPct = timing.budgetPct;
    state.framesRequested = timing.framesRequested;
    state.framesRendered = timing.framesRendered;
    state.callbackRateHz = timing.callbackRateHz;
    state.generatingRateFps = timing.generatingRateFps;
    state.trendPct = timing.trendPct;
    state.sampleRate = config.sampleRate();
    state.simulationFrequency = actualSimFrequency;
    state.presetShortName = presetShortName;

    presentation->ShowEngineState(state);
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

void warnWavExportNotSupported(bool outputWavRequested, ILogging* logger) {
    if (outputWavRequested) {
        logger->warning(LogMask::AUDIO, "WAV export not supported in unified mode - use the old engine mode code path");
    }
}

// ============================================================================
// SimulatorSession - Concrete session managing audio hardware + simulator lifecycle
// Owns audio hardware for session lifetime. Recycles ISimulator on preset swap.
// Reuses runUnifiedAudioLoop() for the main tick loop (DRY).
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
        : type_(config.simulatorType)
        , config_(config)
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
        stopRequested_ = false;

        // Drain pending swap from swapPreset() call
        if (hasPendingSwap_.exchange(false)) {
            std::string path;
            {
                std::lock_guard<std::mutex> lock(pendingSwapMutex_);
                path = std::move(pendingPresetPath_);
            }
            doSwapPreset(path);
        }

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
        int exitCode = runUnifiedAudioLoop(
            *simulator_, config_, *audioBuffer_, crankingController_,
            inputProvider_, presentation_,
            telemetryWriter_, telemetryReader_, logger_);

        // Stop audio only on final exit, not on preset cycle
        if (exitCode != EXIT_BUT_CONTINUE_NEXT) {
            audioBuffer_->stopPlayback(simulator_.get());
        }

        return exitCode;
    }

    void stop() override {
        stopRequested_ = true;
    }

    void swapPreset(const std::string& presetPath) override {
        {
            std::lock_guard<std::mutex> lock(pendingSwapMutex_);
            pendingPresetPath_ = presetPath;
        }
        hasPendingSwap_ = true;
    }

    std::unique_ptr<IAudioHardwareProvider> releaseHardwareProvider() override {
        return std::move(hardwareProvider_);
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
    void doSwapPreset(const std::string& presetPath) {
        // Keep old simulator alive — the IOThread may be mid-render with its pointer.
        // swapSimulator() replaces the pointer atomically, but any in-flight render
        // still holds a snapshot of the old pointer. previousSimulator_ prevents
        // use-after-free until the next swap or session close.
        previousSimulator_ = std::move(simulator_);

        // Fresh engine from the new preset — no state transfer.
        // The starter will spin up the new engine naturally.
        simulator_ = SimulatorFactory::createAndConfigure(
            config_, presetPath, config_.assetBasePath,
            logger_, telemetryWriter_);
        simulator_->create(config_.engineConfig, logger_, telemetryWriter_);

        auto* newBridge = dynamic_cast<BridgeSimulator*>(simulator_.get());
        ASSERT(newBridge, "doSwapPreset: factory returned non-BridgeSimulator");

        if (previousSimulator_ != nullptr) {
            // Capture phase BEFORE reset so we know whether to auto-crank
            bool wasActive = crankingController_.currentPhase() != EnginePhase::Stopped;

            // Reset controller for fresh engine — clears stale baseline/ticks
            crankingController_.reset();

            // Auto-crank new engine if the previous one was active
            if (wasActive) {
                auto* newCombustion = dynamic_cast<ICombustionEngine*>(simulator_.get());
                if (newCombustion) crankingController_.engageStarter(*newCombustion, true);
            }

            // Swap the simulator pointer in the audio buffer — no stop/start, no silence.
            // The IOThread picks up the new simulator on its next callback.
            audioBuffer_->swapSimulator(simulator_.get());

            config_.configPath = presetPath;
            logger_->info(LogMask::BRIDGE, "Session hot-swapped to: %s", presetPath.c_str());
        }
    }

    SimulatorType type_;
    SimulationConfig config_;
    std::unique_ptr<ISimulator> simulator_;
    std::unique_ptr<ISimulator> previousSimulator_;  // Kept alive until next swap — prevents IOThread use-after-free
    IAudioBuffer* audioBuffer_;
    std::unique_ptr<IAudioHardwareProvider> hardwareProvider_;
    input::IInputProvider* inputProvider_;
    presentation::IPresentation* presentation_;
    telemetry::ITelemetryWriter* telemetryWriter_;
    telemetry::ITelemetryReader* telemetryReader_;
    ILogging* logger_;
    CrankingController crankingController_;

    std::atomic<bool> stopRequested_{false};
    std::atomic<bool> hasPendingSwap_{false};
    std::string pendingPresetPath_;
    std::mutex pendingSwapMutex_;
    bool closed_{false};
};

} // anonymous namespace

// ============================================================================
// Unified Main Loop Implementation
// ============================================================================

int runUnifiedAudioLoop(
    ISimulator& simulator,
    const SimulationConfig& config,
    IAudioBuffer& audioBuffer,
    CrankingController& crankingController,
    input::IInputProvider* inputProvider,
    presentation::IPresentation* presentation,
    telemetry::ITelemetryWriter* telemetryWriter,
    telemetry::ITelemetryReader* telemetryReader,
    ILogging* logger)
{
    double currentTime = 0.0;
    LoopTimer timer(config.updateInterval());

    logger->info(LogMask::BRIDGE, "runUnifiedAudioLoop starting simulation loop with %s", config.simulatorLabel.c_str());

    input::EngineInput input;

    auto* bridgeSim = dynamic_cast<BridgeSimulator*>(&simulator);
    auto* combustion = dynamic_cast<ICombustionEngine*>(&simulator);

    // If engine was already running (hot-swap from restoreState, or sine mode), skip cranking
    auto existingEnginePhase = bridgeSim->getEnginePhase();
    if (existingEnginePhase != EnginePhase::Stopped) {
        crankingController.setInitialPhase(existingEnginePhase);
    }
    double lastDynoTorqueScale = -1.0;
    const std::string presetShortName = bridgeSim ? bridgeSim->getName() : "";

    do {
        if (combustion) crankingController.engageStarter(*combustion, input.starterButton);
        auto crankingState = combustion
            ? crankingController.step(*combustion, input.throttle, input.ignition, logger)
            : CrankingController::State{input.throttle, false, EnginePhase::Running};

        applyVehicleControls(simulator, combustion, bridgeSim, input, crankingState, lastDynoTorqueScale, logger);
        audioBuffer.updateSimulation(&simulator, config.updateInterval() * SECONDS_TO_MILLISECONDS);

        EngineSimStats stats = simulator.getStats();
        audioBuffer.fillBufferFromEngine(&simulator, config.framesPerUpdate());

        writeTelemetry(telemetryWriter, currentTime, crankingState.startingThrottle, input.ignition, crankingState.starterEngaged);

        currentTime += config.updateInterval();
        updatePresentation(presentation, config, currentTime, stats, crankingState.startingThrottle, input.ignition, crankingState.starterEngaged, crankingState.phase, readUnderrunCount(telemetryReader), audioBuffer, telemetryReader, presetShortName, simulator.getSimulationFrequency());

        timer.waitUntilNextTick();
        input = pollInput(inputProvider, currentTime, config.duration, config.updateInterval());

        if (input.presetCycle) {
            return EXIT_BUT_CONTINUE_NEXT;
        }
    } while (input.shouldContinue);

    return 0;
}

// ============================================================================
// Main Simulation Entry Point - Factory that creates a session
// ============================================================================

std::unique_ptr<ISimulatorSession> initSimulation(
    const SimulationConfig& config,
    const std::string& scriptPath,
    std::unique_ptr<ISimulator> simulator,
    IAudioBuffer* audioBuffer,
    ISimulatorSession* existingSession,
    input::IInputProvider* inputProvider,
    presentation::IPresentation* presentation,
    telemetry::ITelemetryWriter* telemetryWriter,
    telemetry::ITelemetryReader* telemetryReader,
    ILogging* logger)
{
    ASSERT(simulator, "simulator must be provided");
    ASSERT(audioBuffer, "audioBuffer must be provided");
    ASSERT(config.engineConfig.sampleRate > 0, "config.sampleRate must be set");
    ASSERT(config.updateInterval() > 0.0, "config.updateInterval must be set");
    ASSERT(config.framesPerUpdate() > 0, "config.framesPerUpdate must be set");

    // Always initialize the new simulator first (safer — if create fails, old session untouched)
    initializeSimulator(*simulator, config, logger, telemetryWriter, &config.engineConfig);
    SimulationConfig sessionConfig = config;

    std::unique_ptr<IAudioHardwareProvider> hardwareProvider;

    if (existingSession) {
        // Hot-swap: preserve audio hardware, transfer engine state.
        // Don't close() the old session here — its simulator must stay alive
        // until the audio callback has switched to the new one (a few frames).
        // The caller holds the old session in a unique_ptr that gets overwritten
        // after we return, which naturally defers destruction.
        auto state = existingSession->getSimulator()->saveState();
        hardwareProvider = existingSession->releaseHardwareProvider();
        simulator->restoreState(state);
        sessionConfig.configPath = scriptPath;
    } else {
        // First run: initialize audio buffer and create hardware provider
        AudioBufferConfig strategyConfig;
        strategyConfig.channels = EngineSimAudio::STEREO;
        strategyConfig.synthLatency = config.engineConfig.targetSynthesizerLatency;
        audioBuffer->initialize(strategyConfig, config.sampleRate());

        auto callback = [audioBuffer](AudioBufferView& buffer) -> int {
            return audioRenderCallback(audioBuffer, buffer);
        };
        hardwareProvider = createHardwareProvider(config.sampleRate(), callback, logger);
    }

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
