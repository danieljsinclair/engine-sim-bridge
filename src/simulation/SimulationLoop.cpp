// SimulationLoop.cpp - Simulation loop implementation
// Extracted from engine_sim_cli.cpp for SOLID SRP compliance
// Phase E: Uses ISimulator* instead of EngineSimHandle/EngineSimAPI&
// Phase F: Moved to engine-sim-bridge for reusability (GUI, iOS, headless)

#include "simulation/SimulationLoop.h"

#include "simulator/ISimulator.h"
#include "simulator/BridgeSimulator.h"
#include "simulator/EngineSimTypes.h"

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

struct InputResult {
    double throttle = 0.1;
    bool ignition = true;
    bool shouldContinue = true;
    int gearDelta = 0;
    double dynoTorqueScale = -1.0;
};

InputResult pollInput(input::IInputProvider* inputProvider, double currentTime, double duration, double updateInterval) {
    if (inputProvider) {
        auto engineInput = inputProvider->OnUpdateSimulation(updateInterval);
        return {engineInput.throttle, engineInput.ignition, engineInput.shouldContinue,
                engineInput.gearDelta, engineInput.dynoTorqueScale};
    }
    if (currentTime >= duration) {
        return {0.1, true, false, 0, -1.0};
    }
    auto timed = input::EngineInput{};
    timed.throttle = currentTime < THROTTLE_RAMP_DURATION_SECONDS
        ? currentTime / THROTTLE_RAMP_DURATION_SECONDS : FULL_THROTTLE;
    return {timed.throttle, timed.ignition, true, 0, -1.0};
}

struct CrankingState {
    enum Phase { Cranking, Running } phase = Cranking;
    int ticks = 0;

    static constexpr int BASELINE_TICKS = 10;
    static constexpr double CATCH_RATIO = 2.0;
    static constexpr double MIN_CATCH_RPM = 500.0;

    double exhaustFlowSum = 0.0;
    double exhaustFlowBaseline = 0.0;

    struct Result {
        double effectiveThrottle;
        bool starterEngaged;
    };

    bool engineCaught(const EngineSimStats& stats) const {
        if (exhaustFlowBaseline <= 0.0) return false;
        return stats.exhaustFlow > exhaustFlowBaseline * CATCH_RATIO
            && stats.currentRPM > MIN_CATCH_RPM;
    }

    Result step(ISimulator& simulator, double userThrottle, ILogging* logger) {
        ticks++;
        double effectiveThrottle = 0.55;

        if (phase == Cranking) {
            EngineSimStats stats = simulator.getStats();
#if CRANKING_DEBUG
            // Periodic debug during cranking
            if (ticks <= BASELINE_TICKS || ticks % 20 == 0) {
                logger->info(LogMask::BRIDGE,
                             "Crank tick %d: RPM=%.0f exhaust=%.3f load=%.3f",
                             ticks, stats.currentRPM, stats.exhaustFlow, stats.currentLoad);
            }
#endif
            if (ticks <= BASELINE_TICKS) {
                exhaustFlowSum += stats.exhaustFlow;
                if (ticks == BASELINE_TICKS) {
                    exhaustFlowBaseline = exhaustFlowSum / BASELINE_TICKS;
#if CRANKING_DEBUG
                    logger->info(LogMask::BRIDGE, "Exhaust flow baseline: %.3f at %.0f RPM",
                                 exhaustFlowBaseline, stats.currentRPM);
#endif
                }
            } else if (engineCaught(stats)) {
                simulator.setStarterMotor(false);
                phase = Running;
                effectiveThrottle = userThrottle;
#if CRANKING_DEBUG
                logger->info(LogMask::BRIDGE,
                             "Engine caught at tick %d - exhaust %.3f (%.1fx baseline), %.0f RPM",
                             ticks, stats.exhaustFlow,
                             stats.exhaustFlow / exhaustFlowBaseline, stats.currentRPM);
#else
                ILogging* _ = logger;
#endif
            }

            return {effectiveThrottle, phase == Cranking};
        }

        return {userThrottle, false};
    }
};

int readUnderrunCount(telemetry::ITelemetryReader* reader) {
    if (!reader) return 0;
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
    if (!bridgeSim || gearDelta == 0) return;
    if (bridgeSim->changeGear(gearDelta)) {
        logger->info(LogMask::BRIDGE, "Gear change: %+d", gearDelta);
    }
}

void applyDynoControl(BridgeSimulator* bridgeSim, double scale, double& lastScale) {
    if (!bridgeSim || scale < 0.0 || scale == lastScale) return;
    bridgeSim->setDynoTorqueScale(scale);
    lastScale = scale;
}

void applyVehicleControls(BridgeSimulator* bridgeSim, int gearDelta, double dynoTorqueScale,
                          bool engineRunning, double& lastDynoTorqueScale, ILogging* logger) {
    if (!bridgeSim) return;
    applyGearChange(bridgeSim, gearDelta, logger);
    if (engineRunning) {
        applyDynoControl(bridgeSim, dynoTorqueScale, lastDynoTorqueScale);
    }
}

void updatePresentation(presentation::IPresentation* presentation, const SimulationConfig& config,
                        double currentTime,
                        const EngineSimStats& stats, double throttle, bool ignition, bool starterEngaged,
                        int underrunCount, IAudioBuffer& audioBuffer,
                        telemetry::ITelemetryReader* telemetryReader) {
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
    state.simulationFrequency = config.engineConfig.simulationFrequency;

    presentation->ShowEngineState(state);
}

void writeTelemetry(telemetry::ITelemetryWriter* telemetryWriter,
                    double currentTime,
                    double throttle,
                    bool ignition,
                    bool starterEngaged) {
    if (!telemetryWriter) return;

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
    if (!engineConfig) {
        throw std::runtime_error("engineConfig must not be null");
    }

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

} // anonymous namespace

// ============================================================================
// Unified Main Loop Implementation
// ============================================================================

int runUnifiedAudioLoop(
    ISimulator& simulator,
    const SimulationConfig& config,
    IAudioBuffer& audioBuffer,
    input::IInputProvider* inputProvider,
    presentation::IPresentation* presentation,
    telemetry::ITelemetryWriter* telemetryWriter,
    telemetry::ITelemetryReader* telemetryReader,
    ILogging* logger)
{
    double currentTime = 0.0;
    LoopTimer timer(config.updateInterval());
    CrankingState crankingState;

    auto* bridgeSim = dynamic_cast<BridgeSimulator*>(&simulator);
    double lastDynoTorqueScale = -1.0;

    logger->info(LogMask::BRIDGE, "runUnifiedAudioLoop starting simulation loop with %s", config.simulatorLabel.c_str());

    InputResult input;
    do {
        auto cranking = crankingState.step(simulator, input.throttle, logger);

        simulator.setThrottle(cranking.effectiveThrottle);
        simulator.setIgnition(input.ignition);

        applyVehicleControls(bridgeSim, input.gearDelta, input.dynoTorqueScale,
                             crankingState.phase == CrankingState::Running,
                             lastDynoTorqueScale, logger);

        audioBuffer.updateSimulation(&simulator, config.updateInterval() * SECONDS_TO_MILLISECONDS);

        EngineSimStats stats = simulator.getStats();
        audioBuffer.fillBufferFromEngine(&simulator, config.framesPerUpdate());

        writeTelemetry(telemetryWriter, currentTime, cranking.effectiveThrottle, input.ignition, cranking.starterEngaged);

        currentTime += config.updateInterval();
        updatePresentation(presentation, config, currentTime, stats, cranking.effectiveThrottle,
                            input.ignition, cranking.starterEngaged, readUnderrunCount(telemetryReader),
                            audioBuffer, telemetryReader);

        // Loop control
        timer.waitUntilNextTick();
        input = pollInput(inputProvider, currentTime, config.duration, config.updateInterval());
    } while (input.shouldContinue);

    return 0;
}

// ============================================================================
// Main Simulation Entry Point
// ============================================================================

int runSimulation(
    const SimulationConfig& config,
    ISimulator& simulator,
    IAudioBuffer* audioBuffer,
    input::IInputProvider* inputProvider,
    presentation::IPresentation* presentation,
    telemetry::ITelemetryWriter* telemetryWriter,
    telemetry::ITelemetryReader* telemetryReader,
    ILogging* logger)
{
    ASSERT(audioBuffer, "audioBuffer must be provided");
    ASSERT(config.engineConfig.sampleRate > 0, "config.sampleRate must be set");
    ASSERT(config.updateInterval() > 0.0, "config.updateInterval must be set");
    ASSERT(config.framesPerUpdate() > 0, "config.framesPerUpdate must be set");

    // Initialize simulator (throws on failure)
    initializeSimulator(simulator, config, logger, telemetryWriter, &config.engineConfig);

    // Initialize strategy
    AudioBufferConfig strategyConfig;
    strategyConfig.channels = EngineSimAudio::STEREO;
    strategyConfig.synthLatency = config.engineConfig.targetSynthesizerLatency;

    if (!audioBuffer->initialize(strategyConfig, config.sampleRate())) {
        throw std::runtime_error("Failed to initialize audio strategy");
    }

    // Create and initialize audio hardware provider (throws on failure)
    auto callback = [audioBuffer](AudioBufferView& buffer) -> int {
        return audioRenderCallback(audioBuffer, buffer);
    };

    auto hardwareProvider = createHardwareProvider(config.sampleRate(), callback, logger);

    logger->info(LogMask::AUDIO, "Audio initialized: strategy=%s, sr=%d",
                         audioBuffer->getName(), config.sampleRate());

    // Start strategy playback
    if (!audioBuffer->startPlayback(&simulator)) {
        throw std::runtime_error("Failed to start audio playback");
    }

    // Set volume
    hardwareProvider->setVolume(config.volume);

    simulator.setStarterMotor(true);

    // Prepare buffer via strategy (threaded: pre-fills with silence, sync-pull: no-op)
    audioBuffer->prepareBuffer();

    // Start audio hardware playback
    if (!hardwareProvider->startPlayback()) {
        logger->error(LogMask::AUDIO, "Failed to start hardware playback");
    }

    int exitCode = runUnifiedAudioLoop(simulator, config, *audioBuffer, inputProvider, presentation, telemetryWriter, telemetryReader, logger);

    // Cleanup
    audioBuffer->stopPlayback(&simulator);
    cleanupSimulation(hardwareProvider.get(), simulator);

    warnWavExportNotSupported(config.outputWav, logger);

    return exitCode;
}
