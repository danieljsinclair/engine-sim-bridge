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
    return timed;
}

class CrankingController {
public:
    struct State {
        double startingThrottle;
        bool starterEngaged;
        presentation::EnginePhase phase;
    };

    void engageStarter(ISimulator& simulator, bool starterButton) {
        if (starterButton && phase_ != Running) {
            if (phase_ != Cranking) {
                // Stopped → start cranking
                reset();
                simulator.setStarterMotor(true);
            }
            else {
                // Already cranking, toggle it, so disengage starter
                simulator.setStarterMotor(false);
                phase_ = Stopped;
            }
        }
    }

    State step(ISimulator& simulator, double userThrottle, ILogging* logger) {
        // Stall detection: Running → Stopped when RPM drops below threshold
        if (phase_ == Running && simulator.getStats().currentRPM < STOPPED_RPM) {
            phase_ = Stopped;
        }

        // Stopped and Running: pass through user throttle
        if (phase_ != Cranking) {
            return {userThrottle, false};
        }

        // Cranking: measure baseline, detect catch
        ticks_++;
        double startingThrottle = 0.55;
        EngineSimStats stats = simulator.getStats();
#ifdef CRANKING_DEBUG
        if (ticks_ <= BASELINE_TICKS || ticks_ % 20 == 0) {
            logger->info(LogMask::BRIDGE,
                         "Crank tick %d: RPM=%.0f exhaust=%.6f baseline=%.6f",
                         ticks_, stats.currentRPM, stats.exhaustFlow,
                         exhaustFlowBaseline_);
        }
#endif
        if (ticks_ <= BASELINE_TICKS) {
            exhaustFlowSum_ += stats.exhaustFlow;
            if (ticks_ == BASELINE_TICKS) {
                exhaustFlowBaseline_ = exhaustFlowSum_ / BASELINE_TICKS;
#ifdef CRANKING_DEBUG
                logger->info(LogMask::BRIDGE, "Exhaust flow baseline: %.3f at %.0f RPM",
                             exhaustFlowBaseline_, stats.currentRPM);
#endif
            }
        } else if (engineCaught(stats)) {
            simulator.setStarterMotor(false);
            phase_ = Running;
            startingThrottle = userThrottle;
#ifdef CRANKING_DEBUG
            logger->info(LogMask::BRIDGE,
                         "Engine caught at tick %d - exhaust %.3f (%.1fx baseline), %.0f RPM",
                         ticks_, stats.exhaustFlow,
                         stats.exhaustFlow / exhaustFlowBaseline_, stats.currentRPM);
#else
            ILogging* _ = logger;
#endif
        }

        return {startingThrottle, phase_ == Cranking};
    }

private:
    enum Phase { Stopped, Cranking, Running } phase_ = Cranking;
    int ticks_ = 0;

    static constexpr int BASELINE_TICKS = 10;
    static constexpr double CATCH_RATIO = 2.0;
    static constexpr double MIN_CATCH_RPM = 500.0;
    static constexpr double STOPPED_RPM = 5.0;

    double exhaustFlowSum_ = 0.0;
    double exhaustFlowBaseline_ = 0.0;

    void reset() {
        phase_ = Cranking;
        ticks_ = 0;
        exhaustFlowSum_ = 0.0;
        exhaustFlowBaseline_ = 0.0;
    }

    bool engineCaught(const EngineSimStats& stats) const {
        return stats.exhaustFlow > exhaustFlowBaseline_ * CATCH_RATIO
            && stats.currentRPM > MIN_CATCH_RPM;
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

void applyVehicleControls(
    ISimulator& simulator, BridgeSimulator* bridgeSim,
    const input::EngineInput& input, const CrankingController::State& crankingState,
    double& lastDynoTorqueScale, ILogging* logger)
{
    simulator.setThrottle(crankingState.startingThrottle);
    simulator.setIgnition(input.ignition);

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
    CrankingController crankingController;

    auto* bridgeSim = dynamic_cast<BridgeSimulator*>(&simulator);
    double lastDynoTorqueScale = -1.0;

    logger->info(LogMask::BRIDGE, "runUnifiedAudioLoop starting simulation loop with %s", config.simulatorLabel.c_str());

    input::EngineInput input;
    do {
        crankingController.engageStarter(simulator, input.starterButton);
        auto crankingState = crankingController.step(simulator, input.throttle, logger);

        applyVehicleControls(simulator, bridgeSim, input, crankingState, lastDynoTorqueScale, logger);
        audioBuffer.updateSimulation(&simulator, config.updateInterval() * SECONDS_TO_MILLISECONDS);

        EngineSimStats stats = simulator.getStats();
        audioBuffer.fillBufferFromEngine(&simulator, config.framesPerUpdate());

        writeTelemetry(telemetryWriter, currentTime, crankingState.startingThrottle, input.ignition, crankingState.starterEngaged);

        currentTime += config.updateInterval();
        updatePresentation(presentation, config, currentTime, stats, crankingState.startingThrottle,
                            input.ignition, crankingState.starterEngaged, readUnderrunCount(telemetryReader),
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
