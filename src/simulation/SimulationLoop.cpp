// SimulationLoop.cpp - Simulation loop implementation
// Extracted from engine_sim_cli.cpp for SOLID SRP compliance
// Phase E: Uses ISimulator* instead of EngineSimHandle/EngineSimAPI&
// Phase F: Moved to engine-sim-bridge for reusability (GUI, iOS, headless)

#include "simulation/SimulationLoop.h"

#include "simulator/ISimulator.h"
#include "simulator/EngineSimTypes.h"

#include "strategy/AudioLoopConfig.h"
#include "hardware/IAudioHardwareProvider.h"
#include "hardware/IAudioHardwareProvider.h"
#include "strategy/IAudioBuffer.h"
#include "strategy/Diagnostics.h"
#include "io/IInputProvider.h"
#include "io/IPresentation.h"
#include "common/ILogging.h"
#include "telemetry/ITelemetryProvider.h"
#include "common/Verification.h"

#include <cstring>
#include <stdexcept>
#include <atomic>
#include <csignal>
#include <iostream>
#include <thread>
#include <chrono>

// ============================================================================
// SimulationConfig Implementation
// ============================================================================

SimulationConfig::~SimulationConfig() {
    delete engineConfig;
}

// ============================================================================
// Private Helper Functions - SRP Compliance
// ============================================================================

namespace {

// Timed input simulation constants
static constexpr double THROTTLE_RAMP_DURATION_SECONDS = 0.5;  // Time to ramp from 0 to 1
static constexpr double FULL_THROTTLE = 1.0;                     // Maximum throttle value
static constexpr double SECONDS_TO_MICROSECONDS = 1000000.0;

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

void enableStarterMotor(ISimulator& simulator) {
    simulator.setStarterMotor(true);
}

bool checkStarterMotorRPM(ISimulator& simulator, double minSustainedRPM) {
    EngineSimStats stats = simulator.getStats();
    if (stats.currentRPM > minSustainedRPM) {
        simulator.setStarterMotor(false);
        return true;
    }
    return false;
}

// Get input for non-interactive (timed) mode: ramp throttle from 0 to 1 over THROTTLE_RAMP_DURATION_SECONDS
input::EngineInput getTimedInput(double currentTime) {
    input::EngineInput input;  // Uses struct defaults: ignition=true, starterMotor=false, shouldContinue=true
    input.throttle = currentTime < THROTTLE_RAMP_DURATION_SECONDS
        ? currentTime / THROTTLE_RAMP_DURATION_SECONDS
        : FULL_THROTTLE;
    return input;
}

void updatePresentation(presentation::IPresentation* presentation, const SimulationConfig& config,
                        double currentTime,
                        const EngineSimStats& stats, double throttle, bool ignition,
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
    state.starterMotor = false;
    state.exhaustFlow = stats.exhaustFlow;
    state.renderMs = timing.renderMs;
    state.headroomMs = timing.headroomMs;
    state.budgetPct = timing.budgetPct;
    state.framesRequested = timing.framesRequested;
    state.framesRendered = timing.framesRendered;
    state.callbackRateHz = timing.callbackRateHz;
    state.generatingRateFps = timing.generatingRateFps;
    state.trendPct = timing.trendPct;
    state.sampleRate = config.sampleRate();

    presentation->ShowEngineState(state);
}

void writeTelemetry(telemetry::ITelemetryWriter* telemetryWriter,
                    double currentTime,
                    double throttle,
                    bool ignition) {
    if (!telemetryWriter) return;

    // Push vehicle inputs (loop owns throttle/ignition, simulator doesn't)
    telemetry::VehicleInputsTelemetry inputs;
    inputs.throttlePosition = throttle;
    inputs.ignitionOn = ignition;
    inputs.starterMotorEngaged = false;
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

void runWarmupPhase(ISimulator& simulator,
                   const SimulationConfig& config,
                   bool drainDuringWarmup) {
    double smoothedThrottle = 0.6;
    double currentTime = 0.0;

    for (int i = 0; i < AudioLoopConfig::WARMUP_ITERATIONS; i++) {
        EngineSimStats stats = simulator.getStats();

        simulator.setThrottle(smoothedThrottle);
        simulator.update(config.updateInterval());

        currentTime += config.updateInterval();

        if (drainDuringWarmup) {
            std::vector<float> discardBuffer(config.framesPerUpdate() * EngineSimAudio::STEREO);
            int warmupRead = 0;

            for (int retry = 0; retry <= 3 && warmupRead < config.framesPerUpdate(); retry++) {
                int readThisTime = 0;
                simulator.renderOnDemand(
                    discardBuffer.data() + warmupRead * EngineSimAudio::STEREO,
                    config.framesPerUpdate() - warmupRead,
                    &readThisTime);

                if (readThisTime > 0) warmupRead += readThisTime;
            }
        }
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

    const double minSustainedRPM = 550.0;

    double throttle = 0.1;
    bool ignition = true;

    logger->info(LogMask::BRIDGE, "runUnifiedAudioLoop starting simulation loop with %s", config.simulatorLabel.c_str());

    while (true) {
        checkStarterMotorRPM(simulator, minSustainedRPM);

        // Poll input: interactive mode uses OnUpdateSimulation, timed mode uses duration check
        if (inputProvider) {
            input::EngineInput engineInput = inputProvider->OnUpdateSimulation(config.updateInterval());
            if (!engineInput.shouldContinue) {
                break;  // Input provider signalled termination
            }
            throttle = engineInput.throttle;
            ignition = engineInput.ignition;
        } else {
            if (currentTime >= config.duration) {
                break;
            }
            auto timedInput = getTimedInput(currentTime);
            throttle = timedInput.throttle;
            ignition = timedInput.ignition;
        }

        simulator.setThrottle(throttle);
        simulator.setIgnition(ignition);

        // Update simulation via strategy (threaded mode updates here; sync-pull is no-op)
        audioBuffer.updateSimulation(&simulator, config.updateInterval() * 1000.0);

        EngineSimStats stats = simulator.getStats();

        // Generate audio: strategy decides whether to fill buffer (Threaded fills, SyncPull no-ops)
        audioBuffer.fillBufferFromEngine(&simulator, config.framesPerUpdate());

        writeTelemetry(telemetryWriter, currentTime, throttle, ignition);

        // Read underrun count from telemetry (pushed by ThreadedStrategy)
        int underrunCount = 0;
        if (telemetryReader) {
            auto audioDiag = telemetryReader->getAudioDiagnostics();
            underrunCount = audioDiag.underrunCount;
        }

        currentTime += config.updateInterval();

        // Display via presentation (ConsolePresentation formats the complete output line)
        updatePresentation(presentation, config, currentTime, stats, throttle, ignition, underrunCount, audioBuffer, telemetryReader);

        // Pace to 60Hz using sleep_until for accuracy
        timer.waitUntilNextTick();
    }

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
    ASSERT(config.engineConfig, "config.engineConfig must be set");
    ASSERT(config.engineConfig->sampleRate > 0, "config.sampleRate must be set");
    ASSERT(config.updateInterval() > 0.0, "config.updateInterval must be set");
    ASSERT(config.framesPerUpdate() > 0, "config.framesPerUpdate must be set");

    // Initialize simulator (throws on failure)
    initializeSimulator(simulator, config, logger, telemetryWriter, config.engineConfig);

    // Initialize strategy
    AudioBufferConfig strategyConfig;
    strategyConfig.channels = 2;
    strategyConfig.synthLatency = config.engineConfig->targetSynthesizerLatency;

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

    enableStarterMotor(simulator);

    bool drainDuringWarmup = config.playAudio && audioBuffer->shouldDrainDuringWarmup();
    runWarmupPhase(simulator, config, drainDuringWarmup);

    // Prepare buffer via strategy (threaded: pre-fills, sync-pull: no-op)
    audioBuffer->prepareBuffer();

    // Reset buffer after warmup via strategy
    audioBuffer->resetBufferAfterWarmup();

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
