// SimulationLoop.cpp - Simulation loop implementation
// Extracted from engine_sim_cli.cpp for SOLID SRP compliance
// Phase E: Uses ISimulator* instead of EngineSimHandle/EngineSimAPI&
// Phase F: Moved to engine-sim-bridge for reusability (GUI, iOS, headless)

#include "simulation/SimulationLoop.h"

#include "simulator/ISimulator.h"
#include "simulator/EngineSimTypes.h"

#include "hardware/IAudioHardwareProvider.h"
#include "strategy/IAudioBuffer.h"
#include "io/IInputProvider.h"
#include "io/IPresentation.h"
#include "common/ILogging.h"
#include "telemetry/ITelemetryProvider.h"
#include "common/Verification.h"

#include <cstring>
#include <memory>
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
};

InputResult pollInput(input::IInputProvider* inputProvider, double currentTime, double duration, double updateInterval) {
    if (inputProvider) {
        auto engineInput = inputProvider->OnUpdateSimulation(updateInterval);
        return {engineInput.throttle, engineInput.ignition, engineInput.shouldContinue};
    }
    if (currentTime >= duration) {
        return {0.1, true, false};
    }
    auto timed = input::EngineInput{};
    timed.throttle = currentTime < THROTTLE_RAMP_DURATION_SECONDS
        ? currentTime / THROTTLE_RAMP_DURATION_SECONDS : FULL_THROTTLE;
    return {timed.throttle, timed.ignition, true};
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

// ============================================================================
// Audio render callback — extracted concerns (SRP)
// ============================================================================

void applySpeakerProtection(AudioBufferView& buffer, float drive, ILogging* logger) {
    float* dst = buffer.asFloat();

    const size_t totalSamples = static_cast<size_t>(buffer.frameCount) * buffer.channelCount;

    float peakBefore = SpeakerProtection::peakAbs(dst, totalSamples);
    SpeakerProtection::protectBuffer(dst, buffer.frameCount, buffer.channelCount, drive);
    float peakAfter = SpeakerProtection::peakAbs(dst, totalSamples);

        {
            static int count = 0;
            if (++count % 60 == 0) {
                float reductionDb = (peakAfter > 0.0001f) ? 20.0f * std::log10(peakAfter / std::max(peakBefore, 0.0001f)) : 0.0f;
                logger->info(LogMask::DIAGNOSTICS, "Speaker protection: peak %.3f -> %.3f (%.1f dB)", peakBefore, peakAfter, reductionDb);
            }
    }
}

void logBudgetBreaches(const Diagnostics& diagnostics, ILogging* logger) {
    int64_t totalBreaches = diagnostics.breachCount.load();
    if (totalBreaches % 10 == 0) {
        double renderMs = diagnostics.lastRenderMs.load();
        double budgetMs = renderMs + diagnostics.lastHeadroomMs.load();
        logger->info(LogMask::DIAGNOSTICS, "Budget breach: %.1fms render > %.1fms budget (%lld total)", renderMs, budgetMs, totalBreaches);
    }
}

// Sample discontinuity diagnostics — measures audio content quality
// Captures max sample-to-sample delta (discontinuity) and inter-callback continuity
struct SampleDiagnostics {
    static constexpr float DISCONTINUITY_THRESHOLD = 0.15f;
    static constexpr int MAX_CHANNELS = 2;
    float lastSamples[MAX_CHANNELS] = {};
    int callbackCount = 0;

    const float* previousSamples() const { return lastSamples; }

    void measure(const float* buffer, int frameCount, int channelCount, ILogging* logger) {
        int totalSamples = frameCount * channelCount;

        float interDelta = std::fabs(buffer[0] - lastSamples[0]);

        float maxDelta = 0.0f;
        int maxDeltaIdx = 0;
        float prev = lastSamples[0];
        for (int i = 0; i < totalSamples; ++i) {
            float delta = std::fabs(buffer[i] - prev);
            if (delta > maxDelta) {
                maxDelta = delta;
                maxDeltaIdx = i;
            }
            prev = buffer[i];
        }

        float peak = SpeakerProtection::peakAbs(buffer, totalSamples);
        for (int ch = 0; ch < std::min(channelCount, MAX_CHANNELS); ++ch) {
            lastSamples[ch] = buffer[(frameCount - 1) * channelCount + ch];
        }
        callbackCount++;

        bool anomaly = maxDelta >= DISCONTINUITY_THRESHOLD;
        if (callbackCount % 60 == 0 || anomaly) {
            logger->info(LogMask::DIAGNOSTICS,
                "Audio content: peak=%.4f maxDelta=%.4f@%d interDelta=%.4f first=%.4f last=%.4f%s",
                peak, maxDelta, maxDeltaIdx, interDelta,
                buffer[0], buffer[totalSamples - 1],
                anomaly ? " ** DISCONTINUITY **" : "");
        }
    }
};

// Audio render callback — orchestrates extracted concerns
int audioRenderCallback(IAudioBuffer* strategy, AudioBufferView& buffer,
                        bool enableProtection, float drive,
                        bool enableBreachRecovery,
                        bool enableTrendHold,
                        SpeakerProtection::BreachRecoveryState* breachState,
                        SpeakerProtection::TrendHoldState* trendState,
                        SampleDiagnostics* sampleDiag,
                        ILogging* logger) {
    int result = 0;

    if (strategy->isPlaying()) {
        float* dst = buffer.asFloat();

        {
            // Capture previous render's headroom before this render overwrites diagnostics
            double previousHeadroomMs = strategy->diagnostics().lastHeadroomMs.load();

            strategy->render(buffer);

            // Save tail for breach cross-fade (after every render)
            if (enableBreachRecovery) {
                breachState->saveTail(dst, buffer.frameCount, buffer.channelCount);
            }

            // Breach recovery: cross-fade from held tail on budget breach
            // Log every breach event with actual sample values for evidence
            if (enableBreachRecovery && previousHeadroomMs < SpeakerProtection::BREACH_SMOOTH_THRESHOLD_MS) {
                float heldFirst = breachState->heldTail[0];
                float heldLast = breachState->heldTail[SpeakerProtection::BREACH_XFADE_SAMPLES * 2 - 1];
                float newFirst = dst[0];
                float newLast = dst[buffer.frameCount * buffer.channelCount - 1];
                logger->info(LogMask::DIAGNOSTICS,
                    "BREACH RECOVERY: prev=%.2fms held=[%.4f..%.4f] new=[%.4f..%.4f] delta=%.4f",
                    previousHeadroomMs, heldFirst, heldLast, newFirst, newLast,
                    std::fabs(newFirst - heldLast));
                bool applied = breachState->applyCrossfade(dst, buffer.frameCount, buffer.channelCount, previousHeadroomMs);
                if (applied) {
                    logger->info(LogMask::DIAGNOSTICS,
                        "BREACH RECOVERY: cross-fade applied, after=[%.4f,%.4f,%.4f]",
                        dst[0], dst[2], dst[4]);
                }
            }
        }

        // Content-based discontinuity smoothing — detects sharp sample-to-sample
        // deltas and replaces them with linear ramps. Catches crackles from any source.
        SpeakerProtection::smoothDiscontinuities(dst, buffer.frameCount, buffer.channelCount,
                                                  0.3f, 24, sampleDiag->previousSamples());

        // Speaker protection: soft-clip + peak limiting
        if (enableProtection) {
            applySpeakerProtection(buffer, drive, logger);
        }

        // Measure audio content quality — what the DAC actually receives
        sampleDiag->measure(dst, buffer.frameCount, buffer.channelCount, logger);
    }

    return result;
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
    state.starterMotor = starterEngaged;
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

    logger->info(LogMask::BRIDGE, "runUnifiedAudioLoop starting simulation loop with %s", config.simulatorLabel.c_str());

    InputResult input;
    do {
        auto cranking = crankingState.step(simulator, input.throttle, logger);

        simulator.setThrottle(cranking.effectiveThrottle);
        simulator.setIgnition(input.ignition);

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
    ASSERT(logger, "logger must be provided");
    ASSERT(audioBuffer, "audioBuffer must be provided");
    ASSERT(config.engineConfig.sampleRate > 0, "config.sampleRate must be set");
    ASSERT(config.updateInterval() > 0.0, "config.updateInterval must be set");
    ASSERT(config.framesPerUpdate() > 0, "config.framesPerUpdate must be set");

    // Initialize simulator (throws on failure)
    initializeSimulator(simulator, config, logger, telemetryWriter, &config.engineConfig);

    // Initialize strategy
    AudioBufferConfig strategyConfig;
    strategyConfig.channels = 2;
    strategyConfig.synthLatency = config.engineConfig.targetSynthesizerLatency;

    if (!audioBuffer->initialize(strategyConfig, config.sampleRate())) {
        throw std::runtime_error("Failed to initialize audio strategy");
    }

    // Create and initialize audio hardware provider (throws on failure)
    auto breachState = std::make_shared<SpeakerProtection::BreachRecoveryState>();
    auto trendState = std::make_shared<SpeakerProtection::TrendHoldState>();
    auto sampleDiag = std::make_shared<SampleDiagnostics>();

    auto callback = [audioBuffer, config, logger, breachState, trendState, sampleDiag](AudioBufferView& buffer) -> int {
        return audioRenderCallback(audioBuffer, buffer,
                                   config.engineConfig.speakerProtection,
                                   config.engineConfig.speakerProtectionDrive,
                                   config.engineConfig.breachRecovery,
                                   config.engineConfig.trendHold,
                                   breachState.get(),
                                   trendState.get(),
                                   sampleDiag.get(),
                                   logger);
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
