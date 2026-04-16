// SyncPullStrategy.cpp - Lock-step audio strategy
// Implements synchronous audio generation where simulation advances with audio playback
// SRP: Single responsibility - only implements synchronous lock-step rendering
// OCP: New strategies can be added without modifying existing code
// DIP: Depends on abstractions (ISimulator), not concrete implementations
// Phase F: Moved to engine-sim-bridge submodule

#include "strategy/SyncPullStrategy.h"
#include "strategy/IAudioStrategy.h"
#include "simulator/ISimulator.h"
#include "common/ILogging.h"
#include "common/Verification.h"
#include "simulator/engine_sim_bridge.h"

#include <cstring>

// ============================================================================
// NullTelemetryWriter - Silently discards all telemetry writes
// ============================================================================

namespace {

class NullTelemetryWriter : public telemetry::ITelemetryWriter {
public:
    void writeEngineState(const telemetry::EngineStateTelemetry&) override {}
    void writeFramePerformance(const telemetry::FramePerformanceTelemetry&) override {}
    void writeAudioDiagnostics(const telemetry::AudioDiagnosticsTelemetry&) override {}
    void writeAudioTiming(const telemetry::AudioTimingTelemetry&) override {}
    void writeVehicleInputs(const telemetry::VehicleInputsTelemetry&) override {}
    void writeSimulatorMetrics(const telemetry::SimulatorMetricsTelemetry&) override {}
    void reset() override {}
    const char* getName() const override { return "NullTelemetryWriter"; }
};

} // anonymous namespace

// ============================================================================
// SyncPullStrategy Implementation
// ============================================================================

SyncPullStrategy::SyncPullStrategy(ILogging* logger, telemetry::ITelemetryWriter* telemetry)
    : defaultLogger_(logger ? nullptr : new ConsoleLogger())
    , logger_(logger ? logger : defaultLogger_.get())
    , defaultTelemetry_(telemetry ? nullptr : new NullTelemetryWriter())
    , telemetry_(telemetry ? telemetry : defaultTelemetry_.get())
{
    ASSERT(logger_, "SyncPullStrategy: logger must not be null");
    ASSERT(telemetry_, "SyncPullStrategy: telemetry must not be null");
}

// ============================================================================
// IAudioStrategy Implementation
// ============================================================================

const char* SyncPullStrategy::getName() const {
    return "SyncPull";
}

bool SyncPullStrategy::isEnabled() const {
    return true;
}

bool SyncPullStrategy::isPlaying() const {
    return audioState_.isPlaying.load();
}

bool SyncPullStrategy::shouldDrainDuringWarmup() const {
    return false;
}

void SyncPullStrategy::fillBufferFromEngine(ISimulator*, int) {
    // SyncPull generates audio on-demand in the render callback via ISimulator.
}

// ============================================================================
// Lifecycle Method Implementations
// ============================================================================

bool SyncPullStrategy::initialize(const AudioStrategyConfig& config) {
    ASSERT(logger_, "SyncPullStrategy::initialize: logger must not be null");

    audioState_.sampleRate = config.sampleRate;
    audioState_.isPlaying = false;
    shuttingDown_.store(false);
    diagnostics_.setSampleRate(config.sampleRate);

    logger_->info(LogMask::AUDIO,
                  "SyncPullStrategy initialized: sampleRate=%dHz, channels=%d",
                  config.sampleRate, config.channels);

    return true;
}

void SyncPullStrategy::prepareBuffer() {
    logger_->debug(LogMask::AUDIO, "SyncPullStrategy::prepareBuffer: No-op for sync-pull mode");
}

bool SyncPullStrategy::startPlayback(ISimulator* simulator) {
    shuttingDown_.store(false);
    simulator_ = simulator;
    audioState_.isPlaying.store(true);
    lastThroughputTime_ = std::chrono::steady_clock::now();

    logger_->info(LogMask::AUDIO, "SyncPullStrategy::startPlayback: On-demand rendering started");

    return true;
}

void SyncPullStrategy::stopPlayback(ISimulator* simulator) {
    shuttingDown_.store(true);
    simulator_ = nullptr;
    audioState_.isPlaying.store(false);

    logger_->info(LogMask::AUDIO, "SyncPullStrategy::stopPlayback: On-demand rendering stopped");
}

void SyncPullStrategy::resetBufferAfterWarmup() {
    logger_->debug(LogMask::AUDIO, "SyncPullStrategy::resetBufferAfterWarmup: No-op for sync-pull mode");
}

void SyncPullStrategy::updateSimulation(ISimulator* simulator, double deltaTimeMs) {
    // Sync-pull mode updates simulation during render callback
}

bool SyncPullStrategy::render(
    AudioBufferList* ioData,
    UInt32 numberFrames
) {
    if (!ioData) {
        return false;
    }

    if (!simulator_ || shuttingDown_.load()) {
        // Fill silence on shutdown or null simulator to prevent crackles
        if (ioData->mBuffers[0].mData) {
            float* data = static_cast<float*>(ioData->mBuffers[0].mData);
            EngineSimAudio::fillSilence(data, static_cast<int>(numberFrames));
        }
        return true;
    }

    auto callbackStart = std::chrono::high_resolution_clock::now();

    constexpr int MAX_RETRIES = 3;

    int framesToGenerate = static_cast<int>(numberFrames);
    int framesRendered = 0;

    float* audioData = static_cast<float*>(ioData->mBuffers[0].mData);
    int remainingFrames = framesToGenerate;

    while (remainingFrames > 0 && framesRendered < framesToGenerate && !shuttingDown_.load()) {
        int32_t framesWritten = 0;
        bool result = simulator_->renderOnDemand(
            audioData + (framesRendered * 2),
            remainingFrames,
            &framesWritten
        );

        if (!result) {
            logger_->error(LogMask::AUDIO, "SyncPullStrategy::render: renderOnDemand failed, filling silence");
            EngineSimAudio::fillSilence(audioData, framesToGenerate);
            return true;
        }

        framesRendered += framesWritten;
        remainingFrames -= framesWritten;

        // Partial render: advance simulation and retry up to MAX_RETRIES
        if (framesWritten > 0 && framesWritten < remainingFrames + framesWritten) {
            // Successfully got some frames but not all -- continue loop naturally
            continue;
        }

        if (framesWritten == 0 && remainingFrames > 0) {
            // No frames produced: advance simulation and retry
            int retryCount = 0;
            while (retryCount < MAX_RETRIES && !shuttingDown_.load()) {
                simulator_->update(1.0 / 48000.0);  // One sample period to feed synthesizer
                result = simulator_->renderOnDemand(
                    audioData + (framesRendered * 2),
                    remainingFrames,
                    &framesWritten
                );

                if (!result) {
                    logger_->error(LogMask::AUDIO, "SyncPullStrategy::render: renderOnDemand failed during retry, filling silence");
                    EngineSimAudio::fillSilence(audioData, framesToGenerate);
                    return true;
                }

                if (framesWritten > 0) {
                    framesRendered += framesWritten;
                    remainingFrames -= framesWritten;
                    break;
                }
                retryCount++;
            }

            if (framesWritten == 0 && remainingFrames > 0) {
                logger_->warning(LogMask::AUDIO,
                    "SyncPullStrategy::render: exhausted %d retries, filling silence for %d frames",
                    MAX_RETRIES, remainingFrames);
                break;
            }
        }
    }

    // Fill remaining buffer with silence on partial render to prevent crackles
    if (framesRendered < framesToGenerate) {
        float* remaining = audioData + (framesRendered * 2);
        EngineSimAudio::fillSilence(remaining, framesToGenerate - framesRendered);
    }

    auto callbackEnd = std::chrono::high_resolution_clock::now();
    double renderMs = std::chrono::duration<double, std::milli>(callbackEnd - callbackStart).count();

    diagnostics_.recordRender(renderMs, framesRendered, framesToGenerate);

    // Update throughput rates once per second
    auto now = std::chrono::steady_clock::now();
    double elapsedSec = std::chrono::duration<double>(now - lastThroughputTime_).count();
    if (elapsedSec >= 1.0) {
        diagnostics_.updateThroughput(elapsedSec);
        lastThroughputTime_ = now;
    }

    // Push timing diagnostics to telemetry
    auto snap = diagnostics_.getSnapshot();
    telemetry::AudioTimingTelemetry timing;
    timing.renderMs = snap.lastRenderMs;
    timing.headroomMs = snap.lastHeadroomMs;
    timing.budgetPct = snap.lastBudgetPct;
    timing.framesRequested = snap.lastFramesRequested;
    timing.framesRendered = snap.lastFramesRendered;
    timing.callbackRateHz = snap.callbackRateHz;
    timing.generatingRateFps = snap.generatingRateFps;
    timing.trendPct = snap.trendPct;
    telemetry_->writeAudioTiming(timing);

    return true;
}

bool SyncPullStrategy::AddFrames(
    float* buffer,
    int frameCount
) {
    logger_->debug(LogMask::AUDIO, "SyncPullStrategy::AddFrames: No-op for sync-pull mode");
    return true;
}

void SyncPullStrategy::reset() {
    logger_->debug(LogMask::AUDIO, "SyncPullStrategy reset: No-op for sync-pull mode");
}

std::string SyncPullStrategy::getModeString() const {
    return "SYNC-PULL";
}
