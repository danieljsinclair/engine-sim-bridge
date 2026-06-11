// SyncPullStrategy.cpp - Lock-step audio strategy
// Implements synchronous audio generation where simulation advances with audio playback
// SRP: Single responsibility - only implements synchronous lock-step rendering
// OCP: New strategies can be added without modifying existing code
// DIP: Depends on abstractions (ISimulator), not concrete implementations
// Phase F: Moved to engine-sim-bridge submodule

#include "strategy/SyncPullStrategy.h"
#include "strategy/IAudioBuffer.h"
#include "simulator/ISimulator.h"
#include "common/ILogging.h"
#include "common/Verification.h"
#include "simulator/EngineSimTypes.h"
#include "telemetry/NullTelemetryWriter.h"

#include <cstring>
#include <memory>

// ============================================================================
// SyncPullStrategy Implementation
// ============================================================================

SyncPullStrategy::SyncPullStrategy(ILogging* logger, telemetry::ITelemetryWriter* telemetry)
    : defaultLogger_(logger ? nullptr : std::make_unique<ConsoleLogger>())
    , logger_(logger ? logger : defaultLogger_.get())
    , defaultTelemetry_(telemetry ? nullptr : std::make_unique<NullTelemetryWriter>())
    , telemetry_(telemetry ? telemetry : defaultTelemetry_.get())
{
    ASSERT(logger_, "SyncPullStrategy: logger must not be null");
    ASSERT(telemetry_, "SyncPullStrategy: telemetry must not be null");
}

// ============================================================================
// IAudioBuffer Implementation
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

bool SyncPullStrategy::initialize(const AudioBufferConfig& config, int sampleRate) {
    ASSERT(logger_, "SyncPullStrategy::initialize: logger must not be null");

    audioState_.sampleRate = sampleRate;
    sampleRate_ = sampleRate;
    audioState_.isPlaying = false;
    shuttingDown_.store(false);
    diagnostics_.setSampleRate(sampleRate);

    logger_->info(LogMask::AUDIO,
                  "SyncPullStrategy initialized: sampleRate=%dHz, channels=%d",
                  sampleRate, config.channels);

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

    // Reset crossfade state on start
    lastLeftSample_ = 0.0f;
    lastRightSample_ = 0.0f;
    crossfadeSamplesRemaining_ = 0;

    logger_->info(LogMask::AUDIO, "SyncPullStrategy::startPlayback: On-demand rendering started");

    return true;
}

void SyncPullStrategy::stopPlayback(ISimulator* /* simulator */) {
    shuttingDown_.store(true);
    simulator_ = nullptr;
    audioState_.isPlaying.store(false);

    logger_->info(LogMask::AUDIO, "SyncPullStrategy::stopPlayback: On-demand rendering stopped");
}

void SyncPullStrategy::swapSimulator(ISimulator* newSimulator) {
    // Seamless hot-swap: replace the simulator pointer without stopping audio.
    // The IOThread callback will see the new pointer on its next invocation.
    // Caller must keep the old simulator alive (via previousSimulator_) to
    // prevent use-after-free for any in-flight render.
    simulator_ = newSimulator;

    // Enable crossfade for next render to prevent clicks/pops
    crossfadeSamplesRemaining_ = CROSSFADE_SAMPLES;
}

void SyncPullStrategy::resetBufferAfterWarmup() {
    logger_->debug(LogMask::AUDIO, "SyncPullStrategy::resetBufferAfterWarmup: No-op for sync-pull mode");
}

void SyncPullStrategy::updateSimulation(ISimulator* /* simulator */, double /* deltaTimeMs */) {
    // Sync-pull mode updates simulation during render callback
}

bool SyncPullStrategy::retryRender(float* dst, int offset, int framesNeeded,
                                    int& framesWritten, int maxRetries) {
    int retryCount = 0;
    while (retryCount < maxRetries && !shuttingDown_.load()) {
        simulator_->update(1.0 / sampleRate_);
        bool result = simulator_->renderOnDemand(
            dst + (offset * 2),
            framesNeeded,
            &framesWritten
        );

        if (!result) {
            return false;
        }

        if (framesWritten > 0) {
            return true;
        }
        retryCount++;
    }
    return true;
}

// ============================================================================
// Render Helpers
// ============================================================================

bool SyncPullStrategy::attemptRender(float* dst, int offset, int framesNeeded,
                                      int32_t& framesWritten) {
    bool result = simulator_->renderOnDemand(
        dst + (offset * 2),
        framesNeeded,
        &framesWritten
    );

    if (!result) {
        logger_->error(LogMask::AUDIO, "SyncPullStrategy::render: renderOnDemand failed, filling silence");
    }
    return result;
}

bool SyncPullStrategy::render(AudioBufferView& buffer) {
    float* dst = buffer.asFloat();
    if (!dst) {
        return false;
    }

    if (!simulator_ || shuttingDown_.load()) {
        EngineSimAudio::fillSilence(dst, buffer.frameCount);
        return true;
    }

    auto callbackStart = std::chrono::high_resolution_clock::now();

    constexpr int MAX_RETRIES = 3;

    int framesToGenerate = buffer.frameCount;
    int framesRendered = 0;
    int remainingFrames = framesToGenerate;

    while (remainingFrames > 0 && framesRendered < framesToGenerate && !shuttingDown_.load()) {
        int32_t framesWritten = 0;
        if (!attemptRender(dst, framesRendered, remainingFrames, framesWritten)) {
            EngineSimAudio::fillSilence(dst, framesToGenerate);
            return true;
        }

        framesRendered += framesWritten;
        remainingFrames -= framesWritten;

        bool gotPartial = (framesWritten > 0 && remainingFrames > 0);
        if (gotPartial) {
            continue;
        }

        bool needsRetry = (framesWritten == 0 && remainingFrames > 0);
        if (needsRetry) {
            int32_t retryFrames = 0;
            bool retryOk = retryRender(dst, framesRendered, remainingFrames, retryFrames, MAX_RETRIES);

            if (!retryOk) {
                logger_->error(LogMask::AUDIO, "SyncPullStrategy::render: renderOnDemand failed during retry, filling silence");
                EngineSimAudio::fillSilence(dst, framesToGenerate);
                return true;
            }

            if (retryFrames > 0) {
                framesRendered += retryFrames;
                remainingFrames -= retryFrames;
            } else {
                logger_->warning(LogMask::AUDIO,
                    "SyncPullStrategy::render: exhausted %d retries, filling silence for %d frames",
                    MAX_RETRIES, remainingFrames);
                break;
            }
        }
    }

    fillRemainingSilence(dst, framesRendered, framesToGenerate, remainingFrames);
    applyCrossfade(dst, framesRendered);
    resetFrameRender(framesToGenerate, framesRendered, dst, callbackStart);
    updateTelemetry();

    return true;
}

void SyncPullStrategy::fillRemainingSilence(float* dst, int framesRendered, int framesToGenerate, int remainingFrames) {
    // Fill remaining buffer with silence on partial render to prevent crackles
    if (framesRendered < framesToGenerate) {
        // NOTE: Should never happen in practice - could probably just throw an exception here
        logger_->warning(LogMask::AUDIO,
            "SyncPullStrategy::render: rendered %d/%d frames, filling remaining %d with silence",
            framesRendered, framesToGenerate, remainingFrames);
        float* remaining = dst + (framesRendered * 2);
        EngineSimAudio::fillSilence(remaining, framesToGenerate - framesRendered);
    }
}


void SyncPullStrategy::applyCrossfade(float* dst, int framesRendered) {
    // Apply crossfade if hot-swap is in progress
    if (crossfadeSamplesRemaining_ > 0 && framesRendered > 0) {
        int samplesToCrossfade = std::min(framesRendered * 2, crossfadeSamplesRemaining_);
        float inv = 1.0f / static_cast<float>(CROSSFADE_SAMPLES);

        for (int i = 0; i < samplesToCrossfade; i += 2) {
            float mix = (static_cast<float>(CROSSFADE_SAMPLES - crossfadeSamplesRemaining_ + i) * inv);
            mix = std::max(0.0f, std::min(1.0f, mix)); // Clamp to [0, 1]

            // Left channel
            float newLeft = dst[i];
            dst[i] = lastLeftSample_ * (1.0f - mix) + newLeft * mix;

            // Right channel
            float newRight = dst[i + 1];
            dst[i + 1] = lastRightSample_ * (1.0f - mix) + newRight * mix;
        }

        crossfadeSamplesRemaining_ -= samplesToCrossfade;
        if (crossfadeSamplesRemaining_ < 0) {
            crossfadeSamplesRemaining_ = 0;
        }
    }

}

void SyncPullStrategy::resetFrameRender(int framesToGenerate, int framesRendered, float* dst, std::chrono::high_resolution_clock::time_point callbackStart) {
    // Track last sample values for next crossfade (update after crossfade)
    if (framesRendered > 0) {
        lastLeftSample_ = dst[(framesRendered - 1) * 2];
        lastRightSample_ = dst[(framesRendered - 1) * 2 + 1];
    }

    auto callbackEnd = std::chrono::high_resolution_clock::now();
    double renderMs = std::chrono::duration<double, std::milli>(callbackEnd - callbackStart).count();

    diagnostics_.recordRender(renderMs, framesRendered, framesToGenerate);

    // Update throughput rates once per second
    auto now = std::chrono::steady_clock::now();
    if (double elapsedSec = std::chrono::duration<double>(now - lastThroughputTime_).count(); elapsedSec >= 1.0) {
        diagnostics_.updateThroughput(elapsedSec);
        lastThroughputTime_ = now;
    }

}

void SyncPullStrategy::updateTelemetry() {
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
}

bool SyncPullStrategy::AddFrames(
    float* /* buffer */,
    int /* frameCount */
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
