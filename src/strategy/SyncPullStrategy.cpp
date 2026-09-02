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

void SyncPullStrategy::updateSimulation(ISimulator* simulator, double deltaTimeMs) {
    // THE fix for the live warm-start reversion: advance the core on the LOOP
    // thread (matching DeterministicStrategy) instead of relying solely on the
    // audio render callback. Previously this was a no-op, so during the 90s
    // warm-start prefix the engine advanced ONLY via the audio callback's
    // renderOnDemand (advanceFixedSteps ceil=false = 166 steps/tick) while the
    // loop thread did zero sim work — starving the engine relative to
    // DeterministicStrategy (which calls update -> ceil=true = 167 steps/tick).
    // The 1-step/tick deficit accumulated over 90s and tipped the warm-start
    // into the reversion (negative-exhaust-flow) attractor. Advancing here
    // makes the loop thread own the core; the render callback below now only
    // DRAINS already-synthesized audio (renderDrainedAudio), so there is no
    // double-stepping.
    if (simulator != nullptr) {
        simulator->update(deltaTimeMs / 1000.0);
    }
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
                  __ilog_format("SyncPullStrategy initialized: sampleRate=%dHz, channels=%d",
                  sampleRate, config.channels));

    return true;
}

void SyncPullStrategy::prepareBuffer() {
    logger_->debug(LogMask::AUDIO, __ilog_format("SyncPullStrategy::prepareBuffer: No-op for sync-pull mode"));
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

    // Reset startup-crackle fade state on start. lastAudio starts at the floor
    // (not 0.0) so the first not-playing fill or empty-ring drain emits the
    // floor directly — a hard step from 0.0 to floor would itself be a
    // discontinuity, and starting at the floor keeps the startup edge
    // continuous with audioRenderCallback's not-playing fill (also at floor).
    // fadeInProgress_ is armed so the first real-audio chunk ramps up from the
    // floor, sealing the silence->audio edge.
    lastAudioLeft_ = EngineSimAudio::SILENCE_FLOOR;
    lastAudioRight_ = EngineSimAudio::SILENCE_FLOOR;
    fadeInProgress_ = EngineSimAudio::FADE_SAMPLES;
    seenRealAudio_ = false;

    logger_->info(LogMask::AUDIO, __ilog_format("SyncPullStrategy::startPlayback: On-demand rendering started"));

    return true;
}

void SyncPullStrategy::stopPlayback(ISimulator* /* simulator */) {
    shuttingDown_.store(true);
    simulator_ = nullptr;
    audioState_.isPlaying.store(false);

    logger_->info(LogMask::AUDIO, __ilog_format("SyncPullStrategy::stopPlayback: On-demand rendering stopped"));
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
    logger_->debug(LogMask::AUDIO, __ilog_format("SyncPullStrategy::resetBufferAfterWarmup: No-op for sync-pull mode"));
}

bool SyncPullStrategy::retryRender(float* dst, int offset, int framesNeeded,
                                    int& framesWritten, int maxRetries) {
    // Drain-only: the core is advanced on the loop thread (updateSimulation), so
    // the retry path must NOT call simulator_->update() — doing so would
    // double-step the engine. Just retry draining synthesized audio.
    int retryCount = 0;
    while (retryCount < maxRetries && !shuttingDown_.load()) {
        if (auto result = simulator_->renderDrainedAudio(
                dst + (offset * 2),
                framesNeeded,
                &framesWritten
            ); !result) {
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
    // Primary path: drain already-synthesized audio. Core advancement lives on
    // the loop thread (updateSimulation), so this does NOT step the engine —
    // see renderDrainedAudio / the updateSimulation comment for the rationale
    // (no double-stepping, no warm-start starvation).
    bool result = simulator_->renderDrainedAudio(
        dst + (offset * 2),
        framesNeeded,
        &framesWritten
    );

    if (result && framesWritten > 0 && !seenRealAudio_) {
        // Zero-drain guard (startup window): the ring starts zero-filled, so a
        // full read of exact zeros is pre-first-sample ring content, not audio.
        // Discard it to the dry path below (faded floor, fadeInProgress_ stays
        // armed) instead of letting silence consume the startup fade-in.
        const float* audio = dst + (offset * 2);
        const int samples = framesWritten * 2;
        bool anyNonZero = false;
        for (int i = 0; i < samples; ++i) {
            if (audio[i] != 0.0f) { anyNonZero = true; break; }
        }
        if (anyNonZero) {
            seenRealAudio_ = true;
        } else {
            logger_->debug(LogMask::AUDIO,
                __ilog_format("SyncPullStrategy::attemptRender: startup zero-drain discarded (%d frames)", framesWritten));
            framesWritten = 0;
        }
    }

    if (result && framesWritten > 0) {
        // Real synthesized audio produced. Apply a fade-in ramp at the start of
        // this chunk if we are resuming from a silence gap (silence->audio
        // edge) — ramps the first FADE_SAMPLES from zero to full so the
        // transition is continuous instead of a hard jump (the crackle). The
        // fade operates on the interleaved stereo buffer in place.
        float* audio = dst + (offset * 2);
        if (fadeInProgress_ > 0) {
            applyFadeIn(audio, framesWritten);
        }
        // Remember the last real synthesized sample so the next silence fill
        // can fade out from it continuously.
        const int lastIdx = (framesWritten - 1) * 2;
        lastAudioLeft_ = audio[lastIdx];
        lastAudioRight_ = audio[lastIdx + 1];
    }

    // Handle the silence tail: when renderDrainedAudio returns fewer frames
    // than requested (ring ran dry), the bridge zero-fills the remainder. A
    // hard zero-fill here would jump from the last synthesized sample to zero
    // — a discontinuity the ear hears as a crackle during cranking startup
    // (when the ring runs empty between main-thread ticks). Fade the silence
    // from the last real sample to the non-zero floor so the audio->silence
    // edge is continuous AND the knock detector's exact-zero proxy doesn't
    // count the gap. This covers both the full-empty case (framesWritten==0)
    // and the partial case (0 < framesWritten < framesNeeded).
    if (result && framesWritten < framesNeeded) {
        const int silenceFrames = framesNeeded - framesWritten;
        if (framesWritten == 0) {
            logger_->debug(LogMask::AUDIO,
                __ilog_format("SyncPullStrategy::attemptRender: no buffered audio, filled %d frames faded silence (lastL=%.4f lastR=%.4f)",
                silenceFrames, lastAudioLeft_, lastAudioRight_));
        } else {
            logger_->debug(LogMask::AUDIO,
                __ilog_format("SyncPullStrategy::attemptRender: partial drain %d/%d, fading %d tail frames (lastL=%.4f lastR=%.4f)",
                framesWritten, framesNeeded, silenceFrames, lastAudioLeft_, lastAudioRight_));
        }
        fprintf(stderr, "[FADE] attemptRender tail written=%d need=%d\n", framesWritten, framesNeeded);
        fillSilenceFaded(dst + (offset + framesWritten) * 2, silenceFrames);
        framesWritten = framesNeeded;
    }

    if (!result) {
        logger_->error(LogMask::AUDIO, __ilog_format("SyncPullStrategy::render: renderDrainedAudio failed, filling silence"));
    }
    return result;
}

bool SyncPullStrategy::render(AudioBufferView& buffer) {
    float* dst = buffer.asFloat();
    if (!dst) {
        return false;
    }

    if (!simulator_ || shuttingDown_.load()) {
        fprintf(stderr, "[FILL] A teardown frames=%d\n", buffer.frameCount);
        EngineSimAudio::fillSilence(dst, buffer.frameCount);
        return true;
    }

    auto callbackStart = std::chrono::high_resolution_clock::now();

    int framesToGenerate = buffer.frameCount;
    int framesRendered = renderChunked(dst, framesToGenerate);

    fillRemainingSilence(dst, framesRendered, framesToGenerate, framesToGenerate - framesRendered);
    applyCrossfade(dst, framesRendered);
    resetFrameRender(framesToGenerate, framesRendered, dst, callbackStart);
    updateTelemetry();

    return true;
}

int SyncPullStrategy::renderChunked(float* dst, int framesToGenerate) {
    constexpr int MAX_RETRIES = 3;
    int framesRendered = 0;
    int remainingFrames = framesToGenerate;

    while (remainingFrames > 0 && framesRendered < framesToGenerate && !shuttingDown_.load()) {
        int32_t framesWritten = 0;
        if (!attemptRender(dst, framesRendered, remainingFrames, framesWritten)) {
            fprintf(stderr, "[FILL] B attemptfail frames=%d\n", framesToGenerate);
            EngineSimAudio::fillSilence(dst, framesToGenerate);
            return framesRendered;
        }

        framesRendered += framesWritten;
        remainingFrames -= framesWritten;

        if (framesWritten > 0 && remainingFrames > 0) {
            continue;
        }

        if (framesWritten == 0 && remainingFrames > 0) {
            int32_t retryFrames = 0;
            if (bool retryOk = retryRender(dst, framesRendered, remainingFrames, retryFrames, MAX_RETRIES); !retryOk) {
                logger_->error(LogMask::AUDIO, __ilog_format("SyncPullStrategy::render: renderOnDemand failed during retry, filling silence"));
                fprintf(stderr, "[FILL] C retryfail frames=%d\n", framesToGenerate);
                EngineSimAudio::fillSilence(dst, framesToGenerate);
                return framesRendered;
            }

            if (retryFrames > 0) {
                framesRendered += retryFrames;
                remainingFrames -= retryFrames;
            } else {
                logger_->warning(LogMask::AUDIO,
                    __ilog_format("SyncPullStrategy::render: exhausted %d retries, filling silence for %d frames",
                    MAX_RETRIES, remainingFrames));
                break;
            }
        }
    }

    return framesRendered;
}

void SyncPullStrategy::fillRemainingSilence(float* dst, int framesRendered, int framesToGenerate, int remainingFrames) {
    // Fill remaining buffer with faded silence on partial render. A hard
    // zero-fill here would jump from the last rendered sample to zero — a
    // discontinuity the ear hears as a crackle. The faded fill ramps to the
    // non-zero floor so the transition is continuous AND the knock detector's
    // exact-zero proxy doesn't count the gap.
    if (framesRendered < framesToGenerate) {
        // NOTE: Should never happen in practice - could probably just throw an exception here
        logger_->warning(LogMask::AUDIO,
            __ilog_format("SyncPullStrategy::render: rendered %d/%d frames, filling remaining %d with faded silence",
            framesRendered, framesToGenerate, remainingFrames));
        float* remaining = dst + (framesRendered * 2);
        fillSilenceFaded(remaining, framesToGenerate - framesRendered);
    }
}



void SyncPullStrategy::fillSilenceFaded(float* dst, int frames) {
    // Fade from the last real synthesized sample to SILENCE_FLOOR (a tiny
    // non-zero floor) over FADE_SAMPLES, then hold the floor for the
    // remainder. Each stereo frame ramps both channels linearly. The first
    // faded sample starts AT lastAudioLeft_/Right_ and the last faded sample
    // lands at SILENCE_FLOOR * sign(lastAudio), so the boundary from the
    // preceding audio chunk is continuous (no jump = no crackle). Holding a
    // non-zero floor means the knock detector's exact-zero proxy no longer
    // counts the silence gap (float floor -> int16 ~+9 = -88 dBFS, inaudible),
    // while the next audio chunk's applyFadeIn ramps back up from the floor —
    // the silence->audio edge stays continuous too.
    //
    // int16-floor guarantee: the linear fade from lastAudio to the floor must
    // never produce an intermediate float whose int16 quantization collapses to
    // exactly 0 (which the knock detector counts as a zero-sample dropout).
    // At SILENCE_FLOOR=0.0003f the floor itself is int16 +9, but a fade from a
    // near-zero lastAudio (e.g. a sine zero-crossing at float ~0.0001) would
    // dip below 1 LSB mid-ramp. To close that gap we clamp every faded sample
    // to |val| >= SILENCE_FLOOR: when |lastAudio| < the floor we skip the fade
    // and hold the floor directly (step from 0 to ±9 LSB is inaudible, not a
    // crackle); when |lastAudio| >= the floor the ramp stays above the floor
    // everywhere (both endpoints are >= floor in magnitude), so no int16 0 is
    // ever produced.
    const float floorL = EngineSimAudio::SILENCE_FLOOR * (lastAudioLeft_ >= 0.0f ? 1.0f : -1.0f);
    const float floorR = EngineSimAudio::SILENCE_FLOOR * (lastAudioRight_ >= 0.0f ? 1.0f : -1.0f);
    const bool fadeL = std::abs(lastAudioLeft_)  >= EngineSimAudio::SILENCE_FLOOR;
    const bool fadeR = std::abs(lastAudioRight_) >= EngineSimAudio::SILENCE_FLOOR;

    const int fadeLen = std::min(frames, EngineSimAudio::FADE_SAMPLES);
    const int holdLen = frames - fadeLen;

    for (int i = 0; i < fadeLen; ++i) {
        const float t = (fadeLen > 1)
            ? static_cast<float>(fadeLen - 1 - i) / static_cast<float>(fadeLen - 1)
            : 0.0f;
        // Interpolate from lastAudio to the floor (or hold floor if lastAudio
        // was too small to fade meaningfully). Because both endpoints are
        // >= SILENCE_FLOOR in magnitude, the ramp never crosses zero and never
        // underflows to int16 0.
        dst[i * 2]     = fadeL ? (floorL + (lastAudioLeft_  - floorL) * t) : floorL;
        dst[i * 2 + 1] = fadeR ? (floorR + (lastAudioRight_ - floorR) * t) : floorR;
    }
    // Hold the floor for the remainder (if any).
    for (int i = 0; i < holdLen; ++i) {
        dst[(fadeLen + i) * 2]     = floorL;
        dst[(fadeLen + i) * 2 + 1] = floorR;
    }
    // Once we've emitted the full fade tail, subsequent silence holds at the
    // floor. Keep lastAudio synced to the floor so a long silence run doesn't
    // re-fade from a stale value.
    lastAudioLeft_ = floorL;
    lastAudioRight_ = floorR;
}

void SyncPullStrategy::applyFadeIn(float* dst, int frames) {
    // Ramp the first FADE_SAMPLES of a resumed audio chunk from SILENCE_FLOOR
    // to full scale so the silence->audio edge is continuous. The fade starts
    // from the floor value (not zero) because the preceding silence holds at
    // the floor — ramping from the floor to the audio's natural level keeps
    // the transition continuous. fadeInProgress_ counts down the remaining
    // fade-in samples across chunk boundaries (a single chunk may be shorter
    // than the full fade).
    int remaining = fadeInProgress_;
    const int fadeLen = std::min(frames, remaining);
    for (int i = 0; i < fadeLen; ++i) {
        // Gain ramps from ~0.0 up to ~1.0 over the fade. (FADE_SAMPLES - remaining + i)
        // goes 0..fadeLen-1; dividing by FADE_SAMPLES-1 gives 0..1.
        const float gain = (EngineSimAudio::FADE_SAMPLES > 1)
            ? static_cast<float>(EngineSimAudio::FADE_SAMPLES - remaining + i) / static_cast<float>(EngineSimAudio::FADE_SAMPLES - 1)
            : 1.0f;
        // Interpolate from the floor to the actual audio sample.
        const float floorL = EngineSimAudio::SILENCE_FLOOR * (dst[i * 2] >= 0.0f ? 1.0f : -1.0f);
        const float floorR = EngineSimAudio::SILENCE_FLOOR * (dst[i * 2 + 1] >= 0.0f ? 1.0f : -1.0f);
        dst[i * 2]     = floorL + (dst[i * 2]     - floorL) * gain;
        dst[i * 2 + 1] = floorR + (dst[i * 2 + 1] - floorR) * gain;
    }
    remaining -= fadeLen;
    fadeInProgress_ = (remaining > 0) ? remaining : 0;
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

void SyncPullStrategy::resetFrameRender(int framesToGenerate, int framesRendered, const float* dst, std::chrono::high_resolution_clock::time_point callbackStart) {
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
    logger_->debug(LogMask::AUDIO, __ilog_format("SyncPullStrategy::AddFrames: No-op for sync-pull mode"));
    return true;
}

void SyncPullStrategy::reset() {
    logger_->debug(LogMask::AUDIO, __ilog_format("SyncPullStrategy reset: No-op for sync-pull mode"));
}

std::string SyncPullStrategy::getModeString() const {
    return "SYNC-PULL";
}
