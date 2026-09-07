// SpeakerProtection.h - Audio output protection (soft clipping, peak limiting, breach recovery)
// SRP: Prevent speaker damage from clipping and smooth audio discontinuities
// Phase F: Extracted from EngineSimTypes.h for SRP compliance

#ifndef SPEAKER_PROTECTION_H
#define SPEAKER_PROTECTION_H

#include <cmath>
#include <cstddef>
#include <cstring>
#include <algorithm>

namespace SpeakerProtection {
    // Padé [5/4] approximant coefficients for tanh(x)
    // Numerator: x * (x⁴ + N_x2·x² + N_c)  where N_x2=105, N_c=945
    // Denominator:        (D_x4·x⁴ + D_x2·x² + D_c)  where D_x4=15, D_x2=420, D_c=945
    // Max error < 0.5% for |x| <= 3.0. Uses only +,-,*,/ (hardware FPU on ESP32-S3).
    constexpr float PADE_NUM_X2 = 105.0f;
    constexpr float PADE_NUM_C  = 945.0f;
    constexpr float PADE_DEN_X4 = 15.0f;
    constexpr float PADE_DEN_X2 = 420.0f;
    constexpr float PADE_DEN_C  = 945.0f;

    inline float fastTanh(float x) {
        float x2 = x * x;
        float x4 = x2 * x2;
        float result = x * (x4 + PADE_NUM_X2 * x2 + PADE_NUM_C)
                              / (PADE_DEN_X4 * x4 + PADE_DEN_X2 * x2 + PADE_DEN_C);
        if (result > 1.0f) return 1.0f;
        if (result < -1.0f) return -1.0f;
        return result;
    }

    // Soft-clip a single sample via tanh waveshaping
    inline float softClip(float sample, float drive) {
        return fastTanh(sample * drive);
    }

    // Scan buffer for peak absolute value
    inline float peakAbs(const float* buffer, size_t totalSamples) {
        float peak = 0.0f;
        for (size_t i = 0; i < totalSamples; ++i) {
            float absSample = std::fabs(buffer[i]);
            if (absSample > peak) peak = absSample;
        }
        return peak;
    }

    // Scale buffer down if peak exceeds threshold. Returns applied gain.
    inline float applyPeakLimiter(float* buffer, size_t totalSamples, float threshold) {
        if (totalSamples == 0) {
            return 1.0f;
        }

        float peak = peakAbs(buffer, totalSamples);

        if (peak > threshold) {
            float gain = threshold / peak;
            for (size_t i = 0; i < totalSamples; ++i) {
                buffer[i] *= gain;
            }
            return gain;
        }

        return 1.0f;
    }

    // Full pipeline: soft-clip each sample, then peak-limit the buffer
    inline void protectBuffer(float* buffer, int frameCount, int channelCount,
                              float drive = 1.0f, float threshold = 0.95f) {
        size_t totalSamples = static_cast<size_t>(frameCount) * static_cast<size_t>(channelCount);

        for (size_t i = 0; i < totalSamples; ++i) {
            buffer[i] = softClip(buffer[i], drive);
        }

        applyPeakLimiter(buffer, totalSamples, threshold);
    }

    // Breach recovery: cross-fade from held tail when callback exceeded budget.
    // Any breach (headroom < 0) can cause a hardware-level discontinuity because
    // CoreAudio plays stale/silence while we're late returning. The cross-fade
    // is self-correcting — transparent on continuous audio, only modifies when
    // there's an actual discontinuity between held tail and new buffer.
    constexpr double BREACH_SMOOTH_THRESHOLD_MS = 0.0;
    constexpr int BREACH_FADE_SAMPLES = 32;

    inline bool applyBreachRecoveryFade(float* buffer, int frameCount, int channelCount,
                                         double headroomMs) {
        if (frameCount <= 0 || channelCount <= 0) return false;
        if (headroomMs >= BREACH_SMOOTH_THRESHOLD_MS) return false;

        int fadeFrames = std::min(BREACH_FADE_SAMPLES, frameCount);
        for (int i = 0; i < fadeFrames; ++i) {
            float gain = static_cast<float>(i) / static_cast<float>(fadeFrames);
            for (int ch = 0; ch < channelCount; ++ch) {
                buffer[i * channelCount + ch] *= gain;
            }
        }
        return true;
    }

    // Cross-fade breach recovery: blends from held tail into new buffer
    // Replaces fade-in-from-zero with a proper cross-fade from previous audio
    constexpr int BREACH_XFADE_SAMPLES = 48;  // ~1.1ms at 44100Hz

    struct BreachRecoveryState {
        float heldTail[BREACH_XFADE_SAMPLES * 2] = {};  // stereo, last N frames of previous buffer

        void saveTail(const float* buffer, int frameCount, int channelCount) {
            int xfade = std::min(BREACH_XFADE_SAMPLES, frameCount);
            int srcStart = (frameCount - xfade) * channelCount;
            int samples = xfade * channelCount;
            // Copy tail into the END of heldTail (so alignment is correct for cross-fade)
            std::memmove(heldTail, heldTail + samples, (BREACH_XFADE_SAMPLES * 2 - samples) * sizeof(float));
            std::memcpy(heldTail + (BREACH_XFADE_SAMPLES - xfade) * channelCount,
                        buffer + srcStart, samples * sizeof(float));
        }

        bool applyCrossfade(float* buffer, int frameCount, int channelCount, double headroomMs) const {
            if (frameCount <= 0 || channelCount <= 0) return false;
            if (headroomMs >= BREACH_SMOOTH_THRESHOLD_MS) return false;

            int fadeFrames = std::min(BREACH_XFADE_SAMPLES, frameCount);
            int heldOffset = (BREACH_XFADE_SAMPLES - fadeFrames) * channelCount;

            for (int i = 0; i < fadeFrames; ++i) {
                float gain = static_cast<float>(i) / static_cast<float>(fadeFrames);
                for (int ch = 0; ch < channelCount; ++ch) {
                    int idx = i * channelCount + ch;
                    float held = heldTail[heldOffset + idx];
                    buffer[idx] = held * (1.0f - gain) + buffer[idx] * gain;
                }
            }
            return true;
        }
    };

    // Trend drop threshold: callback rate drop severe enough to warrant hold/replay
    constexpr double TREND_HOLD_THRESHOLD_PCT = -20.0;

    // Trend hold state: replay last good buffer when callback rate drops significantly
    // Prevents crackles at the expense of quality — user can adjust sim-freq next run
    struct TrendHoldState {
        static constexpr int MAX_HOLD_FRAMES = 1024;
        static constexpr int MAX_CHANNELS = 2;
        float heldBuffer[MAX_HOLD_FRAMES * MAX_CHANNELS] = {};
        int heldFrameCount = 0;
        int heldChannelCount = 0;

        void saveBuffer(const float* buffer, int frameCount, int channelCount) {
            int frames = std::min(frameCount, MAX_HOLD_FRAMES);
            int channels = std::min(channelCount, MAX_CHANNELS);
            std::memcpy(heldBuffer, buffer, frames * channels * sizeof(float));
            heldFrameCount = frames;
            heldChannelCount = channels;
        }

        bool replayHeld(float* buffer, int frameCount, int channelCount) const {
            if (heldFrameCount <= 0) return false;
            int frames = std::min(frameCount, heldFrameCount);
            int channels = std::min(channelCount, heldChannelCount);
            std::memcpy(buffer, heldBuffer, frames * channels * sizeof(float));
            return true;
        }
    };

    // Detects sharp sample-to-sample deltas and smooths them with linear interpolation.
    // Returns count of smoothed regions.
    // previousSample: last sample from the previous callback (per-channel), enables
    // inter-buffer discontinuity detection. Size must equal channelCount. Pass nullptr
    // to use buffer[0..channelCount-1] as the baseline (no inter-buffer detection).
    inline int smoothDiscontinuities(float* buffer, int frameCount, int channelCount,
                                      float threshold = 0.3f, int fadeSamples = 24,
                                      const float* previousSample = nullptr) {
        if (frameCount <= 0 || channelCount <= 0) return 0;

        int smoothed = 0;

        for (int ch = 0; ch < channelCount; ++ch) {
            float prev = (previousSample != nullptr) ? previousSample[ch] : buffer[ch];
            int i = 1;

            while (i < frameCount) {
                float current = buffer[i * channelCount + ch];
                float delta = std::fabs(current - prev);

                if (delta > threshold) {
                    float before = prev;
                    int fadeEndFrame = std::min(i + fadeSamples, frameCount);
                    int fadeLength = fadeEndFrame - i;
                    float after = buffer[(fadeEndFrame - 1) * channelCount + ch];

                    for (int j = 0; j < fadeLength; ++j) {
                        float t = static_cast<float>(j) / static_cast<float>(fadeLength);
                        buffer[(i + j) * channelCount + ch] = before + t * (after - before);
                    }

                    prev = buffer[(fadeEndFrame - 1) * channelCount + ch];
                    i = fadeEndFrame;
                    ++smoothed;
                } else {
                    prev = current;
                    ++i;
                }
            }
        }

        return smoothed;
    }

} // namespace SpeakerProtection

#endif // SPEAKER_PROTECTION_H
