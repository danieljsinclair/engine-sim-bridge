// OutputStageDynamics.h - bridge-side output-stage audio dynamics.
//
// Two small, pure, single-threaded processors applied at the mono int16
// source -> stereo float conversion seam (BridgeSimulator), so every
// consumer (playback strategies, --output WAV tap, iOS render) sees the
// same processed stream:
//
//   EmissionOnsetEnvelope - the startup-crackle fix (always on). When the
//     synth's emission output goes from pre-fire silence to loud (combustion
//     attach — crank catch or bump-start), the raw output steps to rail with
//     several hard amplitude steps in the first ~200 ms. The envelope fades
//     the output in over a short linear attack instead. Gain is EXACTLY 1.0
//     outside an attack window (x * 1.0f is bit-exact), so steady-state
//     audio is untouched.
//
//   VolumeTamer - the quiet-on-decel loudness-leveler (--volume-tame, OFF by
//     default = fully bypassed = bit-identical). An upward compressor keyed
//     on the signal itself: a fast envelope follower against a slow session
//     average; quiet sections lift toward the average, loud sections sit
//     back, gain moves are smoothed so the processor itself never steps.
//
// Both are header-only and deterministic (no timing reads, no allocation).
// Threading contract: exactly one thread feeds each instance (whichever
// strategy owns the audio pull); they are not internally synchronized.

#ifndef ENGINE_SIM_BRIDGE_INCLUDE_SIMULATOR_OUTPUT_STAGE_DYNAMICS_H
#define ENGINE_SIM_BRIDGE_INCLUDE_SIMULATOR_OUTPUT_STAGE_DYNAMICS_H

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace bridge_audio {

// ============================================================================
// EmissionOnsetEnvelope
// ============================================================================
//
// State machine per source frame (int16 mono, pre-conversion):
//   OPEN   - output passes at gain 1. Quiet frames accumulate a silence
//            timer; once the source has been quiet >= holdMs the gate ARMS.
//   ARMED  - still gain 1 (what passes through is quiet by definition). The
//            first frame above onsetThreshold starts the ATTACK.
//   ATTACK - linear fade-in 0 -> 1 over attackMs, then back to OPEN.
//
// Detection thresholds are in normalized units (|sample|/32768): cranking /
// pre-fire synth output sits far below 0.02, the attach transient is
// rail-scale (>= 0.25 by an order of magnitude). Normal driving never
// returns to true digital silence, so the gate only fires on real
// silence->loud transitions (engine attach after crank or bump-start).
class EmissionOnsetEnvelope {
public:
    struct Params {
        float silenceEps = 0.02f;     // below = pre-fire quiet
        float onsetThreshold = 0.25f; // above (while armed) = attach
        int silenceHoldFrames;        // quiet time before arming
        int attackFrames;             // fade-in length
    };

    EmissionOnsetEnvelope(int sampleRate, float attackMs = 75.0f, float holdMs = 40.0f)
        : params_{
              0.02f,
              0.25f,
              msToFrames(holdMs, sampleRate),
              msToFrames(attackMs, sampleRate)} {}

    // Feed one SOURCE frame (int16 mono). Returns the gain to apply to that
    // frame's converted output (both channels).
    float frameGain(int16_t sourceSample) {
        const float level = std::fabs(static_cast<float>(sourceSample)) * (1.0f / 32768.0f);
        switch (state_) {
        case State::Open:
            if (level < params_.silenceEps) {
                if (++quietFrames_ >= params_.silenceHoldFrames) {
                    state_ = State::Armed;
                }
            } else {
                quietFrames_ = 0;
            }
            return 1.0f;
        case State::Armed:
            if (level >= params_.onsetThreshold) {
                state_ = State::Attack;
                attackPos_ = 0;
                quietFrames_ = 0;
                // Fall through to emit the first attack frame below.
                break;
            }
            if (level >= params_.silenceEps) {
                // Quiet-ish noise above eps but below onset: not an attach;
                // restart the quiet accumulation.
                quietFrames_ = 0;
                state_ = State::Open;
            }
            return 1.0f;
        case State::Attack:
            break;
        }
        // ATTACK: linear ramp. The first loud frame lands at one ramp step
        // (rail/attackFrames ~ 10 int16-equivalent) instead of the rail, and
        // the ramp slope per frame stays two orders below the 5000-count
        // discontinuity threshold the knock smokes gate on.
        const float gain = static_cast<float>(attackPos_ + 1) /
                           static_cast<float>(params_.attackFrames);
        if (++attackPos_ >= params_.attackFrames) {
            state_ = State::Open;
            quietFrames_ = 0;
        }
        return gain;
    }

    bool attacking() const { return state_ == State::Attack; }

    void reset() {
        state_ = State::Open;
        quietFrames_ = 0;
        attackPos_ = 0;
    }

private:
    enum class State { Open, Armed, Attack };

    static int msToFrames(float ms, int sampleRate) {
        return std::max(1, static_cast<int>(std::lround(ms * 0.001f * sampleRate)));
    }

    Params params_;
    State state_ = State::Open;
    int quietFrames_ = 0;
    int attackPos_ = 0;
};

// ============================================================================
// VolumeTamer
// ============================================================================
//
// Upward compressor keyed on the signal itself (no throttle plumbing into
// the audio path). Per source frame:
//   fast  - peak follower, ~5 ms up / ~300 ms release one-pole
//   slow  - ~2 s one-pole of fast (the session loudness average)
//   target gain = clamp((slow + floor) / max(fast, floor), minGain, maxGain)
//     fast < slow  -> gain > 1 (quiet section lifts toward the average)
//     fast > slow  -> gain < 1 (loud section sits back)
//   gain - smoothed one-pole (~50 ms) so the gain never steps
//
// strength in [0,1] interpolates the exponent applied to the raw ratio:
//   gain = ratio^strength  (strength 0 -> ratio^0 = 1 -> bypass, strength 1
//   -> full leveling). Disabled (strength <= 0) instances are never fed —
//   BridgeSimulator bypasses the tamer entirely — keeping OFF bit-identical.
class VolumeTamer {
public:
    struct Params {
        float strength = 1.0f;  // 0..1 (only constructed when enabled)
        float floorLevel = 0.005f;  // noise floor guarding the divide
        float maxGain = 4.0f;       // +12 dB lift ceiling
        float minGain = 0.7f;       // -3.1 dB loudest cut
    };

    VolumeTamer(int sampleRate, float strength) {
        params_.strength = std::clamp(strength, 0.0f, 1.0f);
        fastCoefUp_ = coefForMs(5.0f, sampleRate);
        fastCoefDown_ = coefForMs(300.0f, sampleRate);
        slowCoef_ = coefForMs(2000.0f, sampleRate);
        gainCoef_ = coefForMs(50.0f, sampleRate);
    }

    // Feed one SOURCE frame (int16 mono). Returns the gain for that frame.
    float frameGain(int16_t sourceSample) {
        const float level = std::fabs(static_cast<float>(sourceSample)) * (1.0f / 32768.0f);

        // Fast peak follower (attack fast, release slow).
        const float coef = (level > fast_) ? fastCoefUp_ : fastCoefDown_;
        fast_ += coef * (level - fast_);
        // Slow session-average follower.
        slow_ += slowCoef_ * (fast_ - slow_);

        const float denom = std::max(fast_, params_.floorLevel);
        const float numer = slow_ + params_.floorLevel;
        const float ratio = numer / denom;
        // ratio > 1 (quiet) -> gain > 1; ratio < 1 (loud) -> gain < 1. The
        // exponent bends how far we go: strength 0.3 = gentle, 1.0 = full.
        const float target = std::clamp(std::pow(ratio, params_.strength),
                                        params_.minGain, params_.maxGain);
        // Smooth the gain itself (steps here would be audible discontinuities).
        gain_ += gainCoef_ * (target - gain_);
        return gain_;
    }

    void reset() {
        fast_ = 0.0f;
        slow_ = 0.0f;
        gain_ = 1.0f;
    }

private:
    static float coefForMs(float ms, int sampleRate) {
        // Standard one-pole smoothing coefficient for a ~ms time constant.
        return 1.0f - std::exp(-1.0f / (ms * 0.001f * static_cast<float>(sampleRate)));
    }

    Params params_;
    float fastCoefUp_ = 0.1f;
    float fastCoefDown_ = 0.01f;
    float slowCoef_ = 0.01f;
    float gainCoef_ = 0.1f;
    float fast_ = 0.0f;
    float slow_ = 0.0f;
    float gain_ = 1.0f;
};

}  // namespace bridge_audio

#endif  // ENGINE_SIM_BRIDGE_INCLUDE_SIMULATOR_OUTPUT_STAGE_DYNAMICS_H
