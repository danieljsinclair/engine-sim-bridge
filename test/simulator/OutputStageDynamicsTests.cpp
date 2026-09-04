// OutputStageDynamicsTests.cpp - the two bridge output-stage processors.
//
// EmissionOnsetEnvelope: the startup attach-crackle fix (always on). Contract:
//   - steady signal never gated (gain 1 from frame 0 — the sine smokes)
//   - sustained silence then a rail-scale onset fades in over the attack
//     (first loud frame's OUTPUT step is rail/attackFrames, not rail)
//   - after the attack, gain returns to exactly 1 (steady state untouched)
//   - silence below eps does not itself trigger an attack
//
// VolumeTamer: the quiet-on-decel leveler. Contract:
//   - quiet sustained input lifts; loud sustained input sits back
//   - gain moves are smooth (no step > the smoothing bound between frames)
//   - ratio exponent: strength 0 never leaves unity gain (bypass semantics
//     are also enforced structurally — disabled instances are never fed)

#include <gtest/gtest.h>

#include "simulator/OutputStageDynamics.h"

#include <cmath>
#include <cstdint>
#include <vector>

namespace {

constexpr int kSampleRate = 44100;
constexpr int16_t kRail = 32767;
constexpr float kEps = 1.0f / 32768.0f;

TEST(EmissionOnsetEnvelopeTest, SteadySignalNeverGated) {
    bridge_audio::EmissionOnsetEnvelope env(kSampleRate);
    // Loud from frame 0: no prior silence, the gate is OPEN and stays open.
    for (int i = 0; i < kSampleRate; ++i) {  // a full second of rail
        EXPECT_EQ(env.frameGain(kRail), 1.0f) << "frame " << i;
    }
}

TEST(EmissionOnsetEnvelopeTest, AttachAfterSilenceFadesInOverAttack) {
    bridge_audio::EmissionOnsetEnvelope env(kSampleRate, /*attackMs=*/75.0f, /*holdMs=*/40.0f);
    // Sustain silence long enough to arm (>> 40 ms).
    for (int i = 0; i < kSampleRate / 10; ++i) {
        env.frameGain(0);
    }
    // Onset: a rail-scale step. The first gain must be one attack step, not 1.
    const float firstGain = env.frameGain(kRail);
    EXPECT_GT(firstGain, 0.0f);
    EXPECT_LT(firstGain, 0.01f) << "first loud frame must land at rail/attackFrames, not rail";
    // The emitted OUTPUT step is rail*firstGain ~ tens of int16-equivalents,
    // two orders below the 5000-count discontinuity threshold.
    EXPECT_LT(std::fabs(static_cast<float>(kRail) * firstGain), 500.0f);

    // Gain climbs monotonically to exactly 1 within the attack window.
    float prev = firstGain;
    int frames = 1;
    while (env.attacking() && frames < kSampleRate / 2) {
        const float g = env.frameGain(kRail);
        EXPECT_GE(g, prev) << "attack ramp must be monotonic";
        prev = g;
        ++frames;
    }
    // 75 ms at 44.1 kHz = ~3308 frames.
    EXPECT_NEAR(frames, static_cast<int>(0.075f * kSampleRate), 3);
    EXPECT_NEAR(prev, 1.0f, 1e-6f);
    // After the attack: unity.
    EXPECT_EQ(env.frameGain(kRail), 1.0f);
}

TEST(EmissionOnsetEnvelopeTest, SilenceAloneNeverAttacks) {
    bridge_audio::EmissionOnsetEnvelope env(kSampleRate);
    for (int i = 0; i < 5 * kSampleRate; ++i) {  // 5 s of silence
        EXPECT_EQ(env.frameGain(0), 1.0f);
        EXPECT_FALSE(env.attacking());
    }
    // Quiet sub-threshold noise (below onsetThreshold but above eps) disarms
    // rather than attacking: it is not an attach transient.
    const float g = env.frameGain(static_cast<int16_t>(0.05f / kEps));
    EXPECT_EQ(g, 1.0f);
    EXPECT_FALSE(env.attacking());
}

TEST(EmissionOnsetEnvelopeTest, ResetRestoresOpen) {
    bridge_audio::EmissionOnsetEnvelope env(kSampleRate);
    for (int i = 0; i < kSampleRate / 10; ++i) env.frameGain(0);
    env.frameGain(kRail);  // start an attack
    ASSERT_TRUE(env.attacking());
    env.reset();
    EXPECT_FALSE(env.attacking());
    EXPECT_EQ(env.frameGain(kRail), 1.0f);
}

TEST(VolumeTamerTest, QuietSectionLiftsTowardAverage) {
    // Loud for seconds (average settles high), then quiet: gain must rise
    // above unity — the quiet-on-decel lift.
    bridge_audio::VolumeTamer tamer(kSampleRate, /*strength=*/0.6f);
    for (int i = 0; i < 3 * kSampleRate; ++i) tamer.frameGain(kRail);
    // Now decel-quiet (~1% rail) for half a second; the smoothed gain should
    // be lifting.
    float g = 1.0f;
    for (int i = 0; i < kSampleRate / 2; ++i) g = tamer.frameGain(static_cast<int16_t>(kRail / 100));
    EXPECT_GT(g, 1.0f) << "quiet section must lift above unity";
}

TEST(VolumeTamerTest, LoudSectionSitsBack) {
    // Quiet for seconds (average settles low), then loud: gain must fall
    // below unity — the loud section is leveled back.
    bridge_audio::VolumeTamer tamer(kSampleRate, /*strength=*/0.6f);
    for (int i = 0; i < 3 * kSampleRate; ++i) tamer.frameGain(kRail / 100);
    float g = 1.0f;
    for (int i = 0; i < kSampleRate / 2; ++i) g = tamer.frameGain(kRail);
    EXPECT_LT(g, 1.0f) << "loud section must sit below unity";
    EXPECT_GT(g, 0.699f) << "cut is bounded by minGain";
}

TEST(VolumeTamerTest, GainMovesSmoothly) {
    // The processor itself must never step the gain (a gain step is an
    // amplitude discontinuity — the defect class this batch exists to kill).
    bridge_audio::VolumeTamer tamer(kSampleRate, /*strength=*/1.0f);
    std::vector<int16_t> stimulus;
    for (int i = 0; i < kSampleRate; ++i) stimulus.push_back(kRail);                       // 1 s loud
    for (int i = 0; i < kSampleRate / 2; ++i) stimulus.push_back(static_cast<int16_t>(kRail / 200));  // .5 s quiet
    for (int i = 0; i < kSampleRate / 2; ++i) stimulus.push_back(kRail);                   // .5 s loud
    float prev = tamer.frameGain(stimulus[0]);
    for (size_t i = 1; i < stimulus.size(); ++i) {
        const float g = tamer.frameGain(stimulus[i]);
        EXPECT_LT(std::fabs(g - prev), 0.01f) << "gain step at frame " << i;
        prev = g;
    }
}

TEST(VolumeTamerTest, ZeroStrengthHoldsUnity) {
    bridge_audio::VolumeTamer tamer(kSampleRate, /*strength=*/0.0f);
    for (int i = 0; i < 2 * kSampleRate; ++i) tamer.frameGain(kRail);
    for (int i = 0; i < kSampleRate / 2; ++i) {
        EXPECT_NEAR(tamer.frameGain(static_cast<int16_t>(kRail / 100)), 1.0f, 1e-6f);
    }
}

}  // namespace
