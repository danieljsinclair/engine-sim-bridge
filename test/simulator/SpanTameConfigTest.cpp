// SpanTameConfigTest.cpp
//
// The bridge seam of the --span-tame output-stage tamer:
// ISimulatorConfig.spanTame must reach the synthesizer's AudioParameters via
// SimulatorInitHelpers::applySpanTame (read-modify-write — nothing else about
// the audio parameters may change). The renderAudio-side behavior (bit
// identity at 0, peak reduction when active) is pinned in the nested
// engine-sim test suite (test/span_tame_tests.cpp); this file pins the
// bridge-side plumbing only.

#include <gtest/gtest.h>

#include "simulator/SimulatorInitHelpers.h"
#include "simulator/SineSimulator.h"

#include <cmath>

namespace {

// A minimal real Simulator subclass — its synthesizer exists from
// construction, so the AudioParameters read-modify-write is directly
// observable without loading a full engine preset.
SineSimulator makeSimulator() {
    return SineSimulator();
}

}  // namespace

// The taming amount lands on the synthesizer's AudioParameters.spanTame —
// the single knob renderAudio consumes.
TEST(SpanTameConfigTest, ApplySpanTame_SetsSynthesizerAudioParameter) {
    SineSimulator sim = makeSimulator();

    SimulatorInitHelpers::applySpanTame(&sim, 0.75f);

    EXPECT_FLOAT_EQ(sim.synthesizer().getAudioParameters().spanTame, 0.75f);
}

// Read-modify-write: applying the taming must not clobber the other audio
// parameters (the script-loaded volume/noise/leveler settings ride along
// untouched — the .mr's audio_volume tuning stays authoritative).
TEST(SpanTameConfigTest, ApplySpanTame_PreservesOtherAudioParameters) {
    SineSimulator sim = makeSimulator();
    Synthesizer::AudioParameters original = sim.synthesizer().getAudioParameters();
    original.volume = 2.0f;
    original.airNoise = 0.25f;
    original.levelerTarget = 21000.0f;
    sim.synthesizer().setAudioParameters(original);

    SimulatorInitHelpers::applySpanTame(&sim, 0.25f);

    const Synthesizer::AudioParameters after = sim.synthesizer().getAudioParameters();
    EXPECT_FLOAT_EQ(after.spanTame, 0.25f);
    EXPECT_FLOAT_EQ(after.volume, 2.0f);
    EXPECT_FLOAT_EQ(after.airNoise, 0.25f);
    EXPECT_FLOAT_EQ(after.levelerTarget, 21000.0f);
    EXPECT_FLOAT_EQ(after.convolution, original.convolution);
    EXPECT_FLOAT_EQ(after.dF_F_mix, original.dF_F_mix);
}

// 0.0 is the explicit OFF value and must be a plain applicable value (no
// rejection, no special casing beyond writing 0.0) — the default-off,
// bit-identical contract starts here.
TEST(SpanTameConfigTest, ApplySpanTame_ZeroIsApplicableExplicitOff) {
    SineSimulator sim = makeSimulator();

    SimulatorInitHelpers::applySpanTame(&sim, 0.0f);

    EXPECT_FLOAT_EQ(sim.synthesizer().getAudioParameters().spanTame, 0.0f);
}
