// AudioRingHealthDiagnosticsTests.cpp - the diagnostics that were missing
// during the sync-pull knock
//
// The knock (bridge d09c726 "single-clock" era: loop thread produces synth
// input at 60 Hz, audio callback renders on demand at 86 Hz -> 1.44x
// overproduction -> the 44100-frame audio ring laps every ~2.25 s, discarding
// ~1 s of audio and restarting mid-waveform) was invisible to every existing
// metric: req/got/took/room/budget are all SERVICE metrics, underruns count
// device-level short reads, and the throughput line measures frames the
// callback OBTAINED (consumption) with no producer-side counter to ratio
// against.
//
// These tests pin the three detectors that close the gap, at the bridge
// Diagnostics layer where they become observable:
//   1. ring lap count                 - the exact event class of the knock
//   2. sustained production/consumption ratio window - catches 1.44x-class
//      overproduction while remaining silent at ~1.00x
//   3. seam discontinuity count       - the audible artifact's fingerprint
// plus their propagation into EngineState for presentation.

#include <gtest/gtest.h>

#include "strategy/Diagnostics.h"
#include "simulator/EngineSimTypes.h"
#include "simulation/PresentationStateBuilders.h"
#include "io/IPresentation.h"
#include "mocks/MockSimulator.h"

#include <cmath>

// ============================================================================
// 1+2+3 at the Diagnostics layer
// ============================================================================

// The 1.44x condition: production pulls ahead of consumption every window.
// The ratio must be reported and, once sustained, flagged.
TEST(AudioRingHealthDiagnosticsTests, OverproductionRatioIsReportedAndSustained) {
    Diagnostics diagnostics;

    // Simulate 3 one-second windows of 1.44x production (63504 written vs
    // 44100 consumed per second - the knock's measured rates).
    uint64_t written = 0;
    uint64_t consumed = 0;
    for (int window = 0; window < 3; ++window) {
        written += 63504;
        consumed += 44100;
        diagnostics.recordRingHealth(written, consumed, 0, 0);
        diagnostics.updateThroughput(1.0);
    }

    const Diagnostics::Snapshot snap = diagnostics.getSnapshot();
    EXPECT_NEAR(snap.prodConsRatio, 63504.0 / 44100.0, 0.01)
        << "the windowed production/consumption ratio must be reported";
    EXPECT_GE(snap.sustainedOverproductionWindows, 2)
        << "consecutive overproduction windows must accumulate";

    // Recovery: balanced windows reset the sustained alarm.
    for (int window = 0; window < 2; ++window) {
        written += 44100;
        consumed += 44100;
        diagnostics.recordRingHealth(written, consumed, 0, 0);
        diagnostics.updateThroughput(1.0);
    }
    const Diagnostics::Snapshot recovered = diagnostics.getSnapshot();
    EXPECT_NEAR(recovered.prodConsRatio, 1.0, 0.01);
    EXPECT_EQ(recovered.sustainedOverproductionWindows, 0)
        << "a balanced window must clear the sustained-overproduction alarm";
}

// The healthy case (the fix/syncpull-boundary behaviour): ratio ~1.00 must
// never trip the alarm, no matter how long it runs.
TEST(AudioRingHealthDiagnosticsTests, BalancedProductionNeverAlarms) {
    Diagnostics diagnostics;

    uint64_t written = 0;
    uint64_t consumed = 0;
    for (int window = 0; window < 30; ++window) {
        written += 44100;
        consumed += 44100;
        diagnostics.recordRingHealth(written, consumed, 0, 0);
        diagnostics.updateThroughput(1.0);
    }

    const Diagnostics::Snapshot snap = diagnostics.getSnapshot();
    EXPECT_NEAR(snap.prodConsRatio, 1.0, 0.001);
    EXPECT_EQ(snap.sustainedOverproductionWindows, 0);
}

// A single burst above threshold must not latch the sustained alarm - only
// sustained overproduction is a defect (a transient catch-up after a stall is
// normal pipeline behaviour).
TEST(AudioRingHealthDiagnosticsTests, TransientSpikeDoesNotLatch) {
    Diagnostics diagnostics;

    uint64_t written = 0;
    uint64_t consumed = 0;

    // one spiky window (2x), then balanced
    written += 88200;
    consumed += 44100;
    diagnostics.recordRingHealth(written, consumed, 0, 0);
    diagnostics.updateThroughput(1.0);

    written += 44100;
    consumed += 44100;
    diagnostics.recordRingHealth(written, consumed, 0, 0);
    diagnostics.updateThroughput(1.0);

    const Diagnostics::Snapshot snap = diagnostics.getSnapshot();
    EXPECT_EQ(snap.sustainedOverproductionWindows, 0)
        << "a single overproduction window followed by balance is a transient, not an alarm";
}

// Laps and seams are counters, not rates: they pass through monotonically.
TEST(AudioRingHealthDiagnosticsTests, LapsAndSeamsPassThrough) {
    Diagnostics diagnostics;

    diagnostics.recordRingHealth(1000, 1000, 0, 0);
    diagnostics.updateThroughput(1.0);
    EXPECT_EQ(diagnostics.getSnapshot().ringLapCount, 0u);
    EXPECT_EQ(diagnostics.getSnapshot().seamDiscontinuityCount, 0u);

    diagnostics.recordRingHealth(2000, 2000, 1, 2);
    diagnostics.updateThroughput(1.0);
    EXPECT_EQ(diagnostics.getSnapshot().ringLapCount, 1u);
    EXPECT_EQ(diagnostics.getSnapshot().seamDiscontinuityCount, 2u);
}

// ============================================================================
// Propagation into EngineState
// ============================================================================

namespace {

// Minimal IAudioBuffer double: buildAudioState only reads getModeString().
class StubAudioBuffer : public IAudioBuffer {
public:
    const char* getName() const override { return "stub"; }
    bool isEnabled() const override { return true; }
    bool isPlaying() const override { return true; }
    bool render(AudioBufferView&) override { return true; }
    bool AddFrames(float*, int) override { return true; }
    bool initialize(const AudioBufferConfig&, int) override { return true; }
    void prepareBuffer() override {}
    bool startPlayback(ISimulator*) override { return true; }
    void stopPlayback(ISimulator*) override {}
    void swapSimulator(ISimulator*) override {}
    void resetBufferAfterWarmup() override {}
    bool shouldDrainDuringWarmup() const override { return false; }
    void fillBufferFromEngine(ISimulator*, int) override {}
    std::string getModeString() const override { return "SYNC-PULL"; }
    void reset() override {}
    void updateSimulation(ISimulator*, double) override {}
};

} // namespace

using namespace presentation;
using namespace presentation::builders;

TEST(AudioRingHealthStateTests, RingHealthFieldsFlowIntoEngineState) {
    telemetry::AudioTimingTelemetry timing;
    timing.ringLaps = 3;
    timing.prodConsRatio = 1.44;
    timing.seamDiscontinuities = 5;
    timing.sustainedOverproductionWindows = 4;

    StubAudioBuffer audio;
    const SimulationConfig config;
    MockSimulator simulator;

    const EngineState::Audio audioState = builders::buildAudioState(
        timing, nullptr, audio, config, 0.0, simulator);

    EXPECT_EQ(audioState.ringLaps, 3);
    EXPECT_DOUBLE_EQ(audioState.prodConsRatio, 1.44);
    EXPECT_EQ(audioState.seamDiscontinuities, 5);
    EXPECT_EQ(audioState.sustainedOverproductionWindows, 4);
}
