// AfterfireShortPullShapeTest.cpp — INPUT-SHAPE reproduction of the real-world
// "pops fire immediately on release" timing bug.
//
// The existing AfterfireCrackleTimingTest holds throttle=1.0 for ~3s (engine
// reaches a stable ~7300 peak, THEN lifts) and passes O1 (first pop <= 0.85 of
// peak). Real C63 free-revving at no load spikes to ~7300 in a FRACTION of a
// second. The suspected defect lives in CombustionChamber::updateAfterfire():
// the on-throttle peak seed uses m_afterfireWasOnThrottle to choose between
//   max(peak, rpmNow)  (accumulate, when already on throttle)
//   rpmNow             (RESET, when re-applying after a dip)
// so any frame where throttle dips below cutoff during the pull sets
// m_afterfireWasOnThrottle=false, and the next on-throttle frame RESETS the
// frozen peak to the current (possibly-falling) RPM. The DFCO threshold
// (dfcoRevDropFraction * collapsedPeak) is then trivially satisfied and pops
// fire immediately.
//
// This test drives the REAL CLI bridge path with realistic SHORT pulls and a
// key-repeat-gap dip to characterise whether that collapses O1.

#include <gtest/gtest.h>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include "simulator/SimulatorFactory.h"
#include "simulator/BridgeSimulator.h"
#include "simulator/EngineSimTypes.h"
#include "simulator/SimulatorInitHelpers.h"

#ifndef TEST_PRESET_DIR
#error "TEST_PRESET_DIR must be defined by CMake"
#endif
#ifndef TEST_ENGINE_SIM_ASSETS
#error "TEST_ENGINE_SIM_ASSETS must be defined by CMake"
#endif

namespace {

constexpr double kO1FirstPopFraction = 0.85;
constexpr double kDt = 1.0 / 60.0;

struct PopSample {
    double timeSinceCutMs;
    double rpm;
    int chamber;
};

struct ShapeMeasurement {
    double peakRevRpm = 0.0;
    double firstPopDelayMs = -1.0;
    double firstPopRpm = -1.0;
    int totalPops = 0;
    std::vector<PopSample> pops;
    bool anyOnThrottlePop = false;
};

// Drives a SHORT rev shape, then a throttle-cut coast, recording every pop.
//
// The rev phase mirrors the EXISTING control test exactly (starter motor ON for
// the first 60 frames, throttle=1.0 throughout) so every scenario starts from the
// same fresh-engine state. This isolates the INPUT-SHAPE variable — hold length
// and in-hold dips — and avoids a confounder where idling at throttle=0 first lets
// raw fuel accumulate in the runner and pop instantly at blip-start.
//
//   holdFrames            : total frames at throttle=1.0 (spike to redline). The
//                           starter is released at frame 60, matching the control.
//   dipFrame              : a single frame (within the hold) forced to throttle=0
//                           (-1 = no dip) — simulates a key-repeat gap
//   buzzGapPeriod         : after the hold, run `buzzFrames` MORE frames at
//                           throttle=1.0 but force ONE throttle=0 frame every
//                           `buzzGapPeriod` frames (key-repeat buzz while PINNED
//                           at redline — the dip occurs AFTER the peak is reached,
//                           where the reset mechanism actually bites). 0 = no buzz.
//   buzzFrames            : length of the redline-buzz phase (only used if
//                           buzzGapPeriod > 0).
//   coastThenBounceFrames : after the hold, coast this many frames at 0.0 (rpm
//                           falls), THEN force ONE frame at throttle=1.0 (a stray
//                           blip) before the real lift. 0 = no bounce. This is
//                           the strong-collapse probe: the bounce re-applies
//                           after rpm has already dropped.
ShapeMeasurement measureShape(BridgeSimulator& bridge, int holdFrames,
                              int dipFrame, int buzzGapPeriod, int buzzFrames,
                              int coastThenBounceFrames) {
    ShapeMeasurement m;

    // --- Hold phase: WOT blip (also cranks the engine via the starter). ---
    for (int i = 0; i < holdFrames; ++i) {
        const double throttle = (i == dipFrame) ? 0.0 : 1.0;
        if (i == 60) bridge.setStarterMotor(false);
        bridge.setThrottle(throttle);
        bridge.update(kDt);
        m.peakRevRpm = std::max(m.peakRevRpm, bridge.getEngineRpm());
    }

    // --- Redline key-repeat buzz: brief gaps while already at peak. ---
    if (buzzGapPeriod > 0) {
        for (int b = 0; b < buzzFrames; ++b) {
            const double throttle = ((b % buzzGapPeriod) == 0) ? 0.0 : 1.0;
            bridge.setThrottle(throttle);
            bridge.update(kDt);
            m.peakRevRpm = std::max(m.peakRevRpm, bridge.getEngineRpm());
        }
    }

    if (coastThenBounceFrames > 0) {
        // Partial coast: rpm falls off the peak.
        for (int j = 0; j < coastThenBounceFrames; ++j) {
            bridge.setThrottle(0.0);
            bridge.update(kDt);
            m.peakRevRpm = std::max(m.peakRevRpm, bridge.getEngineRpm());
        }
        // The stray blip: one on-throttle frame after the coast. This is the
        // frame that (per the hypothesis) RESETS m_afterfireRefPeakRpm to the
        // now-fallen rpm via the else branch.
        bridge.setThrottle(1.0);
        bridge.update(kDt);
        m.peakRevRpm = std::max(m.peakRevRpm, bridge.getEngineRpm());
    }

    // --- THROTTLE CUT + coast ~4s. Watch each chamber's eventCount tick up. ---
    const auto baseline = bridge.getAfterfireDiagnostics();
    std::vector<int> prevEvents(baseline.size(), 0);
    for (size_t c = 0; c < baseline.size(); ++c) prevEvents[c] = baseline[c].eventCount;
    for (const auto& d : baseline) m.anyOnThrottlePop |= (d.eventCount > 0);

    for (int i = 0; i < 360; ++i) {
        bridge.setThrottle(0.0);
        bridge.update(kDt);

        const double tCutMs = i * kDt * 1000.0;
        const double rpm = bridge.getEngineRpm();
        const auto diags = bridge.getAfterfireDiagnostics();
        for (size_t c = 0; c < diags.size(); ++c) {
            const int ec = diags[c].eventCount;
            if (ec > prevEvents[c]) {
                const int newPops = ec - prevEvents[c];
                for (int k = 0; k < newPops; ++k) {
                    m.pops.push_back(PopSample{tCutMs, rpm, static_cast<int>(c)});
                    if (m.firstPopDelayMs < 0.0) {
                        m.firstPopDelayMs = tCutMs;
                        m.firstPopRpm = rpm;
                    }
                }
                prevEvents[c] = ec;
            }
        }
    }

    const auto finalDiags = bridge.getAfterfireDiagnostics();
    for (const auto& d : finalDiags) m.totalPops += d.eventCount;

    return m;
}

void runShapeScenario(int holdFrames, int dipFrame, int buzzGapPeriod, int buzzFrames,
                      int coastThenBounceFrames, const std::string& label) {
    const std::string presetPath = std::string(TEST_PRESET_DIR) + "/C63_M156_V3.json";
    ASSERT_TRUE(std::filesystem::exists(presetPath)) << "Missing preset: " << presetPath;

    ISimulatorConfig config;
    config.sampleRate = 48000;
    config.simulationFrequency = 10000;

    std::filesystem::path savedCwd = std::filesystem::current_path();
    auto sim = SimulatorFactory::create(
        SimulatorType::PistonEngine,
        presetPath,
        std::string(TEST_ENGINE_SIM_ASSETS) + "/",
        config);
    std::filesystem::current_path(savedCwd);
    ASSERT_NE(sim, nullptr) << "SimulatorFactory::create returned null";

    auto* bridge = dynamic_cast<BridgeSimulator*>(sim.get());
    ASSERT_NE(bridge, nullptr) << "Factory did not return a BridgeSimulator";
    ASSERT_TRUE(bridge->create(config, nullptr, nullptr)) << "BridgeSimulator::create failed";

    Simulator* innerSim = bridge->getInternalSimulator();
    ASSERT_NE(innerSim, nullptr);
    SimulatorInitHelpers::initializeConvolutionFilters(innerSim);

    AfterfireConfig af;
    af.enabled = true;
    af.ignitionDelayRefS = 0.3;
    af.diagnostics = true;
    SimulatorFactory::configureAfterfire(sim.get(), af, nullptr);

    bridge->setIgnition(true);
    bridge->setStarterMotor(true);

    const ShapeMeasurement m = measureShape(*bridge, holdFrames, dipFrame, buzzGapPeriod, buzzFrames, coastThenBounceFrames);

    {
        const auto diags = bridge->getAfterfireDiagnostics();
        double maxT = 0.0, maxProg = 0.0, maxRawFrac = 0.0;
        for (const auto& d : diags) {
            maxT = std::max(maxT, d.maxRunnerTempK);
            maxProg = std::max(maxProg, d.maxIgnitionProgress);
            maxRawFrac = std::max(maxRawFrac, d.maxRawFuelFraction);
        }
        printf("[SHAPE-DIAG/%s] maxRunnerT=%.0fK maxIgnProgress=%.3f maxRawFuelFrac=%.5g "
               "anyOnThrottlePop=%d\n",
               label.c_str(), maxT, maxProg, maxRawFrac, m.anyOnThrottlePop ? 1 : 0);
        fflush(stdout);
    }

    const double firstPopRatio = (m.peakRevRpm > 0.0) ? m.firstPopRpm / m.peakRevRpm : 0.0;
    const bool o1 = (m.firstPopRpm <= kO1FirstPopFraction * m.peakRevRpm);
    printf("[SHAPE-RESULT/%s] peakRevRpm=%.0f firstPopDelayMs=%.1f firstPopRpm=%.0f "
           "firstPopRatio=%.3f O1_threshold=%.0f totalPops=%d O1_pass=%d\n",
           label.c_str(), m.peakRevRpm, m.firstPopDelayMs, m.firstPopRpm, firstPopRatio,
           kO1FirstPopFraction * m.peakRevRpm, m.totalPops, o1 ? 1 : 0);
    fflush(stdout);

    ::testing::Test::RecordProperty("scenario", label);
    ::testing::Test::RecordProperty("peak_rev_rpm", std::to_string(m.peakRevRpm));
    ::testing::Test::RecordProperty("first_pop_delay_ms", std::to_string(m.firstPopDelayMs));
    ::testing::Test::RecordProperty("first_pop_rpm", std::to_string(m.firstPopRpm));
    ::testing::Test::RecordProperty("first_pop_ratio", std::to_string(firstPopRatio));
    ::testing::Test::RecordProperty("total_pops", std::to_string(m.totalPops));
    ::testing::Test::RecordProperty("O1_pass", std::to_string(o1));

    // We do NOT assert O1 here — this is a MEASUREMENT/diagnosis test. It reports
    // the numbers so the collapse can be localised; the assertion lives in the
    // brittle-characterisation test, not here. We DO assert the scenario produced
    // real pops (otherwise the measurement is meaningless for timing).
    EXPECT_GT(m.totalPops, 0) << "[" << label << "] no pops at all — scenario never lit off";
}

}  // namespace

// (a) BASELINE: long 3s hold — the existing passing shape. Captures the stable
// ~7300 peak then lifts. Should PASS O1 (first pop after the rev-drop).
TEST(AfterfireShortPullShapeTest, Baseline_LongHold_PassesO1) {
#ifndef ATG_ENGINE_SIM_AFTERFIRE_SPIKE
    GTEST_SKIP() << "ATG_ENGINE_SIM_AFTERFIRE_SPIKE not compiled in";
#else
    runShapeScenario(/*holdFrames=*/180, /*dipFrame=*/-1, /*buzzGap=*/0, /*buzzFrames=*/0,
                     /*coastThenBounce=*/0, "baseline-3s-hold");
#endif
}

// (b) SHORT pull: ~1.5s at WOT (spike to redline, then lift). No dip. In-sim the
// C63 free-revs to ~7300 in ~1.5s, not 0.4s, so the "short" pull here is the
// shortest hold that still reaches the pop regime. Does the short shape alone
// break O1, or does the frozen peak hold?
TEST(AfterfireShortPullShapeTest, ShortHold_NoDip) {
#ifndef ATG_ENGINE_SIM_AFTERFIRE_SPIKE
    GTEST_SKIP() << "ATG_ENGINE_SIM_AFTERFIRE_SPIKE not compiled in";
#else
    runShapeScenario(/*holdFrames=*/90, /*dipFrame=*/-1, /*buzzGap=*/0, /*buzzFrames=*/0,
                     /*coastThenBounce=*/0, "short-hold-no-dip");
#endif
}

// (c) REDLINE KEY-REPEAT BUZZ: reach peak, then keep the pedal "down" but insert
// brief 1-frame gaps every few frames (the key-repeat gap that the input target's
// momentary-throttle decay or a driver tap produces). The dip now occurs AFTER
// the peak is reached — exactly where the else-branch reset collapses the frozen
// peak to the (still near-peak) rpm. Per the hypothesis this should collapse O1.
TEST(AfterfireShortPullShapeTest, RedlineBuzz_DropsPeak) {
#ifndef ATG_ENGINE_SIM_AFTERFIRE_SPIKE
    GTEST_SKIP() << "ATG_ENGINE_SIM_AFTERFIRE_SPIKE not compiled in";
#else
    runShapeScenario(/*holdFrames=*/90, /*dipFrame=*/-1, /*buzzGap=*/4, /*buzzFrames=*/40,
                     /*coastThenBounce=*/0, "redline-buzz");
#endif
}

// (d) STRONG-COLLAPSE probe: short hold, then a partial coast (rpm falls), then
// ONE stray throttle blip before the real lift. This is the exact condition under
// which the else-branch reset collapses the peak to a fallen RPM — the real
// mechanism behind immediate pops.
TEST(AfterfireShortPullShapeTest, ShortHold_CoastThenBounce) {
#ifndef ATG_ENGINE_SIM_AFTERFIRE_SPIKE
    GTEST_SKIP() << "ATG_ENGINE_SIM_AFTERFIRE_SPIKE not compiled in";
#else
    runShapeScenario(/*holdFrames=*/90, /*dipFrame=*/-1, /*buzzGap=*/0, /*buzzFrames=*/0,
                     /*coastThenBounce=*/30, "short-hold-coast-then-bounce");
#endif
}
