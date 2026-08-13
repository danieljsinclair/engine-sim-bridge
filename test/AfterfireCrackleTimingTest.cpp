// AfterfireCrackleTimingTest.cpp — MEASUREMENT-ONLY test that characterises WHEN
// the afterfire pops fire relative to the rev-drop on a throttle cut.
//
// This is the RED-phase evidence for the "immediate-release pop" bug:
//   pops fire the instant the throttle is released, while revs are still at
//   their peak, instead of waiting until revs have fallen (real overrun crackle).
//
// It drives the REAL CLI bridge path (SimulatorFactory -> BridgeSimulator ->
// setThrottle -> update -> afterfire) using the REALISTIC DEFAULT AfterfireConfig
// (enabled=true, ignitionDelayRefS=0.3 — the real default, NOT the aggressive
// 0.001 used by the wiring-proof test). It revs to a peak, then cuts the
// throttle and coasts, recording each pop's (time-since-cut, RPM) so we can
// assert O1: first-pop-RPM <= 0.85 * peak-revRPM.
//
// The assertion is EXPECTED TO FAIL (RED) — that failure is the proof of the bug.
// No model fix is attempted here; that is a separate step.

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

// A single observed pop: when it happened relative to the cut, and at what RPM.
struct PopSample {
    double timeSinceCutMs;
    double rpm;
    int chamber;
};

// Drives a rev-to-peak then throttle-cut coast and records every afterfire pop.
// Uses the realistic default AfterfireConfig. Returns the measurement summary.
struct CrackleMeasurement {
    double peakRevRpm = 0.0;
    double firstPopDelayMs = -1.0;
    double firstPopRpm = -1.0;
    int totalPops = 0;
    std::vector<PopSample> pops;
};

CrackleMeasurement measureCrackle(BridgeSimulator& bridge) {
    CrackleMeasurement m;

    const double dt = 1.0 / 60.0;

    // --- REV to peak: ~3s at full throttle ---
    for (int i = 0; i < 360; ++i) {
        bridge.setThrottle(1.0);
        if (i == 60) bridge.setStarterMotor(false);
        bridge.update(dt);
        m.peakRevRpm = std::max(m.peakRevRpm, bridge.getEngineRpm());
    }

    // --- THROTTLE CUT + coast ~4s. Watch each chamber's eventCount tick up. ---
    const auto baseline = bridge.getAfterfireDiagnostics();
    std::vector<int> prevEvents(baseline.size(), 0);
    for (size_t c = 0; c < baseline.size(); ++c) prevEvents[c] = baseline[c].eventCount;

    for (int i = 0; i < 360; ++i) {
        bridge.setThrottle(0.0);
        bridge.update(dt);

        const double tCutMs = i * dt * 1000.0;
        const double rpm = bridge.getEngineRpm();
        const auto diags = bridge.getAfterfireDiagnostics();
        for (size_t c = 0; c < diags.size(); ++c) {
            const int ec = diags[c].eventCount;
            if (ec > prevEvents[c]) {
                // One or more pops this step on chamber c.
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

}  // namespace

// O1 (KEY): first-pop-RPM must be <= 0.85 * peak-revRPM — revs must drop >=15%
// before the first pop. This is expected to FAIL (RED) because the current model
// fires the first pop on the very tick of the cut, while revs are still at peak.
TEST(AfterfireCrackleTimingTest, RevsHighThenCuts_PopsAfterRevsDrop) {
#ifndef ATG_ENGINE_SIM_AFTERFIRE_SPIKE
    GTEST_SKIP() << "ATG_ENGINE_SIM_AFTERFIRE_SPIKE not compiled in";
#else
    // C63_M156_V3 reaches the runner temperatures (~1880-1910 K) the realistic
    // default config is tuned for; v8_gm_ls tops out near ~1100 K and never
    // lights off at ignitionDelayRefS=0.3 (induction never completes). The
    // EngineSimTypes.h benchmark table (0.3 -> 0.050 s, 20 pops) is measured on
    // this C63 scenario, so it is the correct engine to characterise the bug on.
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

    // REALISTIC DEFAULT config: construct AfterfireConfig defaults, enable it,
    // and use the real default ignitionDelayRefS (0.3). NOT the aggressive 0.001
    // wiring-proof config and NOT a hardcoded timer — this must be physical.
    AfterfireConfig af;
    af.enabled = true;
    af.ignitionDelayRefS = 0.3;
    af.diagnostics = true;        // also surface the [AFTERFIRE] pop lines
    SimulatorFactory::configureAfterfire(sim.get(), af, nullptr);

    // --- Start the engine ---
    bridge->setIgnition(true);
    bridge->setStarterMotor(true);

    const CrackleMeasurement m = measureCrackle(*bridge);

    // --- Pop-timing distribution (bucketted by 500 ms windows after the cut) ---
    const int bucketMs = 500;
    std::vector<int> buckets(8, 0);
    for (const auto& p : m.pops) {
        const int b = static_cast<int>(p.timeSinceCutMs / bucketMs);
        if (b >= 0 && b < static_cast<int>(buckets.size())) ++buckets[b];
        else if (b >= static_cast<int>(buckets.size())) buckets.back()++;
    }

    // --- Clear summary line ---
    printf("[CRACKLE-RESULT] peakRevRpm=%.0f firstPopDelayMs=%.1f firstPopRpm=%.0f "
           "totalPops=%d O1_threshold=%.0f\n",
           m.peakRevRpm, m.firstPopDelayMs, m.firstPopRpm, m.totalPops,
           0.85 * m.peakRevRpm);
    printf("[CRACKLE-DIST] buckets(500ms):");
    for (size_t b = 0; b < buckets.size(); ++b) {
        printf(" [%d-%dms]=%d", static_cast<int>(b) * bucketMs,
               static_cast<int>(b + 1) * bucketMs, buckets[b]);
    }
    printf("\n");
    fflush(stdout);

    RecordProperty("peak_rev_rpm", std::to_string(m.peakRevRpm));
    RecordProperty("first_pop_delay_ms", std::to_string(m.firstPopDelayMs));
    RecordProperty("first_pop_rpm", std::to_string(m.firstPopRpm));
    RecordProperty("total_pops", std::to_string(m.totalPops));
    RecordProperty("o1_threshold_rpm", std::to_string(0.85 * m.peakRevRpm));

    // O1 (KEY) — expected RED: first pop should wait until revs have dropped
    // >=15%. With the current model the first pop lands on the cut tick at peak
    // RPM, so this fails and proves the bug.
    EXPECT_LE(m.firstPopRpm, 0.85 * m.peakRevRpm)
        << "BUG PROOF (O1): first pop fired at " << m.firstPopRpm << " RPM, which is "
        << (m.firstPopRpm / m.peakRevRpm * 100.0) << "% of peak " << m.peakRevRpm
        << " RPM — only " << (100.0 - m.firstPopRpm / m.peakRevRpm * 100.0)
        << "% below peak, not the required >=15%. firstPopDelayMs="
        << m.firstPopDelayMs;
#endif
}
