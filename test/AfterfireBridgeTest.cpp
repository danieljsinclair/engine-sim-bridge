// AfterfireBridgeTest.cpp — drives the REAL CLI path (SimulatorFactory → BridgeSimulator)
// to prove the throttle-polarity fix and the afterfire wiring produce pops on a
// throttle-cut decel. Ported from the backfires-wip snapshot
// (da8dbe9:test/AfterfireBridgeTest.cpp), rewritten for the live physics-based
// afterfire model (manifold-pressure misfire -> Arrhenius auto-ignition ->
// scavenging reset). See EngineSimTypes.h (AfterfireConfig).
//
// This is the API-driven verification of what the interactive CLI does:
// setThrottle → update → afterfire. It drives BridgeSimulator::setThrottle()
// (the exact call the CLI's SimulationLoop makes), so it catches the bridge
// throttle mapping including the DirectThrottleLinkage polarity inversion.

#include <gtest/gtest.h>
#include <cmath>
#include <string>
#include <filesystem>
#include <vector>

#include "simulator/SimulatorFactory.h"
#include "simulator/BridgeSimulator.h"
#include "simulator/EngineSimTypes.h"
#include "simulator/SimulatorInitHelpers.h"

#include "engine.h"
#include "throttle.h"
#include "governor.h"
#include "direct_throttle_linkage.h"

#ifndef TEST_PRESET_DIR
#error "TEST_PRESET_DIR must be defined by CMake"
#endif
#ifndef TEST_ENGINE_SIM_ASSETS
#error "TEST_ENGINE_SIM_ASSETS must be defined by CMake"
#endif

namespace {
double sumAfterfireEvents(BridgeSimulator& bridge) {
    double total = 0.0;
    for (const auto& d : bridge.getAfterfireDiagnostics()) total += d.eventCount;
    return total;
}
double sumSkippedThrottle(BridgeSimulator& bridge) {
    double total = 0.0;
    for (const auto& d : bridge.getAfterfireDiagnostics()) total += d.skippedThrottle;
    return total;
}
}  // namespace

// =============================================================================
// Prove the full bridge path: factory-created real engine, driven via
// BridgeSimulator::setThrottle, revs on full throttle and pops on throttle-cut.
// =============================================================================
TEST(AfterfireBridgeTest, RevsOnFullThrottleAndPopsOnCut) {
#ifndef ATG_ENGINE_SIM_AFTERFIRE_SPIKE
    GTEST_SKIP() << "ATG_ENGINE_SIM_AFTERFIRE_SPIKE not compiled in";
#else
    const std::string presetPath = std::string(TEST_PRESET_DIR) + "/v8_gm_ls.json";
    ASSERT_TRUE(std::filesystem::exists(presetPath)) << "Missing preset: " << presetPath;

    ISimulatorConfig config;
    config.sampleRate = 48000;
    config.simulationFrequency = 10000;

    // SimulatorFactory resolves asset paths relative to CWD — save/restore it.
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

    // SimulatorFactory constructs the BridgeSimulator but does NOT call create() —
    // the consumer must (it runs initDependencies, wiring the telemetry writer that
    // update()/pushTelemetry dereferences). The CLI does this; so must we.
    ASSERT_TRUE(bridge->create(config, nullptr, nullptr)) << "BridgeSimulator::create failed";

    // Diagnose the throttle component type so the polarity assertions match reality.
    {
        Engine* e = bridge->getInternalSimulator()->getEngine();
        Throttle* t = e ? e->getThrottleObject() : nullptr;
        printf("[diag] throttle=%p isGovernor=%d isDirectLinkage=%d speedControl=%.3f\n",
               (void*)t, (bool)dynamic_cast<Governor*>(t),
               (bool)dynamic_cast<DirectThrottleLinkage*>(t),
               t ? t->getSpeedControl() : -1.0);
        fflush(stdout);
    }

    // Load impulse responses (same as the CLI / preset-isomorphism smoke test).
    Simulator* innerSim = bridge->getInternalSimulator();
    ASSERT_NE(innerSim, nullptr);
    SimulatorInitHelpers::initializeConvolutionFilters(innerSim);

    // Configure afterfire AGGRESSIVELY — this is a path/wiring proof, not a realism
    // test, so guarantee events fire during the decel. The physics model lights off
    // when the manifold pressure collapses (overrun), the runner is hot and oxygen-
    // rich enough, and the pedal is below the throttle cutoff. Relax every gate so a
    // cut reliably produces pops within the short coast window.
    AfterfireConfig af;
    af.enabled = true;
    af.misfireManifoldPressurePa = 101325.0;  // ~1 atm: any low-MAP condition qualifies
    af.throttleCutoff = 0.5;                  // generous: cut is well below this
    af.ignitionDelayRefS = 0.001;             // near-instant light-off
    af.activationTempK = 8000.0;
    af.refTempK = 1000.0;
    af.autoIgnitionTempK = 300.0;             // low floor so a hot pipe always lights
    af.minRawFuelFraction = 1e-6;             // tiny: ensure raw fuel accumulates
    af.minOxygenMoleFraction = 1e-4;          // tiny: ensure oxygen present
    af.energyScale = 5.0;                     // loud, clearly-audible pop
    af.diagnostics = true;
    SimulatorFactory::configureAfterfire(sim.get(), af, nullptr);
    ASSERT_GE(sumAfterfireEvents(*bridge), 0.0) << "Afterfire diagnostics must be readable";

    // --- Start the engine: ignition + starter + full throttle ---
    bridge->setIgnition(true);
    bridge->setStarterMotor(true);

    const double dt = 1.0 / 60.0;
    double peakRpm = 0.0;
    const double skippedThrottleAtStart = sumSkippedThrottle(*bridge);  // baseline before the rev
    // REV for ~3s at full throttle. Disable the starter once it should be running.
    for (int i = 0; i < 180; ++i) {
        bridge->setThrottle(1.0);           // full throttle (naive user intent)
        if (i == 60) bridge->setStarterMotor(false);
        bridge->update(dt);
        peakRpm = std::max(peakRpm, bridge->getEngineRpm());
    }
    const double throttleAtFull = innerSim->getEngine()->getSpeedControl();

    // --- CUT throttle and let it coast; afterfire should fire on the cut ---
    const double eventsBeforeDecel = sumAfterfireEvents(*bridge);
    const double skippedThrottleBefore = sumSkippedThrottle(*bridge);
    double minDecelRpm = peakRpm;
    for (int i = 0; i < 240; ++i) {          // ~4s of coast-down
        bridge->setThrottle(0.0);           // throttle CUT (naive user intent)
        bridge->update(dt);
        minDecelRpm = std::min(minDecelRpm, bridge->getEngineRpm());
    }
    const double throttleAtCut = innerSim->getEngine()->getSpeedControl();
    const double eventsAfterDecel = sumAfterfireEvents(*bridge);
    const double skippedThrottleAfter = sumSkippedThrottle(*bridge);

    // POLARITY PROOF: the afterfire gate reads getSpeedControl() (the pedal), NOT
    // getThrottle() — getThrottle() is the plate restriction (0=open/full, 1=closed)
    // because DirectThrottleLinkage::update() overwrites it each tick. So setThrottle(x)
    // must produce getSpeedControl() ≈ x. (Asserting getThrottle here would be wrong.)
    EXPECT_NEAR(throttleAtFull, 1.0, 0.02)
        << "setThrottle(1.0) did not yield full pedal (getSpeedControl=" << throttleAtFull;
    EXPECT_NEAR(throttleAtCut, 0.0, 0.02)
        << "setThrottle(0.0) did not yield closed pedal (getSpeedControl=" << throttleAtCut;

    // Afterfire fired once the throttle was cut and RPM was in range — the core proof
    // that the full bridge path (setThrottle → engine → afterfire) works.
    EXPECT_GT(eventsAfterDecel, eventsBeforeDecel)
        << "Afterfire did not fire after the throttle cut. "
        << "before=" << eventsBeforeDecel << " after=" << eventsAfterDecel;

    // During the full-throttle rev, the pedal was ABOVE the cutoff, so the gate must
    // have counted those steps as skippedThrottle. skippedThrottleBefore is snapshotted
    // AFTER the rev, so it must exceed the pre-rev baseline (skippedThrottleAtStart) —
    // NOT the post-decel value (a decel is a throttle cut, so it adds zero skips).
    EXPECT_GT(skippedThrottleBefore, skippedThrottleAtStart)
        << "Throttle gate did not record skippedThrottle during full-throttle rev.";

    // RPM dynamics are engine-specific (some presets free-rev, some don't) so they are
    // recorded as info, not asserted.
    RecordProperty("peak_rpm", std::to_string(peakRpm));
    RecordProperty("min_decel_rpm", std::to_string(minDecelRpm));
    RecordProperty("throttle_at_full", std::to_string(throttleAtFull));
    RecordProperty("throttle_at_cut", std::to_string(throttleAtCut));
    RecordProperty("afterfire_events", std::to_string(eventsAfterDecel));
    printf("[result] throttle full=%.3f cut=%.3f peakRpm=%.0f events=%.0f skippedThrottle=%.0f\n",
           throttleAtFull, throttleAtCut, peakRpm, eventsAfterDecel,
           skippedThrottleAfter - skippedThrottleBefore);
    fflush(stdout);
#endif
}

// =============================================================================
// Afterfire MASTER VOLUME: --afterfire-gain (customGain) must scale the PHYSICAL
// combustion crackle, not just the WAV overlay. At customGain=0 the combustion
// releases no energy into the exhaust runner, so even though pops FIRE (the
// ignition event happens) the recorded physical energy is ~0 and the pop is
// silent; at customGain=1.0 the same decel releases real energy. This is the
// "gain 0 = no pop at all" contract end-to-end.
// =============================================================================
TEST(AfterfireBridgeTest, MasterVolumeScalesPhysicalCrackle) {
#ifndef ATG_ENGINE_SIM_AFTERFIRE_SPIKE
    GTEST_SKIP() << "ATG_ENGINE_SIM_AFTERFIRE_SPIKE not compiled in";
#else
    const std::string presetPath = std::string(TEST_PRESET_DIR) + "/v8_gm_ls.json";
    ASSERT_TRUE(std::filesystem::exists(presetPath)) << "Missing preset: " << presetPath;

    // Drive one decel at a given customGain and return (events, sum of recorded
    // physical energy released across all chambers).
    auto runDecelAtGain = [&](double gain) -> std::pair<double, double> {
        ISimulatorConfig config;
        config.sampleRate = 48000;
        config.simulationFrequency = 10000;

        std::filesystem::path savedCwd = std::filesystem::current_path();
        auto sim = SimulatorFactory::create(
            SimulatorType::PistonEngine, presetPath,
            std::string(TEST_ENGINE_SIM_ASSETS) + "/", config);
        std::filesystem::current_path(savedCwd);
        auto* bridge = dynamic_cast<BridgeSimulator*>(sim.get());
        EXPECT_NE(bridge, nullptr);
        if (bridge == nullptr) return {0.0, 0.0};
        const bool created = bridge->create(config, nullptr, nullptr);
        EXPECT_TRUE(created);
        if (!created) return {0.0, 0.0};

        SimulatorInitHelpers::initializeConvolutionFilters(bridge->getInternalSimulator());

        AfterfireConfig af;
        af.enabled = true;
        af.misfireManifoldPressurePa = 101325.0;
        af.throttleCutoff = 0.5;
        af.ignitionDelayRefS = 0.001;
        af.activationTempK = 8000.0;
        af.refTempK = 1000.0;
        af.autoIgnitionTempK = 300.0;
        af.minRawFuelFraction = 1e-6;
        af.minOxygenMoleFraction = 1e-4;
        af.energyScale = 5.0;
        af.customGain = gain;
        af.diagnostics = true;
        SimulatorFactory::configureAfterfire(sim.get(), af, nullptr);

        bridge->setIgnition(true);
        bridge->setStarterMotor(true);
        const double dt = 1.0 / 60.0;
        for (int i = 0; i < 180; ++i) {
            bridge->setThrottle(1.0);
            if (i == 60) bridge->setStarterMotor(false);
            bridge->update(dt);
        }
        for (int i = 0; i < 240; ++i) {
            bridge->setThrottle(0.0);
            bridge->update(dt);
        }

        double totalEvents = 0.0, totalEnergy = 0.0;
        for (const auto& d : bridge->getAfterfireDiagnostics()) {
            totalEvents += d.eventCount;
            totalEnergy += d.lastEventEnergyReleased;
        }
        return {totalEvents, totalEnergy};
    };

    const auto zero = runDecelAtGain(0.0);
    const auto full = runDecelAtGain(1.0);

    // Pops must still FIRE at gain 0 (the ignition event is independent of the
    // master volume) — this isolates "silent" from "didn't happen".
    EXPECT_GT(zero.first, 0.0)
        << "Pops must still fire at customGain=0 (ignition is gain-independent)";

    // But the physical energy released must be ~0 at gain 0 (no energy in the
    // pipe => no pressure spike => no audible crackle) and clearly > 0 at gain 1.
    EXPECT_NEAR(zero.second, 0.0, 1e-9)
        << "customGain=0 must release ~0 combustion energy (silent physical pop); got "
        << zero.second;
    EXPECT_GT(full.second, 0.0)
        << "customGain=1.0 must release real combustion energy (audible physical crackle)";

    RecordProperty("gain0_events", std::to_string(zero.first));
    RecordProperty("gain0_energy", std::to_string(zero.second));
    RecordProperty("gain1_events", std::to_string(full.first));
    RecordProperty("gain1_energy", std::to_string(full.second));
    printf("[master-volume] gain0 events=%.0f energy=%.3g | gain1 events=%.0f energy=%.3g\n",
           zero.first, zero.second, full.first, full.second);
    fflush(stdout);
#endif
}
