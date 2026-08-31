// TorqueConverterTest.cpp
//
// Behavioural tests for the fluid-coupling torque-converter coupling model.
// Validates the rescue-branch physics (TR curve, pump law, hysteretic lockup)
// as ported into the bridge's clutch-pressure axis, and the property that
// matters most: the pressure is a SMOOTH function of its inputs (no bang-bang,
// so the engine RPM cannot cycle).

#include <gtest/gtest.h>
#include <twin/TorqueConverter.h>
#include <twin/CouplingModelSelector.h>

#include <memory>
#include <vector>

using namespace twin;

namespace {
constexpr double kIdleRpm = 950.0;
constexpr double kRedlineRpm = 6500.0;

CouplingOutput run(TorqueConverter& tc, double engineRpm, double roadImpliedRpm,
                   double throttle = 0.0) {
    return tc.compute(CouplingInput{
        engineRpm, roadImpliedRpm, throttle, kIdleRpm, kRedlineRpm,
        /*maxClutchTorqueNm=*/12000.0, /*dt=*/0.01});
}
}  // namespace

// Torque ratio curve: 2.0 at stall (SR=0), monotonically falling to 1.0 at the
// coupling point (SR>=0.8). This is the MathWorks characteristic ported verbatim.
TEST(TorqueConverterTest, TorqueRatioCurveIsStallHighCouplingOne) {
    TorqueConverter tc;
    EXPECT_NEAR(tc.lookupTorqueRatio(0.0), 2.0, 1e-6);   // stall multiplication
    EXPECT_NEAR(tc.lookupTorqueRatio(0.8), 1.0, 1e-6);   // coupling point: 1:1
    EXPECT_NEAR(tc.lookupTorqueRatio(1.0), 1.0, 1e-6);   // locked: 1:1
    // Monotonic non-increasing across the slip range.
    double prev = tc.lookupTorqueRatio(0.0);
    for (int i = 1; i <= 20; ++i) {
        const double sr = static_cast<double>(i) / 20.0;
        const double tr = tc.lookupTorqueRatio(sr);
        EXPECT_LE(tr, prev + 1e-9);
        prev = tr;
    }
}

// STANDSTILL CREEP + ROAD-DRIVEN (STABLE) COUPLING.
// (a) At standstill / creep (road-implied < idle) the pressure holds a MODERATE
//     creep capacity: high enough that the fluid LOADS the engine (the engine
//     cannot free-rev unloaded), low enough that it slips (the pinned stationary
//     wheel cannot lug the engine to stall). NOT 0 (free-rev) and NOT 1.0
//     (rigid-couple oscillation).
// (b) In the coupling band the pressure rises with road-implied rpm, and is
//     INDEPENDENT of engine rpm at a fixed road. That independence is the no-
//     oscillation guarantee: the pressure cannot feed back into engine rpm, so it
//     cannot form a limit cycle (the friction clutch converges by pulling the
//     engine toward road speed instead of cycling).
TEST(TorqueConverterTest, StandsStillHoldsModerateCreepAndCouplesOnRoadImplied) {
    TorqueConverter tc;
    // (a) road-implied < idle: moderate pressure for an idle engine AND a
    //     full flare (the load must scale the flare down, not vanish).
    for (const double engine : {kIdleRpm, kIdleRpm * 2.0, kIdleRpm * 5.0}) {
        const double p = run(tc, engine, /*road=*/0.0).clutchPressure;
        EXPECT_GE(p, 0.2) << "standstill pressure too small to load the engine";
        EXPECT_LE(p, 0.6) << "standstill pressure rigidly couples the stationary wheel";
    }
    const double pCreep = run(tc, kIdleRpm, kIdleRpm * 0.4).clutchPressure;
    EXPECT_GE(pCreep, 0.2);
    EXPECT_LE(pCreep, 0.6);

    // (b) Coupling band: pressure rises with road-implied rpm ...
    const double engine = kIdleRpm * 2.0;  // a flaring engine — the ramp must not
                                           // depend on it (see also (c) below)
    const double pLow  = run(tc, engine, kIdleRpm * 1.1).clutchPressure;
    const double pHigh = run(tc, engine, kIdleRpm * 1.5).clutchPressure;
    EXPECT_GT(pHigh, pLow);

    // ... and does NOT depend on engine rpm at a fixed road (no engine feedback).
    const double road = kIdleRpm * 1.3;
    const double pAtIdleRpm = run(tc, kIdleRpm,       road).clutchPressure;
    const double pAtFlare   = run(tc, kIdleRpm * 5.0, road).clutchPressure;
    EXPECT_NEAR(pAtIdleRpm, pAtFlare, 1e-6);
}

// THE BENCH FAILURE (UpLeckHill, 16 mph DA2, 36% throttle): at road-implied
// 1544 rpm the engine free-revved to 6700-7259 rpm because the pressure (the
// converter capacity scale) sat at ~0.03. At cruise road-implied (>= idle*1.6)
// the pressure must be high enough that the converter's lockup capacity can
// hold the engine AT road speed — the engine cannot free-rev 3-4x above it.
TEST(TorqueConverterTest, ModerateRoadSpeedLoadsTheEngine) {
    TorqueConverter tc;
    // 16 mph DA2: road-implied 1544 = idle*1.626 (just past the lock point),
    // engine flaring at redline, part throttle.
    const double p = run(tc, /*engine=*/7000.0, /*road=*/1544.0, /*throttle=*/0.36)
                         .clutchPressure;
    EXPECT_GT(p, 0.5) << "engine free-revs above road speed at moderate road rpm";
    // Fully locked at cruise regardless of engine rpm / throttle.
    EXPECT_NEAR(run(tc, 2000.0, kIdleRpm * 2.0, 0.36).clutchPressure, 1.0, 1e-6);
    EXPECT_NEAR(run(tc, 7000.0, kIdleRpm * 2.0, 1.0).clutchPressure, 1.0, 1e-6);
}

// NO CHATTER: the pressure curve is MONOTONIC non-decreasing in road-implied
// rpm and frame-to-frame stable (max |delta| < 0.15 for a realistic per-frame
// road-implied step). The bench showed 63%->100%->17%->83%->92%->3%->66%
// frame-to-frame at 16 mph — an engine-rpm-fed lockup blend cycling through a
// narrow speed-ratio band. Sweeping road-implied at a FIXED (flaring) engine
// rpm exercises exactly that path: the pressure must be a function of the road
// only, so the sweep is smooth.
TEST(TorqueConverterTest, PressureIsMonotonicAndFrameStableInRoadImplied) {
    TorqueConverter tc;
    const double engine = 7000.0;  // flaring engine (the instability amplifier)
    double prev = -1.0;
    double maxStep = 0.0;
    for (int road = 0; road <= 4000; road += 100) {
        const double p = run(tc, engine, static_cast<double>(road), 0.36).clutchPressure;
        if (prev >= 0.0) {
            EXPECT_GE(p, prev - 1e-9)
                << "pressure decreased as road-implied rpm rose (non-monotonic at road="
                << road << ")";
            maxStep = std::max(maxStep, std::abs(p - prev));
        }
        prev = p;
    }
    EXPECT_LT(maxStep, 0.15) << "pressure jumped " << maxStep
                             << " in one 100-rpm road step (chatter)";

    // ENGINE-INDEPENDENCE at a fixed road: sweeping the ENGINE across the very
    // speed-ratio band where an engine-fed lockup blend would cycle (SR
    // 0.85..0.97 around road 1544 rpm) must not move the pressure at all.
    const double pRef = run(tc, 1544.0, 1544.0, 0.36).clutchPressure;
    for (int engine = 1200; engine <= 3000; engine += 50) {
        const double p = run(tc, static_cast<double>(engine), 1544.0, 0.36).clutchPressure;
        EXPECT_NEAR(p, pRef, 1e-6)
            << "pressure depends on engine rpm at fixed road (feedback loop)";
    }
}

// SMOOTHNESS: sweeping engine RPM across its range must NOT produce any
// step/discontinuity. A bang-bang clutch would show large frame-to-frame deltas;
// the fluid coupling's pressure is a continuous curve. This is the no-oscillation
// guarantee. Sweeps at a FIXED speed ratio (SR=0.7) so the slip gate is constant
// and the sweep exercises the pump gate's ramp across idle.
TEST(TorqueConverterTest, PressureIsSmoothAcrossEngineRpmSweep) {
    TorqueConverter tc;
    std::vector<double> pressures;
    for (int rpm = 0; rpm <= 6500; rpm += 50) {
        pressures.push_back(run(tc, static_cast<double>(rpm),
                                /*roadImplied=*/static_cast<double>(rpm) * 0.7).clutchPressure);
    }
    // No single step may exceed the ramp a smoothstep can produce over a 50-rpm
    // increment at this resolution (~ a few percent). A bang-bang model jumps
    // 0<->1 in one step; assert the max step is well below that.
    double maxStep = 0.0;
    for (size_t i = 1; i < pressures.size(); ++i) {
        maxStep = std::max(maxStep, std::abs(pressures[i] - pressures[i - 1]));
    }
    EXPECT_LT(maxStep, 0.10) << "pressure jumped " << maxStep << " in one 50-rpm step";
}

// SMOOTHNESS across speed ratio: as the turbine catches the pump (SR 0->1) the
// pressure must vary continuously, including through the lockup blend.
TEST(TorqueConverterTest, PressureIsSmoothAcrossSpeedRatioSweep) {
    TorqueConverter tc;
    const double engineRpm = 3000.0;  // above lockupRpm so lockup can engage
    std::vector<double> pressures;
    for (int i = 0; i <= 100; ++i) {
        const double sr = static_cast<double>(i) / 100.0;
        pressures.push_back(run(tc, engineRpm, engineRpm * sr).clutchPressure);
    }
    double maxStep = 0.0;
    for (size_t i = 1; i < pressures.size(); ++i) {
        maxStep = std::max(maxStep, std::abs(pressures[i] - pressures[i - 1]));
    }
    // 0.15 still rejects a bang-bang clutch (the legacy relief jumped ~0.9/frame,
    // 0↔full-lock) by ~6x, while allowing the smooth lockup-clutch ramp (the fluid
    // base is far below 1.0, so the lockup blend covers a wide pressure range over
    // a narrow SR band; the twin rate-limits the actual pressure on top of this).
    EXPECT_LT(maxStep, 0.15) << "pressure jumped " << maxStep << " across an SR step";
}

// Lockup engages at cruise road-implied rpm (>= idle * 1.6) and releases below
// it. The engaged state is sticky (hysteresis): just dipping under the threshold
// must not release it. The lock state follows the ROAD (exogenous), never the
// engine, so it cannot chatter with engine feedback.
TEST(TorqueConverterTest, LockupEngagesAtCruiseAndIsHysteretic) {
    TorqueConverter tc;
    // Cruise: high road-implied -> locked, full pressure.
    const auto cruise = run(tc, 3000.0, 3000.0 * 0.95);
    EXPECT_TRUE(cruise.locked);
    EXPECT_GT(cruise.clutchPressure, 0.9);

    // Low road-implied (launch / slip) -> not locked.
    const auto launch = run(tc, 3000.0, 3000.0 * 0.3);
    EXPECT_FALSE(launch.locked);

    // Hysteresis: re-engage at cruise, then a shallow dip below the lock point
    // stays locked; a deep drop (below the release band) releases.
    const double lockPoint = kIdleRpm * 1.6;
    EXPECT_TRUE(run(tc, 1600.0, lockPoint + 100.0).locked);  // re-engage
    EXPECT_TRUE(run(tc, 1600.0, lockPoint - 75.0).locked);   // shallow dip
    EXPECT_FALSE(run(tc, 1600.0, lockPoint - 300.0).locked); // deep drop
}

// The pressure never leaves its physical [0, 1] range for any input.
TEST(TorqueConverterTest, PressureStaysInUnitRange) {
    TorqueConverter tc;
    for (int e = 0; e <= 7000; e += 500) {
        for (int r = 0; r <= e; r += 250) {
            const double p = run(tc, static_cast<double>(e), static_cast<double>(r)).clutchPressure;
            EXPECT_GE(p, 0.0);
            EXPECT_LE(p, 1.0);
        }
    }
}

// ---------------------------------------------------------------------------
// CREEP-FIX (2026-08-31): the standstill capacity is SPLIT BY REGIME on
// driver throttle (the resolution of the "single-number trap" documented on
// TorqueConverterParameters::creepPressure). Owner driveability evidence
// (2026-08-30/31): with one flat 0.6 the in-gear idle hunted and stalled
// without throttle (bench: D-idle droop ~-350 rpm vs a real C63's ~50-150,
// exhaust-flow sign-flutter ~3x Park); a flat 0.35/0.4 relieved the standstill
// load but scaled the part-throttle launch-flare equilibrium past the free-rev
// design point (3041 -> 3486/3603 rpm in the grid). The split: zero throttle
// idles at the reduced capacity; launch throttle keeps the full one.
// ---------------------------------------------------------------------------

// Zero-throttle standstill must idle on the REDUCED capacity; launch throttle
// must keep the FULL capacity (flare protection). Both ends pinned via the
// production default the selector installs.
TEST(TorqueConverterTest, StandstillCreepIsSplit_IdleReduced_LaunchFull_CreepFix) {
    auto model = makeCouplingModel(CouplingModelKind::TorqueConverter);
    CouplingInput in{
        /*engineRpm=*/kIdleRpm, /*roadSpeedImpliedRpm=*/0.0,
        /*throttle=*/0.0, kIdleRpm, kRedlineRpm,
        /*maxClutchTorqueNm=*/12000.0, /*dt=*/0.01};

    in.throttleFraction = 0.0;  // stoplight: zero driver throttle
    EXPECT_NEAR(model->compute(in).clutchPressure, 0.35, 1e-9);

    in.throttleFraction = 0.25;  // launch (top of the blend band)
    EXPECT_NEAR(model->compute(in).clutchPressure, 0.6, 1e-9);

    in.throttleFraction = 1.0;  // WOT launch
    EXPECT_NEAR(model->compute(in).clutchPressure, 0.6, 1e-9);

    // The blend is monotonic in throttle and passes through its midpoint.
    in.throttleFraction = 0.15;
    const double pMid = model->compute(in).clutchPressure;
    EXPECT_GT(pMid, 0.35);
    EXPECT_LT(pMid, 0.6);
    EXPECT_NEAR(pMid, 0.35 + (0.6 - 0.35) * 0.5, 0.02)  // smoothstep(0.05..0.25, 0.15) ~ 0.5
        << "the throttle blend must be the smooth ramp, not a step";

    // Cruise is unaffected by the split: locked at 1.0 at any throttle.
    in.roadSpeedImpliedRpm = kIdleRpm * 2.0;
    in.throttleFraction = 0.0;
    EXPECT_NEAR(model->compute(in).clutchPressure, 1.0, 1e-9);

    // The engine-rpm independence guarantee survives the split: at fixed road
    // AND fixed throttle the pressure cannot see engine rpm (no feedback loop).
    in.roadSpeedImpliedRpm = 0.0;
    in.throttleFraction = 0.15;
    const double pAtIdle = model->compute(in).clutchPressure;
    in.engineRpm = kRedlineRpm;
    EXPECT_NEAR(model->compute(in).clutchPressure, pAtIdle, 1e-9);
}
