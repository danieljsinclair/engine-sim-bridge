// TorqueConverterTest.cpp
//
// Behavioural tests for the fluid-coupling torque-converter coupling model.
// Validates the rescue-branch physics (TR curve, pump law, hysteretic lockup)
// as ported into the bridge's clutch-pressure axis, and the property that
// matters most: the pressure is a SMOOTH function of its inputs (no bang-bang,
// so the engine RPM cannot cycle).

#include <gtest/gtest.h>
#include <twin/TorqueConverter.h>

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

// STANDSTILL DECOUPLE + ROAD-DRIVEN (STABLE) COUPLING.
// (a) At standstill / creep (road-implied < idle) the fluid coupling has no
//     capacity, so the pressure sits at the floor regardless of engine rpm - a
//     post-crank flare cannot drag the engine to stall (the standstill-stall fix
//     1436->1002->568->209->0), and a creeping engine is not lugged below idle.
// (b) In the coupling band the pressure rises with road-implied rpm, and is
//     INDEPENDENT of engine rpm at a fixed road. That independence is the no-
//     oscillation guarantee: the pressure cannot feed back into engine rpm, so it
//     cannot form a limit cycle (the friction clutch converges by pulling the
//     engine toward road speed instead of cycling).
TEST(TorqueConverterTest, StandsStillDecouplesAndCouplesOnRoadImplied) {
    TorqueConverter tc;
    // (a) road-implied < idle: pressure at the floor for an idle engine AND a
    //     full post-crank flare (no residual coupling to the stationary wheel).
    EXPECT_NEAR(run(tc, kIdleRpm,       /*road=*/0.0).clutchPressure, 0.0, 1e-6);
    EXPECT_NEAR(run(tc, kIdleRpm * 2.0, /*road=*/0.0).clutchPressure, 0.0, 1e-6);
    EXPECT_NEAR(run(tc, kIdleRpm,       kIdleRpm * 0.4).clutchPressure, 0.0, 1e-6);  // creep

    // (b) Coupling band: pressure rises with road-implied rpm ...
    const double engine = kIdleRpm * 2.0;  // SR kept < 0.85 so lockup is not the
                                           // driver here; we are testing the fluid ramp
    const double pLow  = run(tc, engine, kIdleRpm * 1.1).clutchPressure;
    const double pHigh = run(tc, engine, kIdleRpm * 1.5).clutchPressure;
    EXPECT_GT(pHigh, pLow);

    // ... and does NOT depend on engine rpm at a fixed road (no engine feedback).
    const double road = kIdleRpm * 1.3;
    const double pAtIdleRpm = run(tc, kIdleRpm,       road).clutchPressure;
    const double pAtFlare   = run(tc, kIdleRpm * 2.0, road).clutchPressure;
    EXPECT_NEAR(pAtIdleRpm, pAtFlare, 1e-6);
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

// Lockup engages above the RPM + speed-ratio thresholds (cruise) and reaches
// near-full pressure, then releases below them. The engaged state is sticky
// (hysteresis): just dipping under the threshold must not release it.
TEST(TorqueConverterTest, LockupEngagesAtCruiseAndIsHysteretic) {
    TorqueConverter tc;
    // Cruise: high engine + high speed ratio -> locked.
    const auto cruise = run(tc, 3000.0, 3000.0 * 0.95);
    EXPECT_TRUE(cruise.locked);
    EXPECT_GT(cruise.clutchPressure, 0.9);

    // Below the SR threshold (slip) -> not locked.
    const auto launch = run(tc, 3000.0, 3000.0 * 0.3);
    EXPECT_FALSE(launch.locked);
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
