// TorqueConverterLaunchTest.cpp
//
// Deterministic (input -> output) tests for the torque-converter LAUNCH
// pressure function. Pure function — no engine-sim physics, no CLI.
//
// The launch model (see TorqueConverterLaunch.h):
//   - Wheels coupled (roadImplied >= idle):       return DEFER (use slip-lock lock-up)
//   - Launch phase (roadImplied < idle):
//       loadFactor = clamp((engineRpm - idleRpm) / launchBand, 0, 1)
//       pressure   = loadFactor * throttle * stallPressureMax
// where launchBand = idleRpm * launchBandFactor (default -> stall @ 2*idle).
//
// Anti-bog property: at standstill, pressure rises with engine RPM above idle
// and is ~0 near/below idle, so a just-caught engine is never yanked down by a
// fixed creep pressure (the velocity-match catch-22).

#include <gtest/gtest.h>
#include <twin/TorqueConverterLaunch.h>

using namespace twin;

namespace {
// Common engine parameters (match zf8hp45 profile range).
constexpr double kIdleRpm = 750.0;
constexpr double kRedlineRpm = 6500.0;

double launch(double engineRpm, double roadImplied, double throttle,
              LaunchPressureOptions opts = LaunchPressureOptions{}) {
    return computeLaunchPressure(LaunchPressureInput{
        engineRpm, roadImplied, throttle, kIdleRpm, kRedlineRpm}, opts);
}
}  // namespace

// ---------------------------------------------------------------------------
// Phase boundary: defer to the slip-lock lock-up once the wheels are coupled.
// ---------------------------------------------------------------------------

TEST(TorqueConverterLaunchTest, CoupledWheelsAtIdle_DeferToSlipLock) {
    EXPECT_DOUBLE_EQ(launch(1500.0, kIdleRpm, 1.0), LAUNCH_PRESSURE_DEFER);
}

TEST(TorqueConverterLaunchTest, CoupledWheelsAboveIdle_DeferToSlipLock) {
    EXPECT_DOUBLE_EQ(launch(3000.0, 2500.0, 0.8), LAUNCH_PRESSURE_DEFER);
}

// ---------------------------------------------------------------------------
// Anti-bog: at standstill the pressure is ~0 near/below idle, so a just-caught
// engine is NOT yanked down by a fixed creep load.
// ---------------------------------------------------------------------------

TEST(TorqueConverterLaunchTest, StandstillEngineBelowIdle_UnloadsToZero_NoBog) {
    // Engine dragged below idle at standstill -> clutch must open so it recovers.
    EXPECT_NEAR(launch(500.0, 0.0, 1.0), 0.0, 1e-9);
}

TEST(TorqueConverterLaunchTest, StandstillEngineAtIdle_NearZero_NoBog) {
    // A just-caught engine sitting at idle must not be loaded.
    EXPECT_NEAR(launch(kIdleRpm, 0.0, 1.0), 0.0, 1e-9);
}

TEST(TorqueConverterLaunchTest, StandstillJustAboveIdle_GentlePressure) {
    // 50 RPM above idle -> a small, gentle pressure (not a fixed 0.10 creep).
    const double p = launch(kIdleRpm + 50.0, 0.0, 1.0);
    EXPECT_GT(p, 0.0);
    EXPECT_LT(p, 0.02);
}

// ---------------------------------------------------------------------------
// Launch torque: at stall speed under throttle, a sustainable pressure is
// transmitted — enough to creep, not enough to yank the engine.
// ---------------------------------------------------------------------------

TEST(TorqueConverterLaunchTest, StandstillAtStallSpeedFullThrottle_SustainablePressure) {
    // Engine well above the stall band, full throttle, wheels still: capped at
    // stallPressureMax (0.06 -> ~720 Nm with maxClutchTorque=12000).
    const double p = launch(2500.0, 0.0, 1.0);
    EXPECT_NEAR(p, 0.06, 1e-9);
    EXPECT_LT(p, 0.10);  // must NOT be a bog-inducing fixed creep
}

TEST(TorqueConverterLaunchTest, LaunchPressureNeverExceedsStallMax) {
    // Even screaming at redline under WOT the launch pressure is capped.
    EXPECT_NEAR(launch(kRedlineRpm, 0.0, 1.0), 0.06, 1e-9);
}

TEST(TorqueConverterLaunchTest, ZeroThrottleAtStandstill_OpenClutch) {
    // No gas -> no launch torque (clutch open), even with the engine revved.
    EXPECT_NEAR(launch(2500.0, 0.0, 0.0), 0.0, 1e-9);
}

// ---------------------------------------------------------------------------
// Negative feedback shape: pressure ramps with engine RPM and scales with
// throttle — the mechanism that holds the engine near its stall speed.
// ---------------------------------------------------------------------------

TEST(TorqueConverterLaunchTest, PressureRampsWithEngineRpm) {
    // At idle -> 0; mid-band -> half; at/above stall -> full cap.
    EXPECT_NEAR(launch(kIdleRpm,            0.0, 1.0), 0.0,  1e-9);  // idle
    EXPECT_NEAR(launch(kIdleRpm + 375.0,    0.0, 1.0), 0.03, 1e-9);  // mid-band (1125)
    EXPECT_NEAR(launch(kIdleRpm + 750.0,    0.0, 1.0), 0.06, 1e-9);  // stall (1500)
    EXPECT_NEAR(launch(kIdleRpm + 1500.0,   0.0, 1.0), 0.06, 1e-9);  // above stall (capped)
}

TEST(TorqueConverterLaunchTest, PressureScalesWithThrottle) {
    const double full = launch(2500.0, 0.0, 1.0);
    const double half = launch(2500.0, 0.0, 0.5);
    EXPECT_NEAR(full, 0.06, 1e-9);
    EXPECT_NEAR(half, 0.03, 1e-9);
}

// ---------------------------------------------------------------------------
// Tunability: custom options are honoured.
// ---------------------------------------------------------------------------

TEST(TorqueConverterLaunchTest, CustomStallMaxRespected) {
    LaunchPressureOptions opts;
    opts.stallPressureMax = 0.10;
    // Above the stall band under WOT -> the custom cap.
    EXPECT_NEAR(launch(2500.0, 0.0, 1.0, opts), 0.10, 1e-9);
}

TEST(TorqueConverterLaunchTest, CustomLaunchBandRespected) {
    LaunchPressureOptions opts;
    opts.launchBandFactor = 2.0;  // launchBand = 1500 -> stall @ idle+1500 = 2250
    // Half-way through the wider band at full throttle -> half the cap.
    EXPECT_NEAR(launch(kIdleRpm + 750.0, 0.0, 1.0, opts), 0.03, 1e-9);
    // Full pressure only at the new stall speed.
    EXPECT_NEAR(launch(kIdleRpm + 1500.0, 0.0, 1.0, opts), 0.06, 1e-9);
}

// ---------------------------------------------------------------------------
// Out-of-range throttle is clamped (defensive at the boundary input only).
// ---------------------------------------------------------------------------

TEST(TorqueConverterLaunchTest, OverscaleThrottleClampedToCap) {
    EXPECT_NEAR(launch(2500.0, 0.0, 1.5), 0.06, 1e-9);
    EXPECT_NEAR(launch(2500.0, 0.0, -0.5), 0.0, 1e-9);
}
