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
    // Engine well above the stall band, full throttle, wheels still: a
    // sustainable pressure - non-zero (it transmits launch torque) but well
    // below a bog-inducing fixed creep. The exact cap is declarative tuning.
    const double p = launch(2500.0, 0.0, 1.0);
    EXPECT_GT(p, 0.0);
    EXPECT_LT(p, 0.10);  // must NOT be a bog-inducing fixed creep
}

TEST(TorqueConverterLaunchTest, LaunchPressureNeverExceedsStallMax) {
    // Even screaming at redline under WOT the launch pressure is capped: it is
    // flat (no further rise) beyond the stall band and stays in the sustainable
    // band, whatever the default cap is tuned to.
    const double atStall = launch(2500.0, 0.0, 1.0);
    const double atRedline = launch(kRedlineRpm, 0.0, 1.0);
    EXPECT_NEAR(atRedline, atStall, 1e-9)
        << "Pressure must be flat (capped) above the stall band";
    EXPECT_GT(atRedline, 0.0);
    EXPECT_LT(atRedline, 0.10);
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
    // The SHAPE is the spec: ~0 at idle, strictly rising through the launch
    // band, flat (capped) at/above stall. The exact curve values are
    // declarative tuning and may change.
    const double atIdle = launch(kIdleRpm, 0.0, 1.0);
    const double midBand = launch(kIdleRpm + 375.0, 0.0, 1.0);
    const double atStall = launch(kIdleRpm + 750.0, 0.0, 1.0);
    const double aboveStall = launch(kIdleRpm + 1500.0, 0.0, 1.0);
    EXPECT_NEAR(atIdle, 0.0, 1e-9) << "No load on a just-caught engine at idle";
    EXPECT_GT(midBand, atIdle) << "Pressure must ramp up through the band";
    EXPECT_GT(atStall, midBand) << "Pressure must keep rising to the stall point";
    EXPECT_NEAR(aboveStall, atStall, 1e-9) << "Capped (flat) above the stall band";
}

TEST(TorqueConverterLaunchTest, PressureScalesWithThrottle) {
    // More throttle -> more launch pressure; full throttle stays in the
    // sustainable band (the absolute values are tuning).
    const double full = launch(2500.0, 0.0, 1.0);
    const double half = launch(2500.0, 0.0, 0.5);
    EXPECT_GT(half, 0.0);
    EXPECT_GT(full, half);
    EXPECT_LT(full, 0.10);
}

// ---------------------------------------------------------------------------
// Tunability: custom options are honoured.
// ---------------------------------------------------------------------------

TEST(TorqueConverterLaunchTest, CustomStallMaxRespected) {
    LaunchPressureOptions opts;
    opts.stallPressureMax = 0.10;
    // Above the stall band under WOT the pressure reaches exactly the
    // caller-supplied cap (the cap is an input contract, not tuning here).
    EXPECT_NEAR(launch(2500.0, 0.0, 1.0, opts), opts.stallPressureMax, 1e-9);
}

TEST(TorqueConverterLaunchTest, CustomLaunchBandRespected) {
    LaunchPressureOptions opts;
    opts.launchBandFactor = 2.0;  // launchBand = 1500 -> stall @ idle+1500 = 2250
    // A WIDER band must delay the ramp: at idle+750 the wider band applies
    // LESS pressure than the default band, and the cap is reached by the new
    // stall speed (relative shape, not absolute values).
    const double midDefault = launch(kIdleRpm + 750.0, 0.0, 1.0);
    const double midWide = launch(kIdleRpm + 750.0, 0.0, 1.0, opts);
    EXPECT_LT(midWide, midDefault)
        << "A wider launch band must apply less pressure mid-band";
    EXPECT_NEAR(launch(kIdleRpm + 1500.0, 0.0, 1.0, opts),
                opts.stallPressureMax, 1e-9)
        << "Full pressure at the new (wider) stall speed";
}

// ---------------------------------------------------------------------------
// Out-of-range throttle is clamped (defensive at the boundary input only).
// ---------------------------------------------------------------------------

TEST(TorqueConverterLaunchTest, OverscaleThrottleClampedToCap) {
    // Out-of-range throttle behaves exactly like its clamped value
    // (self-relative: no tuning constant is pinned).
    EXPECT_NEAR(launch(2500.0, 0.0, 1.5), launch(2500.0, 0.0, 1.0), 1e-9);
    EXPECT_NEAR(launch(2500.0, 0.0, -0.5), launch(2500.0, 0.0, 0.0), 1e-9);
}
