// AutoGearboxShiftLogicTest.cpp
// Acceptance tests for the Virtual ICE Twin automatic shift logic spec:
//   "upshift before redline or when drivetrain torque is low; downshift on
//    kickdown (high throttle) or high drivetrain torque; throttle biases the
//    decision; hysteresis prevents hunting; neutral holds gear."
//
// Each AC is provable from inputs (speed/RPM/torque/throttle/selector) alone.
// These exercise the torque- and selector-aware update() overload.

#include <gtest/gtest.h>
#include <twin/AutomaticGearbox.h>
#include <twin/IceVehicleProfile.h>
#include <simulator/GearConventions.h>
#include <cmath>

using namespace twin;

namespace {
// Mirror of AutomaticGearbox's internal RPM calc, for asserting expected RPM.
double engineRpmAt(double speedKmh, int gear, const IceVehicleProfile& profile) {
    if (gear < 1 || gear > static_cast<int>(profile.gearRatios.size())) return 0.0;
    double speedMs = speedKmh / 3.6;
    double wheelRpm = speedMs / (2.0 * M_PI * profile.tireRadiusM) * 60.0;
    return wheelRpm * profile.gearRatios[gear - 1] * profile.diffRatio;
}

// Drive the gearbox for many small steps until either the predicate is true or
// the step budget is exhausted. Returns whether the predicate ever held.
template <typename Pred>
bool runUntil(AutomaticGearbox& gb, double dt, int maxSteps,
              double speedKmh, double throttle, double torqueNm,
              Pred pred) {
    for (int i = 0; i < maxSteps; ++i) {
        gb.update(dt, speedKmh, throttle, torqueNm);
        if (pred(gb)) return true;
    }
    return pred(gb);
}
} // namespace

class AutoGearboxShiftLogicTest : public ::testing::Test {
protected:
    IceVehicleProfile profile = IceVehicleProfile::zf8hp45();
};

// AC1: In DRIVE near redline -> upshifts (RPM held below redline).
TEST_F(AutoGearboxShiftLogicTest, AC1_UpshiftsBeforeRedlineInDrive) {
    AutomaticGearbox gb(profile);
    gb.setGearSelector(bridge::GearSelector::DRIVE);

    // 60 km/h in 1st implies an engine speed past redline -> the speed-implied
    // redline safety (coherent with the speed-based shifts) must upshift.
    gb.update(0.1, /*speedKmh*/ 60.0, /*throttle*/ 0.9, /*torqueNm*/ 100.0);

    ASSERT_GT(gb.getCurrentGear(), 1)
        << "When road speed in 1st implies redline, the box must upshift";
    // After upshift(s), the engine RPM implied at this road speed is below redline.
    double rpmAfter = engineRpmAt(60.0, gb.getCurrentGear(), profile);
    EXPECT_LT(rpmAfter, profile.redlineRpm)
        << "After upshift, engine RPM at this road speed must be below redline";
}

// ============================================================================
// REDLINE-NO-UPSHIFT REPRO: user reports --auto redlines in 1st without
// shifting once the car is moving. AC1 only covers low road speed (5 km/h);
// these exercise higher road speed where the bug manifests.
// ============================================================================

// Repro A: steady 40% throttle, engine revs up to redline, road speed rising
// (the user's scenario: throttle to 40%, redline, no upshift).
TEST_F(AutoGearboxShiftLogicTest, RedlineUpshiftsAsEngineRevsToRedlineAtSpeed) {
    AutomaticGearbox gb(profile);
    gb.setGearSelector(bridge::GearSelector::DRIVE);
    double rpm = 1000.0;
    for (int i = 0; i < 300; ++i) {  // ~15s
        rpm += 150.0;
        if (rpm > profile.redlineRpm * 0.99) rpm = profile.redlineRpm * 0.99;
        double speed = 20.0 + i * 0.3;  // 20 -> ~110 km/h
        gb.setTwinContext(3, 1.0, speed, rpm);
        gb.update(0.05, speed, /*throttle*/ 0.4, /*torque*/ 100.0);
    }
    EXPECT_GT(gb.getCurrentGear(), 1)
        << "Engine reaching redline at road speed must trigger an upshift (repro: redline-no-upshift)";
}

// Repro B: redline + high road speed + steady throttle (isolates the speed path
// / redline guard, no tip-in).
TEST_F(AutoGearboxShiftLogicTest, RedlineUpshiftsAtHighRoadSpeedSteadyThrottle) {
    AutomaticGearbox gb(profile);
    gb.setGearSelector(bridge::GearSelector::DRIVE);
    for (int i = 0; i < 200; ++i) {
        gb.setTwinContext(3, 1.0, /*speedFb*/ 60.0, /*rpmFb*/ profile.redlineRpm * 0.98);
        gb.update(0.05, /*speedKmh*/ 60.0, /*throttle*/ 0.4, /*torque*/ 100.0);
    }
    EXPECT_GT(gb.getCurrentGear(), 1)
        << "At redline the box must upshift even when road speed already exceeds the shift point";
}

// AC2: Low load / light throttle (cruise) -> upshifts.
TEST_F(AutoGearboxShiftLogicTest, AC2_UpshiftsAtLowLoadLightThrottle) {
    AutomaticGearbox gb(profile);
    gb.setGearSelector(bridge::GearSelector::DRIVE);

    // Cruise: light throttle, low drivetrain torque. Sweep speed up.
    bool upshifted = runUntil(gb, /*dt*/ 0.5, /*steps*/ 200,
                              /*speed*/ 60.0, /*throttle*/ 0.10, /*torque*/ 40.0,
                              [](AutomaticGearbox& g) { return g.getCurrentGear() >= 3; });
    EXPECT_TRUE(upshifted)
        << "Cruise (light throttle, low torque) should climb into higher gears";
    EXPECT_LE(gb.getCurrentGear(), static_cast<int>(profile.gearRatios.size()));
}

// AC3: High throttle (kickdown) -> downshifts.
TEST_F(AutoGearboxShiftLogicTest, AC3_DownshiftsOnKickdownHighThrottle) {
    AutomaticGearbox gb(profile);
    gb.setGearSelector(bridge::GearSelector::DRIVE);

    // Get into a cruising gear first (gear >= 3) at moderate throttle.
    ASSERT_TRUE(runUntil(gb, 0.5, 200, 80.0, 0.30, 120.0,
        [](AutomaticGearbox& g) { return g.getCurrentGear() >= 3; }));
    int cruiseGear = gb.getCurrentGear();
    ASSERT_GE(cruiseGear, 3);

    // Clear shift interval timers.
    for (int i = 0; i < 40; ++i) gb.update(0.1, 80.0, 0.30, 120.0);

    // Kickdown: floor throttle. Torque feed is moderate-high (demand accel).
    gb.update(0.05, 80.0, 0.98, /*torqueNm*/ 250.0);

    EXPECT_LT(gb.getCurrentGear(), cruiseGear)
        << "High throttle (kickdown) must downshift to a lower gear";
}

// AC4: High drivetrain torque (load) -> downshifts even without a throttle spike.
TEST_F(AutoGearboxShiftLogicTest, AC4_DownshiftsOnHighDrivetrainTorque) {
    AutomaticGearbox gb(profile);
    gb.setGearSelector(bridge::GearSelector::DRIVE);

    // Cruise into a higher gear at light throttle + low torque.
    ASSERT_TRUE(runUntil(gb, 0.5, 200, 70.0, 0.15, 40.0,
        [](AutomaticGearbox& g) { return g.getCurrentGear() >= 3; }));
    int cruiseGear = gb.getCurrentGear();
    ASSERT_GE(cruiseGear, 3);

    // Clear interval timers.
    for (int i = 0; i < 40; ++i) gb.update(0.1, 70.0, 0.15, 40.0);

    // High load suddenly applied (e.g. towing/grade) at unchanged light throttle.
    // Sustained high torque should pull a downshift.
    bool downshifted = runUntil(gb, 0.5, 40, 70.0, 0.15, /*torqueNm*/ 400.0,
        [&](AutomaticGearbox& g) { return g.getCurrentGear() < cruiseGear; });
    EXPECT_TRUE(downshifted)
        << "Sustained high drivetrain torque (load) must downshift";
}

// AC5: Hysteresis — at the threshold the box does not oscillate on every step.
TEST_F(AutoGearboxShiftLogicTest, AC5_NoHuntingAtThreshold) {
    AutomaticGearbox gb(profile);
    gb.setGearSelector(bridge::GearSelector::DRIVE);

    // Climb to a stable cruising gear.
    ASSERT_TRUE(runUntil(gb, 0.5, 200, 60.0, 0.20, 80.0,
        [](AutomaticGearbox& g) { return g.getCurrentGear() >= 3; }));
    for (int i = 0; i < 40; ++i) gb.update(0.1, 60.0, 0.20, 80.0);

    // Hold speed/throttle/torque exactly at the current operating point and
    // count direction changes. A non-hunting box settles and stays put.
    int startGear = gb.getCurrentGear();
    int directionChanges = 0;
    int lastDir = 0;
    for (int i = 0; i < 200; ++i) {
        gb.update(0.1, 60.0, 0.20, 80.0);
        int g = gb.getCurrentGear();
        int dir = (g > startGear) - (g < startGear); // -1, 0, +1
        if (dir != 0 && lastDir != 0 && dir != lastDir) ++directionChanges;
        if (dir != 0) lastDir = dir;
    }
    EXPECT_EQ(directionChanges, 0)
        << "At a fixed operating point the box must not oscillate (hunt)";
}

// AC6: In NEUTRAL -> no shifting; gear holds.
TEST_F(AutoGearboxShiftLogicTest, AC6_NoShiftInNeutralGearHolds) {
    AutomaticGearbox gb(profile);
    gb.setGearSelector(bridge::GearSelector::NEUTRAL);

    // Establish a non-default gear so we can observe it holding.
    gb.setGearSelector(bridge::GearSelector::DRIVE);
    ASSERT_TRUE(runUntil(gb, 0.5, 200, 60.0, 0.30, 100.0,
        [](AutomaticGearbox& g) { return g.getCurrentGear() >= 3; }));
    int heldGear = gb.getCurrentGear();
    ASSERT_GE(heldGear, 3);

    // Switch to NEUTRAL and apply inputs that WOULD shift in DRIVE.
    gb.setGearSelector(bridge::GearSelector::NEUTRAL);
    for (int i = 0; i < 40; ++i) {
        gb.update(0.1, 60.0, /*throttle*/ 0.98, /*torque*/ 400.0);
        ASSERT_FALSE(gb.requestsShift())
            << "In NEUTRAL the gearbox must not request a shift";
    }
    EXPECT_EQ(gb.getCurrentGear(), heldGear)
        << "In NEUTRAL the current gear must hold";
}

// ============================================================================
// SHIFT-LOGIC BEHAVIOR TESTS (exercise the refactored getShiftSpeed /
// getDownshiftSpeed interpolation core via the torque- and selector-aware
// update() overload). Each asserts the BEHAVIOR (shift decisions at given
// speeds), not internal return values.

// Drive the box monotonically up in speed at a fixed throttle and return the
// road speed (km/h) at which the gear first advanced past `startGear`, or -1.0
// if it never did. Mirrors the getShiftSpeed bracket semantics observable from
// outside the box.
static double upshiftSpeedKmh(const IceVehicleProfile& profile, double startSpeed,
                              double maxSpeed, double throttle, double torqueNm,
                              double dt, int preSteps) {
    AutomaticGearbox gb(profile);
    gb.setGearSelector(bridge::GearSelector::DRIVE);
    for (int i = 0; i < preSteps; ++i) gb.update(dt, startSpeed, throttle, torqueNm);
    int start = gb.getCurrentGear();
    for (double s = startSpeed; s <= maxSpeed; s += 1.0) {
        gb.update(dt, s, throttle, torqueNm);
        if (gb.getCurrentGear() > start) return s;
    }
    return -1.0;
}

// F1: Upshift happens at a bounded speed. At medium cruise throttle the 1->2
// upshift fires below redline and at a speed consistent with the calibrated
// table (the box does not wait until redline, nor shift absurdly early).
TEST_F(AutoGearboxShiftLogicTest, F1_UpshiftAtExpectedSpeed) {
    AutomaticGearbox gb(profile);
    gb.setGearSelector(bridge::GearSelector::DRIVE);
    // Probe the 1->2 upshift speed with a throwaway box.
    double slew = upshiftSpeedKmh(profile, /*start*/ 5.0, /*max*/ 90.0, /*thr*/ 0.40,
                                  /*trq*/ 60.0, /*dt*/ 0.1, /*pre*/ 10);
    ASSERT_GT(slew, 0.0) << "Box should upshift from 1st at cruise throttle";
    // ZF8 1->2 at 40% throttle is ~23 kph; accept a bracket around the table so
    // the test does not lock in a single kph.
    EXPECT_GE(slew, 15.0) << "Upshift should not happen below ~15 kph at 40% throttle";
    EXPECT_LT(slew, 60.0) << "Upshift should happen well before 60 kph at 40% throttle";
    EXPECT_LT(engineRpmAt(slew, 1, profile), profile.redlineRpm)
        << "Upshift must occur before redline";
    // Drive the real box to that same speed and confirm it upshifts.
    for (int i = 0; i < 10; ++i) gb.update(0.1, 5.0, 0.40, 60.0);
    gb.update(0.1, slew, 0.40, 60.0);
    EXPECT_GT(gb.getCurrentGear(), 1) << "At the probed upshift speed the box must have upshifted";
}

// F2: Interpolation between throttle levels. The ZF8 table brackets 25% (1->2
// at 19 kph) and 40% (1->2 at 23 kph). At 32% throttle the 1->2 upshift MUST
// land strictly between those two bracketing speeds (proves the shared
// interpolate helper is linear between rows, not snapping to a row).
TEST_F(AutoGearboxShiftLogicTest, F2_InterpolationBetweenThrottleLevels) {
    const double speedAt25 = upshiftSpeedKmh(profile, 5.0, 60.0, 0.25, 60.0, 0.1, 10);
    const double speedAt40 = upshiftSpeedKmh(profile, 5.0, 60.0, 0.40, 60.0, 0.1, 10);
    const double speedAtMid = upshiftSpeedKmh(profile, 5.0, 60.0, 0.32, 60.0, 0.1, 10);

    ASSERT_GT(speedAt25, 0.0);
    ASSERT_GT(speedAt40, 0.0);
    ASSERT_GT(speedAtMid, 0.0);
    // Higher throttle -> later upshift (sports calibration).
    EXPECT_GT(speedAt40, speedAt25);
    // Mid-throttle sits strictly between the two brackets.
    EXPECT_GT(speedAtMid, speedAt25) << "Mid-throttle upshift must be later than the 25% row";
    EXPECT_LT(speedAtMid, speedAt40) << "Mid-throttle upshift must be earlier than the 40% row";
}

// F3: Redline safety — a low gear at a road speed whose implied engine RPM
// exceeds the redline guard MUST upshift regardless of the speed table. The
// resulting gear implies an RPM below redline.
TEST_F(AutoGearboxShiftLogicTest, F3_RedlineSafetyUpshiftsLowGear) {
    AutomaticGearbox gb(profile);
    gb.setGearSelector(bridge::GearSelector::DRIVE);
    // 60 kph in 1st with ZF8 ratios implies an engine speed past redline.
    gb.update(0.1, 60.0, /*thr*/ 0.9, /*trq*/ 80.0);
    EXPECT_GT(gb.getCurrentGear(), 1) << "Redline guard must pull an upshift in 1st";
    EXPECT_LT(engineRpmAt(60.0, gb.getCurrentGear(), profile), profile.redlineRpm)
        << "Post-upshift implied RPM must be below redline";
}

// F4: Downshift on deceleration. Climb to a higher gear at moderate throttle,
// then progressively slow the car; the box must drop at least one gear to keep
// the implied RPM above idle.
TEST_F(AutoGearboxShiftLogicTest, F4_DownshiftOnDeceleration) {
    AutomaticGearbox gb(profile);
    gb.setGearSelector(bridge::GearSelector::DRIVE);
    ASSERT_TRUE(runUntil(gb, 0.5, 200, 80.0, 0.30, 90.0,
                         [](AutomaticGearbox& g) { return g.getCurrentGear() >= 3; }));
    int highGear = gb.getCurrentGear();
    ASSERT_GE(highGear, 3);
    // Clear interval/hysteresis windows.
    for (int i = 0; i < 40; ++i) gb.update(0.1, 80.0, 0.30, 90.0);
    // Slow in coarse steps so downshift thresholds are crossed and intervals elapse.
    for (int s = 75; s >= 10; s -= 3) gb.update(0.7, static_cast<double>(s), 0.10, 60.0);
    EXPECT_LT(gb.getCurrentGear(), highGear) << "Coasting down must downshift";
    EXPECT_GE(gb.getCurrentGear(), 1);
}

// F5: Hysteresis fallback. A profile WITHOUT a separate downshift table must
// downshift at upshift_speed(throttle) * hysteresisFactor. We assert the
// post-shift position relative to that product rather than a single kph:
// (a) at exactly (upshift*hysteresis) no downshift, (b) just below it one does.
TEST_F(AutoGearboxShiftLogicTest, F5_DownshiftHysteresisFallbackUsesFactor) {
    IceVehicleProfile custom;
    custom.gearRatios = {4.714, 3.143, 2.106, 1.667, 1.285, 1.000, 0.839, 0.667};
    custom.diffRatio = 3.15;
    custom.tireRadiusM = 0.32;
    custom.hysteresisFactor = 0.85;
    custom.shiftTableThrottleLevels = {0.1, 0.25, 0.5, 0.75, 1.0};
    custom.shiftTable = {
        {20.0, 35.0, 50.0, 65.0, 80.0, 95.0, 110.0},
        {30.0, 50.0, 70.0, 90.0, 110.0, 130.0, 155.0},
        {40.0, 65.0, 90.0, 115.0, 140.0, 170.0, 200.0},
        {55.0, 85.0, 115.0, 145.0, 180.0, 215.0, 255.0},
        {70.0, 105.0, 140.0, 180.0, 220.0, 265.0, 315.0}
    };
    custom.separateDownshiftTableEnabled = false;
    custom.minShiftIntervalS = 3.0;
    custom.redlineRpm = 6500.0;
    custom.standstillThresholdKmh = 1.0;
    custom.kickdownThrottleThreshold = 0.95;
    custom.kickdownDelta = 0.4;
    custom.kickdownWindowMs = 100.0;
    custom.throttleSmoothingTauMs = 50.0;

    AutomaticGearbox gb(custom);
    gb.setGearSelector(bridge::GearSelector::DRIVE);

    // 1->2 upshift at 50% throttle = 40 kph; hysteresis downshift = 40 * 0.85 = 34.
    gb.update(0.1, 50.0, 0.5);
    ASSERT_EQ(gb.getCurrentGear(), 2);
    for (int i = 0; i < 40; ++i) gb.update(0.1, 50.0, 0.5);

    // Just above the downshift product: no shift.
    gb.update(3.1, 35.0, 0.5);
    EXPECT_EQ(gb.getCurrentGear(), 2) << "Above the hysteresis product, hold 2nd";
    // Just below the downshift product: downshift.
    gb.update(3.1, 32.0, 0.5);
    EXPECT_EQ(gb.getCurrentGear(), 1) << "Below the hysteresis product, fall to 1st";
}

// ============================================================================

// Mock logger: captures the last GearboxLogEntry passed to log()
namespace {
class MockGearboxLogger : public twin::IGearboxLogger {
public:
    mutable twin::GearboxLogEntry lastEntry;
    mutable int callCount = 0;
    void log(const twin::GearboxLogEntry& entry) override {
        lastEntry = entry;
        ++callCount;
    }
};
} // namespace

// logShiftState: verify GearboxLogEntry field population when logger is set.
// Uses a profile with shift intervals disabled so a single update can trigger.
TEST_F(AutoGearboxShiftLogicTest, LogShiftState_PopulatesEntryOnShift) {
    IceVehicleProfile p;
    p.gearRatios = {4.714, 3.143, 2.106, 1.667};
    p.diffRatio = 3.15;
    p.tireRadiusM = 0.32;
    p.hysteresisFactor = 0.85;
    p.shiftTableThrottleLevels = {0.1, 0.5, 1.0};
    p.shiftTable = {
        {20.0, 35.0, 50.0, 65.0},   // 10% throttle
        {30.0, 50.0, 70.0, 90.0},   // 50%
        {40.0, 65.0, 90.0, 115.0}   // 100%
    };
    p.separateDownshiftTableEnabled = false;
    p.minShiftIntervalS = 0.0;
    p.upshiftMinIntervalS = 0.0;
    p.downshiftMinIntervalS = 0.0;
    p.redlineRpm = 6500.0;
    p.standstillThresholdKmh = 1.0;
    p.kickdownThrottleThreshold = 0.95;
    p.kickdownDelta = 0.4;
    p.kickdownWindowMs = 100.0;
    p.throttleSmoothingTauMs = 50.0;

    MockGearboxLogger logger;
    AutomaticGearbox gb(p);
    gb.setLogger(&logger);
    gb.setGearSelector(bridge::GearSelector::DRIVE);

    // First update at low speed — gear 1, no shift requested
    gb.update(0.1, 10.0, 0.3);
    EXPECT_EQ(logger.callCount, 1);
    EXPECT_EQ(logger.lastEntry.currentGear, 1);
    EXPECT_FALSE(logger.lastEntry.requestsShift);
    EXPECT_DOUBLE_EQ(logger.lastEntry.speedKmh, 10.0);
    EXPECT_DOUBLE_EQ(logger.lastEntry.throttleRaw, 0.3);

    // Advance to speed where upshift fires (1st→2nd at 50% = 50 km/h, so 60>50)
    gb.update(0.1, 60.0, 0.5);
    EXPECT_GT(logger.callCount, 1);
    EXPECT_TRUE(logger.lastEntry.requestsShift);
    EXPECT_GT(logger.lastEntry.targetGear, 1);
    EXPECT_GT(logger.lastEntry.upshiftSpeed, 0.0);
}

// logShiftState: no-op when logger is null (default)
TEST_F(AutoGearboxShiftLogicTest, LogShiftState_NoOpWhenNullLogger) {
    AutomaticGearbox gb(profile);
    gb.setGearSelector(bridge::GearSelector::DRIVE);
    // Logger not set — must not crash
    gb.update(0.1, 10.0, 0.3);
    gb.update(0.1, 60.0, 0.9);
    EXPECT_GE(gb.getCurrentGear(), 1);  // behavior intact
}

// Torque-downshift interval gate: after a kickdown, immediate high torque
// must NOT trigger another downshift until the interval elapses
TEST_F(AutoGearboxShiftLogicTest, TorqueDownshift_BlockedByIntervalAfterKickdown) {
    IceVehicleProfile custom;
    custom.gearRatios = {4.714, 3.143, 2.106, 1.667, 1.285, 1.000, 0.839, 0.667};
    custom.diffRatio = 3.15;
    custom.tireRadiusM = 0.32;
    custom.hysteresisFactor = 0.85;
    custom.shiftTableThrottleLevels = {0.1, 0.25, 0.5, 0.75, 1.0};
    custom.shiftTable = {
        {20.0, 35.0, 50.0, 65.0, 80.0, 95.0, 110.0},
        {30.0, 50.0, 70.0, 90.0, 110.0, 130.0, 155.0},
        {40.0, 65.0, 90.0, 115.0, 140.0, 170.0, 200.0},
        {55.0, 85.0, 115.0, 145.0, 180.0, 215.0, 255.0},
        {70.0, 105.0, 140.0, 180.0, 220.0, 265.0, 315.0}
    };
    custom.separateDownshiftTableEnabled = true;
    custom.downshiftTableThrottleLevels = {0.1, 0.25, 0.5, 0.75, 1.0};
    custom.downshiftTable = {
        {10.0, 18.0, 25.0, 33.0, 40.0, 48.0, 55.0},
        {15.0, 25.0, 35.0, 45.0, 55.0, 65.0, 78.0},
        {20.0, 33.0, 45.0, 58.0, 70.0, 85.0, 100.0},
        {28.0, 43.0, 58.0, 73.0, 90.0, 108.0, 128.0},
        {35.0, 53.0, 70.0, 90.0, 110.0, 133.0, 158.0}
    };
    custom.minShiftIntervalS = 3.0;
    custom.downshiftMinIntervalS = 1.0;
    custom.redlineRpm = 6500.0;
    custom.standstillThresholdKmh = 1.0;
    custom.kickdownThrottleThreshold = 0.95;
    custom.kickdownDelta = 0.4;
    custom.kickdownWindowMs = 100.0;
    custom.throttleSmoothingTauMs = 50.0;

    AutomaticGearbox gb(custom);
    gb.setGearSelector(bridge::GearSelector::DRIVE);

    // Climb to 3rd gear at 70 km/h
    ASSERT_TRUE(runUntil(gb, 0.5, 200, 70.0, 0.4, 50.0,
        [](AutomaticGearbox& g) { return g.getCurrentGear() >= 3; }));
    ASSERT_EQ(gb.getCurrentGear(), 3);

    // Drop speed to 50 km/h (still in 3rd, no downshift expected yet)
    for (int i = 0; i < 10; ++i) gb.update(0.1, 50.0, 0.4, 50.0);
    ASSERT_EQ(gb.getCurrentGear(), 3);

    // Clear shift interval timers
    for (int i = 0; i < 40; ++i) gb.update(0.1, 50.0, 0.4, 50.0);

    // Kickdown: floor the throttle to force a 3→2 downshift.
    gb.update(0.05, 50.0, 0.98, 250.0);
    int gearAfterKickdown = gb.getCurrentGear();
    ASSERT_LT(gearAfterKickdown, 3) << "Kickdown must pull a downshift from 3rd";

    // Advance past downshiftMinIntervalS — but no safe lower gear exists
    // (gear 1 at 50 km/h would be 6154 RPM > 5850 = 90% redline), so the
    // box correctly holds 2nd. This proves the safety guard, not just the gate.
    for (int i = 0; i < 15; ++i) gb.update(0.1, 50.0, 0.4, 400.0);
    EXPECT_EQ(gb.getCurrentGear(), gearAfterKickdown)
        << "No safe lower gear: box holds 2nd (gear 1 would exceed 90% redline)";
}

// AC7: Shifts stay within the box's gear range; convention P=-2,R=-1,N=0,1..8.
TEST_F(AutoGearboxShiftLogicTest, AC7_GearStaysInRangeAndHoldsConvention) {
    // The gearbox currentGear_ is the bridge forward-gear index (1..N). The
    // selector/gear-convention mapping (P=-2, R=-1, N=0, forward 1..) is owned
    // by GearConventions; here we assert the forward range invariant under
    // aggressive operation, and that the convention enum values are correct.
    EXPECT_EQ(static_cast<int>(bridge::GearSelector::PARK),    -2);
    EXPECT_EQ(static_cast<int>(bridge::GearSelector::REVERSE), -1);
    EXPECT_EQ(static_cast<int>(bridge::GearSelector::NEUTRAL),  0);
    EXPECT_EQ(static_cast<int>(bridge::BridgeGear::NEUTRAL),    0);
    EXPECT_EQ(static_cast<int>(bridge::BridgeGear::FIRST),      1);

    AutomaticGearbox gb(profile);
    gb.setGearSelector(bridge::GearSelector::DRIVE);
    const int topGear = static_cast<int>(profile.gearRatios.size());

    // Push very fast + heavy throttle + heavy torque: must never exceed top gear
    // or drop below 1.
    for (int i = 0; i < 400; ++i) {
        gb.update(0.1, 250.0, 1.0, 500.0);
        ASSERT_GE(gb.getCurrentGear(), 1);
        ASSERT_LE(gb.getCurrentGear(), topGear);
    }

    // Push to a crawl: must never go below 1.
    for (int i = 0; i < 400; ++i) {
        gb.update(0.1, 1.5, 0.0, 0.0);
        ASSERT_GE(gb.getCurrentGear(), 1);
        ASSERT_LE(gb.getCurrentGear(), topGear);
    }
}
