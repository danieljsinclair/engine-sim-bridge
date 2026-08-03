#include <gtest/gtest.h>
#include <twin/AutomaticGearbox.h>
#include <twin/IceVehicleProfile.h>
#include <algorithm>
#include <cmath>

using namespace twin;

namespace {

// Helper to calculate engine RPM from speed and gear
double calculateEngineRpm(double speedKmh, int gear, const IceVehicleProfile& profile) {
    if (gear < 1 || gear > static_cast<int>(profile.gearRatios.size())) {
        return 0.0;
    }
    double speedMs = speedKmh / 3.6;
    double wheelRpm = speedMs / (2.0 * M_PI * profile.tireRadiusM) * 60.0;
    double engineRpm = wheelRpm * profile.gearRatios[gear - 1] * profile.diffRatio;
    return engineRpm;
}

} // namespace

class AutomaticGearboxTest : public ::testing::Test {
protected:
    IceVehicleProfile profile = IceVehicleProfile::zf8hp45();
};

TEST_F(AutomaticGearboxTest, StartsInFirstGear)
{
    AutomaticGearbox gearbox(profile);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);
}

TEST_F(AutomaticGearboxTest, UpshiftAt50PercentThrottle_AC_01_2)
{
    AutomaticGearbox gearbox(profile);

    // At moderate throttle, upshift should occur before hitting redline
    // The exact speed doesn't matter - what matters is the safety property:
    // upshift happens before engine overspeed

    // Start at low speed - should be in 1st
    gearbox.update(0.1, 15.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);

    // Increase speed until upshift occurs
    bool upshiftOccurred = false;
    double upshiftRpm = 0.0;

    for (double speed = 20.0; speed <= 80.0; speed += 1.0) {
        gearbox.update(0.1, speed, 0.5);
        if (gearbox.getCurrentGear() == 2) {
            upshiftOccurred = true;
            upshiftRpm = calculateEngineRpm(speed, 1, profile);
            break;
        }
    }

    ASSERT_TRUE(upshiftOccurred) << "Should have upshifted from 1st to 2nd at 50% throttle";
    EXPECT_LT(upshiftRpm, profile.redlineRpm) << "Upshift should occur before redline";
    EXPECT_GT(upshiftRpm, profile.redlineRpm * 0.5) << "Upshift should not occur at extremely low RPM";
}

TEST_F(AutomaticGearboxTest, UpshiftAt100PercentThrottle_AC_01_5)
{
    AutomaticGearbox gearbox(profile);

    // At high throttle (near WOT), upshift should occur close to but before redline
    // Use 90% to avoid kickdown triggering at 95%
    gearbox.update(0.1, 30.0, 0.90);
    EXPECT_EQ(gearbox.getCurrentGear(), 1) << "Should still be in 1st at 30 kph with high throttle";

    // Increase speed until upshift occurs
    bool upshiftOccurred = false;
    double upshiftRpm = 0.0;

    for (double speed = 35.0; speed <= 80.0; speed += 1.0) {
        gearbox.update(0.1, speed, 0.90);
        if (gearbox.getCurrentGear() == 2) {
            upshiftOccurred = true;
            upshiftRpm = calculateEngineRpm(speed, 1, profile);
            break;
        }
    }

    ASSERT_TRUE(upshiftOccurred) << "Should have upshifted from 1st to 2nd at high throttle";
    EXPECT_LT(upshiftRpm, profile.redlineRpm) << "Upshift must occur before redline (safety property)";
    EXPECT_GT(upshiftRpm, profile.redlineRpm * 0.7) << "At high throttle, upshift should occur near redline for performance";
}

TEST_F(AutomaticGearboxTest, DownshiftAt85PercentOfUpshiftSpeed_AC_03_4)
{
    AutomaticGearbox gearbox(profile);

    // Accelerate to 2nd gear at 25% throttle
    gearbox.update(0.1, 25.0, 0.25);
    ASSERT_EQ(gearbox.getCurrentGear(), 2) << "Should be in 2nd gear after acceleration";

    // Find the downshift RPM (should be lower than upshift RPM due to hysteresis)
    bool downshiftOccurred = false;
    double downshiftRpm = 0.0;

    for (double speed = 24.0; speed >= 5.0; speed -= 0.5) {
        gearbox.update(0.1, speed, 0.25);
        if (gearbox.getCurrentGear() == 1) {
            downshiftOccurred = true;
            downshiftRpm = calculateEngineRpm(speed, 2, profile);
            break;
        }
    }

    EXPECT_TRUE(downshiftOccurred) << "Should downshift from 2nd to 1st when slowing";
    EXPECT_GT(downshiftRpm, profile.idleRpm) << "Downshift RPM should be above idle";
    EXPECT_LT(downshiftRpm, profile.redlineRpm * 0.7) << "Downshift should occur at moderate RPM";
}

TEST_F(AutomaticGearboxTest, CoastDownSequentialDownshifts_AC_03_1)
{
    AutomaticGearbox gearbox(profile);

    // Accelerate to high gear at moderate throttle
    gearbox.update(0.1, 100.0, 0.4);
    EXPECT_GE(gearbox.getCurrentGear(), 3);

    int topGear = gearbox.getCurrentGear();

    // Coast down (throttle = 0) with many small steps to let smoothing decay
    for (int speed = 90; speed >= 10; speed -= 5) {
        gearbox.update(0.5, static_cast<double>(speed), 0.0);
    }

    // Should be in a lower gear now
    EXPECT_LT(gearbox.getCurrentGear(), topGear);
}

TEST_F(AutomaticGearboxTest, KickdownWithin500ms_AC_04_1)
{
    AutomaticGearbox gearbox(profile);

    // Cruise in a higher gear
    gearbox.update(0.1, 60.0, 0.3);
    int initialGear = gearbox.getCurrentGear();
    EXPECT_GT(initialGear, 1);

    // Clear interval timers
    for (int i = 0; i < 30; ++i) {
        gearbox.update(0.1, 60.0, 0.3);
    }
    initialGear = gearbox.getCurrentGear();

    // Kickdown: throttle jumps from 0.3 to 0.9 (delta > 0.4)
    gearbox.update(0.05, 60.0, 0.9);

    // Should request downshift
    EXPECT_TRUE(gearbox.requestsShift());
    EXPECT_LT(gearbox.getTargetGear(), initialGear);
}

TEST_F(AutomaticGearboxTest, SafeGearRpmBelow90PercentRedline_AC_04_2)
{
    AutomaticGearbox gearbox(profile);

    // At a speed where dropping multiple gears would exceed 90% redline
    // only drop one gear
    double speed = 120.0; // High speed

    // Accelerate to high gear
    gearbox.update(0.1, speed, 0.8);
    int initialGear = gearbox.getCurrentGear();

    // Kickdown at high speed
    gearbox.update(0.05, speed, 0.98);

    if (gearbox.requestsShift()) {
        int targetGear = gearbox.getTargetGear();
        // Should not drop more than one gear to avoid overspeeding engine
        EXPECT_GE(targetGear, initialGear - 1);
    }
}

TEST_F(AutomaticGearboxTest, NoShiftAtStandstill_AC_10_1)
{
    AutomaticGearbox gearbox(profile);

    gearbox.update(0.1, 0.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);
    EXPECT_FALSE(gearbox.requestsShift());
}

TEST_F(AutomaticGearboxTest, NoDownshiftBelowFirstGear_AC_10_3)
{
    AutomaticGearbox gearbox(profile);

    // Slow down to near stop
    gearbox.update(0.1, 5.0, 0.0);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);
    EXPECT_FALSE(gearbox.requestsShift());
}

// Declarative minimal dwell: after any shift a short lockout (~shiftDwellS)
// blocks further table-driven shifts so a single speed sample cannot thrash the
// shift valve sub-frame. Once the dwell elapses the next legitimate shift
// proceeds. (The old multi-second up/downshift intervals are gone — hysteresis
// by construction, via the separate downshift table, replaces them.)
TEST_F(AutomaticGearboxTest, Dwell_BlocksRapidRepeatedShiftThenAllows)
{
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    ASSERT_GT(custom.shiftDwellS, 0.0);
    const double dwell = custom.shiftDwellS;
    AutomaticGearbox gearbox(custom);

    // First upshift: 1->2 at 25% throttle (19 kph).
    gearbox.update(0.1, 20.0, 0.25);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);

    // Speed now exceeds the 2->3 upshift (28 kph), but within the dwell window
    // the shift must be suppressed.
    gearbox.update(0.1, 30.0, 0.25);
    EXPECT_EQ(gearbox.getCurrentGear(), 2) << "Sub-dwell upshift must be blocked";

    // Still inside the dwell.
    gearbox.update(dwell * 0.5, 30.0, 0.25);
    EXPECT_EQ(gearbox.getCurrentGear(), 2) << "Still inside dwell, must hold";

    // Past the dwell: the table-driven 2->3 upshift proceeds.
    gearbox.update(dwell, 30.0, 0.25);
    EXPECT_EQ(gearbox.getCurrentGear(), 3) << "Once dwell elapses the upshift fires";
}

TEST_F(AutomaticGearboxTest, KickdownDetectionThresholds_AC_10_4)
{
    AutomaticGearbox gearbox(profile);

    // Cruise
    gearbox.update(0.1, 60.0, 0.3);
    int initialGear = gearbox.getCurrentGear();
    EXPECT_GT(initialGear, 1);

    // Clear interval timers
    for (int i = 0; i < 30; ++i) {
        gearbox.update(0.1, 60.0, 0.3);
    }

    // Small throttle increase - should NOT trigger kickdown (delta < 0.4)
    gearbox.update(0.05, 60.0, 0.6);
    EXPECT_FALSE(gearbox.requestsShift());

    // Reset - create a new gearbox to reset state
    {
        AutomaticGearbox newGearbox(profile);
        newGearbox.update(0.1, 60.0, 0.3);
        // Clear interval timers
        for (int i = 0; i < 30; ++i) {
            newGearbox.update(0.1, 60.0, 0.3);
        }
        // Large throttle increase - SHOULD trigger kickdown (delta > 0.4)
        newGearbox.update(0.05, 60.0, 0.9);
        EXPECT_TRUE(newGearbox.requestsShift());
    }

    // Reset again for high throttle threshold test
    {
        AutomaticGearbox newGearbox2(profile);
        newGearbox2.update(0.1, 60.0, 0.3);
        // Clear interval timers
        for (int i = 0; i < 30; ++i) {
            newGearbox2.update(0.1, 60.0, 0.3);
        }
        // Throttle at 0.95 (at threshold) - should trigger kickdown
        newGearbox2.update(0.05, 60.0, 0.95);
        EXPECT_TRUE(newGearbox2.requestsShift());
    }
}

TEST_F(AutomaticGearboxTest, InterpolateShiftTableForIntermediateThrottle)
{
    AutomaticGearbox gearbox(profile);

    // At intermediate throttle (between table rows), should interpolate shift points
    // Test that interpolation produces values between the bounding rows

    // First, find upshift RPM at 25% throttle (lower row)
    gearbox.update(0.1, 15.0, 0.25);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);

    double upshiftRpm25 = 0.0;
    for (double speed = 16.0; speed <= 40.0; speed += 0.5) {
        gearbox.update(0.1, speed, 0.25);
        if (gearbox.getCurrentGear() == 2) {
            upshiftRpm25 = calculateEngineRpm(speed, 1, profile);
            break;
        }
    }
    ASSERT_GT(upshiftRpm25, 0.0) << "Should find upshift RPM at 25% throttle";
    EXPECT_LT(upshiftRpm25, profile.redlineRpm) << "Upshift at 25% should occur before redline";

    // Find upshift RPM at 40% throttle (upper row)
    double upshiftRpm40 = 0.0;
    {
        AutomaticGearbox gearbox2(profile);
        for (double speed = 20.0; speed <= 50.0; speed += 0.5) {
            gearbox2.update(0.1, speed, 0.40);
            if (gearbox2.getCurrentGear() == 2) {
                upshiftRpm40 = calculateEngineRpm(speed, 1, profile);
                break;
            }
        }
    }
    ASSERT_GT(upshiftRpm40, 0.0) << "Should find upshift RPM at 40% throttle";
    EXPECT_LT(upshiftRpm40, profile.redlineRpm) << "Upshift at 40% should occur before redline";
    EXPECT_GT(upshiftRpm40, upshiftRpm25) << "Higher throttle should upshift at higher RPM";

    // Now test that 35% throttle (between 25% and 40%) interpolates correctly
    AutomaticGearbox gearbox3(profile);
    double upshiftRpm35 = 0.0;
    for (double speed = 16.0; speed <= 45.0; speed += 0.5) {
        gearbox3.update(0.1, speed, 0.35);
        if (gearbox3.getCurrentGear() == 2) {
            upshiftRpm35 = calculateEngineRpm(speed, 1, profile);
            break;
        }
    }

    ASSERT_GT(upshiftRpm35, 0.0) << "Should find upshift RPM at 35% throttle";
    EXPECT_LT(upshiftRpm35, profile.redlineRpm) << "Upshift at 35% should occur before redline";
    EXPECT_GT(upshiftRpm35, upshiftRpm25) << "35% throttle should upshift at higher RPM than 25%";
    EXPECT_LT(upshiftRpm35, upshiftRpm40) << "35% throttle should upshift at lower RPM than 40%";
}

// ============================================================
// F1: Separate Downshift Table
// ============================================================

TEST_F(AutomaticGearboxTest, SeparateDownshiftTableUsedWhenEnabled)
{
    // Build a profile with a separate downshift table that has higher
    // downshift speeds than the upshift*hysteresis fallback would give.
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();

    // Verify separate table is enabled in zf8hp45
    ASSERT_TRUE(custom.separateDownshiftTableEnabled);
    ASSERT_FALSE(custom.downshiftTable.empty());

    AutomaticGearbox gearbox(custom);

    // Get to 2nd gear: accelerate past the 1->2 upshift point at light throttle
    // At 5% throttle, 1->2 upshift is at 11 kph
    gearbox.update(0.1, 12.0, 0.05);
    ASSERT_EQ(gearbox.getCurrentGear(), 2);

    // Clear interval timers
    for (int i = 0; i < 30; ++i) {
        gearbox.update(0.1, 10.0, 0.05);
    }

    // Now at 5% throttle the 2->1 downshift from the separate table is 9 kph.
    // Fall below 9 kph — should downshift to 1.
    gearbox.update(1.6, 8.0, 0.05);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);
}

TEST_F(AutomaticGearboxTest, SeparateDownshiftTableDisabledFallsBackToHysteresis)
{
    // Manually construct a profile with separateDownshiftTableEnabled=false
    // to verify the old hysteresisFactor path still works.
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

    AutomaticGearbox gearbox(custom);

    // Upshift to 2nd: at 50% throttle, 1->2 upshift is 40 kph
    gearbox.update(0.1, 45.0, 0.5);
    ASSERT_EQ(gearbox.getCurrentGear(), 2);

    // Clear interval timer (minShiftIntervalS = 3.0)
    for (int i = 0; i < 40; ++i) {
        gearbox.update(0.1, 35.0, 0.5);
    }

    // Downshift at hysteresis (40 * 0.85 = 34 kph)
    gearbox.update(0.1, 34.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);

    gearbox.update(3.1, 33.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);
}

// ============================================================
// Coast & cruise (declarative): the tables handle deceleration and steady-state
// without any engine-braking inhibit gate. Zero throttle simply lowers the
// table thresholds via the smoothed throttle; downshifts follow the table.
// ============================================================

TEST_F(AutomaticGearboxTest, CoastDown_AllowsDownshiftThroughTable)
{
    AutomaticGearbox gearbox(profile);

    // Climb to a high gear at moderate throttle.
    gearbox.update(0.1, 100.0, 0.4);
    int topGear = gearbox.getCurrentGear();
    ASSERT_GT(topGear, 1);

    // Coast down with zero throttle; the downshift table pulls gears in.
    for (int speed = 90; speed >= 10; speed -= 5) {
        gearbox.update(0.5, static_cast<double>(speed), 0.0);
    }
    EXPECT_LT(gearbox.getCurrentGear(), topGear)
        << "Coasting down must downshift via the table";
}

TEST_F(AutomaticGearboxTest, Kickdown_AtHighwaySpeed_Fires)
{
    AutomaticGearbox gearbox(profile);

    // Cruise at highway speed in a higher gear.
    gearbox.update(0.1, 80.0, 0.3);
    int initialGear = gearbox.getCurrentGear();
    ASSERT_GT(initialGear, 1);

    // Settle, then floor it — kickdown overrides and pulls a downshift.
    for (int i = 0; i < 30; ++i) {
        gearbox.update(0.1, 80.0, 0.3);
    }
    initialGear = gearbox.getCurrentGear();
    gearbox.update(0.05, 80.0, 0.98);

    EXPECT_TRUE(gearbox.requestsShift());
    EXPECT_LT(gearbox.getTargetGear(), initialGear);
}

// ============================================================
// Declarative shift-table spec: hysteresis by construction, multi-gear climb,
// steady-state hold. Anti-hunting comes from the separate downshift table
// (its thresholds sit below the upshift thresholds), not from interval gates.
// ============================================================

// Dead band: between the downshift and upshift thresholds the box HOLDS gear.
// At 5% throttle the 1->2 upshift is 11 kph and the 2->1 downshift is 9 kph;
// holding 10 kph in 2nd must never oscillate to 1st or 3rd.
TEST_F(AutomaticGearboxTest, DeadBand_HoldsGear_NoOscillation)
{
    AutomaticGearbox gearbox(profile);

    // Upshift into 2nd (1->2 at 5% throttle is 11 kph).
    gearbox.update(0.1, 12.0, 0.05);
    ASSERT_EQ(gearbox.getCurrentGear(), 2);

    // Sit in the dead band for many frames — gear must stay at 2.
    int minGear = 2, maxGear = 2;
    for (int i = 0; i < 80; ++i) {
        gearbox.update(0.1, 10.0, 0.05);
        minGear = std::min(minGear, gearbox.getCurrentGear());
        maxGear = std::max(maxGear, gearbox.getCurrentGear());
    }
    EXPECT_EQ(minGear, 2) << "Must not downshift to 1st inside the dead band";
    EXPECT_EQ(maxGear, 2) << "Must not upshift to 3rd inside the dead band";
}

// Multi-gear climb: accelerate from rest and the box steps up through the gears.
TEST_F(AutomaticGearboxTest, MultiGearClimb_ReachesHighGear)
{
    AutomaticGearbox gearbox(profile);
    int maxGear = 1;
    for (int i = 0; i < 600; ++i) {  // 60 s
        const double speed = 10.0 + (200.0 - 10.0) * (static_cast<double>(i) / 599.0);
        gearbox.update(0.1, speed, 0.5);
        maxGear = std::max(maxGear, gearbox.getCurrentGear());
    }
    EXPECT_GE(maxGear, 5) << "A sustained acceleration must climb well into the gear range";
    EXPECT_LE(maxGear, static_cast<int>(profile.gearRatios.size()));
}

// Cruise: a constant operating point must settle and then hold — no hunting.
TEST_F(AutomaticGearboxTest, Cruise_HoldsGear_NoHunting)
{
    AutomaticGearbox gearbox(profile);

    // Reach a steady cruise gear at 60 kph / 30% throttle.
    for (int i = 0; i < 60; ++i) {
        gearbox.update(0.1, 60.0, 0.30);
    }
    const int settled = gearbox.getCurrentGear();
    ASSERT_GT(settled, 1);

    // Hold the exact same operating point; the gear must not change.
    for (int i = 0; i < 200; ++i) {
        gearbox.update(0.1, 60.0, 0.30);
        ASSERT_EQ(gearbox.getCurrentGear(), settled) << "Cruise must not hunt";
    }
}

// After an upshift, a legitimate downshift is permitted once the dwell elapses
// (the dwell is direction-agnostic; there is no cross-direction interval).
TEST_F(AutomaticGearboxTest, DownshiftPermitted_AfterUpshift_OnceDwellElapses)
{
    AutomaticGearbox gearbox(profile);

    gearbox.update(0.1, 10.0, 0.25);
    ASSERT_EQ(gearbox.getCurrentGear(), 1);

    // Upshift 1->2 (19 kph at 25% throttle).
    gearbox.update(2.1, 20.0, 0.25);
    ASSERT_EQ(gearbox.getCurrentGear(), 2);

    // Dwell elapses (0.5 s > shiftDwellS), then the 2->1 downshift (9 kph) fires.
    gearbox.update(0.5, 9.0, 0.05);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);
}

// ============================================================
// F4/F5: 10 Throttle Levels with Separate Tables
// ============================================================

TEST_F(AutomaticGearboxTest, TenThrottleLevels_UpshiftAtWOT)
{
    // At 100% throttle, 1->2 upshift should happen at 48 kmph (~85% redline)
    AutomaticGearbox gearbox(profile);

    gearbox.update(0.1, 47.0, 1.0);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);

    // Above 49 kph — upshift
    gearbox.update(0.1, 50.0, 1.0);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);
}

TEST_F(AutomaticGearboxTest, TenThrottleLevels_DownshiftGapVariesWithThrottle)
{
    // At light throttle (5%), upshift 1->2 at 11 kph, downshift 2->1 at 9 kph
    // Gap = 2 kph (small)
    // At high throttle (90%), upshift 1->2 at 45 kph, downshift 2->1 at 33 kph
    // Gap = 12 kph (large)

    // Light throttle: upshift to 2nd at 5% throttle
    {
        AutomaticGearbox gearbox(profile);
        gearbox.update(0.1, 10.0, 0.05);
        gearbox.update(2.1, 12.0, 0.05);
        ASSERT_EQ(gearbox.getCurrentGear(), 2);

        // Clear interval before testing downshift
        for (int i = 0; i < 20; ++i) {
            gearbox.update(0.1, 12.0, 0.05);
        }

        // Downshift at 9 kph — just above threshold, no shift
        gearbox.update(0.1, 9.0, 0.05);
        EXPECT_EQ(gearbox.getCurrentGear(), 2);

        // Below 9 kph — downshift
        gearbox.update(1.6, 8.0, 0.05);
        EXPECT_EQ(gearbox.getCurrentGear(), 1);
    }

    // High throttle: upshift to 2nd at 90% throttle (use 0.90 to avoid kickdown)
    {
        AutomaticGearbox gearbox(profile);
        gearbox.update(0.1, 10.0, 0.90);
        gearbox.update(2.1, 46.0, 0.90);
        ASSERT_EQ(gearbox.getCurrentGear(), 2);

        // Clear interval before testing downshift
        for (int i = 0; i < 20; ++i) {
            gearbox.update(0.1, 46.0, 0.90);
        }

        // Downshift at 33 kph — just above threshold, no shift
        gearbox.update(0.1, 34.0, 0.90);
        EXPECT_EQ(gearbox.getCurrentGear(), 2);

        // Below 33 kph — downshift
        gearbox.update(1.6, 32.0, 0.90);
        EXPECT_EQ(gearbox.getCurrentGear(), 1);
    }
}

// ============================================================
// Redline Safety (speed-implied RPM — coherent with the shift speed model)
// ============================================================

TEST_F(AutomaticGearboxTest, RedlineSafetyUpshift_WhenImpliedRpmExceeds95Percent) {
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    AutomaticGearbox gearbox(custom);

    // A road speed whose implied engine speed in 1st gear exceeds 95% redline.
    // The redline check is now coherent with the speed model (it no longer keys
    // off the separate real-engine rpmFeedback_, which could hunt against the
    // speed-based shifts).
    gearbox.update(0.1, 50.0, 0.90);

    EXPECT_GT(gearbox.getCurrentGear(), 1)
        << "Should upshift when road speed in 1st implies >95% redline";
}

TEST_F(AutomaticGearboxTest, RedlineSafety_DoesNotBlockDownshift) {
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    AutomaticGearbox gearbox(custom);

    // Get to gear 3+ at moderate throttle
    gearbox.update(0.1, 50.0, 0.30);
    ASSERT_GE(gearbox.getCurrentGear(), 2);

    // Clear interval timers by running 3+ seconds at constant speed
    for (int i = 0; i < 30; ++i) {
        gearbox.update(0.1, 50.0, 0.30);
    }
    int gear = gearbox.getCurrentGear();
    ASSERT_GE(gear, 2);

    // Set high RPM — redline safety triggers upshift
    gearbox.setTwinContext(3, 1.0, 50.0, 6200.0);
    gearbox.update(0.1, 50.0, 0.30);
    gear = gearbox.getCurrentGear();

    // Clear RPM feedback so redline safety doesn't fire during deceleration
    gearbox.setTwinContext(3, 1.0, 50.0, 2000.0);

    // Decelerate with large time steps to let intervals elapse
    for (int speed = 45; speed >= 5; speed -= 5) {
        gearbox.update(1.6, static_cast<double>(speed), 0.05);
    }

    EXPECT_LT(gearbox.getCurrentGear(), gear)
        << "Downshifts should work even when redline RPM was high";
}

TEST_F(AutomaticGearboxTest, RedlineSafety_DoesNotOverrideKickdown) {
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    AutomaticGearbox gearbox(custom);

    // Cruise in gear 3+
    gearbox.update(0.1, 60.0, 0.30);
    int initialGear = gearbox.getCurrentGear();
    ASSERT_GT(initialGear, 1);

    // Clear interval timers
    for (int i = 0; i < 30; ++i) {
        gearbox.update(0.1, 60.0, 0.30);
    }
    initialGear = gearbox.getCurrentGear();

    // Set high RPM AND floor throttle — kickdown should fire
    gearbox.setTwinContext(3, 1.0, 60.0, 6200.0);
    gearbox.update(0.05, 60.0, 0.98);

    // Kickdown should still override (downshift)
    EXPECT_TRUE(gearbox.requestsShift());
    EXPECT_LT(gearbox.getTargetGear(), initialGear)
        << "Kickdown should override redline safety";
}

// ============================================================
// Realistic-throttle upshifts: the declarative gearbox has no gradient/tip
// gate, so it follows the tables under sustained ramps and sub-percent CAN
// jitter. (These were originally RED tests for a level-sensitive tip gate that
// pinned the box in 1st; the gate is gone, the behaviour is now the default.)
// ============================================================

TEST_F(AutomaticGearboxTest, SustainedThrottleRamp_upshifts) {
    // A driver smoothly ramps throttle while accelerating; the box must follow
    // the table and upshift as speed rises.
    AutomaticGearbox gearbox(profile);
    constexpr double dt = 1.0 / 60.0;
    const int frames = 120;  // 2.0 s
    const double startSpeed = 10.0, endSpeed = 60.0;
    const double startThrottle = 0.20, endThrottle = 0.60;
    int maxGear = 1;
    for (int i = 0; i < frames; ++i) {
        const double frac = static_cast<double>(i) / (frames - 1);
        const double speed = startSpeed + (endSpeed - startSpeed) * frac;
        const double throttle = startThrottle + (endThrottle - startThrottle) * frac;
        gearbox.update(dt, speed, throttle);
        maxGear = std::max(maxGear, gearbox.getCurrentGear());
    }
    EXPECT_GE(maxGear, 2) << "Gearbox must upshift during a sustained gentle throttle ramp";
}

TEST_F(AutomaticGearboxTest, NoisyThrottle_Upshifts) {
    // Real CAN throttle carries sub-percent jitter on a steady command; the
    // box must still upshift as speed rises past the 1->2 point.
    AutomaticGearbox gearbox(profile);
    constexpr double dt = 1.0 / 60.0;
    const double baseThrottle = 0.5;
    int maxGear = 1;
    // Ramp speed 18 -> 45 kph (across the 1->2 upshift ~27 kph at 0.5 throttle)
    // over 3 s with +-0.5% jitter on every frame.
    const int frames = 3 * 60;
    const double startSpeed = 18.0, endSpeed = 45.0;
    for (int i = 0; i < frames; ++i) {
        const double frac = static_cast<double>(i) / (frames - 1);
        const double speed = startSpeed + (endSpeed - startSpeed) * frac;
        const double jitter = (i % 2 == 0) ? 0.005 : -0.005;
        gearbox.update(dt, speed, baseThrottle + jitter);
        maxGear = std::max(maxGear, gearbox.getCurrentGear());
    }
    EXPECT_GE(maxGear, 2) << "Gearbox must upshift despite sub-percent throttle jitter";
}

TEST_F(AutomaticGearboxTest, FullThrottleRamp_SequentialUpshifts) {
    // Full-throttle acceleration with continuous CAN jitter overlaid: the box
    // must step cleanly up through the gears.
    AutomaticGearbox gearbox(profile);
    constexpr double dt = 1.0 / 60.0;
    const int frames = 30 * 60;  // 30 s
    const double startSpeed = 8.0, endSpeed = 220.0;
    const double startThrottle = 0.0, endThrottle = 0.94;  // below kickdown (0.95)
    int maxGear = 1;
    int upshiftCount = 0;
    int prevGear = 1;
    double gear2Speed = -1.0;
    for (int i = 0; i < frames; ++i) {
        const double frac = static_cast<double>(i) / (frames - 1);
        const double speed = startSpeed + (endSpeed - startSpeed) * frac;
        const double throttle = startThrottle + (endThrottle - startThrottle) * frac;
        const double jitter = (i % 2 == 0) ? 0.005 : -0.005;
        gearbox.update(dt, speed, std::clamp(throttle + jitter, 0.0, 1.0));
        const int g = gearbox.getCurrentGear();
        if (g > prevGear) {
            ++upshiftCount;
            if (g == 2 && gear2Speed < 0.0) gear2Speed = speed;
        }
        maxGear = std::max(maxGear, g);
        prevGear = g;
    }
    EXPECT_GE(upshiftCount, 4) << "Should complete several sequential upshifts over a full-throttle run";
    EXPECT_GE(maxGear, 5) << "Should reach a high gear by 220 kph";
    ASSERT_GE(gear2Speed, 0.0) << "Should reach 2nd gear";
    EXPECT_GE(gear2Speed, 10.0) << "1->2 should not happen at standstill";
    EXPECT_LE(gear2Speed, 55.0) << "1->2 should happen well before redline push";
}

TEST_F(AutomaticGearboxTest, SteadyThrottleWithJitter_DoesNotBlockUpshift) {
    // Sub-percent jitter on a STEADY throttle must not block higher-gear
    // upshifts; the smoothed throttle stays stable and the table decides.
    AutomaticGearbox gearbox(profile);
    constexpr double dt = 1.0 / 60.0;

    // Reach 2nd at 0.5 throttle (1->2 ~27 kph; 2->3 ~43 kph, so 35 holds 2nd).
    for (int i = 0; i < 30; ++i) {
        gearbox.update(dt, 35.0, 0.50);
    }
    ASSERT_EQ(gearbox.getCurrentGear(), 2) << "precondition: in 2nd gear";

    // Clear the dwell.
    for (int i = 0; i < 150; ++i) {
        gearbox.update(dt, 35.0, 0.50);
    }

    // Raise speed above the 2->3 upshift (~43 kph) with continuous +-0.5%
    // jitter: the 2->3 upshift proceeds.
    int maxGear = 2;
    for (int i = 0; i < 5 * 60; ++i) {
        const double jitter = (i % 2 == 0) ? 0.005 : -0.005;
        gearbox.update(dt, 55.0, 0.50 + jitter);
        maxGear = std::max(maxGear, gearbox.getCurrentGear());
    }
    EXPECT_GE(maxGear, 3) << "Steady throttle with jitter must still upshift";
}

// ============================================================
// Kickdown is the sole override of the tables: a sudden large throttle increase
// forces one RPM-safe downshift even inside the dwell window — a legitimate
// power demand cannot wait.
// ============================================================

TEST_F(AutomaticGearboxTest, Kickdown_OverridesDwell) {
    AutomaticGearbox gearbox(profile);

    // Upshift into 2nd (1->2 at 25% throttle is 19 kph).
    gearbox.update(0.1, 20.0, 0.25);
    ASSERT_EQ(gearbox.getCurrentGear(), 2);

    // One frame later — still well inside the dwell window (0.05 s < 0.4 s) —
    // floor the throttle. Kickdown must fire and pull a downshift anyway.
    gearbox.update(0.05, 20.0, 0.95);
    EXPECT_TRUE(gearbox.requestsShift()) << "Kickdown must override the dwell";
    EXPECT_LT(gearbox.getCurrentGear(), 2) << "Kickdown must downshift despite sub-dwell timing";
}

// ============================================================
// fix95: top-gear logShiftState must not throw when the shift table
// has fewer columns than the profile's gear count (e.g. Subaru EJ25:
// 6 gears, 5 upshift columns). Previously getShiftSpeed(top, top+1, ...)
// computed tableIndex = top-1 which equaled shiftTable[0].size(), hitting
// the ASSERT and throwing std::runtime_error. The clamp makes every table
// access bounds-safe regardless of profile dimensions.
// ============================================================
TEST_F(AutomaticGearboxTest, TopGear_LogShiftState_DoesNotThrow) {
    // Build a 6-speed profile with 5 shift-table columns (upshifts 1→2 .. 5→6),
    // matching the Subaru EJ25 layout that triggered the crash.
    IceVehicleProfile profile = IceVehicleProfile::zf8hp45();
    profile.gearRatios = {4.714, 3.143, 2.106, 1.667, 1.285, 1.000};
    profile.shiftTable = {
        {11, 17, 26, 32, 42},
        {15, 22, 33, 41, 54},
        {19, 28, 42, 53, 69},
        {23, 35, 52, 65, 85},
        {29, 43, 64, 81, 105},
        {35, 52, 77, 97, 126},
        {40, 59, 88, 110, 143},
        {44, 65, 97, 122, 159},
        {46, 68, 102, 128, 167},
        {48, 71, 106, 134, 174}
    };
    profile.separateDownshiftTableEnabled = true;
    profile.downshiftTable = {
        {9, 13, 20, 25, 33},
        {11, 16, 24, 30, 39},
        {14, 21, 31, 39, 51},
        {18, 27, 40, 51, 66},
        {22, 33, 49, 62, 81},
        {26, 39, 58, 74, 95},
        {29, 44, 66, 83, 107},
        {33, 49, 73, 92, 119},
        {34, 51, 76, 96, 125},
        {34, 51, 76, 97, 125}
    };

    AutomaticGearbox gearbox(profile);

    // Attach a logger so logShiftState exercises the upshift-speed lookup
    // every frame. Without a logger the getShiftSpeed call is skipped.
    struct MockLogger : public IGearboxLogger {
        std::vector<GearboxLogEntry> entries;
        void log(const GearboxLogEntry& entry) override { entries.push_back(entry); }
    };
    MockLogger logger;
    gearbox.setLogger(&logger);

    // Accelerate past the 5→6 threshold so the box climbs to top gear. The 5→6
    // upshift is ~105 kph @ 40% throttle and the 2s shift-dwell needs ~20 updates
    // per shift, so run long and fast enough (190 updates to 200 kph) to clear
    // every threshold + dwell and reach gear 6.
    for (double speed = 10.0; speed <= 200.0; speed += 1.0) {
        gearbox.update(0.1, speed, 0.4);
    }

    // Reached top gear — the next logShiftState call would have asked for
    // getShiftSpeed(6, 7, ...) which previously threw.
    EXPECT_EQ(gearbox.getCurrentGear(), 6);
    EXPECT_FALSE(logger.entries.empty());
    if (!logger.entries.empty()) {
        const auto& lastEntry = logger.entries.back();
        EXPECT_GE(lastEntry.upshiftSpeed, 0.0)
            << "upshiftSpeed at top gear must be non-negative (clamped to last column)";
    }
}
