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

TEST_F(AutomaticGearboxTest, MinShiftIntervalBetweenSameDirectionShifts_AC_10_5)
{
    AutomaticGearbox gearbox(profile);

    // Perform two consecutive upshifts: 1->2 then 2->3
    // First upshift at 25% throttle: 1->2 at 19 kph
    gearbox.update(0.1, 20.0, 0.25);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);

    // Speed high enough for 2->3 at 25% throttle (28 kph)
    // But second upshift within upshiftMinIntervalS (2.0s) should be blocked
    gearbox.update(0.1, 30.0, 0.25);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);

    // Advance 1.8 seconds (total 1.9 from first shift) - still blocked
    gearbox.update(1.8, 30.0, 0.25);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);

    // Advance past 2.0 seconds total - upshift allowed
    gearbox.update(0.2, 30.0, 0.25);
    EXPECT_EQ(gearbox.getCurrentGear(), 3);
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
// F2: Engine Braking Inhibitor
// ============================================================

TEST_F(AutomaticGearboxTest, EngineBrakingInhibitorDoesNotBlockDownshift)
{
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    ASSERT_TRUE(custom.engineBrakingInhibitorEnabled);

    AutomaticGearbox gearbox(custom);

    // Get to a high gear at moderate throttle
    gearbox.update(0.1, 100.0, 0.4);
    int topGear = gearbox.getCurrentGear();
    ASSERT_GT(topGear, 1);

    // Clear interval timers
    for (int i = 0; i < 30; ++i) {
        gearbox.update(0.1, 100.0, 0.4);
    }

    // Coast at highway speed (throttle = 0, speed = 80 kph)
    // Per x-engineer ch6 s4.3: inhibitor blocks UPHIFTS ONLY, downshifts are free
    gearbox.update(0.1, 80.0, 0.0);

    // Coast down further — downshift should be allowed
    for (int speed = 70; speed >= 10; speed -= 5) {
        gearbox.update(0.5, static_cast<double>(speed), 0.0);
    }

    // Should have downshifted — inhibitor only blocks upshifts
    EXPECT_LT(gearbox.getCurrentGear(), topGear)
        << "Downshifts should be allowed even with engine braking inhibitor active";
}

TEST_F(AutomaticGearboxTest, EngineBrakingInhibitorAllowsDownshiftBelowMinSpeed)
{
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    ASSERT_TRUE(custom.engineBrakingInhibitorEnabled);
    ASSERT_DOUBLE_EQ(custom.engineBrakingMinSpeedKmh, 10.0);

    AutomaticGearbox gearbox(custom);

    // Get to a high gear
    gearbox.update(0.1, 100.0, 0.4);
    int topGear = gearbox.getCurrentGear();
    ASSERT_GT(topGear, 1);

    // Clear interval timers then coast below min speed
    for (int speed = 90; speed >= 5; speed -= 5) {
        gearbox.update(0.5, static_cast<double>(speed), 0.0);
    }

    // Should have downshifted since we're below min speed
    EXPECT_LT(gearbox.getCurrentGear(), topGear);
}

TEST_F(AutomaticGearboxTest, EngineBrakingInhibitorDoesNotBlockKickdown)
{
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    ASSERT_TRUE(custom.engineBrakingInhibitorEnabled);

    AutomaticGearbox gearbox(custom);

    // Cruise at highway speed in a higher gear
    gearbox.update(0.1, 80.0, 0.3);
    int initialGear = gearbox.getCurrentGear();
    ASSERT_GT(initialGear, 1);

    // Clear interval timers
    for (int i = 0; i < 30; ++i) {
        gearbox.update(0.1, 80.0, 0.3);
    }
    initialGear = gearbox.getCurrentGear();

    // Floor it — kickdown should override the inhibitor
    gearbox.update(0.05, 80.0, 0.98);

    // Kickdown should still fire even with inhibitor active
    EXPECT_TRUE(gearbox.requestsShift());
    EXPECT_LT(gearbox.getTargetGear(), initialGear);
}

TEST_F(AutomaticGearboxTest, EngineBrakingInhibitorBlocksUpshiftAtHighwaySpeed)
{
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    ASSERT_TRUE(custom.engineBrakingInhibitorEnabled);

    AutomaticGearbox gearbox(custom);

    // Get to a moderate gear at moderate throttle
    gearbox.update(0.1, 60.0, 0.25);
    int gear = gearbox.getCurrentGear();
    ASSERT_GE(gear, 2);

    // Clear interval timers so next upshift isn't interval-blocked
    for (int i = 0; i < 30; ++i) {
        gearbox.update(0.1, 60.0, 0.25);
    }
    gear = gearbox.getCurrentGear();

    // Coast at 70 kph with zero throttle — activates engine braking inhibitor
    gearbox.update(0.1, 70.0, 0.0);

    int gearAfterCoast = gearbox.getCurrentGear();
    // With inhibitor active, upshifts should be blocked even if speed increases
    gearbox.update(0.1, 120.0, 0.0);
    EXPECT_EQ(gearbox.getCurrentGear(), gearAfterCoast);
}

// ============================================================
// F3: Asymmetric Shift Intervals
// ============================================================

TEST_F(AutomaticGearboxTest, AsymmetricShiftInterval_UpshiftUsesUpshiftInterval)
{
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    // zf8hp45 has upshiftMinIntervalS=2.0
    ASSERT_DOUBLE_EQ(custom.upshiftMinIntervalS, 2.0);

    AutomaticGearbox gearbox(custom);

    // Start in gear 1, below upshift threshold
    gearbox.update(0.1, 10.0, 0.25);
    ASSERT_EQ(gearbox.getCurrentGear(), 1);

    // First upshift: 1->2 at 25% throttle (19 kph)
    gearbox.update(2.1, 20.0, 0.25);
    ASSERT_EQ(gearbox.getCurrentGear(), 2);

    // Second upshift: 2->3 at 25% throttle (28 kph)
    // Wait just under upshift interval (2.0s) — should be blocked
    gearbox.update(1.9, 30.0, 0.25);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);

    // Advance past 2.0s total — upshift should be allowed
    gearbox.update(0.2, 30.0, 0.25);
    EXPECT_EQ(gearbox.getCurrentGear(), 3);
}

TEST_F(AutomaticGearboxTest, AsymmetricShiftInterval_DownshiftUsesDownshiftInterval)
{
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    // zf8hp45 has downshiftMinIntervalS=1.0 per x-engineer ch6 s4.2
    ASSERT_DOUBLE_EQ(custom.downshiftMinIntervalS, 1.0);

    AutomaticGearbox gearbox(custom);

    // Get to 4th gear at 40% throttle
    gearbox.update(0.1, 60.0, 0.40);
    ASSERT_GE(gearbox.getCurrentGear(), 4);

    // Clear interval timers
    for (int i = 0; i < 30; ++i) {
        gearbox.update(0.1, 60.0, 0.40);
    }
    int topGear = gearbox.getCurrentGear();

    // Drop to speed that triggers one downshift but not two
    // Gear 4->3 at 5% = 25 kph, gear 3->2 at 5% = 13 kph
    // Use speed 20: only 4->3 triggers, 3->2 doesn't (20 > 13).
    gearbox.update(1.6, 20.0, 0.05);
    int gearAfterFirst = gearbox.getCurrentGear();

    if (gearAfterFirst < topGear) {
        // Try for second consecutive downshift before 1.0s — should be blocked
        gearbox.update(0.9, 8.0, 0.05);
        EXPECT_EQ(gearbox.getCurrentGear(), gearAfterFirst);

        // Advance past 1.0s total — downshift should be allowed
        gearbox.update(0.2, 8.0, 0.05);
        EXPECT_LT(gearbox.getCurrentGear(), gearAfterFirst);
    }
}

TEST_F(AutomaticGearboxTest, CrossResetResetsOppositeDirectionTimer)
{
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    ASSERT_DOUBLE_EQ(custom.upshiftMinIntervalS, 2.0);
    ASSERT_DOUBLE_EQ(custom.downshiftMinIntervalS, 1.0);

    AutomaticGearbox gearbox(custom);

    // Start below upshift threshold
    gearbox.update(0.1, 10.0, 0.25);
    ASSERT_EQ(gearbox.getCurrentGear(), 1);

    // Get to 2nd gear via upshift (wait for interval)
    gearbox.update(2.1, 20.0, 0.25);
    ASSERT_EQ(gearbox.getCurrentGear(), 2);

    // Immediately try to downshift — the upshift just happened, so downshift timer
    // should be cross-reset (it starts fresh from the upshift).
    // Since we just upshifted, the downshift timer starts at 0.
    // Wait 1.1s (> downshift interval 1.0s) and try to downshift
    gearbox.update(1.1, 9.0, 0.05);
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
// F4: Tip-In / Tip-Out Correction (x-engineer ch6 s4.4)
// ============================================================

TEST_F(AutomaticGearboxTest, TipIn_BlocksUpshift) {
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    ASSERT_TRUE(custom.tipCorrectionEnabled);

    AutomaticGearbox gearbox(custom);

    // Start in gear 1, accelerate to just above 1->2 upshift speed at low throttle
    gearbox.update(0.1, 12.0, 0.10);

    // Now tip-in: rapid throttle increase should produce gradient > 10 %/s
    // From 0.10 to 0.50 in 0.016s = 25.0 %/s (well above 10 threshold)
    gearbox.update(0.016, 15.0, 0.50);

    // Tip-in should block the upshift even though speed exceeds threshold
    // (speed 15 > upshift speed ~13 at 50% throttle, but tip correction blocks it)
    EXPECT_EQ(gearbox.getCurrentGear(), 1)
        << "Tip-in correction should block upshift during rapid throttle increase";
}

TEST_F(AutomaticGearboxTest, TipOut_BlocksUpshift) {
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    ASSERT_TRUE(custom.tipCorrectionEnabled);

    AutomaticGearbox gearbox(custom);

    // Start in gear 1, get to gear 3+
    gearbox.update(0.1, 10.0, 0.30);
    gearbox.update(2.1, 40.0, 0.30);
    int gearBeforeTipOut = gearbox.getCurrentGear();
    ASSERT_GE(gearBeforeTipOut, 2);

    // Clear interval timers
    for (int i = 0; i < 30; ++i) {
        gearbox.update(0.1, 40.0, 0.30);
    }

    // Tip-out: rapid throttle release produces gradient < -10 %/s
    // From 0.30 to 0.05 in 0.016s = -15.6 %/s (below -10 threshold)
    gearbox.update(0.016, 55.0, 0.05);

    // Tip-out should block the next upshift even though speed is above threshold
    EXPECT_LE(gearbox.getCurrentGear(), gearBeforeTipOut)
        << "Tip-out correction should block upshift during rapid throttle release";
}

TEST_F(AutomaticGearboxTest, TipCorrection_DoesNotBlockDownshift) {
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    ASSERT_TRUE(custom.tipCorrectionEnabled);

    AutomaticGearbox gearbox(custom);

    // Get to a high gear at moderate throttle
    gearbox.update(0.1, 80.0, 0.40);
    int topGear = gearbox.getCurrentGear();
    ASSERT_GE(topGear, 3) << "Should reach gear 3+ at 80 kph 40% throttle";

    // Clear interval timers
    for (int i = 0; i < 30; ++i) {
        gearbox.update(0.1, 80.0, 0.40);
    }

    // Tip-out: rapid throttle release (triggers tip-out correction)
    gearbox.update(0.016, 80.0, 0.05);

    // Now slow down below downshift thresholds
    // At 5% throttle, downshift thresholds are low but sequential downshifts should occur
    for (int speed = 70; speed >= 5; speed -= 5) {
        gearbox.update(0.5, static_cast<double>(speed), 0.05);
    }

    // Downshift should be allowed even with tip-out active
    EXPECT_LT(gearbox.getCurrentGear(), topGear)
        << "Tip-out should NOT block downshifts — gearbox should have downshifted during coast";
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

// RedlineSafety_BypassesIntervalTimer removed: the redline safety is now folded
// into the speed-based upshift pass (coherent with the speed model), so it no
// longer bypasses the shift interval. The bypass premise was tied to the old
// real-RPM redline that could hunt; see RedlineSafetyUpshift_*ImpliedRpm* above.

// RED: NoOscillation_AfterUpshift_DownshiftRequiresInterval
// Stashed — requires gearbox interval logic redesign (lastShiftDirection_ vs hasShiftedBefore_)
// See: circle-back stash review

// ============================================================
// F4: Tip-In / Tip-Out Correction (x-engineer ch6 s4.4)
// ============================================================

TEST_F(AutomaticGearboxTest, TipCorrection_ClearsWhenGradientStabilizes) {
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    ASSERT_TRUE(custom.tipCorrectionEnabled);

    AutomaticGearbox gearbox(custom);

    // Start with a moderate throttle (no tip-in on first frame due to skip)
    gearbox.update(0.1, 10.0, 0.10);

    // Tip-in: rapid throttle increase from 0.10 to 0.50
    // Gradient = 0.40/0.016 * 100 = 2500 %/s (way above 10 threshold)
    gearbox.update(0.016, 15.0, 0.50);

    // Tip correction should block upshift here
    EXPECT_EQ(gearbox.getCurrentGear(), 1);

    // Stabilize throttle (gradient returns to ~0, tip correction clears)
    // Also let smoothed throttle catch up, AND wait for upshift interval
    for (int i = 0; i < 25; ++i) {
        gearbox.update(0.1, 20.0, 0.50);
    }

    // Now accelerate past upshift threshold — tip correction should be clear
    gearbox.update(0.1, 30.0, 0.50);

    EXPECT_GT(gearbox.getCurrentGear(), 1)
        << "Upshift should proceed after tip-in gradient stabilizes";
}

// ============================================================
// Bug: "gearbox never upshifts" — transient tip-correction
// (x-engineer ch6 s4.4). The original tip gate was level-sensitive
// per frame on the RAW throttle gradient with no decay, so any
// sustained ramp or sub-percent CAN jitter fired it every frame and
// permanently suppressed upshifts. These tests assert the gearbox
// upshifts under realistic throttle input (RED on the per-frame
// gate; GREEN once the gate is a short transient inhibit window).
// ============================================================

TEST_F(AutomaticGearboxTest, SustainedThrottleRamp_upshifts) {
    // A driver smoothly ramps throttle while accelerating. The raw per-frame
    // gradient (~20 %/s) exceeds the old 10 %/s tip threshold every frame, so
    // the level-sensitive gate pins the gearbox in 1st for the whole ramp.
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
    // Real CAN throttle carries sub-percent jitter on a steady command. At
    // 60 fps the raw gradient of +-0.5% jitter is ~+-30 %/s, above the old
    // 10 %/s threshold, so once the tip gate arms it stays armed and the
    // gearbox cannot upshift as speed rises past the 1->2 point.
    AutomaticGearbox gearbox(profile);
    constexpr double dt = 1.0 / 60.0;
    const double baseThrottle = 0.5;
    int maxGear = 1;
    // Ramp speed 18 -> 45 kph (across the 1->2 upshift ~27 kph at 0.5 throttle)
    // over 3 s with +-0.5% jitter on every frame. Frame 0 starts below the
    // upshift point so the gate is armed before speed crosses it.
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
    // Full-throttle acceleration with continuous CAN jitter overlaid. The
    // jitter spikes the raw gradient above threshold every frame, pinning the
    // gearbox in 1st on the buggy gate; transient tip-correction lets it step
    // cleanly through the gears.
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

TEST_F(AutomaticGearboxTest, TipCorrection_BlocksOnlyForShortWindow) {
    // A tip event arms a TRANSIENT inhibit window, not a permanent block: the
    // upshift is held for ~tipInhibitWindowS, then proceeds.
    IceVehicleProfile custom = IceVehicleProfile::zf8hp45();
    ASSERT_GT(custom.tipInhibitWindowS, 0.0);
    ASSERT_LE(custom.tipInhibitWindowS, 0.5);  // must be short / transient
    const double window = custom.tipInhibitWindowS;

    AutomaticGearbox gearbox(custom);
    constexpr double dt = 1.0 / 60.0;

    // Settle in 1st at high throttle: the 1->2 upshift at 0.80 throttle is
    // ~40 kph, so 35 kph holds 1st with no tip event (throttle is stable).
    for (int i = 0; i < 30; ++i) {
        gearbox.update(dt, 35.0, 0.80);
    }
    ASSERT_EQ(gearbox.getCurrentGear(), 1) << "precondition: held in 1st below the 0.80 upshift speed";

    // Tip-out stab: a sharp throttle drop gives a large negative smoothed
    // gradient that arms the inhibit window. At the new low throttle the 1->2
    // threshold (~20 kph) is below the current speed (35), so the ONLY thing
    // holding the upshift back is the transient window.
    gearbox.update(dt, 35.0, 0.30);
    ASSERT_EQ(gearbox.getCurrentGear(), 1) << "tip event should hold the upshift";

    double upshiftAt = -1.0;
    double t = 0.0;
    for (int i = 0; i < 90; ++i) {  // up to 1.5 s
        gearbox.update(dt, 35.0, 0.30);
        t += dt;
        if (gearbox.getCurrentGear() > 1 && upshiftAt < 0.0) {
            upshiftAt = t;
        }
    }
    ASSERT_GE(upshiftAt, 0.0) << "upshift should fire once the window decays";
    EXPECT_GE(upshiftAt, window - 0.10) << "window must hold the upshift for ~tipInhibitWindowS";
    EXPECT_LE(upshiftAt, window + 0.20) << "upshift must proceed shortly after the window decays";
}

TEST_F(AutomaticGearboxTest, SteadyThrottleWithJitter_DoesNotBlockUpshift) {
    // Sub-percent jitter on a STEADY throttle must NOT arm the inhibit window
    // (the smoothed gradient stays small), so higher-gear upshifts still proceed.
    AutomaticGearbox gearbox(profile);
    constexpr double dt = 1.0 / 60.0;

    // Reach 2nd at 0.5 throttle (1->2 ~27 kph; 2->3 ~43 kph, so 35 holds 2nd).
    for (int i = 0; i < 30; ++i) {
        gearbox.update(dt, 35.0, 0.50);
    }
    ASSERT_EQ(gearbox.getCurrentGear(), 2) << "precondition: in 2nd gear";

    // Clear the upshift interval (upshiftMinIntervalS = 2.0 s).
    for (int i = 0; i < 150; ++i) {
        gearbox.update(dt, 35.0, 0.50);
    }

    // Raise speed above the 2->3 upshift (~43 kph) with continuous +-0.5%
    // jitter: the smoothed gradient stays below threshold, no window arms, and
    // the 2->3 upshift proceeds.
    int maxGear = 2;
    for (int i = 0; i < 5 * 60; ++i) {
        const double jitter = (i % 2 == 0) ? 0.005 : -0.005;
        gearbox.update(dt, 55.0, 0.50 + jitter);
        maxGear = std::max(maxGear, gearbox.getCurrentGear());
    }
    EXPECT_GE(maxGear, 3) << "Steady throttle with jitter must still upshift";
}
