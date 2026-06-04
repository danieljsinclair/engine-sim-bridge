#include <gtest/gtest.h>
#include <twin/AutomaticGearbox.h>
#include <twin/IceVehicleProfile.h>

using namespace twin;

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

    // At 50% throttle, 1st->2nd happens at interpolated ~28 km/h
    // (between 40% row: 24 and 55% row: 30, t=0.667)
    gearbox.update(0.1, 27.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);

    gearbox.update(0.1, 29.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);
}

TEST_F(AutomaticGearboxTest, UpshiftAt100PercentThrottle_AC_01_5)
{
    AutomaticGearbox gearbox(profile);

    // At 100% throttle, 1st->2nd happens at 49 km/h (from shiftTable row 9)
    gearbox.update(0.1, 48.0, 1.0);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);

    gearbox.update(0.1, 50.0, 1.0);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);
}

TEST_F(AutomaticGearboxTest, DownshiftAt85PercentOfUpshiftSpeed_AC_03_4)
{
    AutomaticGearbox gearbox(profile);

    // Accelerate to 2nd gear at 50% throttle (1->2 upshift at ~28 km/h,
    // but below 2->3 upshift at ~42 km/h)
    gearbox.update(0.1, 35.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);

    // Downshift table at 50%: 2->1 happens at interpolated ~20.67 km/h
    // (between 40% row: 18 and 55% row: 22)
    gearbox.update(0.1, 22.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);

    gearbox.update(0.1, 20.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);
}

TEST_F(AutomaticGearboxTest, CoastDownSequentialDownshifts_AC_03_1)
{
    AutomaticGearbox gearbox(profile);

    // Accelerate to high gear at 80% throttle
    gearbox.update(0.1, 100.0, 0.8);
    EXPECT_GE(gearbox.getCurrentGear(), 3);

    int topGear = gearbox.getCurrentGear();

    // Coast down (throttle = 0) — downshift table at 5% (clamped)
    // Downshift thresholds at 5%: 2->1=9, 3->2=13, 4->3=20, 5->4=25
    // Need to drop below each threshold sequentially
    gearbox.update(0.1, 50.0, 0.0);
    gearbox.update(0.1, 30.0, 0.0);
    gearbox.update(0.1, 15.0, 0.0);  // Below 4->3=20 and 3->2=13 triggers
    gearbox.update(0.1, 5.0, 0.0);   // Below 2->1=9

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

    // Kickdown: throttle jumps from 0.3 to 0.9 within 100ms (delta > 0.4)
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

TEST_F(AutomaticGearboxTest, MinShiftIntervalBetweenConsecutiveShifts_AC_10_5)
{
    AutomaticGearbox gearbox(profile);

    // Upshift to 2nd gear at 50% throttle (1->2 at ~28 km/h)
    gearbox.update(0.1, 35.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);

    // Immediately try to upshift to 3rd (2->3 at ~42 km/h at 50%)
    // Should be blocked by upshift interval (2.0s for ZF 8HP45)
    gearbox.update(0.1, 50.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);

    // Advance 1.8 seconds (total 1.9s since last shift) - still blocked
    gearbox.update(1.8, 50.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);

    // Advance past 2.0 seconds total - upshift allowed
    gearbox.update(0.2, 50.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 3);
}

TEST_F(AutomaticGearboxTest, KickdownDetectionThresholds_AC_10_4)
{
    AutomaticGearbox gearbox(profile);

    // Cruise
    gearbox.update(0.1, 60.0, 0.3);
    int initialGear = gearbox.getCurrentGear();
    EXPECT_GT(initialGear, 1);

    // Small throttle increase - should NOT trigger kickdown (delta < 0.4)
    gearbox.update(0.05, 60.0, 0.6);
    EXPECT_FALSE(gearbox.requestsShift());

    // Reset - create a new gearbox to reset state
    {
        AutomaticGearbox newGearbox(profile);
        newGearbox.update(0.1, 60.0, 0.3);
        // Large throttle increase - SHOULD trigger kickdown (delta > 0.4)
        newGearbox.update(0.05, 60.0, 0.9);
        EXPECT_TRUE(newGearbox.requestsShift());
    }

    // Reset again for high throttle threshold test
    {
        AutomaticGearbox newGearbox2(profile);
        newGearbox2.update(0.1, 60.0, 0.3);
        // Throttle at 0.95 (at threshold) - should trigger kickdown
        newGearbox2.update(0.05, 60.0, 0.95);
        EXPECT_TRUE(newGearbox2.requestsShift());
    }
}

TEST_F(AutomaticGearboxTest, InterpolateShiftTableForIntermediateThrottle)
{
    AutomaticGearbox gearbox(profile);

    // At 47% throttle (between 40% row and 55% row in new 10-level table)
    // 40%: 1st->2nd at 24 km/h
    // 55%: 1st->2nd at 30 km/h
    // Interpolated at 47%: 24 + (0.47-0.40)/(0.55-0.40) * (30-24) = 24 + 0.467*6 = 26.8 km/h
    gearbox.update(0.1, 26.0, 0.47);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);

    gearbox.update(0.1, 28.0, 0.47);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);
}
