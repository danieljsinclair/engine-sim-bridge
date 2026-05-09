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

    // At 50% throttle, 1st->2nd should happen around 40 km/h
    gearbox.update(0.1, 39.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);

    gearbox.update(0.1, 40.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);

    gearbox.update(0.1, 41.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);
}

TEST_F(AutomaticGearboxTest, UpshiftAt100PercentThrottle_AC_01_5)
{
    AutomaticGearbox gearbox(profile);

    // At 100% throttle, 1st->2nd should happen around 70 km/h
    gearbox.update(0.1, 69.0, 1.0);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);

    gearbox.update(0.1, 70.0, 1.0);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);

    gearbox.update(0.1, 71.0, 1.0);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);
}

TEST_F(AutomaticGearboxTest, DownshiftAt85PercentOfUpshiftSpeed_AC_03_4)
{
    AutomaticGearbox gearbox(profile);

    // Accelerate to 2nd gear at 50% throttle
    gearbox.update(0.1, 45.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);

    // Upshift speed for 1st->2nd at 50% is 40 km/h
    // Downshift should occur at 85% = 34 km/h
    gearbox.update(0.1, 35.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);

    gearbox.update(0.1, 34.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);

    gearbox.update(0.1, 33.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);
}

TEST_F(AutomaticGearboxTest, CoastDownSequentialDownshifts_AC_03_1)
{
    AutomaticGearbox gearbox(profile);

    // Accelerate to high gear
    gearbox.update(0.1, 100.0, 0.8);
    EXPECT_GE(gearbox.getCurrentGear(), 3);

    int topGear = gearbox.getCurrentGear();

    // Coast down (throttle = 0)
    gearbox.update(0.1, 80.0, 0.0);
    gearbox.update(0.1, 60.0, 0.0);
    gearbox.update(0.1, 40.0, 0.0);
    gearbox.update(0.1, 20.0, 0.0);

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

TEST_F(AutomaticGearboxTest, Min3SecondsBetweenShifts_AC_10_5)
{
    AutomaticGearbox gearbox(profile);

    // Perform an upshift
    gearbox.update(0.1, 45.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);

    // Try to shift again immediately - should be blocked
    gearbox.update(0.1, 35.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);

    // Advance 2.9 seconds - still blocked
    gearbox.update(2.9, 35.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);

    // Advance past 3 seconds - shift allowed
    gearbox.update(0.2, 33.0, 0.5);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);
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

    // At 40% throttle (between 25% and 50% rows)
    // 25%: 1st->2nd at 30 km/h
    // 50%: 1st->2nd at 40 km/h
    // Interpolated at 40%: 36 km/h
    gearbox.update(0.1, 35.0, 0.4);
    EXPECT_EQ(gearbox.getCurrentGear(), 1);

    gearbox.update(0.1, 37.0, 0.4);
    EXPECT_EQ(gearbox.getCurrentGear(), 2);
}
