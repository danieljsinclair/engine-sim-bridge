#include <gtest/gtest.h>
#include <twin/IceVehicleProfile.h>

using namespace twin;

TEST(IceVehicleProfile, Zf8hp45FactoryDefaults)
{
    IceVehicleProfile profile = IceVehicleProfile::zf8hp45();

    // ZF 8HP45 gear ratios
    ASSERT_EQ(profile.gearRatios.size(), 8);
    EXPECT_DOUBLE_EQ(profile.gearRatios[0], 4.714);
    EXPECT_DOUBLE_EQ(profile.gearRatios[1], 3.143);
    EXPECT_DOUBLE_EQ(profile.gearRatios[2], 2.106);
    EXPECT_DOUBLE_EQ(profile.gearRatios[3], 1.667);
    EXPECT_DOUBLE_EQ(profile.gearRatios[4], 1.285);
    EXPECT_DOUBLE_EQ(profile.gearRatios[5], 1.000);
    EXPECT_DOUBLE_EQ(profile.gearRatios[6], 0.839);
    EXPECT_DOUBLE_EQ(profile.gearRatios[7], 0.667);

    // Drivetrain parameters
    EXPECT_DOUBLE_EQ(profile.diffRatio, 3.15);
    EXPECT_DOUBLE_EQ(profile.tireRadiusM, 0.32);
    EXPECT_DOUBLE_EQ(profile.vehicleMassKg, 1800.0);

    // RPM limits
    EXPECT_DOUBLE_EQ(profile.redlineRpm, 6500.0);
    EXPECT_DOUBLE_EQ(profile.idleRpm, 750.0);

    // Shift timing (ms)
    EXPECT_DOUBLE_EQ(profile.shiftDisengageMs, 50.0);
    EXPECT_DOUBLE_EQ(profile.shiftPauseMs, 200.0);
    EXPECT_DOUBLE_EQ(profile.shiftReengageMs, 100.0);

    // Throttle smoothing
    EXPECT_DOUBLE_EQ(profile.throttleSmoothingTauMs, 50.0);

    // Minimum shift interval
    EXPECT_DOUBLE_EQ(profile.minShiftIntervalS, 3.0);

    // Hysteresis factor
    EXPECT_DOUBLE_EQ(profile.hysteresisFactor, 0.85);

    // Kickdown parameters
    EXPECT_DOUBLE_EQ(profile.kickdownThrottleThreshold, 0.95);
    EXPECT_DOUBLE_EQ(profile.kickdownDelta, 0.4);
    EXPECT_DOUBLE_EQ(profile.kickdownWindowMs, 100.0);
}

TEST(IceVehicleProfile, ShiftTableCalibration)
{
    IceVehicleProfile profile = IceVehicleProfile::zf8hp45();

    // Verify shift table structure: 5 throttle levels (10%, 25%, 50%, 75%, 100%)
    // Each with 7 upshift thresholds (1->2, 2->3, ..., 7->8)
    ASSERT_EQ(profile.shiftTable.size(), 5);

    // 10% throttle row
    const auto& throttle10 = profile.shiftTable[0];
    ASSERT_EQ(throttle10.size(), 7);
    EXPECT_DOUBLE_EQ(throttle10[0], 20.0);
    EXPECT_DOUBLE_EQ(throttle10[1], 35.0);
    EXPECT_DOUBLE_EQ(throttle10[2], 50.0);
    EXPECT_DOUBLE_EQ(throttle10[3], 65.0);
    EXPECT_DOUBLE_EQ(throttle10[4], 80.0);
    EXPECT_DOUBLE_EQ(throttle10[5], 95.0);
    EXPECT_DOUBLE_EQ(throttle10[6], 110.0);

    // 25% throttle row
    const auto& throttle25 = profile.shiftTable[1];
    ASSERT_EQ(throttle25.size(), 7);
    EXPECT_DOUBLE_EQ(throttle25[0], 30.0);
    EXPECT_DOUBLE_EQ(throttle25[1], 50.0);
    EXPECT_DOUBLE_EQ(throttle25[2], 70.0);
    EXPECT_DOUBLE_EQ(throttle25[3], 90.0);
    EXPECT_DOUBLE_EQ(throttle25[4], 110.0);
    EXPECT_DOUBLE_EQ(throttle25[5], 130.0);
    EXPECT_DOUBLE_EQ(throttle25[6], 155.0);

    // 50% throttle row
    const auto& throttle50 = profile.shiftTable[2];
    ASSERT_EQ(throttle50.size(), 7);
    EXPECT_DOUBLE_EQ(throttle50[0], 40.0);
    EXPECT_DOUBLE_EQ(throttle50[1], 65.0);
    EXPECT_DOUBLE_EQ(throttle50[2], 90.0);
    EXPECT_DOUBLE_EQ(throttle50[3], 115.0);
    EXPECT_DOUBLE_EQ(throttle50[4], 140.0);
    EXPECT_DOUBLE_EQ(throttle50[5], 170.0);
    EXPECT_DOUBLE_EQ(throttle50[6], 200.0);

    // 75% throttle row
    const auto& throttle75 = profile.shiftTable[3];
    ASSERT_EQ(throttle75.size(), 7);
    EXPECT_DOUBLE_EQ(throttle75[0], 55.0);
    EXPECT_DOUBLE_EQ(throttle75[1], 85.0);
    EXPECT_DOUBLE_EQ(throttle75[2], 115.0);
    EXPECT_DOUBLE_EQ(throttle75[3], 145.0);
    EXPECT_DOUBLE_EQ(throttle75[4], 180.0);
    EXPECT_DOUBLE_EQ(throttle75[5], 215.0);
    EXPECT_DOUBLE_EQ(throttle75[6], 255.0);

    // 100% throttle row
    const auto& throttle100 = profile.shiftTable[4];
    ASSERT_EQ(throttle100.size(), 7);
    EXPECT_DOUBLE_EQ(throttle100[0], 70.0);
    EXPECT_DOUBLE_EQ(throttle100[1], 105.0);
    EXPECT_DOUBLE_EQ(throttle100[2], 140.0);
    EXPECT_DOUBLE_EQ(throttle100[3], 180.0);
    EXPECT_DOUBLE_EQ(throttle100[4], 220.0);
    EXPECT_DOUBLE_EQ(throttle100[5], 265.0);
    EXPECT_DOUBLE_EQ(throttle100[6], 315.0);
}

TEST(IceVehicleProfile, CustomConstruction)
{
    std::vector<double> customGearRatios = {5.0, 3.0, 2.0, 1.5, 1.0};
    std::vector<std::vector<double>> customShiftTable = {
        {30.0, 50.0, 70.0},
        {40.0, 60.0, 80.0}
    };

    IceVehicleProfile profile(
        customGearRatios,
        3.5,
        0.35,
        2000.0,
        customShiftTable,
        0.9,
        0.90,
        0.35,
        0.3,
        50.0,
        200.0,
        100.0,
        40.0,
        2.5,
        7000.0,
        800.0
    );

    EXPECT_EQ(profile.gearRatios.size(), 5);
    EXPECT_DOUBLE_EQ(profile.gearRatios[0], 5.0);
    EXPECT_DOUBLE_EQ(profile.diffRatio, 3.5);
    EXPECT_DOUBLE_EQ(profile.vehicleMassKg, 2000.0);
    EXPECT_DOUBLE_EQ(profile.redlineRpm, 7000.0);
    EXPECT_DOUBLE_EQ(profile.idleRpm, 800.0);
    EXPECT_DOUBLE_EQ(profile.hysteresisFactor, 0.9);
    EXPECT_DOUBLE_EQ(profile.kickdownThrottleThreshold, 0.90);
}

TEST(IceVehicleProfile, ParameterRangesAreValid)
{
    IceVehicleProfile profile = IceVehicleProfile::zf8hp45();

    // All gear ratios should be positive
    for (double ratio : profile.gearRatios) {
        EXPECT_GT(ratio, 0.0);
    }

    // Ratios should be strictly decreasing
    for (size_t i = 1; i < profile.gearRatios.size(); ++i) {
        EXPECT_LT(profile.gearRatios[i], profile.gearRatios[i - 1]);
    }

    // Diff ratio should be positive
    EXPECT_GT(profile.diffRatio, 0.0);

    // Tire radius should be positive and reasonable
    EXPECT_GT(profile.tireRadiusM, 0.2);
    EXPECT_LT(profile.tireRadiusM, 0.5);

    // Vehicle mass should be positive
    EXPECT_GT(profile.vehicleMassKg, 500.0);

    // Redline should be greater than idle
    EXPECT_GT(profile.redlineRpm, profile.idleRpm);

    // Shift timing should be positive
    EXPECT_GT(profile.shiftDisengageMs, 0.0);
    EXPECT_GT(profile.shiftPauseMs, 0.0);
    EXPECT_GT(profile.shiftReengageMs, 0.0);

    // Hysteresis factor should be between 0 and 1
    EXPECT_GT(profile.hysteresisFactor, 0.0);
    EXPECT_LT(profile.hysteresisFactor, 1.0);

    // Kickdown thresholds should be valid
    EXPECT_GE(profile.kickdownThrottleThreshold, 0.0);
    EXPECT_LE(profile.kickdownThrottleThreshold, 1.0);
    EXPECT_GT(profile.kickdownDelta, 0.0);
    EXPECT_LT(profile.kickdownDelta, 1.0);

    // Shift table speeds should be positive and increasing
    for (const auto& row : profile.shiftTable) {
        for (size_t i = 1; i < row.size(); ++i) {
            EXPECT_GT(row[i], row[i - 1]);
        }
    }
}
