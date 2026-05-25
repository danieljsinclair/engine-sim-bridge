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

    // Asymmetric shift intervals (ZF 8HP45)
    EXPECT_DOUBLE_EQ(profile.upshiftMinIntervalS, 2.0);
    EXPECT_DOUBLE_EQ(profile.downshiftMinIntervalS, 1.5);

    // Engine braking inhibitor
    EXPECT_TRUE(profile.engineBrakingInhibitorEnabled);
    EXPECT_DOUBLE_EQ(profile.engineBrakingMaxThrottle, 0.01);
    EXPECT_DOUBLE_EQ(profile.engineBrakingMinSpeedKmh, 10.0);

    // Tip-in/tip-out correction
    EXPECT_TRUE(profile.tipCorrectionEnabled);
    EXPECT_DOUBLE_EQ(profile.tipInGradientThreshold, 10.0);
    EXPECT_DOUBLE_EQ(profile.tipOutGradientThreshold, -10.0);

    // Separate downshift table
    EXPECT_TRUE(profile.separateDownshiftTableEnabled);

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

    // Verify shift table structure: 10 throttle levels
    // Each with 7 upshift thresholds (1->2, 2->3, ..., 7->8)
    ASSERT_EQ(profile.shiftTable.size(), 10u);
    ASSERT_EQ(profile.shiftTableThrottleLevels.size(), 10u);

    // Verify throttle level breakpoints
    EXPECT_DOUBLE_EQ(profile.shiftTableThrottleLevels[0], 0.05);
    EXPECT_DOUBLE_EQ(profile.shiftTableThrottleLevels[3], 0.40);
    EXPECT_DOUBLE_EQ(profile.shiftTableThrottleLevels[9], 1.00);

    // 5% throttle row
    const auto& throttle5 = profile.shiftTable[0];
    ASSERT_EQ(throttle5.size(), 7u);
    EXPECT_DOUBLE_EQ(throttle5[0], 11.0);
    EXPECT_DOUBLE_EQ(throttle5[6], 64.0);

    // 25% throttle row
    const auto& throttle25 = profile.shiftTable[2];
    ASSERT_EQ(throttle25.size(), 7u);
    EXPECT_DOUBLE_EQ(throttle25[0], 19.0);
    EXPECT_DOUBLE_EQ(throttle25[6], 105.0);

    // 50% interpolated between 40% and 55% rows — just verify key rows exist
    const auto& throttle40 = profile.shiftTable[3];
    ASSERT_EQ(throttle40.size(), 7u);
    EXPECT_DOUBLE_EQ(throttle40[0], 24.0);
    EXPECT_DOUBLE_EQ(throttle40[6], 137.0);

    const auto& throttle55 = profile.shiftTable[4];
    ASSERT_EQ(throttle55.size(), 7u);
    EXPECT_DOUBLE_EQ(throttle55[0], 30.0);
    EXPECT_DOUBLE_EQ(throttle55[6], 169.0);

    // 100% throttle row (WOT)
    const auto& throttle100 = profile.shiftTable[9];
    ASSERT_EQ(throttle100.size(), 7u);
    EXPECT_DOUBLE_EQ(throttle100[0], 49.0);
    EXPECT_DOUBLE_EQ(throttle100[6], 274.0);

    // Verify separate downshift table
    ASSERT_EQ(profile.downshiftTable.size(), 10u);
    ASSERT_EQ(profile.downshiftTableThrottleLevels.size(), 10u);

    // 5% downshift row
    const auto& downshift5 = profile.downshiftTable[0];
    ASSERT_EQ(downshift5.size(), 7u);
    EXPECT_DOUBLE_EQ(downshift5[0], 9.0);
    EXPECT_DOUBLE_EQ(downshift5[6], 50.0);

    // 100% downshift row
    const auto& downshift100 = profile.downshiftTable[9];
    ASSERT_EQ(downshift100.size(), 7u);
    EXPECT_DOUBLE_EQ(downshift100[0], 34.0);
    EXPECT_DOUBLE_EQ(downshift100[6], 192.0);
}

TEST(IceVehicleProfile, CustomConstruction)
{
    IceVehicleProfile profile;
    profile.gearRatios = {5.0, 3.0, 2.0, 1.5, 1.0};
    profile.shiftTable = {
        {30.0, 50.0, 70.0},
        {40.0, 60.0, 80.0}
    };
    profile.diffRatio = 3.5;
    profile.tireRadiusM = 0.35;
    profile.vehicleMassKg = 2000.0;
    profile.hysteresisFactor = 0.9;
    profile.kickdownThrottleThreshold = 0.90;
    profile.kickdownDelta = 0.35;
    profile.kickdownWindowMs = 0.3;
    profile.shiftDisengageMs = 50.0;
    profile.shiftPauseMs = 200.0;
    profile.shiftReengageMs = 100.0;
    profile.throttleSmoothingTauMs = 40.0;
    profile.minShiftIntervalS = 2.5;
    profile.redlineRpm = 7000.0;
    profile.idleRpm = 800.0;

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
