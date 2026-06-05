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

    // Asymmetric shift intervals (ZF 8HP45 per x-engineer ch6 s4.2)
    EXPECT_DOUBLE_EQ(profile.upshiftMinIntervalS, 2.0);
    EXPECT_DOUBLE_EQ(profile.downshiftMinIntervalS, 1.0);

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
    EXPECT_DOUBLE_EQ(profile.shiftTableThrottleLevels[9], 1.00);

    // Each throttle row should have 7 shift thresholds
    for (const auto& row : profile.shiftTable) {
        EXPECT_EQ(row.size(), 7u);
    }

    // Shift speeds should increase with throttle (higher throttle = higher shift speed)
    for (size_t col = 0; col < 7; ++col) {
        EXPECT_GT(profile.shiftTable[9][col], profile.shiftTable[0][col])
            << "WOT shift speed should be higher than light throttle for column " << col;
    }

    // Shift speeds should increase with gear within a row (1->2 < 2->3 < ... < 7->8)
    for (const auto& row : profile.shiftTable) {
        for (size_t i = 1; i < row.size(); ++i) {
            EXPECT_GT(row[i], row[i - 1])
                << "Shift speeds should increase monotonically across gears";
        }
    }

    // Verify separate downshift table structure
    ASSERT_EQ(profile.downshiftTable.size(), 10u);
    ASSERT_EQ(profile.downshiftTableThrottleLevels.size(), 10u);

    // Each downshift throttle row should have 7 thresholds
    for (const auto& row : profile.downshiftTable) {
        EXPECT_EQ(row.size(), 7u);
    }

    // Downshift speeds should be lower than upshift speeds (separate table allows this)
    // Compare 5% throttle row
    for (size_t col = 0; col < 7; ++col) {
        EXPECT_LT(profile.downshiftTable[0][col], profile.shiftTable[0][col])
            << "Downshift speed should be lower than upshift speed at light throttle";
    }

    // Downshift speeds should increase with throttle (coast-down at higher throttle = higher speeds)
    for (size_t col = 0; col < 7; ++col) {
        EXPECT_GE(profile.downshiftTable[9][col], profile.downshiftTable[0][col])
            << "Downshift speed at WOT should be >= downshift at light throttle";
    }

    // Downshift speeds should increase with gear within a row
    for (const auto& row : profile.downshiftTable) {
        for (size_t i = 1; i < row.size(); ++i) {
            EXPECT_GT(row[i], row[i - 1])
                << "Downshift speeds should increase monotonically across gears";
        }
    }
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
