// SpeedRpmConversionTest.cpp - TDD test for speed→RPM conversion
//
// Tests the pure function computeTargetRpm(speedKmh, gear, gearRatio, tireRadiusM, diffRatio)
// which computes engine RPM from road speed using the formula:
//   engineRPM = (speedKmh / 3.6) / (2 * PI * tireRadius) * 60 * gearRatio * diffRatio
//
// Tests every row of SpeedRpmTable_C63.h to prove the conversion is correct.

#include <gtest/gtest.h>
#include <cmath>
#include "test/data/SpeedRpmTable_C63.h"
#include "twin/SpeedRpmConversion.h"

// ============================================================================
// Parametrized test fixture for C63 speed-RPM table
// ============================================================================
class SpeedRpmConversionC63Test : public ::testing::TestWithParam<test_data::SpeedRpmEntry> {};

// Test every row in the C63 table (98 entries: 14 speeds x 7 gears)
TEST_P(SpeedRpmConversionC63Test, ComputeRpmMatchesTable) {
    const auto& entry = GetParam();

    // Skip invalid entries (RPM exceeds redline)
    if (!entry.isValid) {
        return;
    }

    // Use C63 vehicle parameters
    constexpr double gearRatios[] = {4.34, 2.89, 1.92, 1.37, 1.00, 0.82, 0.73};
    constexpr double diffRatio = test_data::c63::kDiffRatio;
    constexpr double tireRadius = test_data::c63::kTireRadius;
    constexpr double redline = test_data::c63::kRedline;

    // Gear index is 1-based in the table, convert to 0-based
    int gearIndex = entry.gear - 1;
    ASSERT_GE(gearIndex, 0) << "Invalid gear index";
    ASSERT_LT(gearIndex, 7) << "Gear index out of range";

    double gearRatio = gearRatios[gearIndex];

    // Compute RPM using the pure function
    double computedRpm = twin::computeTargetRpm(
        entry.speedKmh,
        gearRatio,
        tireRadius,
        diffRatio,
        redline  // Apply redline clamp
    );

    // Tolerance: 0.1% or 1 RPM, whichever is larger
    double tolerance = std::max(std::abs(entry.expectedRpm) * 0.001, 1.0);
    EXPECT_NEAR(computedRpm, entry.expectedRpm, tolerance)
        << "Speed: " << entry.speedKmh << " km/h, Gear: " << entry.gear;
}

// Instantiate the test with all C63 table entries
INSTANTIATE_TEST_SUITE_P(
    AllC63SpeedRpmEntries,
    SpeedRpmConversionC63Test,
    ::testing::ValuesIn(
        test_data::c63::kSpeedRpmTable,
        test_data::c63::kSpeedRpmTable + test_data::c63::kSpeedRpmTableSize
    )
);

// ============================================================================
// Test redline clamping
// ============================================================================
TEST(SpeedRpmConversionTest, RedlineClamp) {
    // C63 parameters
    constexpr double gearRatio = 4.34;  // First gear
    constexpr double tireRadius = test_data::c63::kTireRadius;
    constexpr double diffRatio = test_data::c63::kDiffRatio;
    constexpr double redline = test_data::c63::kRedline;

    // Speed that would exceed redline in first gear
    double speedKmh = 100.0;  // Should give ~9097 RPM, clamped to 7200

    double computedRpm = twin::computeTargetRpm(
        speedKmh,
        gearRatio,
        tireRadius,
        diffRatio,
        redline
    );

    EXPECT_LE(computedRpm, redline) << "RPM should be clamped to redline";
    EXPECT_NEAR(computedRpm, redline, 1.0) << "Clamped RPM should equal redline";
}

// ============================================================================
// Test zero speed returns zero RPM
// ============================================================================
TEST(SpeedRpmConversionTest, ZeroSpeedReturnsZeroRpm) {
    constexpr double gearRatio = 4.34;
    constexpr double tireRadius = test_data::c63::kTireRadius;
    constexpr double diffRatio = test_data::c63::kDiffRatio;

    double rpm = twin::computeTargetRpm(0.0, gearRatio, tireRadius, diffRatio);
    EXPECT_NEAR(rpm, 0.0, 0.01);
}

// ============================================================================
// Test invalid parameters return zero RPM
// ============================================================================
TEST(SpeedRpmConversionTest, InvalidParametersReturnZero) {
    // Zero gear ratio
    double rpm1 = twin::computeTargetRpm(50.0, 0.0, 0.32, 3.15);
    EXPECT_NEAR(rpm1, 0.0, 0.01);

    // Negative speed
    double rpm2 = twin::computeTargetRpm(-10.0, 4.34, 0.32, 3.15);
    EXPECT_NEAR(rpm2, 0.0, 0.01);

    // Zero tire radius
    double rpm3 = twin::computeTargetRpm(50.0, 4.34, 0.0, 3.15);
    EXPECT_NEAR(rpm3, 0.0, 0.01);
}

// ============================================================================
// Test Ferrari F136 table (additional validation)
// ============================================================================
class SpeedRpmConversionFerrariTest : public ::testing::TestWithParam<test_data::SpeedRpmEntry> {};

TEST_P(SpeedRpmConversionFerrariTest, ComputeRpmMatchesFerrariTable) {
    const auto& entry = GetParam();

    if (!entry.isValid) {
        return;
    }

    constexpr double gearRatios[] = {3.23, 2.19, 1.61, 1.23, 0.97, 0.80};
    constexpr double diffRatio = test_data::ferrari_f136::kDiffRatio;
    constexpr double tireRadius = test_data::ferrari_f136::kTireRadius;
    constexpr double redline = test_data::ferrari_f136::kRedline;

    int gearIndex = entry.gear - 1;
    ASSERT_GE(gearIndex, 0) << "Invalid gear index";
    ASSERT_LT(gearIndex, 6) << "Gear index out of range";

    double gearRatio = gearRatios[gearIndex];
    double computedRpm = twin::computeTargetRpm(
        entry.speedKmh,
        gearRatio,
        tireRadius,
        diffRatio,
        redline
    );

    double tolerance = std::max(std::abs(entry.expectedRpm) * 0.001, 1.0);
    EXPECT_NEAR(computedRpm, entry.expectedRpm, tolerance)
        << "Speed: " << entry.speedKmh << " km/h, Gear: " << entry.gear;
}

INSTANTIATE_TEST_SUITE_P(
    AllFerrariSpeedRpmEntries,
    SpeedRpmConversionFerrariTest,
    ::testing::ValuesIn(
        test_data::ferrari_f136::kSpeedRpmTable,
        test_data::ferrari_f136::kSpeedRpmTable + test_data::ferrari_f136::kSpeedRpmTableSize
    )
);

// ============================================================================
// Test GM LS table (additional validation)
// ============================================================================
class SpeedRpmConversionGmLsTest : public ::testing::TestWithParam<test_data::SpeedRpmEntry> {};

TEST_P(SpeedRpmConversionGmLsTest, ComputeRpmMatchesGmLsTable) {
    const auto& entry = GetParam();

    if (!entry.isValid) {
        return;
    }

    constexpr double gearRatios[] = {2.97, 2.07, 1.43, 1.00, 0.71, 0.57};
    constexpr double diffRatio = test_data::gm_ls::kDiffRatio;
    constexpr double tireRadius = test_data::gm_ls::kTireRadius;
    constexpr double redline = test_data::gm_ls::kRedline;

    int gearIndex = entry.gear - 1;
    ASSERT_GE(gearIndex, 0) << "Invalid gear index";
    ASSERT_LT(gearIndex, 6) << "Gear index out of range";

    double gearRatio = gearRatios[gearIndex];
    double computedRpm = twin::computeTargetRpm(
        entry.speedKmh,
        gearRatio,
        tireRadius,
        diffRatio,
        redline
    );

    double tolerance = std::max(std::abs(entry.expectedRpm) * 0.001, 1.0);
    EXPECT_NEAR(computedRpm, entry.expectedRpm, tolerance)
        << "Speed: " << entry.speedKmh << " km/h, Gear: " << entry.gear;
}

INSTANTIATE_TEST_SUITE_P(
    AllGmLsSpeedRpmEntries,
    SpeedRpmConversionGmLsTest,
    ::testing::ValuesIn(
        test_data::gm_ls::kSpeedRpmTable,
        test_data::gm_ls::kSpeedRpmTable + test_data::gm_ls::kSpeedRpmTableSize
    )
);
