// RedlineClampAndUpshiftTest.cpp - TDD test for redline clamp and upshift behavior
//
// Tests:
// 1. computeTargetRpm clamps to redline
// 2. AutomaticGearbox upshifts when RPM would exceed redline

#include <gtest/gtest.h>
#include "twin/AutomaticGearbox.h"
#include "twin/IceVehicleProfile.h"
#include "twin/SpeedRpmConversion.h"

class RedlineClampAndUpshiftTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a C63-style profile
        profile_ = twin::IceVehicleProfile();
        profile_.gearRatios = {4.34, 2.89, 1.92, 1.37, 1.00, 0.82, 0.73};
        profile_.diffRatio = 2.82;
        profile_.tireRadiusM = 0.35687;
        profile_.redlineRpm = 7200.0;
        profile_.idleRpm = 700.0;
        profile_.standstillThresholdKmh = 1.0;
        profile_.minShiftIntervalS = 0.5;

        gearbox_ = std::make_unique<twin::AutomaticGearbox>(profile_);
    }

    twin::IceVehicleProfile profile_;
    std::unique_ptr<twin::AutomaticGearbox> gearbox_;
};

// ============================================================================
// Test: computeTargetRpm clamps to redline
// ============================================================================
TEST_F(RedlineClampAndUpshiftTest, ComputeTargetRpmClampsToRedline) {
    // C63 first gear parameters
    double gearRatio = profile_.gearRatios[0];  // 4.34
    double tireRadius = profile_.tireRadiusM;   // 0.35687
    double diffRatio = profile_.diffRatio;      // 2.82
    double redline = profile_.redlineRpm;       // 7200.0

    // Speed that would exceed redline in first gear
    // At 100 km/h in first gear: RPM ≈ 9097 (exceeds redline)
    double speedKmh = 100.0;

    // Without redline clamp
    double unclampedRpm = twin::computeTargetRpm(speedKmh, gearRatio, tireRadius, diffRatio, 0.0);
    EXPECT_GT(unclampedRpm, redline)
        << "At 100 km/h in first gear, RPM should exceed redline without clamping";

    // With redline clamp
    double clampedRpm = twin::computeTargetRpm(speedKmh, gearRatio, tireRadius, diffRatio, redline);
    EXPECT_NEAR(clampedRpm, redline, 1.0)
        << "With redline clamp, RPM should be limited to redline";
}

// ============================================================================
// Test: computeTargetRpm doesn't clamp when below redline
// ============================================================================
TEST_F(RedlineClampAndUpshiftTest, ComputeTargetRpmDoesntClampBelowRedline) {
    double gearRatio = profile_.gearRatios[0];
    double tireRadius = profile_.tireRadiusM;
    double diffRatio = profile_.diffRatio;
    double redline = profile_.redlineRpm;

    // Speed that's well below redline in first gear
    double speedKmh = 50.0;  // Should give ~4548 RPM

    double rpm = twin::computeTargetRpm(speedKmh, gearRatio, tireRadius, diffRatio, redline);
    EXPECT_LT(rpm, redline)
        << "At 50 km/h in first gear, RPM should be below redline";
    EXPECT_GT(rpm, redline * 0.5)
        << "RPM should be reasonable (>50% redline)";
}

// ============================================================================
// Test: Redline clamp works with zero redline (no clamp)
// ============================================================================
TEST_F(RedlineClampAndUpshiftTest, ComputeTargetRpmNoClampWithZeroRedline) {
    double gearRatio = profile_.gearRatios[0];
    double tireRadius = profile_.tireRadiusM;
    double diffRatio = profile_.diffRatio;

    double speedKmh = 100.0;

    // Zero redline means no clamping
    double rpm = twin::computeTargetRpm(speedKmh, gearRatio, tireRadius, diffRatio, 0.0);
    EXPECT_GT(rpm, 7200.0)
        << "With zero redline (no clamp), RPM should exceed 7200";
}

// ============================================================================
// Test: AutomaticGearbox upshifts at redline (when RPM feedback is provided)
// ============================================================================
TEST_F(RedlineClampAndUpshiftTest, AutomaticGearboxUpshiftsAtRedline) {
    // Set up gearbox in first gear
    ASSERT_EQ(gearbox_->getCurrentGear(), 1)
        << "Gearbox should start in first gear";

    // Provide RPM feedback above redline threshold (95% of redline)
    double redlineThreshold = profile_.redlineRpm * 0.95;  // 6840 RPM
    gearbox_->setTwinContext(
        0,      // twinState (unused)
        0.0,    // clutchPressure
        50.0,   // speedFeedbackKmh
        redlineThreshold + 100.0  // rpmFeedback (just above 95% redline)
    );

    // Update gearbox with speed that would trigger upshift
    gearbox_->update(0.1, 50.0, 0.5);

    // Should request upshift
    EXPECT_TRUE(gearbox_->requestsShift())
        << "At 95% redline, gearbox should request upshift";

    // Target gear should be second gear
    EXPECT_EQ(gearbox_->getTargetGear(), 2)
        << "Redline upshift should target second gear";

    EXPECT_EQ(gearbox_->getLastShiftDirection(), 1)
        << "Shift direction should be up (+1)";
}

// ============================================================================
// Test: AutomaticGearbox doesn't upshift below redline threshold
// ============================================================================
TEST_F(RedlineClampAndUpshiftTest, AutomaticGearboxNoUpshiftBelowRedlineThreshold) {
    ASSERT_EQ(gearbox_->getCurrentGear(), 1);

    // Provide RPM feedback below redline threshold
    double belowRedline = profile_.redlineRpm * 0.9;  // 6480 RPM
    gearbox_->setTwinContext(
        0,
        0.0,
        50.0,
        belowRedline
    );

    // Update gearbox
    gearbox_->update(0.1, 50.0, 0.5);

    // Should NOT request upshift
    EXPECT_FALSE(gearbox_->requestsShift())
        << "Below 95% redline, gearbox should not request upshift";

    // Should stay in first gear
    EXPECT_EQ(gearbox_->getTargetGear(), 1)
        << "Gear should remain first when below redline threshold";
}

// ============================================================================
// Test: Redline upshift bypasses normal shift interval
// ============================================================================
TEST_F(RedlineClampAndUpshiftTest, RedlineUpshiftBypassesShiftInterval) {
    ASSERT_EQ(gearbox_->getCurrentGear(), 1);

    // Provide RPM feedback above redline threshold
    double redlineThreshold = profile_.redlineRpm * 0.95;
    gearbox_->setTwinContext(
        0,
        0.0,
        50.0,
        redlineThreshold + 100.0
    );

    // Immediately update (no time for normal shift interval to pass)
    gearbox_->update(0.001, 50.0, 0.5);

    // Should still request upshift (redline bypasses interval)
    EXPECT_TRUE(gearbox_->requestsShift())
        << "Redline upshift should bypass normal shift interval";
}
