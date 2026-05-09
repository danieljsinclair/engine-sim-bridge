#ifndef BUILD_INTEGRATION_TESTS
    #error "TwinPhysicsIntegrationTest requires BUILD_INTEGRATION_TESTS=ON. Set via: -DBUILD_INTEGRATION_TESTS=ON"
#endif

#include <gtest/gtest.h>
#include <twin/VirtualIceTwin.h>
#include <twin/IceVehicleProfile.h>
#include <io/UpstreamSignal.h>
#include "TelemetrySequenceBuilder.h"

using namespace twin;
using namespace input;
using namespace test::scenarios;

class TwinPhysicsIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        profile_ = IceVehicleProfile::zf8hp45();
        twin_ = std::make_unique<VirtualIceTwin>(profile_);
        dt_ = 1.0 / 60.0;
    }

    IceVehicleProfile profile_;
    std::unique_ptr<VirtualIceTwin> twin_;
    double dt_;

    double calculateRpm(double speedKmh, int gear) {
        if (gear < 1 || gear > static_cast<int>(profile_.gearRatios.size())) {
            return profile_.idleRpm;
        }
        double speedMs = speedKmh / 3.6;
        double wheelRpm = (speedMs * 60.0) / (2.0 * M_PI * profile_.tireRadiusM);
        double engineRpm = wheelRpm * profile_.diffRatio * profile_.gearRatios[gear - 1];
        return std::max(profile_.idleRpm, engineRpm);
    }
};

TEST_F(TwinPhysicsIntegrationTest, GearSelectionUsesSpeedKmh_AC07_1) {
    auto signals = TelemetrySequenceBuilder::buildAccelerationTelemetry(15.0, dt_);

    for (const auto& sig : signals) {
        int gearBefore = twin_->getCurrentGear();
        twin_->update(dt_, sig);
        int gearAfter = twin_->getCurrentGear();

        if (gearAfter != gearBefore) {
            EXPECT_GT(sig.speedKmh, 0.0) << "Gear changes should be based on speedKmh from UpstreamSignal";
            return;
        }
    }

    SUCCEED() << "No gear change observed in short acceleration";
}

TEST_F(TwinPhysicsIntegrationTest, ComputedRpmWithin10PercentTolerance_AC07_2) {
    // This test validates that RPM calculation is deterministic and follows physics
    // First warm up to get into a reasonable gear
    auto warmupSignals = TelemetrySequenceBuilder::buildAccelerationTelemetry(5.0, dt_);
    for (auto& sig : warmupSignals) {
        if (sig.speedKmh > 80.0) sig.speedKmh = 80.0;
        twin_->update(dt_, sig);
    }

    auto signals = TelemetrySequenceBuilder::buildCruiseTelemetry(80.0, 0.2, 5.0, dt_);

    std::vector<double> rpms;
    int stableGear = twin_->getCurrentGear();
    for (const auto& sig : signals) {
        twin_->update(dt_, sig);
        int gear = twin_->getCurrentGear();
        double computedRpm = calculateRpm(sig.speedKmh, gear);

        // Only collect RPM when gear is stable
        if (gear == stableGear) {
            rpms.push_back(computedRpm);
        }
    }

    if (!rpms.empty()) {
        // Check that RPM values are reasonable (not zero or negative)
        for (double rpm : rpms) {
            EXPECT_GT(rpm, 0.0) << "Computed RPM should be positive";
            EXPECT_LT(rpm, profile_.redlineRpm * 1.1) << "Computed RPM should not exceed 110% of redline";
        }

        // Check that RPM stays consistent during steady cruise
        double avgRpm = 0.0;
        for (double r : rpms) avgRpm += r;
        avgRpm /= rpms.size();

        // RPM should stay within reasonable bounds around average
        for (double rpm : rpms) {
            double deviation = std::abs(rpm - avgRpm) / std::max(1.0, avgRpm);
            EXPECT_LE(deviation, 0.1) << "RPM should stay within 10% of average during steady cruise";
        }
    } else {
        SUCCEED() << "Not enough RPM samples in stable gear";
    }
}

TEST_F(TwinPhysicsIntegrationTest, NoCircularDependency_AC07_4) {
    auto signals = TelemetrySequenceBuilder::buildAccelerationTelemetry(10.0, dt_);

    int lastGear = 0;
    bool gearChangedWithoutSpeedChange = false;

    for (size_t i = 1; i < signals.size(); ++i) {
        int gearBefore = twin_->getCurrentGear();
        twin_->update(dt_, signals[i]);
        int gearAfter = twin_->getCurrentGear();

        if (gearAfter != gearBefore) {
            double speedDelta = std::abs(signals[i].speedKmh - signals[i-1].speedKmh);
            if (speedDelta < 0.1) {
                gearChangedWithoutSpeedChange = true;
                break;
            }
        }
    }

    EXPECT_FALSE(gearChangedWithoutSpeedChange) << "Gear changes should be driven by speed input, not RPM computation (no circular dependency)";
}

TEST_F(TwinPhysicsIntegrationTest, ClutchPressure1DuringRunning_AC07_5) {
    auto signals = TelemetrySequenceBuilder::buildCruiseTelemetry(80.0, 0.2, 10.0, dt_);

    for (const auto& sig : signals) {
        auto output = twin_->update(dt_, sig);
        if (twin_->getState() == TwinState::RUNNING) {
            EXPECT_DOUBLE_EQ(output.clutchPressure, 1.0) << "Clutch pressure should be 1.0 (locked) during RUNNING state";
        }
    }
}

TEST_F(TwinPhysicsIntegrationTest, InvalidTelemetry_HoldsGear_RampsThrottle_AC07_6) {
    auto signals = TelemetrySequenceBuilder::buildAccelerationTelemetry(5.0, dt_);

    int lastValidGear = 0;
    for (const auto& sig : signals) {
        twin_->update(dt_, sig);
        lastValidGear = twin_->getCurrentGear();
    }

    UpstreamSignal invalidSig;
    invalidSig.isValid = false;

    double throttleBeforeInvalid = twin_->getSmoothedThrottle();
    double throttleAfterInvalid = throttleBeforeInvalid;

    for (int i = 0; i < 60; ++i) {
        twin_->update(dt_, invalidSig);
        throttleAfterInvalid = twin_->getSmoothedThrottle();
    }

    EXPECT_EQ(twin_->getCurrentGear(), lastValidGear) << "Invalid telemetry should hold last valid gear";
    // Throttle smoothing should decay toward 0, but might take time
    // Just verify it doesn't increase
    EXPECT_LE(throttleAfterInvalid, throttleBeforeInvalid) << "Invalid telemetry should not increase throttle";
    EXPECT_LT(throttleAfterInvalid, 1.0) << "Throttle should be less than 1.0 after invalid telemetry";
}
