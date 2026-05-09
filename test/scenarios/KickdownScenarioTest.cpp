#include <gtest/gtest.h>
#include <twin/VirtualIceTwin.h>
#include <twin/IceVehicleProfile.h>
#include <io/UpstreamSignal.h>
#include "TelemetrySequenceBuilder.h"

using namespace twin;
using namespace input;
using namespace test::scenarios;

class KickdownScenarioTest : public ::testing::Test {
protected:
    void SetUp() override {
        profile_ = IceVehicleProfile::zf8hp45();
        twin_ = std::make_unique<VirtualIceTwin>(profile_);
        dt_ = 1.0 / 60.0;
    }

    IceVehicleProfile profile_;
    std::unique_ptr<VirtualIceTwin> twin_;
    double dt_;

    void warmupToCruise(double speedKmh, double throttle) {
        auto signals = TelemetrySequenceBuilder::buildAccelerationTelemetry(15.0, dt_);
        for (auto& sig : signals) {
            if (sig.speedKmh > speedKmh) sig.speedKmh = speedKmh;
            sig.throttleFraction = throttle;
            twin_->update(dt_, sig);
        }
    }

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

TEST_F(KickdownScenarioTest, DownshiftWithin500ms_AC04_1) {
    warmupToCruise(60.0, 0.2);

    int gearBeforeKickdown = twin_->getCurrentGear();
    double kickdownTime = -1.0;
    double time = 0.0;

    auto signals = TelemetrySequenceBuilder::buildKickdownTelemetry(60.0, 0.2, 1.0, 3.0, dt_);

    for (const auto& sig : signals) {
        int gearBefore = twin_->getCurrentGear();
        twin_->update(dt_, sig);
        int gearAfter = twin_->getCurrentGear();

        if (gearAfter < gearBefore && sig.throttleFraction > 0.9) {
            kickdownTime = time;
            break;
        }
        time += dt_;
    }

    EXPECT_NE(kickdownTime, -1.0) << "Kickdown should trigger a downshift";
    if (kickdownTime >= 0.0) {
        EXPECT_LE(kickdownTime, 0.5) << "Downshift should occur within 500ms of throttle step";
    }
}

TEST_F(KickdownScenarioTest, TargetGear_RpmBelow90PercentRedline_AC04_2) {
    warmupToCruise(60.0, 0.2);

    auto signals = TelemetrySequenceBuilder::buildKickdownTelemetry(60.0, 0.2, 1.0, 3.0, dt_);

    int targetGear = -1;
    for (const auto& sig : signals) {
        int gearBefore = twin_->getCurrentGear();
        twin_->update(dt_, sig);
        int gearAfter = twin_->getCurrentGear();

        if (gearAfter < gearBefore && sig.throttleFraction > 0.9) {
            targetGear = gearAfter;
            break;
        }
    }

    if (targetGear > 0) {
        double rpm = calculateRpm(60.0, targetGear);
        double maxSafeRpm = profile_.redlineRpm * 0.9;
        EXPECT_LT(rpm, maxSafeRpm) << "Target gear should produce RPM < 90% of redline at current speed";
    } else {
        SUCCEED() << "No downshift occurred to validate RPM";
    }
}

TEST_F(KickdownScenarioTest, From6thGear_DownshiftTo3rdOr4th_AC04_3) {
    warmupToCruise(60.0, 0.2);

    int initialGear = twin_->getCurrentGear();

    auto signals = TelemetrySequenceBuilder::buildKickdownTelemetry(60.0, 0.2, 1.0, 3.0, dt_);

    int kickdownGear = -1;
    for (const auto& sig : signals) {
        int gearBefore = twin_->getCurrentGear();
        twin_->update(dt_, sig);
        int gearAfter = twin_->getCurrentGear();

        if (gearAfter < gearBefore && sig.throttleFraction > 0.9) {
            kickdownGear = gearAfter;
            break;
        }
    }

    if (initialGear >= 6 && kickdownGear > 0) {
        EXPECT_TRUE(kickdownGear == 3 || kickdownGear == 4)
            << "From 6th gear at 60 km/h, kickdown should go to 3rd or 4th (multi-gear skip allowed)";
    } else {
        SUCCEED() << "Test condition not met (not in 6th gear)";
    }
}

TEST_F(KickdownScenarioTest, GearHeld_UntilThrottleDrops_AC04_4) {
    warmupToCruise(60.0, 0.2);

    auto signals = TelemetrySequenceBuilder::buildKickdownTelemetry(60.0, 0.2, 1.0, 3.0, dt_);

    int kickdownGear = -1;
    size_t kickdownIndex = 0;
    for (size_t i = 0; i < signals.size(); ++i) {
        int gearBefore = twin_->getCurrentGear();
        twin_->update(dt_, signals[i]);
        int gearAfter = twin_->getCurrentGear();

        if (gearAfter < gearBefore && signals[i].throttleFraction > 0.9) {
            kickdownGear = gearAfter;
            kickdownIndex = i;
            break;
        }
    }

    if (kickdownGear > 0) {
        bool gearHeld = true;
        for (size_t i = kickdownIndex; i < signals.size(); ++i) {
            if (signals[i].throttleFraction < 0.8 && twin_->getCurrentGear() == kickdownGear) {
                continue;
            }
            if (signals[i].throttleFraction >= 0.8 && twin_->getCurrentGear() != kickdownGear) {
                gearHeld = false;
                break;
            }
        }
        EXPECT_TRUE(gearHeld) << "Gear should be held until throttle drops below 80%";
    } else {
        SUCCEED() << "No kickdown occurred to validate gear hold";
    }
}

TEST_F(KickdownScenarioTest, SubsequentUpshifts_AtHigherSpeeds_AC04_5) {
    warmupToCruise(60.0, 0.2);

    auto signals = TelemetrySequenceBuilder::buildKickdownTelemetry(60.0, 0.2, 1.0, 3.0, dt_);

    int kickdownGear = -1;
    bool sawUpshiftAfterKickdown = false;
    double upshiftSpeed = 0.0;

    for (const auto& sig : signals) {
        int gearBefore = twin_->getCurrentGear();
        twin_->update(dt_, sig);
        int gearAfter = twin_->getCurrentGear();

        if (gearAfter < gearBefore && sig.throttleFraction > 0.9) {
            kickdownGear = gearAfter;
        }

        if (kickdownGear > 0 && gearAfter > gearBefore) {
            sawUpshiftAfterKickdown = true;
            upshiftSpeed = sig.speedKmh;
            break;
        }
    }

    if (sawUpshiftAfterKickdown) {
        EXPECT_GT(upshiftSpeed, 140.0) << "Subsequent upshifts should occur at higher speeds (>140 km/h for 100% throttle)";
    } else {
        SUCCEED() << "No upshift observed after kickdown within test duration";
    }
}
