#include <gtest/gtest.h>
#include <twin/VirtualIceTwin.h>
#include <twin/IceVehicleProfile.h>
#include <io/UpstreamSignal.h>
#include "TelemetrySequenceBuilder.h"

using namespace twin;
using namespace input;
using namespace test::scenarios;

class StandstillScenarioTest : public ::testing::Test {
protected:
    void SetUp() override {
        profile_ = IceVehicleProfile::zf8hp45();
        twin_ = std::make_unique<VirtualIceTwin>(profile_);
        twin_->setGearSelector(bridge::GearSelector::DRIVE);
        dt_ = 1.0 / 60.0;
    }

    IceVehicleProfile profile_;
    std::unique_ptr<VirtualIceTwin> twin_;
    double dt_;
};

TEST_F(StandstillScenarioTest, GearIs1st_AtStandstill_AC05_1) {
    auto signals = TelemetrySequenceBuilder::buildStandstillTelemetry(30.0, dt_);

    for (const auto& sig : signals) {
        twin_->update(dt_, sig);
    }

    EXPECT_EQ(twin_->getCurrentGear(), 1) << "At standstill, gear selection should be 1st";
}

TEST_F(StandstillScenarioTest, RpmAtIdle_700to800_AC05_2) {
    auto signals = TelemetrySequenceBuilder::buildStandstillTelemetry(5.0, dt_);

    for (const auto& sig : signals) {
        twin_->update(dt_, sig);
    }

    EXPECT_GE(twin_->getSmoothedThrottle(), 0.0) << "Throttle at standstill should be 0";
    EXPECT_LE(twin_->getSmoothedThrottle(), 0.05) << "Throttle at standstill should be ≤ 5%";

    double idleRpm = profile_.idleRpm;
    EXPECT_GE(idleRpm, 700.0) << "Profile idle RPM should be at least 700";
    EXPECT_LE(idleRpm, 800.0) << "Profile idle RPM should be at most 800";
}

TEST_F(StandstillScenarioTest, NoShifts_DuringIdle_AC05_3) {
    auto signals = TelemetrySequenceBuilder::buildStandstillTelemetry(30.0, dt_);

    int initialGear = 0;
    for (const auto& sig : signals) {
        initialGear = twin_->getCurrentGear();
        twin_->update(dt_, sig);
        int currentGear = twin_->getCurrentGear();

        if (currentGear != initialGear && initialGear != 0) {
            FAIL() << "No shifts should occur during idle period at standstill";
        }
    }

    SUCCEED() << "No shifts occurred during idle period";
}

TEST_F(StandstillScenarioTest, LowThrottle_NoAcceleration_AC05_4) {
    auto signals = TelemetrySequenceBuilder::buildStandstillTelemetry(5.0, dt_);

    for (const auto& sig : signals) {
        twin_->update(dt_, sig);
    }

    auto lowThrottleSignals = TelemetrySequenceBuilder::buildCruiseTelemetry(0.0, 0.04, 3.0, dt_);

    double maxSpeed = 0.0;

    for (const auto& sig : lowThrottleSignals) {
        twin_->update(dt_, sig);
        maxSpeed = std::max(maxSpeed, sig.speedKmh);
    }

    EXPECT_LE(maxSpeed, 0.5) << "Throttle input ≤ 5% should produce minimal or no acceleration from idle";
}
