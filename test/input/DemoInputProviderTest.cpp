#include <gtest/gtest.h>
#include <input/DemoInputProvider.h>
#include <input/IThrottleSource.h>
#include <twin/IceVehicleProfile.h>

using namespace input;
using namespace twin;

class MockThrottleSource : public IThrottleSource {
public:
    explicit MockThrottleSource(double throttle = 0.0, bool cont = true)
        : throttle_(throttle), shouldContinue_(cont) {}

    double pollThrottle() override { return throttle_; }
    bool shouldContinue() const override { return shouldContinue_; }

    void setThrottle(double t) { throttle_ = t; }
    void setShouldContinue(bool c) { shouldContinue_ = c; }

private:
    double throttle_;
    bool shouldContinue_;
};

class DemoInputProviderTest : public ::testing::Test {
protected:
    IceVehicleProfile profile_{IceVehicleProfile::zf8hp45()};
    std::unique_ptr<MockThrottleSource> throttleSource_{std::make_unique<MockThrottleSource>(0.0)};
    MockThrottleSource* rawSource_{throttleSource_.get()};

    std::unique_ptr<DemoInputProvider> provider_;

    void SetUp() override {
        provider_ = std::make_unique<DemoInputProvider>(
            std::move(throttleSource_), profile_);
    }
};

TEST_F(DemoInputProviderTest, Initialize_Succeeds) {
    ASSERT_TRUE(provider_->Initialize());
    EXPECT_TRUE(provider_->IsConnected());
}

TEST_F(DemoInputProviderTest, GetProviderName_ReturnsDemo) {
    EXPECT_EQ(provider_->GetProviderName(), "DemoInputProvider");
}

TEST_F(DemoInputProviderTest, OnUpdate_ReturnsValidEngineInput) {
    ASSERT_TRUE(provider_->Initialize());
    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_TRUE(input.shouldContinue);
}

TEST_F(DemoInputProviderTest, ThrottlePropagatesToEngineInput) {
    rawSource_->setThrottle(0.8);
    ASSERT_TRUE(provider_->Initialize());

    // Warm up through twin state machine: OFF → CRANKING → IDLE → RUNNING
    for (int i = 0; i < 200; ++i) {
        provider_->OnUpdateSimulation(0.016);
    }

    // Now in RUNNING state, throttle should pass through
    EngineInput input;
    for (int i = 0; i < 20; ++i) {
        input = provider_->OnUpdateSimulation(0.016);
    }
    EXPECT_GT(input.throttle, 0.3);
}

TEST_F(DemoInputProviderTest, GearAbsolute_AppliedNotGearDelta) {
    rawSource_->setThrottle(0.8);
    ASSERT_TRUE(provider_->Initialize());

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    // Twin output uses gearAbsolute, not gearDelta
    EXPECT_GE(input.gearAbsolute, 0);
    EXPECT_EQ(input.gearDelta, 0);
}

TEST_F(DemoInputProviderTest, ClutchPressure_InValidRange) {
    ASSERT_TRUE(provider_->Initialize());

    for (int i = 0; i < 100; ++i) {
        EngineInput input = provider_->OnUpdateSimulation(0.016);
        EXPECT_GE(input.clutchPressure, 0.0);
        EXPECT_LE(input.clutchPressure, 1.0);
    }
}

TEST_F(DemoInputProviderTest, ShouldContinue_False_WhenThrottleSourceExits) {
    rawSource_->setShouldContinue(false);
    ASSERT_TRUE(provider_->Initialize());

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_FALSE(input.shouldContinue);
}

TEST_F(DemoInputProviderTest, SpeedIncreasesOverTime_FullThrottle) {
    rawSource_->setThrottle(1.0);
    ASSERT_TRUE(provider_->Initialize());

    for (int i = 0; i < 500; ++i) {
        provider_->OnUpdateSimulation(0.016);
    }

    double roadSpeed = provider_->getDemoRoadSpeedKmh();
    EXPECT_GT(roadSpeed, 10.0);
}

TEST_F(DemoInputProviderTest, SpeedReported_ViaTelemetry) {
    rawSource_->setThrottle(0.5);
    ASSERT_TRUE(provider_->Initialize());

    provider_->OnUpdateSimulation(0.016);
    double speed = provider_->getDemoRoadSpeedKmh();
    EXPECT_GE(speed, 0.0);
}

TEST_F(DemoInputProviderTest, GearReported_ViaTelemetry) {
    ASSERT_TRUE(provider_->Initialize());
    provider_->OnUpdateSimulation(0.016);
    int gear = provider_->getDemoGear();
    EXPECT_GE(gear, 0);
}
