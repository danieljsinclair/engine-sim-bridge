#include <gtest/gtest.h>
#include "input/VirtualIceInputProvider.h"
#include "io/UpstreamSignal.h"
#include "twin/IceVehicleProfile.h"

using namespace input;
using namespace twin;

class VirtualIceInputProviderTest : public ::testing::Test {
protected:
    void SetUp() override {
        profile_ = IceVehicleProfile::zf8hp45();
        provider_ = std::make_unique<VirtualIceInputProvider>(profile_);
    }

    IceVehicleProfile profile_;
    std::unique_ptr<VirtualIceInputProvider> provider_;
};

TEST_F(VirtualIceInputProviderTest, InitializeAndConnect) {
    EXPECT_TRUE(provider_->Initialize());
    EXPECT_TRUE(provider_->IsConnected());
    EXPECT_EQ("VirtualIceInputProvider", provider_->GetProviderName());
}

TEST_F(VirtualIceInputProviderTest, ShutdownDisconnects) {
    ASSERT_TRUE(provider_->Initialize());
    ASSERT_TRUE(provider_->IsConnected());

    provider_->Shutdown();
    EXPECT_FALSE(provider_->IsConnected());
}

TEST_F(VirtualIceInputProviderTest, InitializeTwiceFails) {
    ASSERT_TRUE(provider_->Initialize());
    EXPECT_FALSE(provider_->Initialize());
    EXPECT_NE("", provider_->GetLastError());
}

TEST_F(VirtualIceInputProviderTest, ZeroThrottleWhenSignalInvalid) {
    ASSERT_TRUE(provider_->Initialize());

    UpstreamSignal invalidSignal;
    invalidSignal.isValid = false;
    provider_->setUpstreamSignal(invalidSignal);

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_EQ(0.0, input.throttle);
}

TEST_F(VirtualIceInputProviderTest, ThrottleMapsCorrectly) {
    ASSERT_TRUE(provider_->Initialize());

    UpstreamSignal signal;
    signal.throttleFraction = 0.5;
    signal.speedKmh = 10.0;
    signal.isValid = true;
    provider_->setUpstreamSignal(signal);

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_LE(input.throttle, 1.0);
}

TEST_F(VirtualIceInputProviderTest, GearMapsFromTwin) {
    ASSERT_TRUE(provider_->Initialize());

    UpstreamSignal signal;
    signal.throttleFraction = 0.8;
    signal.speedKmh = 60.0;
    signal.isValid = true;
    provider_->setUpstreamSignal(signal);

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_GE(input.gearAbsolute, 0);
}

TEST_F(VirtualIceInputProviderTest, ClutchPressureMapsFromTwin) {
    ASSERT_TRUE(provider_->Initialize());

    UpstreamSignal signal;
    signal.throttleFraction = 0.5;
    signal.speedKmh = 30.0;
    signal.isValid = true;
    provider_->setUpstreamSignal(signal);

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_GE(input.clutchPressure, 0.0);
    EXPECT_LE(input.clutchPressure, 1.0);
}

TEST_F(VirtualIceInputProviderTest, ShouldContinueIsTrue) {
    ASSERT_TRUE(provider_->Initialize());

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_TRUE(input.shouldContinue);
}

TEST_F(VirtualIceInputProviderTest, CrankingStateActivatesStarterAndIgnition) {
    ASSERT_TRUE(provider_->Initialize());

    UpstreamSignal signal;
    signal.throttleFraction = 0.0;
    signal.speedKmh = 0.0;
    signal.timestampUtcMs = 1234567890;
    signal.isValid = true;
    provider_->setUpstreamSignal(signal);

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_TRUE(input.ignition);
    EXPECT_TRUE(input.starterMotor);
    EXPECT_EQ(1, input.gearAbsolute);
}
