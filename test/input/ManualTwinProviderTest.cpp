#include <gtest/gtest.h>
#include "input/ManualTwinProvider.h"
#include "input/IThrottleSource.h"
#include "simulator/ISimulator.h"
#include "simulator/EngineSimTypes.h"

#include <memory>

using namespace input;

namespace {

class MockThrottleSource : public IThrottleSource {
public:
    double pollThrottle() override { return throttle; }
    bool shouldContinue() const override { return continueFlag; }

    double throttle = 0.0;
    bool continueFlag = true;
};

class MockSimulator : public ISimulator {
public:
    bool create(const ISimulatorConfig&, ILogging*, telemetry::ITelemetryWriter*) override { return true; }
    void destroy() override {}
    std::string getLastError() const override { return ""; }
    const char* getName() const override { return "MockSimulator"; }
    void update(double) override {}
    EngineSimStats getStats() const override { return {}; }
    void setThrottle(double) override {}
    bool renderOnDemand(float*, int32_t, int32_t*) override { return false; }
    bool readAudioBuffer(float*, int32_t, int32_t*) override { return false; }
    bool start() override { return true; }
    void stop() override {}
    int getSimulationFrequency() const override { return 60; }

    double getEngineRpm() const override { return rpm; }
    double rpm = 0.0;
};

} // anonymous namespace

class ManualTwinProviderTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto throttle = std::make_unique<MockThrottleSource>();
        throttle_ = throttle.get();
        provider_ = std::make_unique<ManualTwinProvider>(
            std::move(throttle),
            sim_
        );
    }

    MockThrottleSource* throttle_ = nullptr;
    MockSimulator sim_;
    std::unique_ptr<ManualTwinProvider> provider_;
};

TEST_F(ManualTwinProviderTest, InitializeAndConnect) {
    EXPECT_TRUE(provider_->Initialize());
    EXPECT_TRUE(provider_->IsConnected());
    EXPECT_EQ("ManualTwinProvider", provider_->GetProviderName());
}

TEST_F(ManualTwinProviderTest, ShutdownDisconnects) {
    ASSERT_TRUE(provider_->Initialize());
    provider_->Shutdown();
    EXPECT_FALSE(provider_->IsConnected());
}

TEST_F(ManualTwinProviderTest, ReturnsDefaultWhenUninitialized) {
    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_DOUBLE_EQ(input.throttle, 0.0);
}

TEST_F(ManualTwinProviderTest, StaysOffWithoutIgnitionAndStarter) {
    ASSERT_TRUE(provider_->Initialize());

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_FALSE(input.starterButton);
    EXPECT_FALSE(input.ignition);
    EXPECT_EQ(input.gearAbsolute, 0); // Neutral
}

TEST_F(ManualTwinProviderTest, CrankingActivatesWithIgnitionAndStarter) {
    ASSERT_TRUE(provider_->Initialize());

    provider_->setIgnitionRequested(true);
    provider_->setStarterRequested(true);
    EngineInput input = provider_->OnUpdateSimulation(0.016);

    EXPECT_TRUE(input.starterButton);
    EXPECT_TRUE(input.ignition);
    EXPECT_EQ(input.gearAbsolute, 0); // Neutral during cranking
    EXPECT_DOUBLE_EQ(input.clutchPressure, 0.0);
}

TEST_F(ManualTwinProviderTest, TransitionsToRunningWhenRpmRises) {
    ASSERT_TRUE(provider_->Initialize());
    provider_->setIgnitionRequested(true);
    provider_->setStarterRequested(true);
    provider_->OnUpdateSimulation(0.016); // OFF -> CRANKING

    sim_.rpm = 500.0;
    throttle_->throttle = 0.7;
    EngineInput input = provider_->OnUpdateSimulation(0.016);

    EXPECT_FALSE(input.starterButton);
    EXPECT_DOUBLE_EQ(input.throttle, 0.7);
}

TEST_F(ManualTwinProviderTest, ThrottlePassthroughWhenRunning) {
    ASSERT_TRUE(provider_->Initialize());
    provider_->setIgnitionRequested(true);
    provider_->setStarterRequested(true);
    provider_->OnUpdateSimulation(0.016); // OFF -> CRANKING

    sim_.rpm = 500.0;
    provider_->OnUpdateSimulation(0.016); // CRANKING -> RUNNING

    throttle_->throttle = 0.5;
    sim_.rpm = 800.0;
    EngineInput input = provider_->OnUpdateSimulation(0.016);

    EXPECT_DOUBLE_EQ(input.throttle, 0.5);
}

TEST_F(ManualTwinProviderTest, GearUpFromNeutralWhenRunning) {
    ASSERT_TRUE(provider_->Initialize());
    provider_->setIgnitionRequested(true);
    provider_->setStarterRequested(true);
    provider_->OnUpdateSimulation(0.016); // OFF -> CRANKING

    sim_.rpm = 500.0;
    provider_->OnUpdateSimulation(0.016); // CRANKING -> RUNNING

    provider_->setGearUpRequested(true);
    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_EQ(input.gearAbsolute, 1); // FIRST
}
