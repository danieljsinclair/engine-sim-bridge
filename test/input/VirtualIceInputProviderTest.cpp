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

// When the twin is in PIN mode and RUNNING, its pinVehicleSpeedTargetKmh (the CSV
// speed) must be surfaced through EngineInput.vehicleSpeedTargetKmh so the
// SimulationLoop pins the sim wheels to the CSV speed. FREE (default) leaves it
// at the -1 sentinel, which SimulationLoop treats as "don't change".
TEST_F(VirtualIceInputProviderTest, PinModeSurfacesVehicleSpeedTarget) {
    ASSERT_TRUE(provider_->Initialize());
    provider_->setIgnition(true);  // twin ignition defaults OFF; commanded here

    // DRIVE selector + a real throttle so the twin reaches RUNNING.
    const int drive = 99;  // bridge::GearSelector::DRIVE
    provider_->setGearSelector(drive);

    UpstreamSignal signal;
    signal.throttleFraction = 0.8;
    signal.speedKmh = 32.0;
    signal.isValid = true;
    signal.timestampUtcMs = 1000;  // non-zero -> twin treats as valid telemetry
    provider_->setUpstreamSignal(signal);

    // Advance through CRANKING (deterministic 3s fallback) into RUNNING.
    for (int i = 0; i < 250; ++i) provider_->OnUpdateSimulation(0.016);
    provider_->setWheelCouplingMode(WheelCouplingMode::Pin);

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_DOUBLE_EQ(input.vehicleSpeedTargetKmh, 32.0)
        << "PIN must carry the CSV speed into EngineInput.vehicleSpeedTargetKmh";

    // FREE (default) must NOT pin: leave the -1 sentinel.
    auto freeProvider = std::make_unique<VirtualIceInputProvider>(profile_);
    ASSERT_TRUE(freeProvider->Initialize());
    freeProvider->setIgnition(true);  // twin ignition defaults OFF; commanded here
    freeProvider->setGearSelector(drive);
    freeProvider->setUpstreamSignal(signal);
    for (int i = 0; i < 250; ++i) freeProvider->OnUpdateSimulation(0.016);
    EngineInput freeInput = freeProvider->OnUpdateSimulation(0.016);
    EXPECT_DOUBLE_EQ(freeInput.vehicleSpeedTargetKmh, -1.0)
        << "FREE must leave vehicleSpeedTargetKmh at the -1 sentinel (no pin)";
}

// --pin-tau-ms plumbing + behavior at the provider seam. PIN mode + a chase
// tau must reach the twin: the surfaced EngineInput.vehicleSpeedTargetKmh
// GLIDES across a CSV road-speed step instead of teleporting; tau=0 keeps it
// exactly the raw speed (the rigid regression contract).
TEST_F(VirtualIceInputProviderTest, PinTauChasesSpeedTargetThroughProvider) {
    ASSERT_TRUE(provider_->Initialize());
    const int drive = 99;
    provider_->setGearSelector(drive);
    provider_->setWheelCouplingMode(WheelCouplingMode::Pin);
    provider_->setPinTauMs(150.0);
    provider_->setIgnition(true);  // twin ignition defaults OFF; commanded here

    UpstreamSignal signal;
    signal.throttleFraction = 0.3;
    signal.speedKmh = 4.0;
    signal.isValid = true;
    signal.timestampUtcMs = 1000;
    provider_->setUpstreamSignal(signal);
    for (int i = 0; i < 250; ++i) provider_->OnUpdateSimulation(0.016);

    // Step the CSV speed 4 -> 10: the very next frame must NOT teleport.
    signal.speedKmh = 10.0;
    provider_->setUpstreamSignal(signal);
    const EngineInput stepped = provider_->OnUpdateSimulation(0.016);
    EXPECT_GT(stepped.vehicleSpeedTargetKmh, 4.0)
        << "the chase starts moving immediately after the step";
    EXPECT_LT(stepped.vehicleSpeedTargetKmh, 5.0)
        << "tau=150 must not teleport the pin to the stepped speed in one frame";

    // And it converges onto the new level (zero steady-state error).
    EngineInput settled{};
    for (int i = 0; i < 200; ++i) settled = provider_->OnUpdateSimulation(0.016);
    EXPECT_NEAR(settled.vehicleSpeedTargetKmh, 10.0, 0.1);
}

TEST_F(VirtualIceInputProviderTest, PinTauZeroIsRigidAtProviderSeam) {
    ASSERT_TRUE(provider_->Initialize());
    const int drive = 99;
    provider_->setGearSelector(drive);
    provider_->setWheelCouplingMode(WheelCouplingMode::Pin);
    provider_->setPinTauMs(0.0);
    provider_->setIgnition(true);  // twin ignition defaults OFF; commanded here

    UpstreamSignal signal;
    signal.throttleFraction = 0.3;
    signal.speedKmh = 4.0;
    signal.isValid = true;
    signal.timestampUtcMs = 1000;
    provider_->setUpstreamSignal(signal);
    for (int i = 0; i < 250; ++i) provider_->OnUpdateSimulation(0.016);

    signal.speedKmh = 10.0;
    provider_->setUpstreamSignal(signal);
    const EngineInput stepped = provider_->OnUpdateSimulation(0.016);
    EXPECT_DOUBLE_EQ(stepped.vehicleSpeedTargetKmh, 10.0)
        << "tau=0 must surface the stepped speed at once (rigid pin)";
}

TEST_F(VirtualIceInputProviderTest, CrankingStateActivatesStarterAndIgnition) {
    ASSERT_TRUE(provider_->Initialize());
    provider_->setIgnition(true);  // twin ignition defaults OFF; commanded here

    UpstreamSignal signal;
    signal.throttleFraction = 0.0;
    signal.speedKmh = 0.0;
    signal.timestampUtcMs = 1234567890;
    signal.isValid = true;
    provider_->setUpstreamSignal(signal);

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_TRUE(input.ignition);
    EXPECT_TRUE(input.starterButton);
    EXPECT_EQ(0, input.gearAbsolute);  // Neutral during cranking
}
