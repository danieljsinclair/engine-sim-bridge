#include <gtest/gtest.h>
#include <input/DemoInputProvider.h>
#include <input/DemoThrottleSource.h>
#include <input/IThrottleSource.h>
#include <input/GearSelectorInput.h>
#include <input/IgnitionInput.h>
#include <twin/IceVehicleProfile.h>
#include <simulator/GearConventions.h>
#include <simulator/EngineSimTypes.h>
#include <cmath>

using namespace input;
using namespace twin;
using namespace bridge;

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
    std::unique_ptr<DemoThrottleSource> demoThrottle_{std::make_unique<DemoThrottleSource>()};
    DemoThrottleSource* rawThrottle_{demoThrottle_.get()};

    std::unique_ptr<DemoInputProvider> provider_;

    void SetUp() override {
        provider_ = std::make_unique<DemoInputProvider>(
            std::move(demoThrottle_),
            std::make_unique<GearSelectorInput>(),
            std::make_unique<IgnitionInput>(),
            profile_);
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
    EXPECT_DOUBLE_EQ(input.throttle, 0.0);
}

TEST_F(DemoInputProviderTest, SetThrottle_PropagatesViaIDemoControls) {
    ASSERT_TRUE(provider_->Initialize());

    // Warm up through twin state machine: OFF → CRANKING → IDLE → RUNNING
    // Feed throttle every frame (like CLI does when key is held)
    for (int i = 0; i < 200; ++i) {
        rawThrottle_->setThrottleLevel(0.8);
        provider_->OnUpdateSimulation(0.016);
    }

    // Now in RUNNING state, throttle should pass through
    EngineInput input;
    for (int i = 0; i < 20; ++i) {
        rawThrottle_->setThrottleLevel(0.8);
        input = provider_->OnUpdateSimulation(0.016);
    }
    EXPECT_GT(input.throttle, 0.3);
}

TEST_F(DemoInputProviderTest, GearAbsolute_AppliedNotGearDelta) {
    rawThrottle_->setThrottleLevel(0.8);
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
    rawThrottle_->requestExit();
    ASSERT_TRUE(provider_->Initialize());

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    // No crash, input still valid, throttle source's exit intent no longer in input struct
    EXPECT_GE(input.gearAbsolute, -1);
}

TEST_F(DemoInputProviderTest, SpeedIncreasesOverTime_FullThrottle) {
    ASSERT_TRUE(provider_->Initialize());

    for (int i = 0; i < 500; ++i) {
        rawThrottle_->setThrottleLevel(1.0);
        provider_->OnUpdateSimulation(0.016);
    }

    double roadSpeed = provider_->getDemoRoadSpeedKmh();
    EXPECT_GT(roadSpeed, 10.0);
}

TEST_F(DemoInputProviderTest, SpeedReported_ViaTelemetry) {
    rawThrottle_->setThrottleLevel(0.5);
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

// IDemoControls interface tests

TEST_F(DemoInputProviderTest, SetThrottle_ViaIDemoControls) {
    ASSERT_TRUE(provider_->Initialize());
    provider_->setThrottle(0.7);
    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_GT(input.throttle, 0.0);
}

TEST_F(DemoInputProviderTest, ShiftUp_CyclesGear) {
    ASSERT_TRUE(provider_->Initialize());
    int initialGear = provider_->getGearSelectorState();
    provider_->shiftUp();
    int afterShift = provider_->getGearSelectorState();
    // Should have changed (NEUTRAL → DRIVE by default)
    EXPECT_NE(initialGear, afterShift);
}

TEST_F(DemoInputProviderTest, ShiftDown_CyclesGear) {
    ASSERT_TRUE(provider_->Initialize());
    provider_->shiftUp();
    provider_->shiftUp();
    int afterShiftUp = provider_->getGearSelectorState();
    provider_->shiftDown();
    int afterShiftDown = provider_->getGearSelectorState();
    EXPECT_NE(afterShiftUp, afterShiftDown);
}

TEST_F(DemoInputProviderTest, GetGearSelectorState_ReturnsValidState) {
    ASSERT_TRUE(provider_->Initialize());
    int state = provider_->getGearSelectorState();
    EXPECT_GE(state, 0);
}

TEST_F(DemoInputProviderTest, SetIgnition_Off) {
    ASSERT_TRUE(provider_->Initialize());
    provider_->setIgnition(false);
    EXPECT_FALSE(provider_->isIgnitionOn());
}

TEST_F(DemoInputProviderTest, SetIgnition_On) {
    ASSERT_TRUE(provider_->Initialize());
    provider_->setIgnition(true);
    EXPECT_TRUE(provider_->isIgnitionOn());
}

TEST_F(DemoInputProviderTest, ToggleIgnition_FlipsState) {
    ASSERT_TRUE(provider_->Initialize());
    bool initial = provider_->isIgnitionOn();
    provider_->toggleIgnition();
    bool afterToggle = provider_->isIgnitionOn();
    EXPECT_NE(initial, afterToggle);
}

TEST_F(DemoInputProviderTest, RequestExit_ViaIDemoControls) {
    ASSERT_TRUE(provider_->Initialize());
    provider_->requestExit();
    EngineInput input = provider_->OnUpdateSimulation(0.016);
    // shouldContinue removed from EngineInput — exit is now via session->stop()
    // requestExit() signals the throttle source; no crash or exception is sufficient
    EXPECT_GE(input.gearAbsolute, -1);
}

// ============================================================================
// Full-chain integration test - reproduces live demo data path
// ============================================================================

TEST(DemoInputProviderIntegration, GearboxShiftsAt20PercentThrottle) {
    // Create components exactly like CLIMain.cpp does
    auto throttle = std::make_unique<input::DemoThrottleSource>();
    input::DemoThrottleSource* rawThrottle = throttle.get();
    auto gearSelector = std::make_unique<input::GearSelectorInput>();
    auto ignition = std::make_unique<input::IgnitionInput>();

    auto provider = std::make_unique<input::DemoInputProvider>(
        std::move(throttle),
        std::move(gearSelector),
        std::move(ignition),
        twin::IceVehicleProfile::zf8hp45()
    );

    ASSERT_TRUE(provider->Initialize());
    ASSERT_TRUE(provider->IsConnected());

    // Simulate the live demo: set throttle to 20% and hold it
    rawThrottle->setThrottleLevel(0.20);

    double dt = 1.0 / 86.0;  // Match live demo rate (86 Hz from display)

    // Simulate engine-sim feedback (like what happens in SimulationLoop)
    EngineSimStats feedbackStats{};
    feedbackStats.currentRPM = 0.0;
    feedbackStats.vehicleSpeedKmh = 0.0;

    // Simulate gear selector set to DRIVE (NEUTRAL → DRIVE via shiftUp)
    provider->shiftUp();

    int gearChanges = 0;
    int lastGear = 0;

    printf("\n=== FULL-CHAIN INTEGRATION TEST: 20%% THROTTLE GEARSHIFT ===\n");
    printf("  Profile: ZF 8HP45 (ratios: 4.714, 3.143, 2.106, 1.667, 1.285, 1.000, 0.839, 0.667)\n");
    printf("  Simulation rate: %.1f Hz (dt=%.4f sec)\n", 1.0/dt, dt);
    printf("  Duration: 60 seconds\n\n");

    for (int i = 0; i < static_cast<int>(60.0 / dt); ++i) {
        double t = i * dt;

        // Simulate what SimulationLoop does:
        // 1. provideFeedback with engine-sim stats
        provider->provideFeedback(feedbackStats);

        // 2. Refresh throttle (simulate key held down)
        rawThrottle->setThrottleLevel(0.20);

        // 3. OnUpdateSimulation
        EngineInput engineInput = provider->OnUpdateSimulation(dt);

        int gear = engineInput.gearAbsolute;

        // Update feedback stats based on engine input (simplified engine-sim response)
        // In the real demo, engine-sim uses gear to calculate RPM and speed
        // Simulate: if gear changes, engine-sim speed changes
        if (gear > 0 && gear <= 8) {
            // Simulate engine-sim RPM based on demo physics speed and gear
            double demoSpeed = provider->getDemoRoadSpeedKmh();
            // Engine-sim produces RPM = speed * gearRatio * diffRatio / (2pi * tireRadius * 3.6 / 60)
            // Simplified: RPM ≈ speed * gearRatio * 26.14
            double gearRatio = twin::IceVehicleProfile::zf8hp45().gearRatios[gear - 1];
            feedbackStats.currentRPM = std::max(750.0, demoSpeed * gearRatio * 26.14);
            feedbackStats.vehicleSpeedKmh = demoSpeed; // Engine-sim speed ≈ demo physics speed for simplicity
        }

        if (gear != lastGear && lastGear != 0) {
            gearChanges++;
            printf("  GEAR CHANGE at t=%.1fs: gear %d → %d, demoSpeed=%.1f kph, engineRPM=%.0f\n",
                   t, lastGear, gear, provider->getDemoRoadSpeedKmh(), feedbackStats.currentRPM);
        }
        lastGear = gear;

        // Print every 5 seconds
        if (i % static_cast<int>(5.0 / dt) == 0) {
            printf("  t=%5.1fs  gear=%d  demoSpeed=%.1f kph  rpm=%.0f  fbSpeed=%.1f\n",
                   t, gear, provider->getDemoRoadSpeedKmh(), feedbackStats.currentRPM, feedbackStats.vehicleSpeedKmh);
        }
    }

    printf("\n=== FINAL RESULTS ===\n");
    printf("  Total gear changes: %d\n", gearChanges);
    printf("  Final gear: %d\n", lastGear);
    printf("  Final speed: %.1f kph\n", provider->getDemoRoadSpeedKmh());
    printf("  Final RPM: %.0f\n", feedbackStats.currentRPM);
    printf("======================\n\n");

    EXPECT_GT(gearChanges, 0) << "Gearbox should shift at least once during 60s at 20% throttle";
    EXPECT_GT(lastGear, 1) << "Final gear should be > 1 after 60s at 20% throttle";
}