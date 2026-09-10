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
    explicit MockThrottleSource(double throttle = 0.0)
        : throttle_(throttle) {}

    double pollThrottle() override { return throttle_; }

    void setThrottle(double t) { throttle_ = t; }

private:
    double throttle_;
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

// D1-D3: the demo path must accept the SAME coupling flags as the live/replay
// telemetry paths. This is a compile-time + intent check: the setters exist and
// are invocable with the same signatures TelemetryProviderFactory uses.
TEST_F(DemoInputProviderTest, CouplingFlags_AcceptedPreInit) {
    // Pre-Initialize: flags must be stored (no crash, no throw) and survive
    // to Initialize() via the store + re-apply contract.
    provider_->setWheelCouplingMode(twin::WheelCouplingMode::Free);
    provider_->setCouplingModel(twin::CouplingModelKind::ClutchMap);
    provider_->setPinTauMs(150.0);
    twin::EffectiveThrottleConfig etc;
    etc.enabled = true;
    provider_->setEffectiveThrottleConfig(etc);
    twin::TorqueInformedGearboxConfig tigc;
    tigc.enabled = true;
    provider_->setTorqueInformedGearboxConfig(tigc);

    ASSERT_TRUE(provider_->Initialize());
    EXPECT_TRUE(provider_->IsConnected());
}

// D2: the demo path must APPLY --coupling-model to the twin, not just store
// it. The twin surfaces the chosen model via EngineInput.couplingIsTorqueConverter
// (true only for TorqueConverter). The demo's DEFAULT coupling model is
// TorqueConverter (DemoInputProvider.h) -> flag true; after selecting ClutchMap
// the flag flips to false. This is the same observable the live/replay telemetry
// paths produce via applyTwinCouplingFlags — proving the demo setters reach the
// twin the same way.
TEST_F(DemoInputProviderTest, CouplingModel_AppliedToTwin) {
    ASSERT_TRUE(provider_->Initialize());
    // Default demo twin is TorqueConverter.
    EngineInput in = provider_->OnUpdateSimulation(0.016);
    EXPECT_TRUE(in.couplingIsTorqueConverter);

    // Selecting clutch-map must reach the twin and flip the flag off.
    provider_->setCouplingModel(twin::CouplingModelKind::ClutchMap);
    in = provider_->OnUpdateSimulation(0.016);
    EXPECT_FALSE(in.couplingIsTorqueConverter)
        << "--coupling-model=clutch-map must reach the demo twin";
}

TEST_F(DemoInputProviderTest, CouplingFlags_AcceptedPostInit) {
    ASSERT_TRUE(provider_->Initialize());
    // Post-Initialize: flags forward straight through to the twin.
    provider_->setWheelCouplingMode(twin::WheelCouplingMode::Torque);
    provider_->setCouplingModel(twin::CouplingModelKind::Legacy);
    provider_->setPinTauMs(75.0);
    twin::EffectiveThrottleConfig etc;
    etc.enabled = false;
    provider_->setEffectiveThrottleConfig(etc);
    twin::TorqueInformedGearboxConfig tigc;
    tigc.enabled = false;
    provider_->setTorqueInformedGearboxConfig(tigc);
    EXPECT_TRUE(provider_->IsConnected());
}

TEST_F(DemoInputProviderTest, RequestExit_ViaIDemoControls_NoCrash) {
    ASSERT_TRUE(provider_->Initialize());
    provider_->requestExit();
    EngineInput input = provider_->OnUpdateSimulation(0.016);
    // requestExit() is now a no-op in the throttle source
    // exit is handled by session->stop() in the simulation loop
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


// ============================================================================
// Full-chain integration test - reproduces live demo data path
// ============================================================================

// ============================================================================
// Brake flow tests — proves setBrake() → OnUpdateSimulation() → brakeLevel
// ============================================================================

TEST_F(DemoInputProviderTest, SetBrake_PropagatesToBrakeLevel) {
    ASSERT_TRUE(provider_->Initialize());

    provider_->setBrake(1.0);
    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_DOUBLE_EQ(input.brakeLevel, 1.0);
}

TEST_F(DemoInputProviderTest, Brake_StaysWhileHeld) {
    ASSERT_TRUE(provider_->Initialize());

    provider_->setBrake(1.0);
    for (int i = 0; i < 20; ++i) {
        EngineInput input = provider_->OnUpdateSimulation(0.016);
        EXPECT_DOUBLE_EQ(input.brakeLevel, 1.0)
            << "brakeLevel should be 1.0 on poll " << (i + 1) << " while held";
    }
}

TEST_F(DemoInputProviderTest, Brake_ReleasesWhenReset) {
    ASSERT_TRUE(provider_->Initialize());

    provider_->setBrake(1.0);
    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_DOUBLE_EQ(input.brakeLevel, 1.0);

    // Simulate keyboard provider resetting brake each frame (key released)
    provider_->setBrake(0.0);
    input = provider_->OnUpdateSimulation(0.016);
    EXPECT_DOUBLE_EQ(input.brakeLevel, 0.0);
}

TEST_F(DemoInputProviderTest, Brake_DefaultIsZero) {
    ASSERT_TRUE(provider_->Initialize());

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_DOUBLE_EQ(input.brakeLevel, 0.0);
}

TEST_F(DemoInputProviderTest, Brake_ThrottleIndependent) {
    ASSERT_TRUE(provider_->Initialize());

    // Apply throttle and brake simultaneously
    rawThrottle_->setThrottleLevel(0.8);
    provider_->setBrake(1.0);

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_DOUBLE_EQ(input.brakeLevel, 1.0);
    // Throttle also flows through (after twin warmup it may vary, but raw value was set)
}

// ============================================================================
// enhanceInput merge-seam tests
//
// enhanceInput is the keyboard/demo path: it receives a base EngineInput
// (keyboard state) and merges the twin's computed gear/clutch diagnostics
// ONTO it. Two fields are dropped by the current merge: couplingIsTorqueConverter
// (Defect A) and vehicleSpeedTargetKmh (Defect B).
//
// These tests drive the twin to RUNNING state (so surfaceCouplingTargets
// populates pinVehicleSpeedTargetKmh), call enhanceInput, and assert the twin's
// values survive the merge into the returned EngineInput.
// ============================================================================

namespace {

// Drive a DemoInputProvider's twin to RUNNING state by feeding it ignition +
// throttle + DRIVE selector over enough frames for the 3s crank fallback to
// expire and the IDLE->RUNNING transition to fire.
void warmTwinToRunning(input::DemoInputProvider& provider, input::DemoThrottleSource* rawThrottle,
                       double dt = 0.016, int maxFrames = 600) {
    // P → R → N → D (3 shift-ups from the default PARK)
    provider.shiftUp(); // P → R
    provider.shiftUp(); // R → N
    provider.shiftUp(); // N → D

    EngineSimStats feedback{};
    feedback.currentRPM = 800.0;

    for (int i = 0; i < maxFrames; ++i) {
        rawThrottle->setThrottleLevel(0.5);
        provider.provideFeedback(feedback);
        double throttle = rawThrottle->pollThrottle();
        input::EngineInput base;
        base.throttle = throttle;
        base.ignition = true;
        base.gearSelector = provider.getGearSelectorState();
        base.brakeLevel = 0.0;
        provider.enhanceInput(base, dt);
    }
}

}  // namespace

// Defect A (red): enhanceInput must pass couplingIsTorqueConverter through from
// the twin. The default demo coupling model is TorqueConverter, so the twin
// surfaces couplingIsTorqueConverter=true. The merge must preserve it.
TEST_F(DemoInputProviderTest, EnhanceInput_PreservesCouplingIsTorqueConverter) {
    ASSERT_TRUE(provider_->Initialize());
    warmTwinToRunning(*provider_, rawThrottle_);

    input::EngineInput base;
    base.throttle = 0.5;
    base.ignition = true;
    base.gearSelector = provider_->getGearSelectorState();
    base.brakeLevel = 0.0;
    // Intentionally set baseInput to the OPPOSITE of what the twin produces,
    // so a bare copy of baseInput would fail the assertion.
    base.couplingIsTorqueConverter = false;

    input::EngineInput result = provider_->enhanceInput(base, 0.016);

    EXPECT_TRUE(result.couplingIsTorqueConverter)
        << "enhanceInput must pass the twin's couplingIsTorqueConverter=true through; "
        << "baseInput had it false so only a twin pass-through can make this true";
}

// Defect A (red, converse): selecting ClutchMap flips couplingIsTorqueConverter
// to false in the twin; enhanceInput must carry that false through too.
TEST_F(DemoInputProviderTest, EnhanceInput_PreservesCouplingIsTorqueConverterFalse) {
    ASSERT_TRUE(provider_->Initialize());
    provider_->setCouplingModel(twin::CouplingModelKind::ClutchMap);
    warmTwinToRunning(*provider_, rawThrottle_);

    input::EngineInput base;
    base.throttle = 0.5;
    base.ignition = true;
    base.gearSelector = provider_->getGearSelectorState();
    base.brakeLevel = 0.0;
    // Opposite of what the twin produces — the merge must overwrite this.
    base.couplingIsTorqueConverter = true;

    input::EngineInput result = provider_->enhanceInput(base, 0.016);

    EXPECT_FALSE(result.couplingIsTorqueConverter)
        << "enhanceInput must pass the twin's couplingIsTorqueConverter=false through";
}

// Defect B (red): enhanceInput must pass vehicleSpeedTargetKmh through from the
// twin. With PIN wheel coupling (the demo default) and the twin in RUNNING,
// surfaceCouplingTargets populates pinVehicleSpeedTargetKmh >= 0 (the CSV road
// speed). The merge must preserve it instead of leaving the baseInput sentinel
// (-1.0), which SimulationLoop gates on >= 0.0 to engage the VehicleSpeedConstraint.
TEST_F(DemoInputProviderTest, EnhanceInput_PreservesVehicleSpeedTargetKmh) {
    // Default: PIN wheel coupling + TorqueConverter model.
    ASSERT_TRUE(provider_->Initialize());
    warmTwinToRunning(*provider_, rawThrottle_);

    input::EngineInput base;
    base.throttle = 0.5;
    base.ignition = true;
    base.gearSelector = provider_->getGearSelectorState();
    base.brakeLevel = 0.0;
    // Sentinel: untouched. A bare copy leaves -1.0; the twin should produce >= 0.
    base.vehicleSpeedTargetKmh = -1.0;

    input::EngineInput result = provider_->enhanceInput(base, 0.016);

    EXPECT_GE(result.vehicleSpeedTargetKmh, 0.0)
        << "enhanceInput must pass the twin's vehicleSpeedTargetKmh (>=0) through; "
        << "baseInput had -1.0 so only a twin pass-through can make this >= 0";
}

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

#ifdef ATG_ENGINE_SIM_TEST_VERBOSE
    printf("\n=== FULL-CHAIN INTEGRATION TEST: 20%% THROTTLE GEARSHIFT ===\n");
    printf("  Profile: ZF 8HP45 (ratios: 4.714, 3.143, 2.106, 1.667, 1.285, 1.000, 0.839, 0.667)\n");
    printf("  Simulation rate: %.1f Hz (dt=%.4f sec)\n", 1.0/dt, dt);
    printf("  Duration: 60 seconds\n\n");
#endif

    for (int i = 0; i < static_cast<int>(60.0 / dt); ++i) {
#ifdef ATG_ENGINE_SIM_TEST_VERBOSE
        double t = i * dt;
#endif

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
#ifdef ATG_ENGINE_SIM_TEST_VERBOSE
            printf("  GEAR CHANGE at t=%.1fs: gear %d → %d, demoSpeed=%.1f kph, engineRPM=%.0f\n",
                   t, lastGear, gear, provider->getDemoRoadSpeedKmh(), feedbackStats.currentRPM);
#endif
        }
        lastGear = gear;

        // Print every 5 seconds
        if (i % static_cast<int>(5.0 / dt) == 0) {
#ifdef ATG_ENGINE_SIM_TEST_VERBOSE
            printf("  t=%5.1fs  gear=%d  demoSpeed=%.1f kph  rpm=%.0f  fbSpeed=%.1f\n",
                   t, gear, provider->getDemoRoadSpeedKmh(), feedbackStats.currentRPM, feedbackStats.vehicleSpeedKmh);
#endif
        }
    }

#ifdef ATG_ENGINE_SIM_TEST_VERBOSE
    printf("\n=== FINAL RESULTS ===\n");
    printf("  Total gear changes: %d\n", gearChanges);
    printf("  Final gear: %d\n", lastGear);
    printf("  Final speed: %.1f kph\n", provider->getDemoRoadSpeedKmh());
    printf("  Final RPM: %.0f\n", feedbackStats.currentRPM);
    printf("======================\n\n");
#endif

    EXPECT_GT(gearChanges, 0) << "Gearbox should shift at least once during 60s at 20% throttle";
    EXPECT_GT(lastGear, 1) << "Final gear should be > 1 after 60s at 20% throttle";
}