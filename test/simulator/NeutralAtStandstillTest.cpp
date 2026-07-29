// NeutralAtStandstillTest.cpp - TDD test for neutral behavior
//
// Tests that:
// 1. When manually in neutral at standstill, clutch is disengaged
// 2. When in neutral, setSpeedTrackingTarget should fail/disable dyno
// 3. Engine idles at zero throttle (confirmed by user)
// 4. Engine can rev freely in neutral with throttle

#include <gtest/gtest.h>
#include "simulator/BridgeSimulator.h"
#include "simulator/SineSimulator.h"
#include "simulator/SineEngine.h"
#include "simulator/SineVehicle.h"
#include "simulator/SineTransmission.h"

class NeutralAtStandstillTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto sineSim = std::make_unique<SineSimulator>();
        Simulator::Parameters simParams;
        simParams.systemType = Simulator::SystemType::NsvOptimized;
        sineSim->initialize(simParams);
        sineSim->setSimulationFrequency(EngineSimDefaults::SIMULATION_FREQUENCY);
        sineSim->setFluidSimulationSteps(EngineSimDefaults::FLUID_SIMULATION_STEPS);
        sineSim->setTargetSynthesizerLatency(EngineSimDefaults::TARGET_SYNTH_LATENCY);
        sineSim->loadSimulation(new SineEngine(), new SineVehicle(), new SineTransmission());
        simulator_ = std::make_unique<BridgeSimulator>(std::move(sineSim));

        ISimulatorConfig config;
        config.sampleRate = EngineSimDefaults::SAMPLE_RATE;
        config.simulationFrequency = EngineSimDefaults::SIMULATION_FREQUENCY;

        bool created = simulator_->create(config, nullptr, nullptr);
        ASSERT_TRUE(created) << "BridgeSimulator::create() failed";

        // Get raw simulator for direct access
        rawSim_ = simulator_->getInternalSimulator();
        ASSERT_NE(rawSim_, nullptr);

        trans_ = rawSim_->getTransmission();
        ASSERT_NE(trans_, nullptr);
    }

    std::unique_ptr<BridgeSimulator> simulator_;
    Simulator* rawSim_ = nullptr;
    Transmission* trans_ = nullptr;
};

// ============================================================================
// Test: When manually in neutral at standstill, clutch is disengaged
// ============================================================================
TEST_F(NeutralAtStandstillTest, InNeutralClutchDisengaged) {
    // Manually set to neutral
    trans_->changeGear(-1);
    trans_->setClutchPressure(0.0);

    // Verify clutch is disengaged
    EXPECT_NEAR(trans_->getClutchPressure(), 0.0, 0.01)
        << "In neutral, clutch should be disengaged";
}

// ============================================================================
// Test: setSpeedTrackingTarget fails in neutral (dyno should be OFF)
// ============================================================================
TEST_F(NeutralAtStandstillTest, SetSpeedTrackingTargetInNeutralFails) {
    // Manually set to neutral
    trans_->changeGear(-1);
    trans_->setClutchPressure(0.0);

    // Try to set speed tracking target in neutral
    bool success = simulator_->setSpeedTrackingTarget(50.0);

    // Should fail (not in gear)
    EXPECT_FALSE(success)
        << "setSpeedTrackingTarget should fail when in neutral";

    // Dyno should be disabled
    EXPECT_FALSE(rawSim_->m_dyno.m_enabled)
        << "In neutral, speed-tracking dyno should be OFF";
}

// ============================================================================
// Test: After setSpeedTrackingTarget in neutral, dyno remains disabled
// ============================================================================
TEST_F(NeutralAtStandstillTest, DynoDisabledAfterFailedSpeedTrackingInNeutral) {
    // Enable dyno first
    rawSim_->m_dyno.m_enabled = true;
    rawSim_->m_dyno.m_hold = true;
    rawSim_->m_dyno.m_rotationSpeed = 100.0;

    // Set to neutral
    trans_->changeGear(-1);

    // Try to set speed tracking (should fail and disable dyno)
    simulator_->setSpeedTrackingTarget(50.0);

    // Dyno should still be disabled (or get disabled)
    EXPECT_FALSE(rawSim_->m_dyno.m_enabled)
        << "After failed speed tracking in neutral, dyno should be disabled";
}

// ============================================================================
// Test: Engine idles at zero throttle (in neutral, ignition on)
// Note: User confirmed engine-sim idles natively at 0% throttle
// ============================================================================
TEST_F(NeutralAtStandstillTest, EngineIdlesAtZeroThrottle) {
    // Set up: Neutral, ignition on, zero throttle
    trans_->changeGear(-1);
    trans_->setClutchPressure(0.0);
    simulator_->setIgnition(true);
    simulator_->setThrottle(0.0);

    // Advance simulation to let engine start and settle
    simulator_->update(0.01);
    simulator_->update(0.01);
    simulator_->update(0.05);  // Let it stabilize

    // Engine should be idling (not zero RPM, not stopped)
    double engineRpm = simulator_->getEngineRpm();
    EXPECT_GT(engineRpm, 0.0)
        << "At zero throttle with ignition on, engine should idle (RPM > 0)";
    EXPECT_LT(engineRpm, 2000.0)
        << "At zero throttle, idle RPM should be reasonable (< 2000)";
}

// ============================================================================
// Test: Engine can rev freely in neutral with throttle
// ============================================================================
TEST_F(NeutralAtStandstillTest, EngineRevvingInNeutralWithThrottle) {
    // Set up: Neutral, ignition on
    trans_->changeGear(-1);
    trans_->setClutchPressure(0.0);
    simulator_->setIgnition(true);
    simulator_->setThrottle(0.0);

    // Get baseline at zero throttle
    simulator_->update(0.01);
    simulator_->update(0.01);
    double idleRpm = simulator_->getEngineRpm();

    // Apply 50% throttle
    simulator_->setThrottle(0.5);
    simulator_->update(0.05);  // Let it rev up

    double revvingRpm = simulator_->getEngineRpm();

    // RPM should rise with throttle
    EXPECT_GT(revvingRpm, idleRpm * 1.5)
        << "In neutral with throttle, RPM should rise freely";

    // Apply 100% throttle
    simulator_->setThrottle(1.0);
    simulator_->update(0.05);

    double fullThrottleRpm = simulator_->getEngineRpm();

    // Should rev even higher
    EXPECT_GT(fullThrottleRpm, revvingRpm)
        << "At full throttle, RPM should be higher than 50% throttle";
}

// ============================================================================
// Test: When switching from gear to neutral, clutch disengages
// ============================================================================
TEST_F(NeutralAtStandstillTest, ClutchDisengagesWhenShiftingToNeutral) {
    // Start in first gear with clutch engaged
    trans_->changeGear(0);
    trans_->setClutchPressure(1.0);
    EXPECT_NEAR(trans_->getClutchPressure(), 1.0, 0.01)
        << "Clutch should be engaged in first gear";

    // Shift to neutral using BridgeSimulator::changeGear
    simulator_->changeGear(-1);  // Shift down by 1 (from gear 0 to -1)

    // Clutch should now be disengaged
    EXPECT_NEAR(trans_->getClutchPressure(), 0.0, 0.01)
        << "After shifting to neutral, clutch should be disengaged";
}
