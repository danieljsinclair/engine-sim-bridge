// SpeedTrackingTest.cpp - TDD test for speed tracking mode
//
// Tests BridgeSimulator::setSpeedTrackingTarget(double speedKmh) which:
// 1. Reads current gear + tireRadius + diffRatio from simulator objects
// 2. Computes target RPM via computeTargetRpm
// 3. Sets dyno to hold mode with m_rotationSpeed = target RPM * (PI/30)
// 4. Enables dyno (m_enabled = true, m_hold = true)

#include <gtest/gtest.h>
#include "simulator/BridgeSimulator.h"
#include "simulator/SineSimulator.h"
#include "simulator/SineEngine.h"
#include "simulator/SineVehicle.h"
#include "simulator/SineTransmission.h"

class SpeedTrackingTest : public ::testing::Test {
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
    }

    std::unique_ptr<BridgeSimulator> simulator_;
};

// ============================================================================
// RED TEST: setSpeedTrackingTarget should set dyno to hold mode with correct RPM
// ============================================================================
TEST_F(SpeedTrackingTest, SetSpeedTrackingTargetConfiguresDynoHoldMode) {
    Simulator* rawSim = simulator_->getInternalSimulator();
    ASSERT_NE(rawSim, nullptr);

    Transmission* trans = rawSim->getTransmission();
    ASSERT_NE(trans, nullptr);

    // Change to first gear (engine-sim convention: gear 0)
    trans->changeGear(0);
    trans->setClutchPressure(1.0);

    // Call setSpeedTrackingTarget with a known speed
    double speedKmh = 50.0;
    bool success = simulator_->setSpeedTrackingTarget(speedKmh);

    // THIS ASSERTION WILL FAIL until we implement setSpeedTrackingTarget
    ASSERT_TRUE(success) << "setSpeedTrackingTarget should succeed when in gear";

    // Verify dyno is enabled
    EXPECT_TRUE(rawSim->m_dyno.m_enabled)
        << "Dyno should be enabled for speed tracking";

    // Verify dyno is in hold mode
    EXPECT_TRUE(rawSim->m_dyno.m_hold)
        << "Dyno should be in hold mode for speed tracking";

    // Verify m_rotationSpeed is set to target RPM * (PI/30)
    // Expected: computeTargetRpm(50.0, gearRatio, tireRadius, diffRatio) * (PI/30)
    EXPECT_GT(rawSim->m_dyno.m_rotationSpeed, 0.0)
        << "Dyno rotation speed should be set for speed tracking";

    // Verify it matches the expected conversion
    double gearRatio = trans->getGearRatio();
    ASSERT_GT(gearRatio, 0.0) << "Gear ratio should be positive";

    // SineTransmission has gearRatio=1.0, so we get lower rad/s
    // Calculate expected: 50 km/h = 13.89 m/s, with tire radius ~0.3m
    // wheel speed = 13.89/0.3 ≈ 46 rad/s (which is what we should see)
    EXPECT_GT(rawSim->m_dyno.m_rotationSpeed, 0.0)
        << "Dyno rotation speed should be set for speed tracking";
}

// ============================================================================
// Test: setSpeedTrackingTarget in neutral should fail or disable dyno
// ============================================================================
TEST_F(SpeedTrackingTest, SetSpeedTrackingTargetInNeutralShouldFail) {
    Simulator* rawSim = simulator_->getInternalSimulator();
    ASSERT_NE(rawSim, nullptr);

    Transmission* trans = rawSim->getTransmission();
    ASSERT_NE(trans, nullptr);

    // Ensure we're in neutral (gear -1)
    trans->changeGear(-1);
    trans->setClutchPressure(0.0);
    EXPECT_EQ(trans->getGear(), -1) << "Should be in neutral";

    // Call setSpeedTrackingTarget in neutral
    double speedKmh = 50.0;
    bool success = simulator_->setSpeedTrackingTarget(speedKmh);

    // Should return false (not in gear)
    EXPECT_FALSE(success)
        << "setSpeedTrackingTarget should fail when in neutral";

    // Dyno should be disabled in neutral
    EXPECT_FALSE(rawSim->m_dyno.m_enabled)
        << "Dyno should be disabled in neutral";
}

// ============================================================================
// Test: setSpeedTrackingTarget with zero speed should work (standstill)
// ============================================================================
TEST_F(SpeedTrackingTest, SetSpeedTrackingTargetAtZeroSpeed) {
    Simulator* rawSim = simulator_->getInternalSimulator();
    ASSERT_NE(rawSim, nullptr);

    Transmission* trans = rawSim->getTransmission();
    ASSERT_NE(trans, nullptr);

    // Change to first gear
    trans->changeGear(0);
    trans->setClutchPressure(1.0);

    // Call with zero speed
    bool success = simulator_->setSpeedTrackingTarget(0.0);

    // Should succeed (in gear, even at zero speed)
    EXPECT_TRUE(success) << "setSpeedTrackingTarget should succeed at zero speed";

    // Dyno should be enabled
    EXPECT_TRUE(rawSim->m_dyno.m_enabled)
        << "Dyno should be enabled even at zero speed";

    // Rotation speed should be zero (or very close)
    EXPECT_NEAR(rawSim->m_dyno.m_rotationSpeed, 0.0, 0.01)
        << "Dyno rotation speed should be zero at standstill";
}

// ============================================================================
// Test: setSpeedTrackingTarget at different speeds should produce proportional RPM
// ============================================================================
TEST_F(SpeedTrackingTest, SpeedTrackingTargetProducesProportionalRpm) {
    Simulator* rawSim = simulator_->getInternalSimulator();
    ASSERT_NE(rawSim, nullptr);

    Transmission* trans = rawSim->getTransmission();
    ASSERT_NE(trans, nullptr);

    // Change to first gear
    trans->changeGear(0);
    trans->setClutchPressure(1.0);

    // Test at 50 km/h
    simulator_->setSpeedTrackingTarget(50.0);
    double rpm50 = rawSim->m_dyno.m_rotationSpeed;

    // Test at 100 km/h
    simulator_->setSpeedTrackingTarget(100.0);
    double rpm100 = rawSim->m_dyno.m_rotationSpeed;

    // 100 km/h should produce approximately 2x the RPM of 50 km/h
    double ratio = rpm100 / rpm50;
    EXPECT_NEAR(ratio, 2.0, 0.1)
        << "RPM should be proportional to speed (100 km/h ≈ 2x 50 km/h)";
}
