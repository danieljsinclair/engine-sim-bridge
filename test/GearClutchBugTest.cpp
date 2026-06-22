// GearClutchBugTest.cpp - TDD test for gear 1 clutch bug
//
// BUG: BridgeSimulator::changeGear() at line 240 has the condition:
//   trans->setClutchPressure(newGear > 0 ? 1.0 : 0.0);
// In engine-sim convention, gear 0 is the FIRST forward gear (indices 0 to gearCount-1).
// This means when newGear=0 (first gear), the condition evaluates to false,
// setting clutch pressure to 0.0 (disengaged), making first gear behave like neutral.
//
// FIX: Change condition to (newGear >= 0) so all forward gears (0 to gearCount-1)
// get clutch pressure 1.0 (engaged), while only neutral (-1) gets 0.0.
//
// This test MUST FAIL with the buggy code and PASS once fixed.

#include <gtest/gtest.h>
#include "simulator/BridgeSimulator.h"
#include "simulator/SineSimulator.h"
#include "simulator/SineEngine.h"
#include "simulator/SineVehicle.h"
#include "simulator/SineTransmission.h"

class GearClutchBugTest : public ::testing::Test {
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
// RED TEST: Proves that first gear (engine-sim gear index 0) currently
// gets clutch pressure 0.0 (disengaged) instead of 1.0 (engaged).
// ============================================================================
TEST_F(GearClutchBugTest, FirstGearShouldHaveClutchEngaged) {
    Simulator* rawSim = simulator_->getInternalSimulator();
    ASSERT_NE(rawSim, nullptr) << "Failed to get raw simulator";

    Transmission* trans = rawSim->getTransmission();
    ASSERT_NE(trans, nullptr) << "Failed to get transmission";

    // Change to first gear (engine-sim convention: gear 0 is first forward gear)
    bool success = simulator_->changeGear(1);  // +1 gear delta from neutral to first
    ASSERT_TRUE(success) << "changeGear(1) should succeed";

    // Verify we're in gear 0 (first gear in engine-sim convention)
    int currentGear = trans->getGear();
    EXPECT_EQ(currentGear, 0) << "Should be in gear 0 (first gear)";

    // THIS ASSERTION FAILS with the bug: clutch pressure is 0.0 instead of 1.0
    double clutchPressure = trans->getClutchPressure();
    EXPECT_NEAR(clutchPressure, 1.0, 0.01)
        << "First gear should have clutch ENGAGED (pressure 1.0), not disengaged (0.0). "
        << "Bug: changeGear uses 'newGear > 0' but should use 'newGear >= 0' "
        << "because gear 0 is first gear in engine-sim convention.";
}

// ============================================================================
// Test that neutral (gear -1) correctly has clutch disengaged
// ============================================================================
TEST_F(GearClutchBugTest, NeutralShouldHaveClutchDisengaged) {
    Simulator* rawSim = simulator_->getInternalSimulator();
    ASSERT_NE(rawSim, nullptr);

    Transmission* trans = rawSim->getTransmission();
    ASSERT_NE(trans, nullptr);

    // Start in neutral (initial state after loadSimulation)
    int currentGear = trans->getGear();
    EXPECT_EQ(currentGear, -1) << "Initial gear should be -1 (neutral)";

    double clutchPressure = trans->getClutchPressure();
    EXPECT_NEAR(clutchPressure, 0.0, 0.01)
        << "Neutral should have clutch disengaged (pressure 0.0)";
}

// ============================================================================
// Test that all forward gears (0 through gearCount-1) have clutch engaged
// ============================================================================
TEST_F(GearClutchBugTest, AllForwardGearsShouldHaveClutchEngaged) {
    Simulator* rawSim = simulator_->getInternalSimulator();
    ASSERT_NE(rawSim, nullptr);

    Transmission* trans = rawSim->getTransmission();
    ASSERT_NE(trans, nullptr);

    int gearCount = trans->getGearCount();
    ASSERT_GT(gearCount, 0) << "Transmission should have at least one gear";

    // Test each forward gear
    for (int i = 0; i < gearCount; ++i) {
        // Change to this gear directly using setGear (not changeGear delta)
        trans->changeGear(i);
        trans->setClutchPressure(i >= 0 ? 1.0 : 0.0);  // Expected behavior after fix

        int currentGear = trans->getGear();
        EXPECT_EQ(currentGear, i) << "Should be in gear " << i;

        double clutchPressure = trans->getClutchPressure();
        EXPECT_NEAR(clutchPressure, 1.0, 0.01)
            << "Forward gear " << i << " should have clutch ENGAGED (pressure 1.0)";
    }
}
