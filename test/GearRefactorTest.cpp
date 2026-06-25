// GearRefactorTest.cpp - TDD tests for gear method refactor
//
// REFACTOR GOAL:
// 1. changeGear() should use bridge convention throughout (like setGear/getGear)
// 2. setGear() and changeGear() should take optional clutch-pressure parameter
// 3. Default clutch: 1.0 for any forward gear, 0.0 for neutral
//
// These tests MUST FAIL before the refactor and PASS after.

#include <gtest/gtest.h>
#include "simulator/BridgeSimulator.h"
#include "simulator/SineSimulator.h"
#include "simulator/SineEngine.h"
#include "simulator/SineVehicle.h"
#include "simulator/SineTransmission.h"
#include "simulator/GearConventions.h"

class GearRefactorTest : public ::testing::Test {
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

        rawSim_ = simulator_->getInternalSimulator();
        ASSERT_NE(rawSim_, nullptr) << "Failed to get raw simulator";
        trans_ = rawSim_->getTransmission();
        ASSERT_NE(trans_, nullptr) << "Failed to get transmission";
    }

    std::unique_ptr<BridgeSimulator> simulator_;
    Simulator* rawSim_ = nullptr;
    Transmission* trans_ = nullptr;
};

// ============================================================================
// TASK 1: changeGear should use bridge convention
// ============================================================================

// RED TEST: Proves that changeGear currently manipulates raw engine-sim indices
// After refactor, this should work in bridge convention (getGear returns bridge values)
TEST_F(GearRefactorTest, ChangeGearUsesBridgeConvention) {
    // Start in neutral (bridge convention: 0 = neutral)
    int startGear = simulator_->getGear();
    EXPECT_EQ(startGear, 0) << "Initial gear should be 0 (neutral in bridge convention)";

    // Shift up by 1 (from neutral to first)
    // In bridge convention: 0 -> 1
    // In engine-sim convention: -1 -> 0
    bool success = simulator_->changeGear(1);
    ASSERT_TRUE(success) << "changeGear(1) should succeed";

    // After refactor, getGear should return 1 (first gear in bridge convention)
    // Before refactor, this might return engine-sim index 0
    int afterUpshift = simulator_->getGear();
    EXPECT_EQ(afterUpshift, 1) << "After +1 from neutral, gear should be 1 (bridge convention)";

    // Verify consistency: getGear should match setGear/getGear convention
    // i.e., if we setGear(1), we should read back 1
    simulator_->setGear(1);
    int setGearResult = simulator_->getGear();
    EXPECT_EQ(setGearResult, 1) << "setGear(1) should make getGear return 1 (bridge convention)";

    // changeGear should follow the same convention
    simulator_->setGear(0);  // Back to neutral
    simulator_->changeGear(1);  // +1 to first gear
    int changeGearResult = simulator_->getGear();
    EXPECT_EQ(changeGearResult, 1) << "changeGear(+1) should result in gear 1 (bridge convention)";
}

// RED TEST: changeGear(+1) multiple times should follow bridge convention
TEST_F(GearRefactorTest, ChangeGearMultipleUpshiftsBridgeConvention) {
    // Note: SineTransmission only has 1 gear, so we can't test multi-gear behavior
    // This test verifies the convention within the available range
    simulator_->setGear(0);  // Neutral

    // Neutral (0) -> First (1)
    EXPECT_TRUE(simulator_->changeGear(1));
    EXPECT_EQ(simulator_->getGear(), 1) << "Upshift: gear should be 1 (bridge convention)";

    // Try to upshift again (should clamp at gear 1 since only 1 gear available)
    simulator_->changeGear(1);
    EXPECT_EQ(simulator_->getGear(), 1) << "Clamped at max gear (1 in bridge convention)";
}

// RED TEST: changeGear(-1) should work in bridge convention
TEST_F(GearRefactorTest, ChangeGearDownshiftBridgeConvention) {
    simulator_->setGear(1);  // First gear (bridge convention)

    EXPECT_TRUE(simulator_->changeGear(-1));
    EXPECT_EQ(simulator_->getGear(), 0) << "Downshift to neutral: gear should be 0 (bridge convention)";

    // Further downshifts should stay at neutral
    simulator_->changeGear(-1);
    EXPECT_EQ(simulator_->getGear(), 0) << "Clamped at neutral (0)";
}

// RED TEST: changeGear should clamp at neutral (0), not -1 (engine-sim neutral)
TEST_F(GearRefactorTest, ChangeGearClampsAtNeutral) {
    simulator_->setGear(1);  // First gear

    simulator_->changeGear(-1);  // Downshift to neutral
    EXPECT_EQ(simulator_->getGear(), 0) << "Should be in neutral (0)";

    // Further downshifts should stay at neutral
    simulator_->changeGear(-1);
    EXPECT_EQ(simulator_->getGear(), 0) << "Should clamp at neutral (0)";
}

// ============================================================================
// TASK 2: Default clutch pressure behavior
// ============================================================================

// RED TEST: setGear(forward) should default to clutch 1.0
TEST_F(GearRefactorTest, SetGearForwardDefaultsToClutchEngaged) {
    // After refactor, setGear(1) should set clutch to 1.0
    simulator_->setGear(1);  // First gear

    double clutchPressure = trans_->getClutchPressure();
    EXPECT_NEAR(clutchPressure, 1.0, 0.01)
        << "setGear(1) should default clutch to 1.0 (engaged)";
}

// RED TEST: setGear(neutral) should default to clutch 0.0
TEST_F(GearRefactorTest, SetGearNeutralDefaultsToClutchDisengaged) {
    simulator_->setGear(0);  // Neutral

    double clutchPressure = trans_->getClutchPressure();
    EXPECT_NEAR(clutchPressure, 0.0, 0.01)
        << "setGear(0) should default clutch to 0.0 (disengaged)";
}

// RED TEST: changeGear to forward should default to clutch 1.0
TEST_F(GearRefactorTest, ChangeGearForwardDefaultsToClutchEngaged) {
    simulator_->setGear(0);  // Start in neutral
    EXPECT_NEAR(trans_->getClutchPressure(), 0.0, 0.01);

    simulator_->changeGear(1);  // Upshift to first

    double clutchPressure = trans_->getClutchPressure();
    EXPECT_NEAR(clutchPressure, 1.0, 0.01)
        << "changeGear(+1) should default clutch to 1.0 for forward gear";
}

// RED TEST: changeGear to neutral should default to clutch 0.0
TEST_F(GearRefactorTest, ChangeGearToNeutralDefaultsToClutchDisengaged) {
    simulator_->setGear(1);  // First gear

    simulator_->changeGear(-1);  // Downshift to neutral

    double clutchPressure = trans_->getClutchPressure();
    EXPECT_NEAR(clutchPressure, 0.0, 0.01)
        << "changeGear(-1 to neutral) should default clutch to 0.0 (disengaged)";
}

// ============================================================================
// TASK 2: Explicit clutch pressure parameter
// ============================================================================

// After refactor: setGear(gear, clutchPressure) should honor the explicit value
TEST_F(GearRefactorTest, SetGearExplicitClutchPressure) {
    // Set gear with explicit clutch pressure
    simulator_->setGear(1, 0.5);

    double clutchPressure = trans_->getClutchPressure();
    EXPECT_NEAR(clutchPressure, 0.5, 0.01)
        << "setGear(1, 0.5) should set clutch to 0.5";
}

// After refactor: Explicit 0.0 clutch should be honored even for forward gear
TEST_F(GearRefactorTest, SetGearForwardWithZeroClutch) {
    simulator_->setGear(2, 0.0);  // Second gear, clutch disengaged

    double clutchPressure = trans_->getClutchPressure();
    EXPECT_NEAR(clutchPressure, 0.0, 0.01)
        << "setGear(2, 0.0) should honor explicit 0.0 clutch";
}

// After refactor: Explicit 1.0 clutch should be honored even for neutral
TEST_F(GearRefactorTest, SetGearNeutralWithFullClutch) {
    simulator_->setGear(0, 1.0);  // Neutral, clutch engaged

    double clutchPressure = trans_->getClutchPressure();
    EXPECT_NEAR(clutchPressure, 1.0, 0.01)
        << "setGear(0, 1.0) should honor explicit 1.0 clutch";
}
