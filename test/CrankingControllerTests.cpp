// CrankingControllerTests.cpp - TDD tests for cranking state management
// Phase is stored on ICombustionEngine (single source of truth).
// These tests assert correct behaviour through the engine's phase.

#include <gtest/gtest.h>
#include "simulation/CrankingController.h"
#include "simulator/ICombustionEngine.h"
#include "simulator/EngineSimTypes.h"

namespace {

// Minimal ICombustionEngine stub that records calls and owns phase
class StubEngine : public ICombustionEngine {
public:
    double throttle_ = 0.0;
    bool ignition_ = false;
    bool starterMotor_ = false;
    EngineSimStats stats_{};
    EnginePhase phase_ = EnginePhase::Stopped;

    bool create(const ISimulatorConfig&, ILogging*, telemetry::ITelemetryWriter*) override { return true; }
    void destroy() override {}
    std::string getLastError() const override { return {}; }
    const char* getName() const override { return "Stub"; }
    void update(double) override {}
    EngineSimStats getStats() const override { return stats_; }
    void setThrottle(double pos) override { throttle_ = pos; }
    bool renderOnDemand(float*, int32_t, int32_t*) override { return true; }
    bool readAudioBuffer(float*, int32_t, int32_t*) override { return true; }
    bool start() override { return true; }
    void stop() override {}
    int getSimulationFrequency() const override { return 10000; }
    void setIgnition(bool on) override { ignition_ = on; }
    void setStarterMotor(bool on) override { starterMotor_ = on; }
    EnginePhase getEnginePhase() const override { return phase_; }
    void setEnginePhase(EnginePhase phase) override { phase_ = phase; }
};

} // namespace

// ============================================================================
// Engage starter from Stopped -> Cranking
// ============================================================================

TEST(CrankingControllerTest, EngageStarterFromStopped_EntersCrankingAndEngagesMotor) {
    CrankingController controller;
    StubEngine engine;

    controller.engageStarter(engine, true);

    EXPECT_EQ(engine.getEnginePhase(), EnginePhase::Cranking);
    EXPECT_TRUE(engine.starterMotor_);
}

TEST(CrankingControllerTest, EngageStarterFromStopped_IgnoresButtonFalse) {
    CrankingController controller;
    StubEngine engine;

    controller.engageStarter(engine, false);

    EXPECT_EQ(engine.getEnginePhase(), EnginePhase::Stopped);
    EXPECT_FALSE(engine.starterMotor_);
}

// ============================================================================
// Toggle: pressing starter while Cranking stops the starter
// ============================================================================

TEST(CrankingControllerTest, EngageStarterWhileCranking_StopsStarterAndReturnsToStopped) {
    CrankingController controller;
    StubEngine engine;

    controller.engageStarter(engine, true);
    ASSERT_EQ(engine.getEnginePhase(), EnginePhase::Cranking);

    controller.engageStarter(engine, true);

    EXPECT_EQ(engine.getEnginePhase(), EnginePhase::Stopped);
    EXPECT_FALSE(engine.starterMotor_);
}

// ============================================================================
// Reset: controller must reset when preparing for a new engine
// Hot-swap creates a fresh engine; the controller must not carry stale state.
// Reset does NOT change phase — that's the engine's job.
// ============================================================================

TEST(CrankingControllerTest, Reset_ClearsInternalCrankingState) {
    CrankingController controller;
    StubEngine engine;

    controller.engageStarter(engine, true);
    ASSERT_EQ(engine.getEnginePhase(), EnginePhase::Cranking);

    controller.reset();

    // Phase stays on the engine — reset only clears ticks/baseline
    EXPECT_EQ(engine.getEnginePhase(), EnginePhase::Cranking);
}

TEST(CrankingControllerTest, Reset_ClearsBaseline) {
    // After reset, the controller should build a fresh exhaust flow baseline
    // when cranking the new engine. Stale baseline from the old engine would
    // make engineCaught() produce wrong results.
    CrankingController controller;
    StubEngine engine;

    controller.engageStarter(engine, true);
    engine.stats_.exhaustFlow = 0.8;
    for (int i = 0; i < 15; ++i) {
        controller.step(engine, 0.5, true, nullptr);
    }

    controller.reset();

    // Simulate cranking a new engine from scratch
    StubEngine newEngine;
    controller.engageStarter(newEngine, true);
    newEngine.stats_.exhaustFlow = 0.2;

    // First step should be building baseline (ticks_ = 1)
    auto state = controller.step(newEngine, 0.5, true, nullptr);
    EXPECT_EQ(state.phase, EnginePhase::Cranking);
    EXPECT_TRUE(state.starterEngaged);
}

// ============================================================================
// Step returns correct throttle for cranking phase
// ============================================================================

TEST(CrankingControllerTest, StepWhileCranking_ReturnsCrankingThrottle) {
    CrankingController controller;
    StubEngine engine;

    controller.engageStarter(engine, true);

    auto state = controller.step(engine, 0.2, true, nullptr);

    EXPECT_GT(state.startingThrottle, 0.2);  // Cranking override > user throttle
    EXPECT_EQ(state.phase, EnginePhase::Cranking);
    EXPECT_TRUE(state.starterEngaged);
}

// ============================================================================
// Engine catches: exhaust flow spike + RPM above threshold + ignition on
// ============================================================================

TEST(CrankingControllerTest, EngineCatches_TransitionsToRunning) {
    CrankingController controller;
    StubEngine engine;

    controller.engageStarter(engine, true);

    // Build baseline with low exhaust flow
    engine.stats_.exhaustFlow = 0.1;
    for (int i = 0; i < 12; ++i) {
        controller.step(engine, 0.5, true, nullptr);
    }

    // Simulate engine catching: high exhaust flow + RPM above MIN_CATCH_RPM
    engine.stats_.exhaustFlow = 1.0;   // Well above baseline * 2.0
    engine.stats_.currentRPM = 800.0;  // Above MIN_CATCH_RPM (500)
    engine.ignition_ = true;

    auto state = controller.step(engine, 0.5, true, nullptr);

    EXPECT_EQ(engine.getEnginePhase(), EnginePhase::Running);
    EXPECT_EQ(state.phase, EnginePhase::Running);
    EXPECT_FALSE(state.starterEngaged);  // Starter disengages on catch
    EXPECT_FALSE(engine.starterMotor_);
}

// ============================================================================
// Engine does NOT catch without ignition
// ============================================================================

TEST(CrankingControllerTest, EngineDoesNotCatch_WithoutIgnition) {
    CrankingController controller;
    StubEngine engine;

    controller.engageStarter(engine, true);

    engine.stats_.exhaustFlow = 0.1;
    for (int i = 0; i < 12; ++i) {
        controller.step(engine, 0.5, true, nullptr);
    }

    // High exhaust + RPM but ignition OFF
    engine.stats_.exhaustFlow = 1.0;
    engine.stats_.currentRPM = 800.0;

    auto state = controller.step(engine, 0.5, false, nullptr);
    EXPECT_EQ(state.phase, EnginePhase::Cranking);

    EXPECT_EQ(engine.getEnginePhase(), EnginePhase::Cranking);  // Still cranking
}

// ============================================================================
// Hot-swap scenario: swapping engines mid-crank
// The controller must reset and re-engage the starter on the new engine.
// ============================================================================

TEST(CrankingControllerTest, HotSwapWhileCranking_ReEngagesStarterOnNewEngine) {
    CrankingController controller;
    StubEngine oldEngine;

    // Engine is cranking
    controller.engageStarter(oldEngine, true);
    ASSERT_EQ(oldEngine.getEnginePhase(), EnginePhase::Cranking);
    ASSERT_TRUE(oldEngine.starterMotor_);

    // Hot-swap: reset controller for fresh engine
    controller.reset();
    StubEngine newEngine;

    // Re-engage starter on new engine
    controller.engageStarter(newEngine, true);

    EXPECT_EQ(newEngine.getEnginePhase(), EnginePhase::Cranking);
    EXPECT_TRUE(newEngine.starterMotor_);  // Starter actually engaged on new engine

    // Step the new engine — should build fresh baseline
    newEngine.stats_.exhaustFlow = 0.2;
    auto state = controller.step(newEngine, 0.5, true, nullptr);
    EXPECT_EQ(state.phase, EnginePhase::Cranking);
    EXPECT_TRUE(state.starterEngaged);
}

// ============================================================================
// Hot-swap scenario: engine was Running, swap should auto-crank new engine
// ============================================================================

TEST(CrankingControllerTest, HotSwapWhileRunning_AutoCranksNewEngine) {
    CrankingController controller;
    StubEngine oldEngine;

    // Get engine to Running state
    controller.engageStarter(oldEngine, true);
    oldEngine.setEnginePhase(EnginePhase::Running);
    ASSERT_EQ(oldEngine.getEnginePhase(), EnginePhase::Running);

    // Hot-swap: reset + engage starter on new engine
    controller.reset();
    StubEngine newEngine;
    controller.engageStarter(newEngine, true);

    EXPECT_EQ(newEngine.getEnginePhase(), EnginePhase::Cranking);
    EXPECT_TRUE(newEngine.starterMotor_);
}

// ============================================================================
// Hot-swap scenario: engine was Stopped, new engine should stay Stopped
// ============================================================================

TEST(CrankingControllerTest, HotSwapWhileStopped_NewEngineStaysStopped) {
    CrankingController controller;
    StubEngine oldEngine;

    ASSERT_EQ(oldEngine.getEnginePhase(), EnginePhase::Stopped);

    // Hot-swap: reset, do NOT engage starter
    controller.reset();
    StubEngine newEngine;

    EXPECT_EQ(newEngine.getEnginePhase(), EnginePhase::Stopped);
    EXPECT_FALSE(newEngine.starterMotor_);
}

// ============================================================================
// Running -> Stopping when ignition off
// ============================================================================

TEST(CrankingControllerTest, RunningWithIgnitionOff_TransitionsToStopping) {
    CrankingController controller;
    StubEngine engine;

    engine.setEnginePhase(EnginePhase::Running);
    engine.stats_.currentRPM = 3000.0;

    auto state = controller.step(engine, 0.5, false, nullptr);

    EXPECT_EQ(engine.getEnginePhase(), EnginePhase::Stopping);
    EXPECT_EQ(state.phase, EnginePhase::Stopping);
}

// ============================================================================
// Stopping -> Stopped when RPM drops below threshold
// ============================================================================

TEST(CrankingControllerTest, StoppingWithLowRPM_TransitionsToStopped) {
    CrankingController controller;
    StubEngine engine;

    engine.setEnginePhase(EnginePhase::Running);
    engine.stats_.currentRPM = 3000.0;
    controller.step(engine, 0.5, false, nullptr);  // -> Stopping
    ASSERT_EQ(engine.getEnginePhase(), EnginePhase::Stopping);

    engine.stats_.currentRPM = 2.0;  // Below STOPPED_RPM (5.0)
    auto state = controller.step(engine, 0.5, false, nullptr);

    EXPECT_EQ(engine.getEnginePhase(), EnginePhase::Stopped);
    EXPECT_EQ(state.phase, EnginePhase::Stopped);
}

// ============================================================================
// Rollover: hot-swap with drivetrain momentum
// No starter motor, RPM-only detection, uses user throttle
// ============================================================================

TEST(CrankingControllerTest, Rollover_TransitionsToRunning_WhenRPMAboveThreshold) {
    CrankingController controller;
    StubEngine engine;

    engine.setEnginePhase(EnginePhase::Rollover);
    engine.stats_.currentRPM = 600.0;

    auto state = controller.step(engine, 0.3, true, nullptr);

    EXPECT_EQ(engine.getEnginePhase(), EnginePhase::Running);
    EXPECT_EQ(state.phase, EnginePhase::Running);
    EXPECT_FALSE(state.starterEngaged);
    EXPECT_DOUBLE_EQ(state.startingThrottle, 0.3);  // User throttle, not cranking override
}

TEST(CrankingControllerTest, Rollover_StaysRollover_WhenRPMTooLow) {
    CrankingController controller;
    StubEngine engine;

    engine.setEnginePhase(EnginePhase::Rollover);
    engine.stats_.currentRPM = 100.0;  // Below MIN_CATCH_RPM

    auto state = controller.step(engine, 0.3, true, nullptr);

    EXPECT_EQ(engine.getEnginePhase(), EnginePhase::Rollover);
    EXPECT_EQ(state.phase, EnginePhase::Rollover);
}

TEST(CrankingControllerTest, Rollover_DoesNotCatch_WithoutIgnition) {
    CrankingController controller;
    StubEngine engine;

    engine.setEnginePhase(EnginePhase::Rollover);
    engine.stats_.currentRPM = 600.0;

    auto state = controller.step(engine, 0.3, false, nullptr);
    EXPECT_EQ(state.phase, EnginePhase::Rollover);

    EXPECT_EQ(engine.getEnginePhase(), EnginePhase::Rollover);
}

TEST(CrankingControllerTest, Rollover_FallsBackToCranking_AtZeroRPM) {
    CrankingController controller;
    StubEngine engine;

    engine.setEnginePhase(EnginePhase::Rollover);
    engine.stats_.currentRPM = 0.0;

    for (int i = 0; i < 3; ++i) {
        controller.step(engine, 0.3, true, nullptr);
    }

    // After ROLLOVER_FALLBACK_TICKS (3) at 0 RPM, should transition to Cranking
    auto state = controller.step(engine, 0.3, true, nullptr);
    EXPECT_EQ(engine.getEnginePhase(), EnginePhase::Cranking);
    EXPECT_TRUE(engine.starterMotor_);
    EXPECT_TRUE(state.starterEngaged);
}

TEST(CrankingControllerTest, Rollover_UsesUserThrottle_NotCrankingOverride) {
    CrankingController controller;
    StubEngine engine;

    engine.setEnginePhase(EnginePhase::Rollover);
    engine.stats_.currentRPM = 100.0;  // Below catch threshold, stays in Rollover

    auto state = controller.step(engine, 0.1, true, nullptr);

    EXPECT_DOUBLE_EQ(state.startingThrottle, 0.1);  // User's throttle, not 0.55
}

TEST(CrankingControllerTest, EngageStarterFromRollover_TransitionsToCranking) {
    CrankingController controller;
    StubEngine engine;

    engine.setEnginePhase(EnginePhase::Rollover);

    controller.engageStarter(engine, true);

    EXPECT_EQ(engine.getEnginePhase(), EnginePhase::Cranking);
    EXPECT_TRUE(engine.starterMotor_);
}

TEST(CrankingControllerTest, Rollover_DoesNotFallBackImmediately_ZeroRPMTick1) {
    CrankingController controller;
    StubEngine engine;

    engine.setEnginePhase(EnginePhase::Rollover);
    engine.stats_.currentRPM = 0.0;

    // First tick at 0 RPM — should NOT fall back immediately
    auto state = controller.step(engine, 0.3, true, nullptr);
    EXPECT_EQ(state.phase, EnginePhase::Rollover);
    EXPECT_EQ(engine.getEnginePhase(), EnginePhase::Rollover);
}

// ============================================================================
// Pure method tests (Step 3) - verify TransitionDecision return values
// ============================================================================

TEST(CrankingControllerTest, EngageStarterPure_FromStopped_ReturnsCrankingDecision) {
    CrankingController controller;
    StubEngine engine;

    auto decision = controller.engageStarterPure(engine, true);

    EXPECT_EQ(decision.targetPhase, EnginePhase::Cranking);
    EXPECT_TRUE(decision.starterMotor);
    EXPECT_TRUE(decision.isTransition);
    EXPECT_EQ(decision.effectiveThrottle, 0.0);
}

TEST(CrankingControllerTest, EngageStarterPure_ButtonFalse_ReturnsNoTransition) {
    CrankingController controller;
    StubEngine engine;

    auto decision = controller.engageStarterPure(engine, false);

    EXPECT_EQ(decision.targetPhase, EnginePhase::Stopped);
    EXPECT_FALSE(decision.starterMotor);
    EXPECT_FALSE(decision.isTransition);
}

TEST(CrankingControllerTest, EngageStarterPure_FromCranking_ReturnsStoppedDecision) {
    CrankingController controller;
    StubEngine engine;
    engine.phase_ = EnginePhase::Cranking;

    auto decision = controller.engageStarterPure(engine, true);

    EXPECT_EQ(decision.targetPhase, EnginePhase::Stopped);
    EXPECT_FALSE(decision.starterMotor);
    EXPECT_TRUE(decision.isTransition);
}

TEST(CrankingControllerTest, StepPure_RunningWithIgnitionOff_ReturnsStoppingDecision) {
    CrankingController controller;
    StubEngine engine;
    engine.phase_ = EnginePhase::Running;
    engine.stats_.currentRPM = 3000.0;

    auto decision = controller.stepPure(engine, 0.5, false);

    EXPECT_EQ(decision.targetPhase, EnginePhase::Stopping);
    EXPECT_TRUE(decision.isTransition);
    EXPECT_FALSE(decision.starterMotor);
}

TEST(CrankingControllerTest, StepPure_CrankingWithCatch RPM_ReturnsRunningDecision) {
    CrankingController controller;
    StubEngine engine;
    engine.phase_ = EnginePhase::Cranking;
    engine.stats_.currentRPM = 1000.0;
    engine.stats_.exhaustFlow = 3.0;

    // Advance past baseline ticks
    for (int i = 0; i < 11; i++) {
        controller.stepPure(engine, 0.5, true);
    }

    auto decision = controller.stepPure(engine, 0.5, true);

    EXPECT_EQ(decision.targetPhase, EnginePhase::Running);
    EXPECT_FALSE(decision.starterMotor);
    EXPECT_TRUE(decision.isTransition);
    EXPECT_DOUBLE_EQ(decision.effectiveThrottle, 0.55);  // CRANKING_THROTTLE
}

TEST(CrankingControllerTest, StepPure_CrankingWithCatchRPM_ReturnsRunningDecision) {
    CrankingController controller;
    StubEngine engine;
    engine.phase_ = EnginePhase::Cranking;
    engine.stats_.currentRPM = 1000.0;
    engine.stats_.exhaustFlow = 3.0;

    // Advance past baseline ticks
    for (int i = 0; i < 11; i++) {
        controller.stepPure(engine, 0.5, true);
    }

    auto decision = controller.stepPure(engine, 0.5, true);

    EXPECT_EQ(decision.targetPhase, EnginePhase::Running);
    EXPECT_FALSE(decision.starterMotor);
    EXPECT_TRUE(decision.isTransition);
    EXPECT_DOUBLE_EQ(decision.effectiveThrottle, 0.55);  // CRANKING_THROTTLE
}
