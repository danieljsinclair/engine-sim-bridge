// CrankingControllerTests.cpp - TDD tests for cranking state management
// Controller methods return TransitionDecision (pure) without mutating engine.
// These tests assert correct behaviour through the returned decision values.

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
};

} // namespace

// ============================================================================
// Engage starter from Stopped -> Cranking (returns decision, no engine mutation)
// ============================================================================

TEST(CrankingControllerTest, EngageStarterFromStopped_ReturnsCrankingDecision) {
    CrankingController controller;
    StubEngine engine;

    auto decision = controller.engageStarter(engine, true);

    EXPECT_EQ(decision.targetPhase, EnginePhase::Cranking);
    EXPECT_TRUE(decision.starterMotor);
    EXPECT_TRUE(decision.isTransition);
}

TEST(CrankingControllerTest, EngageStarterFromStopped_IgnoresButtonFalse) {
    CrankingController controller;
    StubEngine engine;

    auto decision = controller.engageStarter(engine, false);

    EXPECT_EQ(decision.targetPhase, EnginePhase::Stopped);
    EXPECT_FALSE(decision.starterMotor);
    EXPECT_FALSE(decision.isTransition);
}

// ============================================================================
// Toggle: pressing starter while Cranking stops the starter
// ============================================================================

TEST(CrankingControllerTest, EngageStarterWhileCranking_ReturnsStoppedDecision) {
    CrankingController controller;
    StubEngine engine;
    engine.phase_ = EnginePhase::Cranking;

    auto decision = controller.engageStarter(engine, true);

    EXPECT_EQ(decision.targetPhase, EnginePhase::Stopped);
    EXPECT_FALSE(decision.starterMotor);
    EXPECT_TRUE(decision.isTransition);
}

// ============================================================================
// Reset: controller must reset when preparing for a new engine
// Hot-swap creates a fresh engine; the controller must not carry stale state.
// Reset does NOT change phase -- that's the caller's job.
// ============================================================================

TEST(CrankingControllerTest, Reset_ClearsInternalTickState) {
    CrankingController controller;
    StubEngine engine;
    engine.phase_ = EnginePhase::Cranking;

    // Advance ticks by stepping
    engine.stats_.exhaustFlow = 0.1;
    for (int i = 0; i < 5; ++i) {
        controller.step(engine, 0.5, true);
    }

    controller.reset();

    // Engine phase unchanged -- reset only clears ticks/baseline
    EXPECT_EQ(engine.getEnginePhase(), EnginePhase::Cranking);
}

TEST(CrankingControllerTest, Reset_ClearsBaseline) {
    // After reset, the controller should build a fresh exhaust flow baseline
    // when cranking the new engine. Stale baseline from the old engine would
    // make engineCaught() produce wrong results.
    CrankingController controller;
    StubEngine engine;
    engine.phase_ = EnginePhase::Cranking;
    engine.stats_.exhaustFlow = 0.8;
    for (int i = 0; i < 15; ++i) {
        controller.step(engine, 0.5, true);
    }

    controller.reset();

    // Simulate cranking a new engine from scratch
    StubEngine newEngine;
    newEngine.phase_ = EnginePhase::Cranking;
    newEngine.stats_.exhaustFlow = 0.2;

    // First step should be building baseline (ticks_ = 1)
    auto decision = controller.step(newEngine, 0.5, true);
    EXPECT_EQ(decision.targetPhase, EnginePhase::Cranking);
    EXPECT_FALSE(decision.isTransition);
}

// ============================================================================
// Step returns correct throttle for cranking phase
// ============================================================================

TEST(CrankingControllerTest, StepWhileCranking_ReturnsCrankingThrottle) {
    CrankingController controller;
    StubEngine engine;
    engine.phase_ = EnginePhase::Cranking;

    auto decision = controller.step(engine, 0.2, true);

    EXPECT_GT(decision.effectiveThrottle, 0.2);  // Cranking override > user throttle
    EXPECT_EQ(decision.targetPhase, EnginePhase::Cranking);
    EXPECT_FALSE(decision.isTransition);
}

// ============================================================================
// Engine catches: exhaust flow spike + RPM above threshold + ignition on
// ============================================================================

TEST(CrankingControllerTest, EngineCatches_TransitionsToRunning) {
    CrankingController controller;
    StubEngine engine;
    engine.phase_ = EnginePhase::Cranking;

    // Build baseline with low exhaust flow
    engine.stats_.exhaustFlow = 0.1;
    for (int i = 0; i < 20; ++i) {
        controller.step(engine, 0.5, true);
    }

    // Simulate engine catching: exhaust spike and sufficient RPM
    engine.stats_.exhaustFlow = 1.0;   // Exhaust spike well above baseline
    engine.stats_.currentRPM = 2000.0; // Well above catch threshold
    engine.ignition_ = true;

    auto decision = controller.step(engine, 0.5, true);

    EXPECT_EQ(decision.targetPhase, EnginePhase::Running);
    EXPECT_TRUE(decision.isTransition);
    EXPECT_FALSE(decision.starterMotor);  // Starter disengages on catch
}

// ============================================================================
// Engine does NOT catch without ignition
// ============================================================================

TEST(CrankingControllerTest, EngineDoesNotCatch_WithoutIgnition) {
    CrankingController controller;
    StubEngine engine;
    engine.phase_ = EnginePhase::Cranking;

    engine.stats_.exhaustFlow = 0.1;
    for (int i = 0; i < 20; ++i) {
        controller.step(engine, 0.5, true);
    }

    // High exhaust + RPM but ignition OFF
    engine.stats_.exhaustFlow = 1.0;
    engine.stats_.currentRPM = 800.0;

    auto state = controller.step(engine, 0.5, false);
    EXPECT_EQ(state.targetPhase, EnginePhase::Cranking);
    EXPECT_FALSE(state.isTransition);
}

// ============================================================================
// Hot-swap scenario: swapping engines mid-crank
// The controller must reset and re-engage the starter on the new engine.
// ============================================================================

TEST(CrankingControllerTest, HotSwapWhileCranking_ReEngagesStarterOnNewEngine) {
    CrankingController controller;
    StubEngine oldEngine;
    oldEngine.phase_ = EnginePhase::Cranking;

    // Hot-swap: reset controller for fresh engine
    controller.reset();
    StubEngine newEngine;
    newEngine.phase_ = EnginePhase::Stopped;

    // Re-engage starter on new engine
    auto decision = controller.engageStarter(newEngine, true);

    EXPECT_EQ(decision.targetPhase, EnginePhase::Cranking);
    EXPECT_TRUE(decision.starterMotor);
    EXPECT_TRUE(decision.isTransition);

    // Apply the decision to the new engine (caller's responsibility)
    newEngine.phase_ = EnginePhase::Cranking;

    // Step the new engine -- should build fresh baseline
    newEngine.stats_.exhaustFlow = 0.2;
    auto stepDecision = controller.step(newEngine, 0.5, true);
    EXPECT_EQ(stepDecision.targetPhase, EnginePhase::Cranking);
    EXPECT_FALSE(stepDecision.isTransition);
}

// ============================================================================
// Hot-swap scenario: engine was Running, swap should auto-crank new engine
// ============================================================================

TEST(CrankingControllerTest, HotSwapWhileRunning_AutoCranksNewEngine) {
    CrankingController controller;

    // Hot-swap: reset + engage starter on new engine
    controller.reset();
    StubEngine newEngine;
    newEngine.phase_ = EnginePhase::Stopped;

    auto decision = controller.engageStarter(newEngine, true);

    EXPECT_EQ(decision.targetPhase, EnginePhase::Cranking);
    EXPECT_TRUE(decision.starterMotor);
}

// ============================================================================
// Hot-swap scenario: engine was Stopped, new engine should stay Stopped
// ============================================================================

TEST(CrankingControllerTest, HotSwapWhileStopped_NewEngineStaysStopped) {
    CrankingController controller;

    controller.reset();
    StubEngine newEngine;
    newEngine.phase_ = EnginePhase::Stopped;

    auto decision = controller.engageStarter(newEngine, false);

    EXPECT_EQ(decision.targetPhase, EnginePhase::Stopped);
    EXPECT_FALSE(decision.starterMotor);
    EXPECT_FALSE(decision.isTransition);
}

// ============================================================================
// Running -> Stopping when ignition off
// ============================================================================

TEST(CrankingControllerTest, RunningWithIgnitionOff_TransitionsToStopping) {
    CrankingController controller;
    StubEngine engine;

    engine.phase_ = EnginePhase::Running;
    engine.stats_.currentRPM = 3000.0;

    auto decision = controller.step(engine, 0.5, false);

    EXPECT_EQ(decision.targetPhase, EnginePhase::Stopping);
    EXPECT_TRUE(decision.isTransition);
}

// ============================================================================
// Stopping -> Stopped when RPM drops below threshold
// ============================================================================

TEST(CrankingControllerTest, StoppingWithLowRPM_TransitionsToStopped) {
    CrankingController controller;
    StubEngine engine;

    engine.phase_ = EnginePhase::Stopping;
    engine.stats_.currentRPM = 0.0;  // Engine effectively stopped

    auto decision = controller.step(engine, 0.5, false);

    EXPECT_EQ(decision.targetPhase, EnginePhase::Stopped);
    EXPECT_TRUE(decision.isTransition);
}

// ============================================================================
// Rollover: hot-swap with drivetrain momentum
// No starter motor, RPM-only detection, uses user throttle
// ============================================================================

TEST(CrankingControllerTest, Rollover_TransitionsToRunning_WhenRPMAboveThreshold) {
    CrankingController controller;
    StubEngine engine;

    engine.phase_ = EnginePhase::Rollover;
    engine.stats_.currentRPM = 600.0;

    auto state = controller.step(engine, 0.3, true);

    EXPECT_EQ(state.targetPhase, EnginePhase::Running);
    EXPECT_TRUE(state.isTransition);
    EXPECT_FALSE(state.starterMotor);
    EXPECT_DOUBLE_EQ(state.effectiveThrottle, 0.3);  // User throttle, not cranking override
}

TEST(CrankingControllerTest, Rollover_StaysRollover_WhenRPMTooLow) {
    CrankingController controller;
    StubEngine engine;

    engine.phase_ = EnginePhase::Rollover;
    engine.stats_.currentRPM = 10.0;  // Well below catch threshold

    auto state = controller.step(engine, 0.3, true);

    EXPECT_EQ(state.targetPhase, EnginePhase::Rollover);
    EXPECT_FALSE(state.isTransition);
}

TEST(CrankingControllerTest, Rollover_DoesNotCatch_WithoutIgnition) {
    CrankingController controller;
    StubEngine engine;

    engine.phase_ = EnginePhase::Rollover;
    engine.stats_.currentRPM = 600.0;

    auto state = controller.step(engine, 0.3, false);
    EXPECT_EQ(state.targetPhase, EnginePhase::Rollover);
    EXPECT_FALSE(state.isTransition);
}

TEST(CrankingControllerTest, Rollover_FallsBackToCranking_AtZeroRPM) {
    CrankingController controller;
    StubEngine engine;

    engine.phase_ = EnginePhase::Rollover;
    engine.stats_.currentRPM = 0.0;

    for (int i = 0; i < 5; ++i) {
        controller.step(engine, 0.3, true);
    }

    // After sustained zero RPM, should fall back to Cranking
    auto state = controller.step(engine, 0.3, true);
    EXPECT_EQ(state.targetPhase, EnginePhase::Cranking);
    EXPECT_TRUE(state.isTransition);
    EXPECT_TRUE(state.starterMotor);
}

TEST(CrankingControllerTest, Rollover_UsesUserThrottle_NotCrankingOverride) {
    CrankingController controller;
    StubEngine engine;

    engine.phase_ = EnginePhase::Rollover;
    engine.stats_.currentRPM = 100.0;  // Below catch threshold, stays in Rollover

    auto state = controller.step(engine, 0.1, true);

    EXPECT_DOUBLE_EQ(state.effectiveThrottle, 0.1);  // User throttle, not cranking override
}

TEST(CrankingControllerTest, EngageStarterFromRollover_ReturnsCrankingDecision) {
    CrankingController controller;
    StubEngine engine;

    engine.phase_ = EnginePhase::Rollover;

    auto decision = controller.engageStarter(engine, true);

    EXPECT_EQ(decision.targetPhase, EnginePhase::Cranking);
    EXPECT_TRUE(decision.starterMotor);
    EXPECT_TRUE(decision.isTransition);
}

TEST(CrankingControllerTest, Rollover_DoesNotFallBackImmediately_ZeroRPMTick1) {
    CrankingController controller;
    StubEngine engine;

    engine.phase_ = EnginePhase::Rollover;
    engine.stats_.currentRPM = 0.0;

    // First tick at 0 RPM -- should NOT fall back immediately
    auto state = controller.step(engine, 0.3, true);
    EXPECT_EQ(state.targetPhase, EnginePhase::Rollover);
    EXPECT_FALSE(state.isTransition);
}

// ============================================================================
// Pure method tests - verify TransitionDecision return values
// ============================================================================

TEST(CrankingControllerTest, EngageStarter_ButtonFalse_ReturnsNoTransition) {
    CrankingController controller;
    StubEngine engine;

    auto decision = controller.engageStarter(engine, false);

    EXPECT_EQ(decision.targetPhase, EnginePhase::Stopped);
    EXPECT_FALSE(decision.starterMotor);
    EXPECT_FALSE(decision.isTransition);
}

TEST(CrankingControllerTest, EngageStarter_FromCranking_ReturnsStoppedDecision) {
    CrankingController controller;
    StubEngine engine;
    engine.phase_ = EnginePhase::Cranking;

    auto decision = controller.engageStarter(engine, true);

    EXPECT_EQ(decision.targetPhase, EnginePhase::Stopped);
    EXPECT_FALSE(decision.starterMotor);
    EXPECT_TRUE(decision.isTransition);
}

TEST(CrankingControllerTest, Step_RunningWithIgnitionOff_ReturnsStoppingDecision) {
    CrankingController controller;
    StubEngine engine;
    engine.phase_ = EnginePhase::Running;
    engine.stats_.currentRPM = 3000.0;

    auto decision = controller.step(engine, 0.5, false);

    EXPECT_EQ(decision.targetPhase, EnginePhase::Stopping);
    EXPECT_TRUE(decision.isTransition);
    EXPECT_FALSE(decision.starterMotor);
}

TEST(CrankingControllerTest, Step_CrankingWithCatchRPM_ReturnsRunningDecision) {
    CrankingController controller;
    StubEngine engine;
    engine.phase_ = EnginePhase::Cranking;
    engine.stats_.currentRPM = 1000.0;

    // Build baseline with low exhaust flow
    engine.stats_.exhaustFlow = 0.1;
    for (int i = 0; i < 20; i++) {
        controller.step(engine, 0.5, true);
    }

    // Simulate catch: high exhaust flow above baseline ratio
    engine.stats_.exhaustFlow = 1.0;
    auto decision = controller.step(engine, 0.5, true);

    EXPECT_EQ(decision.targetPhase, EnginePhase::Running);
    EXPECT_FALSE(decision.starterMotor);
    EXPECT_TRUE(decision.isTransition);
    EXPECT_GT(decision.effectiveThrottle, 0.0);  // Partial throttle during cranking
    EXPECT_LT(decision.effectiveThrottle, 1.0);
}
