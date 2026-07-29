// CrankingControllerStoppingTests.cpp - Complementary Stopping-entry coverage
//
// The existing CrankingControllerTests.cpp exercises the Stopping phase only on
// the low-RPM finalization path (Stopping -> Stopped). These tests pin the two
// Stopping-entry branches that file does not cover:
//   - step() from Stopping when the engine is still catchable (RPM above the
//     catch floor, ignition on) -> Rollover (hot-swap recovery, no starter);
//   - engageStarter() from Stopping in both outcomes: catch -> Running (no
//     starter) and fallback -> Cranking (starter re-engaged).
//
// engageStarter() handles Stopping and Rollover in the same case block; pinning
// the Stopping entry confirms the shared catch-vs-fallback logic is reached from
// Stopping, not only from Rollover.
//
// We drive the controller through a focused ICombustionEngine stub (the
// interface seam production uses) — faking a boundary, not mocking our logic.

#include "simulation/CrankingController.h"
#include "simulation/EnginePhase.h"
#include "simulator/ICombustionEngine.h"
#include "simulator/EngineSimTypes.h"

#include <gtest/gtest.h>

namespace {

// Minimal ICombustionEngine stub matching the controller's read surface.
class StoppingStubEngine : public ICombustionEngine {
public:
    EngineSimStats stats{};
    EnginePhase phase = EnginePhase::Stopped;

    // ICombustionEngine
    void setStarterMotor(bool) override {}
    void setIgnition(bool) override {}
    void applyTransition(const TransitionDecision&) override {}

    // ISimulator
    bool create(const ISimulatorConfig&, ILogging*, telemetry::ITelemetryWriter*) override { return true; }
    void destroy() override {}
    std::string getLastError() const override { return {}; }
    const char* getName() const override { return "StoppingStub"; }
    void update(double) override {}
    EngineSimStats getStats() const override { return stats; }
    void setThrottle(double) override {}
    bool renderOnDemand(float*, int32_t, int32_t*) override { return true; }
    bool readAudioBuffer(float*, int32_t, int32_t*) override { return true; }
    bool start() override { return true; }
    void stop() override {}
    int getSimulationFrequency() const override { return 10000; }
    EnginePhase getEnginePhase() const override { return phase; }
};

}  // namespace

// step() from Stopping with RPM still above the catch floor -> Rollover.
// This is the untested counterpart to the existing StoppingWithLowRPM test:
// here the engine can still catch, so it enters hot-swap recovery, not Stopped.
TEST(CrankingControllerStoppingTest, StepFromStoppingCatchableTransitionsToRollover) {
    StoppingStubEngine engine;
    engine.phase = EnginePhase::Stopping;
    engine.stats.currentRPM = 900.0;   // > MIN_CATCH_RPM (500)
    engine.stats.exhaustFlow = 1.0;
    CrankingController controller;

    const auto d = controller.step(engine, /*throttle*/ 0.0, /*ignition*/ true);

    EXPECT_EQ(d.targetPhase, EnginePhase::Rollover);
    EXPECT_TRUE(d.isTransition);
    EXPECT_FALSE(d.starterMotor);
}

// engageStarter() from Stopping with a catchable engine -> Running, no starter.
TEST(CrankingControllerStoppingTest, EngageStarterFromStoppingCatchableGoesToRunning) {
    StoppingStubEngine engine;
    engine.phase = EnginePhase::Stopping;
    engine.stats.currentRPM = 900.0;   // catchable
    engine.stats.exhaustFlow = 1.0;
    CrankingController controller;

    const auto d = controller.engageStarter(engine, /*button*/ true, /*ignition*/ true);

    EXPECT_EQ(d.targetPhase, EnginePhase::Running);
    EXPECT_FALSE(d.starterMotor);
    EXPECT_TRUE(d.isTransition);
}

// engageStarter() from Stopping when the engine cannot catch -> restart cranking
// with the starter motor (fallback branch of the shared Stopping/Rollover case).
TEST(CrankingControllerStoppingTest, EngageStarterFromStoppingFallbackRestartsCranking) {
    StoppingStubEngine engine;
    engine.phase = EnginePhase::Stopping;
    engine.stats.currentRPM = 100.0;   // < MIN_CATCH_RPM (500) -> cannot catch
    engine.stats.exhaustFlow = 0.01;
    CrankingController controller;

    const auto d = controller.engageStarter(engine, /*button*/ true, /*ignition*/ true);

    EXPECT_EQ(d.targetPhase, EnginePhase::Cranking);
    EXPECT_TRUE(d.starterMotor);
    EXPECT_TRUE(d.isTransition);
}
