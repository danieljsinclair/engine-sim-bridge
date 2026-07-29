// CommandedSpeedDisplayTest.cpp
// Feature B: the commanded road-speed target (','/'.' keys) must be surfaced
// into EngineState so it is VISIBLE in neutral, where the vehicle-speed
// readout otherwise reflects physics only. Engine behaviour in neutral is
// unchanged — this is display only.

#include <gtest/gtest.h>
#include <simulation/PresentationStateBuilders.h>
#include <simulation/CrankingController.h>
#include <io/IInputProvider.h>
#include <simulation/EnginePhase.h>

using namespace presentation;
using input::EngineInput;

namespace {
CrankingController::State neutralIdleState() {
    // Minimal cranking state — startingThrottle is the only field read by
    // buildControlState for the throttle value.
    CrankingController::State s{};
    s.startingThrottle = 0.0;
    s.starterEngaged = false;
    s.phase = EnginePhase::Running;
    return s;
}
} // namespace

TEST(CommandedSpeedDisplayTest, CommandedSpeedTarget_SurfacedIntoControls) {
    EngineInput in;
    in.roadSpeedKmh = 80.0;  // user commanded 80 km/h via '.' keys

    auto controls = builders::buildControlState(in, neutralIdleState());

    EXPECT_DOUBLE_EQ(controls.commandedSpeedKmh, 80.0)
        << "The commanded road-speed target must flow into EngineState.Controls";
}

TEST(CommandedSpeedDisplayTest, NoCommand_NegativeSentinelPreserved) {
    // Default EngineInput has roadSpeedKmh = -2.0 (no speed commanded).
    EngineInput in;

    auto controls = builders::buildControlState(in, neutralIdleState());

    EXPECT_LT(controls.commandedSpeedKmh, 0.0)
        << "When no speed is commanded the negative sentinel must be preserved "
        << "so the renderer can suppress the target field";
}

TEST(CommandedSpeedDisplayTest, ZeroSpeedCommand_IsSurfacedNotSuppressed) {
    // A deliberate 0 km/h command is a valid command (distinct from "no command")
    // and must surface as 0.0, not be hidden by the negative-sentinel logic.
    EngineInput in;
    in.roadSpeedKmh = 0.0;

    auto controls = builders::buildControlState(in, neutralIdleState());

    EXPECT_DOUBLE_EQ(controls.commandedSpeedKmh, 0.0);
}
