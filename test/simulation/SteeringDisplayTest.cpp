// SteeringDisplayTest.cpp
//
// The display-state seam of the steering-angle readout: EngineInput's
// telemetry-sourced steeringAngleDeg must surface into
// EngineState.Controls.steeringAngleDeg so the CLI console can render it
// (mirrors CommandedSpeedDisplayTest — display only, engine behaviour
// unchanged). Presence flows through; absence stays nullopt so non-DBC
// sources (keyboard/demo) render nothing.

#include <gtest/gtest.h>
#include <simulation/PresentationStateBuilders.h>
#include <simulation/CrankingController.h>
#include <io/IInputProvider.h>
#include <simulation/EnginePhase.h>

using namespace presentation;
using input::EngineInput;

namespace {
CrankingController::State neutralIdleState() {
    // Minimal cranking state — only startingThrottle is read by
    // buildControlState for the throttle value.
    CrankingController::State s{};
    s.startingThrottle = 0.0;
    s.starterEngaged = false;
    s.phase = EnginePhase::Running;
    return s;
}
}  // namespace

// A steering value reported by telemetry (vehicle-sim steering_angle_deg,
// CAN 0x129 SCCM_steeringAngle) must reach EngineState.Controls verbatim —
// signed and precise; no clamping or rounding at the plumbing layer.
TEST(SteeringDisplayTest, SteeringAngle_SurfacedIntoControls) {
    EngineInput in;
    in.steeringAngleDeg = -12.5;

    const auto controls = builders::buildControlState(in, neutralIdleState());

    ASSERT_TRUE(controls.steeringAngleDeg.has_value());
    EXPECT_DOUBLE_EQ(*controls.steeringAngleDeg, -12.5);
}

// Positive (right-hand) angles surface too — the signal is signed and both
// directions are ordinary values.
TEST(SteeringDisplayTest, SteeringAngle_PositiveSurfaces) {
    EngineInput in;
    in.steeringAngleDeg = 340.0;

    const auto controls = builders::buildControlState(in, neutralIdleState());

    ASSERT_TRUE(controls.steeringAngleDeg.has_value());
    EXPECT_DOUBLE_EQ(*controls.steeringAngleDeg, 340.0);
}

// When the feed carries no steering (keyboard/demo/non-DBC sources, or a
// capture without the column), the field must stay nullopt — the renderer's
// cue to degrade to nothing. Zero is NOT substituted for absent.
TEST(SteeringDisplayTest, SteeringAbsent_ControlsStayNullopt) {
    EngineInput in;

    const auto controls = builders::buildControlState(in, neutralIdleState());

    EXPECT_FALSE(controls.steeringAngleDeg.has_value());
}
