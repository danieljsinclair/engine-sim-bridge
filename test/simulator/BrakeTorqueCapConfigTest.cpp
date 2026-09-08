// BrakeTorqueCapConfigTest.cpp
//
// The bridge seam of the --brake-torque drive-cap toggle:
// ISimulatorConfig.brakeTorqueCap must reach the simulator's
// VehicleSpeedConstraint.m_asymmetricDriveCap via
// SimulatorInitHelpers::applyBrakeTorqueCap (a thin setter pass-through to
// Simulator::setBrakeTorqueCap). The constraint-level behaviour (symmetric vs
// asymmetric limits, free-roll when disabled) is pinned in the nested
// engine-sim test suite (test/vehicle_speed_constraint_tests.cpp); this file
// pins the bridge-side plumbing only — that the toggle reaches the constraint
// member through the Simulator setter, uniformly for any Simulator subclass.

#include <gtest/gtest.h>

#include "simulator/SimulatorInitHelpers.h"
#include "simulator/SineSimulator.h"

namespace {

// A minimal real Simulator subclass — its m_vehicleSpeedConstraint exists from
// construction (it is a value member of Simulator, not allocated), so the
// toggle is directly observable without loading a full engine preset.
SineSimulator makeSimulator() {
    return SineSimulator();
}

}  // namespace

// The toggle lands on the constraint's m_asymmetricDriveCap — the single knob
// calculate() consumes to select symmetric vs asymmetric limits. Observed
// through the Simulator accessor (isBrakeTorqueCap) so the test pins the
// public seam, not the private member layout.
TEST(BrakeTorqueCapConfigTest, ApplyBrakeTorqueCap_ReachesConstraintMember) {
    SineSimulator sim = makeSimulator();

    SimulatorInitHelpers::applyBrakeTorqueCap(&sim, true);

    EXPECT_TRUE(sim.isBrakeTorqueCap());
}

// false (the default) writes the pre-cap symmetric state, so the default-off,
// CSV-replay bit-identity contract starts here.
TEST(BrakeTorqueCapConfigTest, ApplyBrakeTorqueCap_DefaultOffWritesSymmetricState) {
    SineSimulator sim = makeSimulator();

    SimulatorInitHelpers::applyBrakeTorqueCap(&sim, false);

    EXPECT_FALSE(sim.isBrakeTorqueCap());
}

// Toggle is idempotent and overwritable: a later call replaces the earlier
// value (the factory applies exactly one call per build, but the setter must
// not latch the first value permanently).
TEST(BrakeTorqueCapConfigTest, ApplyBrakeTorqueCap_OverwritesPreviousValue) {
    SineSimulator sim = makeSimulator();

    SimulatorInitHelpers::applyBrakeTorqueCap(&sim, true);
    EXPECT_TRUE(sim.isBrakeTorqueCap());

    SimulatorInitHelpers::applyBrakeTorqueCap(&sim, false);
    EXPECT_FALSE(sim.isBrakeTorqueCap());
}

// Null-guard: mirrors applySpanTame's contract — a null simulator must not
// crash, because the factory wiring path is the only caller and a mis-wired
// build must fail safe rather than segfault.
TEST(BrakeTorqueCapConfigTest, ApplyBrakeTorqueCap_NullSimulatorIsSafe) {
    // Should not throw or dereference.
    SimulatorInitHelpers::applyBrakeTorqueCap(nullptr, true);
    SimulatorInitHelpers::applyBrakeTorqueCap(nullptr, false);
}