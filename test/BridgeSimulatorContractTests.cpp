// BridgeSimulatorContractTests.cpp - Dyno/transition/snapshot contract coverage
//
// BridgeSimulator is the universal ISimulator wrapper; several of its methods
// are zero-hit per lcov (configureDynoLoad, applyTransition, capture/restore
// drivetrain state, setStarterMotor, setDynoTorqueScale). These tests pin their
// OBSERVABLE contracts without touching fragile internal dyno/transmission
// field values:
//   - configureDynoLoad: documented return-value contract (<=0 -> false, >0 -> true)
//   - applyTransition: no-op when isTransition=false; sets phase when true
//     (observable via the public getEnginePhase())
//   - captureDrivetrainState: snapshots the current enginePhase (read side)
//   - restoreDrivetrainState: does NOT restore phase (per its doc comment — only
//     drivetrain physics), so phase is unaffected by the snapshot's enginePhase
//
// Construction uses the light SineSimulator path (no script compilation),
// mirroring SineWaveRegressionTests. We exercise the REAL BridgeSimulator +
// real SineSimulator; no mocks of our own classes.

#include "simulator/BridgeSimulator.h"
#include "simulator/SineSimulator.h"
#include "simulator/SineEngine.h"
#include "simulator/SineVehicle.h"
#include "simulator/SineTransmission.h"
#include "simulator/EngineSimTypes.h"
#include "simulation/EnginePhase.h"
#include "common/PresetExceptions.h"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

namespace {

// Build a fully-created BridgeSimulator around a SineSimulator (the established
// light path). Returns nullptr-equivalent via ASSERT inside the helper caller.
std::unique_ptr<BridgeSimulator> makeReadyBridgeSimulator() {
    auto sineSim = std::make_unique<SineSimulator>();
    Simulator::Parameters simParams;
    simParams.systemType = Simulator::SystemType::NsvOptimized;
    sineSim->initialize(simParams);
    sineSim->setSimulationFrequency(EngineSimDefaults::SIMULATION_FREQUENCY);
    sineSim->setFluidSimulationSteps(EngineSimDefaults::FLUID_SIMULATION_STEPS);
    sineSim->setTargetSynthesizerLatency(EngineSimDefaults::TARGET_SYNTH_LATENCY);
    sineSim->loadSimulation(new SineEngine(), new SineVehicle(), new SineTransmission());

    auto bridge = std::make_unique<BridgeSimulator>(std::move(sineSim), "TestBridge");
    ISimulatorConfig config;
    config.sampleRate = EngineSimDefaults::SAMPLE_RATE;
    config.simulationFrequency = EngineSimDefaults::SIMULATION_FREQUENCY;
    config.fluidSimulationSteps = EngineSimDefaults::FLUID_SIMULATION_STEPS;
    config.targetSynthesizerLatency = EngineSimDefaults::TARGET_SYNTH_LATENCY;
    [[maybe_unused]] const bool created = bridge->create(config, nullptr, nullptr);
    return bridge;
}

// Build a transition decision targeting a phase (isTransition=true by default).
TransitionDecision transitionTo(EnginePhase phase, bool starterMotor = false,
                                double throttle = 0.0, bool isTransition = true) {
    return TransitionDecision{phase, starterMotor, throttle, isTransition};
}

}  // namespace

// --- configureDynoLoad: documented return-value contract ---------------------

// loadFraction <= 0 declines configuration (no load to apply) -> returns false.
TEST(BridgeSimulatorContractTest, ConfigureDynoLoadRejectsNonPositiveFraction) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    EXPECT_FALSE(sim->configureDynoLoad(0.0));
    EXPECT_FALSE(sim->configureDynoLoad(-0.5));
}

// A positive load fraction configures the dyno -> returns true.
TEST(BridgeSimulatorContractTest, ConfigureDynoLoadAcceptsPositiveFraction) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    EXPECT_TRUE(sim->configureDynoLoad(0.5));
    EXPECT_TRUE(sim->configureDynoLoad(1.0));
}

// --- applyTransition: phase machine observable via getEnginePhase() -----------

// A decision flagged isTransition=false is a no-op: phase must not change.
TEST(BridgeSimulatorContractTest, ApplyTransitionNoOpWhenNotFlagged) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    const EnginePhase before = sim->getEnginePhase();

    // An unflagged decision targeting a different phase must not move the phase.
    sim->applyTransition(transitionTo(EnginePhase::Running, false, 0.0, /*isTransition*/false));
    EXPECT_EQ(sim->getEnginePhase(), before);
}

// A flagged decision sets the engine phase to its target.
TEST(BridgeSimulatorContractTest, ApplyTransitionSetsPhaseWhenFlagged) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);

    sim->applyTransition(transitionTo(EnginePhase::Cranking, /*starter*/true));
    EXPECT_EQ(sim->getEnginePhase(), EnginePhase::Cranking);

    sim->applyTransition(transitionTo(EnginePhase::Running));
    EXPECT_EQ(sim->getEnginePhase(), EnginePhase::Running);

    sim->applyTransition(transitionTo(EnginePhase::Stopped));
    EXPECT_EQ(sim->getEnginePhase(), EnginePhase::Stopped);
}

// --- captureDrivetrainState: snapshots enginePhase (read side) ----------------

// The snapshot records the engine's current phase at capture time.
TEST(BridgeSimulatorContractTest, CaptureDrivetrainStateRecordsCurrentPhase) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    sim->applyTransition(transitionTo(EnginePhase::Running));

    const auto snapshot = sim->captureDrivetrainState();
    EXPECT_EQ(snapshot.enginePhase, EnginePhase::Running);
}

// --- restoreDrivetrainState: phase is NOT restored (per doc comment) ----------

// Per the method's doc: restoreDrivetrainState restores drivetrain PHYSICS only,
// NOT operational phase. So restoring a snapshot whose enginePhase differs from
// the live phase must leave the live phase unchanged.
TEST(BridgeSimulatorContractTest, RestoreDrivetrainStateDoesNotChangePhase) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    sim->applyTransition(transitionTo(EnginePhase::Running));
    ASSERT_EQ(sim->getEnginePhase(), EnginePhase::Running);

    BridgeSimulator::DrivetrainSnapshot snapshot;
    snapshot.enginePhase = EnginePhase::Stopped;   // deliberately different
    snapshot.gear = -1;                              // < 0: transmission branch skipped

    sim->restoreDrivetrainState(snapshot);
    EXPECT_EQ(sim->getEnginePhase(), EnginePhase::Running);  // phase untouched
}

// --- restoreState (byte-vector hot-swap): size validation ---------------------

// restoreState rejects a payload smaller than a DrivetrainSnapshot by throwing
// SimulatorException. Pinning the documented contract (type asserted, not msg).
TEST(BridgeSimulatorContractTest, RestoreStateRejectsUndersizedPayload) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    const std::vector<uint8_t> tooSmall(sizeof(BridgeSimulator::DrivetrainSnapshot) - 1, 0);
    EXPECT_THROW(sim->restoreState(tooSmall), SimulatorException);
}

// A correctly-sized payload does not throw (round-trip of the serialization
// path, irrespective of the values it carries).
TEST(BridgeSimulatorContractTest, RestoreStateAcceptsSizedPayload) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    const std::vector<uint8_t> sized(sizeof(BridgeSimulator::DrivetrainSnapshot), 0);
    EXPECT_NO_THROW(sim->restoreState(sized));
}

// --- saveState: produces a DrivetrainSnapshot-sized payload -------------------
// saveState() was fully unhit per lcov. It must yield a payload exactly the size
// of a DrivetrainSnapshot (the wire format restoreState expects).
TEST(BridgeSimulatorContractTest, SaveStateProducesSizedPayload) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    const auto bytes = sim->saveState();
    EXPECT_EQ(bytes.size(), sizeof(BridgeSimulator::DrivetrainSnapshot));
}

// saveState then restoreState round-trips without throwing: the serialized form
// produced by saveState is valid input to restoreState.
TEST(BridgeSimulatorContractTest, SaveStateRoundTripsThroughRestoreState) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    sim->applyTransition(transitionTo(EnginePhase::Running));
    const auto bytes = sim->saveState();
    ASSERT_EQ(bytes.size(), sizeof(BridgeSimulator::DrivetrainSnapshot));
    EXPECT_NO_THROW(sim->restoreState(bytes));
}

// --- setDynoTorqueScale: negative scale is rejected ---------------------------
// A negative dyno torque scale is a programmer error, not a runtime state. The
// documented contract rejects it by throwing SimulatorException (type asserted,
// not message). Pinning observable behavior, not dyno internals.
TEST(BridgeSimulatorContractTest, SetDynoTorqueScaleRejectsNegative) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    EXPECT_THROW(sim->setDynoTorqueScale(-0.1), SimulatorException);
}

// A non-negative scale is accepted (no throw) whether or not the dyno is enabled
// — when disabled the call is a documented no-op.
TEST(BridgeSimulatorContractTest, SetDynoTorqueScaleAcceptsNonNegative) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    EXPECT_NO_THROW(sim->setDynoTorqueScale(0.0));
    EXPECT_NO_THROW(sim->setDynoTorqueScale(0.5));
}

// --- setSpeedTrackingTarget: declined in neutral ------------------------------
// In neutral there is no gear ratio to map road speed to engine RPM, so the
// speed-tracking target cannot be applied -> returns false. setGear(0) is the
// bridge convention for neutral (engine-sim gear -1).
TEST(BridgeSimulatorContractTest, SetSpeedTrackingTargetDeclinedInNeutral) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    sim->setGear(0);  // neutral
    ASSERT_EQ(sim->getGear(), 0);

    EXPECT_FALSE(sim->setSpeedTrackingTarget(/*speedKmh*/ 50.0));
}

// In a forward gear, speed tracking can be applied -> returns true.
TEST(BridgeSimulatorContractTest, SetSpeedTrackingTargetAcceptedInGear) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    sim->setGear(1);  // 1st (forward)
    ASSERT_EQ(sim->getGear(), 1);

    EXPECT_TRUE(sim->setSpeedTrackingTarget(/*speedKmh*/ 50.0));
}

// --- changeGear: no-op and clamping contracts ---------------------------------

// changeGear(0) requests no change -> declined with false (documented no-op).
TEST(BridgeSimulatorContractTest, ChangeGearZeroDeltaIsNoOp) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    sim->setGear(1);
    EXPECT_FALSE(sim->changeGear(0));
}

// changeGear clamps at the top of the gear range: with a 1-forward-gear
// transmission, requesting +5 from 1st gear lands on the maximum (1st), not
// beyond. Observable via getGear() (bridge convention).
TEST(BridgeSimulatorContractTest, ChangeGearClampsAtTopOfRange) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    sim->setGear(1);                 // only forward gear available
    ASSERT_EQ(sim->getGear(), 1);

    sim->changeGear(5);              // request far beyond max
    EXPECT_EQ(sim->getGear(), 1);    // clamped to max forward gear
}

// changeGear clamps at the bottom (neutral): a large negative delta from 1st
// gear lands at neutral (0 in bridge convention), not below.
TEST(BridgeSimulatorContractTest, ChangeGearClampsAtNeutral) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    sim->setGear(1);

    sim->changeGear(-5);
    EXPECT_EQ(sim->getGear(), 0);    // clamped at neutral
}

// --- setVehicleSpeedTarget: return-value contract (Spike-A inverse model) ------
// Negative target = disable (free-roll): the constraint cannot be applied, so the
// call returns false regardless of gear. Observable via the documented return.
TEST(BridgeSimulatorContractTest, SetVehicleSpeedTargetNegativeIsDeclined) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    sim->setGear(1);
    ASSERT_EQ(sim->getGear(), 1);

    EXPECT_FALSE(sim->setVehicleSpeedTarget(/*speedKmh*/ -1.0));
}

// In neutral the engine must free-rev, so the wheels are not pinned -> false.
TEST(BridgeSimulatorContractTest, SetVehicleSpeedTargetDeclinedInNeutral) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    sim->setGear(0);  // neutral
    ASSERT_EQ(sim->getGear(), 0);

    EXPECT_FALSE(sim->setVehicleSpeedTarget(/*speedKmh*/ 50.0));
}

// A valid positive target in gear is accepted -> true (the constraint is applied).
TEST(BridgeSimulatorContractTest, SetVehicleSpeedTargetAcceptedInGear) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    sim->setGear(1);
    ASSERT_EQ(sim->getGear(), 1);

    EXPECT_TRUE(sim->setVehicleSpeedTarget(/*speedKmh*/ 50.0));
}

// --- captureDrivetrainState: gear stored in ENGINE-SIM convention -------------
// The snapshot's gear field reads the transmission's raw gear (engine-sim
// convention: -1=neutral, 0=1st...), NOT the bridge convention getGear() reports
// (0=neutral, 1=1st...). Pinning this asymmetry: a future "fix" that routes the
// capture through bridge::toBridge would corrupt the hot-swap round-trip, since
// restoreDrivetrainState writes the raw value back. setGear(1) -> engine-sim 0.
TEST(BridgeSimulatorContractTest, CaptureDrivetrainStateStoresEngineSimGearConvention) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    sim->setGear(1);                                  // bridge FIRST
    ASSERT_EQ(sim->getGear(), 1);                     // getGear reports bridge convention

    const auto snapshot = sim->captureDrivetrainState();
    EXPECT_EQ(snapshot.gear, 0);                      // snapshot holds engine-sim FIRST (raw)
}

// --- changeGear(delta, clutchPressure): 2-arg overload clamps identically ------
// The explicit-clutch-pressure overload must observe the same gear clamping as
// the 1-arg form. Observable via getGear(); the applied clutch value itself has
// no public reader, so we assert only the gear result (the documented effect).
TEST(BridgeSimulatorContractTest, ChangeGearWithClutchPressureClampsAtTop) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    sim->setGear(1);
    ASSERT_EQ(sim->getGear(), 1);

    sim->changeGear(/*delta*/ 5, /*clutchPressure*/ 0.5);
    EXPECT_EQ(sim->getGear(), 1);    // clamped to max forward gear
}

// --- capture/restore gear ROUND-TRIP (the load-bearing invariant) -------------
// The raw-convention test above pins the *value*; THIS pins the *invariant* that
// value exists to serve: snapshot.gear captured from one sim must restore to the
// SAME bridge gear on another sim. restoreDrivetrainState feeds snapshot.gear
// straight to trans->changeGear(), guarded by gear>=0 — which only works because
// capture stores engine-sim raw. A future "fix" routing capture through toBridge
// would shift the restored gear up by one every hot-swap. This test survives even
// if the convention numbers change: the contract is gear-in == gear-out.
TEST(BridgeSimulatorContractTest, CaptureRestoreRoundTripsGearAcrossInstances) {
    auto donor = makeReadyBridgeSimulator();
    ASSERT_NE(donor, nullptr);
    donor->setGear(1);
    ASSERT_EQ(donor->getGear(), 1);

    const auto snapshot = donor->captureDrivetrainState();

    auto recipient = makeReadyBridgeSimulator();
    ASSERT_NE(recipient, nullptr);
    recipient->setGear(0);   // start in a different gear (neutral)
    ASSERT_NE(recipient->getGear(), 1);

    recipient->restoreDrivetrainState(snapshot);
    EXPECT_EQ(recipient->getGear(), 1);   // restored to the donor's gear
}

// --- Dyno ON/OFF observable via the public stats surface ----------------------
// getStats() -> getDynoStats() writes dynoTargetRPM ONLY when m_dyno.m_enabled
// (default 0.0 = "disabled"). So dynoTargetRPM != 0 proves the dyno is ON, and
// == 0 proves it's OFF — all through the public API, no private-field poke.

// configureDynoLoad(>0) enables the dyno -> dynoTargetRPM is non-zero (ON).
TEST(BridgeSimulatorContractTest, ConfigureDynoLoadTurnsDynoOnObservableViaStats) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    sim->setGear(1);                  // dyno needs a runnable state
    ASSERT_TRUE(sim->configureDynoLoad(/*loadFraction*/ 0.5));

    EXPECT_NE(sim->getStats().dynoTargetRPM, 0.0);   // dyno ON
}

// setVehicleSpeedTarget(>0) in-gear FORCES the dyno OFF (the "dragged engine"
// fix): it replaces the dyno with the vehicle-speed constraint. Pinned through
// the stats surface — the invariant that had no observable seam before. Setup:
// enable the dyno first (so OFF is a real transition, not just the default).
TEST(BridgeSimulatorContractTest, SetVehicleSpeedTargetForcesDynoOffObservableViaStats) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    sim->setGear(1);
    ASSERT_TRUE(sim->configureDynoLoad(/*loadFraction*/ 0.5));
    ASSERT_NE(sim->getStats().dynoTargetRPM, 0.0);   // precondition: dyno ON

    EXPECT_TRUE(sim->setVehicleSpeedTarget(/*speedKmh*/ 50.0));
    EXPECT_EQ(sim->getStats().dynoTargetRPM, 0.0);   // dyno FORCED OFF
}

// setSpeedTrackingTarget in-gear configures the dyno in hold mode with a derived
// target RPM = max(rpmFloor, computeTargetRpm(speedKmh, gearRatio, tire, diff)).
// Two contracts pinned through the stats surface:
//   (1) the floor is honored when the road-speed-derived RPM is below it;
//   (2) with no floor, the target scales up monotonically with road speed.
TEST(BridgeSimulatorContractTest, SetSpeedTrackingTargetDynoRpmHonorsFloorAndScales) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    sim->setGear(1);

    // (1) Floor dominates when the derived RPM is lower: target == floor exactly.
    constexpr double kFloor = 1500.0;
    ASSERT_TRUE(sim->setSpeedTrackingTarget(/*speedKmh*/ 30.0, kFloor));
    EXPECT_DOUBLE_EQ(sim->getStats().dynoTargetRPM, kFloor);

    // (2) No floor -> target reflects road-speed-derived RPM, which is monotonic
    // in speed (RPM = speedMs/tireRadius * gearRatio * diffRatio * 60/2pi).
    ASSERT_TRUE(sim->setSpeedTrackingTarget(/*speedKmh*/ 30.0, /*rpmFloor*/ 0.0));
    const double lowSpeedRpm = sim->getStats().dynoTargetRPM;
    ASSERT_GT(lowSpeedRpm, 0.0);

    ASSERT_TRUE(sim->setSpeedTrackingTarget(/*speedKmh*/ 120.0, /*rpmFloor*/ 0.0));
    const double highSpeedRpm = sim->getStats().dynoTargetRPM;
    EXPECT_GT(highSpeedRpm, lowSpeedRpm);   // 4x road speed -> higher target RPM
}

