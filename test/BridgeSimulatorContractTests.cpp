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
#include "simulator/ScriptLoadHelpers.h"
#include "simulation/EnginePhase.h"
#include "common/PresetExceptions.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace {

// Build a fully-created BridgeSimulator around a SineSimulator (the established
// light path). Returns nullptr-equivalent via ASSERT inside the helper caller.
std::unique_ptr<BridgeSimulator> makeReadyBridgeSimulatorWith(Transmission* transmission) {
    auto sineSim = std::make_unique<SineSimulator>();
    Simulator::Parameters simParams;
    simParams.systemType = Simulator::SystemType::NsvOptimized;
    sineSim->initialize(simParams);
    sineSim->setSimulationFrequency(EngineSimDefaults::SIMULATION_FREQUENCY);
    sineSim->setFluidSimulationSteps(EngineSimDefaults::FLUID_SIMULATION_STEPS);
    sineSim->setTargetSynthesizerLatency(EngineSimDefaults::TARGET_SYNTH_LATENCY);
    sineSim->loadSimulation(new SineEngine(), new SineVehicle(), transmission);

    auto bridge = std::make_unique<BridgeSimulator>(std::move(sineSim), "TestBridge");
    ISimulatorConfig config;
    config.sampleRate = EngineSimDefaults::SAMPLE_RATE;
    config.simulationFrequency = EngineSimDefaults::SIMULATION_FREQUENCY;
    config.fluidSimulationSteps = EngineSimDefaults::FLUID_SIMULATION_STEPS;
    config.targetSynthesizerLatency = EngineSimDefaults::TARGET_SYNTH_LATENCY;
    [[maybe_unused]] const bool created = bridge->create(config, nullptr, nullptr);
    return bridge;
}

std::unique_ptr<BridgeSimulator> makeReadyBridgeSimulator() {
    return makeReadyBridgeSimulatorWith(new SineTransmission);
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

// ============================================================================
// renderDrainedAudio: drain-only render contract (S886 refactor site).
//
// The method's documented contract: synthesize + drain ALREADY-PRODUCED audio
// without advancing the core (the loop thread owns core advancement via
// update()). The internal bounded drain loop re-renders until the request is
// satisfied or the producer stalls (no new input and no buffered audio), then
// zero-pads whatever remains. These tests pin the OBSERVABLE outcomes of that
// loop through the public ISimulator API — a restructure of the loop must
// keep all four: satisfied requests carry real audio, drained-out requests
// carry silence, partial requests carry audio then a zero tail, and invalid
// arguments fail fast.
// ============================================================================

// A FRESH simulator (no update() yet) has no synthesized audio: the drain must
// stall immediately, report zero frames written, zero-fill the WHOLE buffer,
// and still return true. This also pins drain-only-ness: unlike
// renderOnDemand (which advances the core itself and so always produces — see
// SineWaveRegressionTests), the drain path must NOT manufacture audio from a
// never-advanced core.
TEST(BridgeSimulatorContractTest, RenderDrainedAudioFreshSimulatorWritesZeroSilence) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);

    constexpr int32_t kFrames = 256;
    std::vector<float> buffer(kFrames * 2, 1.0f);  // poison: non-zero start
    int32_t written = -1;

    EXPECT_TRUE(sim->renderDrainedAudio(buffer.data(), kFrames, &written));
    EXPECT_EQ(written, 0);
    for (float sample : buffer) {
        EXPECT_FLOAT_EQ(sample, 0.0f);  // silence fill covers the full request
    }
}

// After the core has advanced (production: update() on the loop thread), a
// modest drain request must be SATISFIED in full — the drain loop re-renders
// until the requested frames exist — and the audio is the real sine signal,
// not silence.
TEST(BridgeSimulatorContractTest, RenderDrainedAudioAfterUpdateSatisfiesRequest) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    for (int i = 0; i < 5; ++i) sim->update(0.05);  // 0.25s of core advance

    constexpr int32_t kFrames = 256;
    std::vector<float> buffer(kFrames * 2, 0.0f);
    int32_t written = 0;

    EXPECT_TRUE(sim->renderDrainedAudio(buffer.data(), kFrames, &written));
    EXPECT_EQ(written, kFrames);  // full request satisfied

    float peak = 0.0f;
    for (int32_t i = 0; i < kFrames; ++i) {
        peak = std::max(peak, std::fabs(buffer[i * 2]));  // left channel
    }
    EXPECT_GT(peak, 0.0f);  // real audio, not silence
}

// A request LARGER than what one limited core advance produced must drain the
// available audio, then zero-pad the remainder (the producer-stall tail).
// 0.01s of advance synthesizes well under the 4096-frame request, so the
// bounded drain loop terminates via its stall break with a partial write.
TEST(BridgeSimulatorContractTest, RenderDrainedAudioPartialProductionPadsSilenceTail) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);
    sim->update(0.01);  // ~441 output frames worth of input — far below request

    constexpr int32_t kFrames = 4096;
    std::vector<float> buffer(kFrames * 2, 1.0f);  // poison: non-zero start
    int32_t written = -1;

    EXPECT_TRUE(sim->renderDrainedAudio(buffer.data(), kFrames, &written));
    EXPECT_GT(written, 0);            // some audio drained
    EXPECT_LT(written, kFrames);      // ...but not the whole request

    // The unwritten tail must be silence — the zero-pad contract.
    for (int32_t i = written; i < kFrames; ++i) {
        EXPECT_FLOAT_EQ(buffer[i * 2], 0.0f) << "left sample past written frames";
        EXPECT_FLOAT_EQ(buffer[i * 2 + 1], 0.0f) << "right sample past written frames";
    }
}

// Argument guards (fail-fast contract): a null buffer or a non-positive frame
// count throws rather than returning a misleading result. Exception TYPE only
// — the message text is not the contract.
TEST(BridgeSimulatorContractTest, RenderDrainedAudioRejectsInvalidArguments) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);

    float scratch[8] = {};
    int32_t written = 0;
    EXPECT_THROW(sim->renderDrainedAudio(nullptr, 8, &written), SimulatorException);
    EXPECT_THROW(sim->renderDrainedAudio(scratch, 0, &written), SimulatorException);
    EXPECT_THROW(sim->renderDrainedAudio(scratch, -1, &written), SimulatorException);
}

// LATENT-HAZARD CHARACTERIZATION (deterministically reproducible; see the
// report note on the S886 site): when the drain's first render is
// free-space-capped it fills the audio ring to EXACTLY capacity, and the
// upstream RingBuffer's blind-write semantics make writeIndex land on start
// — size() then reports 0 (full is indistinguishable from empty). The drain
// loop's stall-break therefore fires and the callback observes a silent
// dropout even though the ring holds a full second of audio. This pins the
// CURRENT observable behaviour of the loop in that state (true return,
// zero written, silence fill) so a behaviour-preserving refactor keeps it
// and any future fix of the ring ambiguity is flagged deliberately. The
// production shape (SyncPull drains every ~16ms callback) never approaches
// ring capacity, so the hazard is latent, not live.
TEST(BridgeSimulatorContractTest, RenderDrainedAudioNearFullRingStallsToSilence) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);

    // Production-shaped chunked driving: small updates each followed by a
    // small satisfied drain. The audio ring fills toward capacity (verified:
    // ~41.5k of the 44.1k ring after 11 chunks) while the latency governor
    // throttles new input to a surplus.
    std::vector<float> scratch(256 * 2);
    int32_t written = 0;
    for (int chunk = 0; chunk < 11; ++chunk) {
        for (int i = 0; i < 6; ++i) sim->update(1.0 / 60.0);
        ASSERT_TRUE(sim->renderDrainedAudio(scratch.data(), 256, &written));
        ASSERT_EQ(written, 256);
    }
    // Top up the staged input beyond the ring's remaining free space: the
    // next render is free-space-capped to an exact fill (the ambiguity above).
    for (int i = 0; i < 20; ++i) sim->update(1.0 / 60.0);

    const int32_t frames = EngineSimDefaults::SAMPLE_RATE * 2;  // over-request
    std::vector<float> buffer(frames * 2, 1.0f);  // poison: non-zero start
    written = 0;
    EXPECT_TRUE(sim->renderDrainedAudio(buffer.data(), frames, &written));
    EXPECT_EQ(written, 0);  // exact-fill ambiguity reads as a producer stall
    for (float sample : buffer) {
        EXPECT_FLOAT_EQ(sample, 0.0f);  // full-request silence fill
    }
}

// ============================================================================
// setUseTorqueConverter: flag-follows-transmission-reality contract
// (S5350 refactor site — the `trans` local at the read path).
//
// The method records the bridge flag by READING the transmission: enabling is
// only honoured when the transmission actually carries a converter (installed
// at factory wiring time); disabling flips the flag and must never REMOVE the
// converter. Pinning both directions through the public usesTorqueConverter()
// plus the getInternalSimulator() test seam, so a pointer-to-const change that
// accidentally alters behaviour (or starts mutating the transmission) fails.
// ============================================================================

// Enabling on a transmission with NO converter is declined: the flag reports
// the transmission's reality (false), not the request. The default sine
// transmission carries no converter.
TEST(BridgeSimulatorContractTest, SetUseTorqueConverterWithoutConverterStaysFalse) {
    auto sim = makeReadyBridgeSimulator();
    ASSERT_NE(sim, nullptr);

    EXPECT_FALSE(sim->usesTorqueConverter());          // flag starts false
    sim->setUseTorqueConverter(true);
    EXPECT_FALSE(sim->usesTorqueConverter());          // no converter -> stays false
    sim->setUseTorqueConverter(false);
    EXPECT_FALSE(sim->usesTorqueConverter());
}

// Round trip on a converter-EQUIPPED transmission: the read path honours the
// converter when enabling, keeps the converter installed when disabling (the
// documented "only ever added, never removed"), and re-evaluates on re-enable.
// The transmission is the factory's own ScriptLoadHelpers::createDefaultTransmission
// (converter installed at initialize time — the documented-safe order), wired
// through the same loadSimulation path production uses.
TEST(BridgeSimulatorContractTest, SetUseTorqueConverterRoundTripOnConverterEquippedTransmission) {
    auto sim = makeReadyBridgeSimulatorWith(
        ScriptLoadHelpers::createDefaultTransmission(/*useTorqueConverter=*/true));
    ASSERT_NE(sim, nullptr);
    const Transmission* trans = sim->getInternalSimulator()->getTransmission();
    ASSERT_NE(trans, nullptr);
    ASSERT_TRUE(trans->hasTorqueConverter());  // precondition: converter installed

    EXPECT_FALSE(sim->usesTorqueConverter());   // flag starts false even with a converter
    sim->setUseTorqueConverter(true);
    EXPECT_TRUE(sim->usesTorqueConverter());    // enable honours the converter's presence

    sim->setUseTorqueConverter(false);
    EXPECT_FALSE(sim->usesTorqueConverter());   // disable flips the flag only...
    EXPECT_TRUE(trans->hasTorqueConverter());   // ...the converter stays installed

    sim->setUseTorqueConverter(true);
    EXPECT_TRUE(sim->usesTorqueConverter());    // re-enable re-evaluates to true
}

