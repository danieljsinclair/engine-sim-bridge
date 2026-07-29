// SimulationLoopHandoverTests.cpp - Blind value tests for the Q2 hot-swap seam
//
// Exercises createSession() + SimulatorSession::handoverSession() /
// transferDrivetrainState() WITHOUT real CoreAudio (NullAudioHardwareProvider)
// and WITHOUT any mocks of our own classes — we drive the REAL production path
// with real sine BridgeSimulators.
//
// Each test asserts ONE observable behavior of the hot-swap contract:
//   1. handoverSession swaps the live simulator pointer in the session.
//   2. drivetrain state (vehicle v_theta) is transferred to the new simulator.
//   3. handoverSession is idempotent — two consecutive swaps complete cleanly.
//   4. createSession with a null existingSession builds a fresh session
//      (proves the audioProvider injection didn't break first-run).
//
// No assert-path / throw-forcing / fragile-seam tests — happy-path value
// assertions only, mirroring BridgeSimulatorContractTests / SimulationLoopStepTests.

#include "simulation/SimulationLoop.h"
#include "session/ISimulatorSession.h"
#include "simulator/BridgeSimulator.h"
#include "simulator/SineSimulator.h"
#include "simulator/SineEngine.h"
#include "simulator/SineVehicle.h"
#include "simulator/SineTransmission.h"
#include "simulator/EngineSimTypes.h"
#include "hardware/NullAudioHardwareProvider.h"
#include "strategy/IAudioBuffer.h"
#include "common/ILogging.h"

#include <gtest/gtest.h>

#include <memory>
#include <string>

namespace {

using namespace input;

// ============================================================================
// Minimal Fake Implementations
// ============================================================================

// Fake audio buffer: only needs to satisfy createSession's initialize() call
// (first-run path) and the swapSimulator() call (handover path). We record the
// current simulator pointer so the swap is externally observable too.
class FakeAudioBuffer : public IAudioBuffer {
public:
    ISimulator* currentSimulator = nullptr;

    const char* getName() const override { return "FakeAudioBuffer"; }
    bool isEnabled() const override { return true; }
    bool isPlaying() const override { return isPlaying_; }

    bool render(AudioBufferView&) override { return true; }
    bool AddFrames(float*, int) override { return true; }

    bool initialize(const AudioBufferConfig&, int) override { return true; }
    void prepareBuffer() override {}

    bool startPlayback(ISimulator* simulator) override {
        currentSimulator = simulator;
        isPlaying_ = true;
        return true;
    }
    void stopPlayback(ISimulator*) override { isPlaying_ = false; }
    void swapSimulator(ISimulator* newSimulator) override { currentSimulator = newSimulator; }

    void resetBufferAfterWarmup() override {}
    bool shouldDrainDuringWarmup() const override { return false; }
    void fillBufferFromEngine(ISimulator*, int) override {}
    std::string getModeString() const override { return "FakeAudioBuffer"; }
    void reset() override {}
    void updateSimulation(ISimulator*, double) override {}

private:
    bool isPlaying_ = false;
};

// Fake logger — no-op (createSession logs via it; must be non-null).
class FakeLogger : public ILogging {
public:
    void setMask(uint32_t) override {}
    uint32_t getMask() const override { return 0; }
protected:
    void _write(uint32_t, const std::string&) override {}
};

// ============================================================================
// Fixture
// ============================================================================

// Build a sine BridgeSimulator that is LOADED but NOT created. createSession
// (first-run and handover) owns the create() call, so we must not pre-create.
// Returns the unique_ptr and stashes the raw ISimulator* for later identity checks.
std::unique_ptr<BridgeSimulator> buildSineBridge(const std::string& name, BridgeSimulator*& outRaw) {
    auto sineSim = std::make_unique<SineSimulator>();
    Simulator::Parameters simParams;
    simParams.systemType = Simulator::SystemType::NsvOptimized;
    sineSim->initialize(simParams);
    sineSim->setSimulationFrequency(EngineSimDefaults::SIMULATION_FREQUENCY);
    sineSim->setFluidSimulationSteps(EngineSimDefaults::FLUID_SIMULATION_STEPS);
    sineSim->setTargetSynthesizerLatency(EngineSimDefaults::TARGET_SYNTH_LATENCY);
    sineSim->loadSimulation(new SineEngine(), new SineVehicle(), new SineTransmission());

    auto bridge = std::make_unique<BridgeSimulator>(std::move(sineSim), name);
    outRaw = bridge.get();
    return bridge;
}

// A SimulationConfig that satisfies createSession's first-run ASSERTs
// (sampleRate > 0, updateInterval > 0, framesPerUpdate > 0).
SimulationConfig makeValidConfig() {
    SimulationConfig cfg;
    cfg.simulatorLabel = "HandoverTest";
    cfg.engineConfig.sampleRate = EngineSimDefaults::SAMPLE_RATE;
    cfg.engineConfig.simulationFrequency = EngineSimDefaults::SIMULATION_FREQUENCY;
    cfg.engineConfig.fluidSimulationSteps = EngineSimDefaults::FLUID_SIMULATION_STEPS;
    cfg.engineConfig.targetSynthesizerLatency = EngineSimDefaults::TARGET_SYNTH_LATENCY;
    return cfg;
}

// Seed an already-built (created) BridgeSimulator with a known engine phase and
// vehicle v_theta via public seams, so transferDrivetrainState's re-engagement
// block (Rollover/Cranking branches) is exercised on hot-swap.
void seedPhaseVtheta(BridgeSimulator& sim, EnginePhase phase, double vtheta) {
    // applyTransition with isTransition=true sets enginePhase_ to targetPhase.
    sim.applyTransition(TransitionDecision{phase, false, 0.0, /*isTransition*/ true});
    sim.getInternalSimulator()->getVehicleMassBody()->v_theta = vtheta;
}

class HandoverTest : public ::testing::Test {
protected:
    void SetUp() override {
        audioBuffer_ = std::make_unique<FakeAudioBuffer>();
        logger_ = std::make_unique<FakeLogger>();

        // SessionDependencies for createSession — only audioBuffer + logger are
        // dereferenced during session construction (no run() in these tests).
        deps_ = SessionDependencies{};
        deps_.audioBuffer = audioBuffer_.get();
        deps_.logger = logger_.get();

        config_ = makeValidConfig();
    }

    // Convenience: build a NULL (headless) audio provider for injection.
    static std::unique_ptr<IAudioHardwareProvider> nullProvider() {
        return std::make_unique<NullAudioHardwareProvider>();
    }

    std::unique_ptr<FakeAudioBuffer> audioBuffer_;
    std::unique_ptr<FakeLogger> logger_;
    SessionDependencies deps_;
    SimulationConfig config_;
};

// ============================================================================
// Tests
// ============================================================================

// 1. handoverSession swaps the live simulator held by the session.
TEST_F(HandoverTest, HandoverSwapsTheSimulator) {
    BridgeSimulator* seedRaw = nullptr;
    auto seed = buildSineBridge("Seed", seedRaw);

    auto session = createSession(config_, "seed.mr", std::move(seed), deps_,
                                 nullptr, nullProvider());
    ASSERT_NE(session, nullptr);
    ASSERT_EQ(session->getSimulator(), seedRaw);

    // Build a distinct NEW simulator and hot-swap it in.
    BridgeSimulator* newRaw = nullptr;
    auto incoming = buildSineBridge("Incoming", newRaw);
    ASSERT_NE(newRaw, seedRaw);  // sanity: distinct objects

    auto swapped = createSession(config_, "new.mr", std::move(incoming), deps_,
                                 std::move(session), nullProvider());
    ASSERT_NE(swapped, nullptr);

    // The session must now expose the NEW simulator, not the seed.
    EXPECT_EQ(swapped->getSimulator(), newRaw);
    EXPECT_NE(swapped->getSimulator(), seedRaw);

    // The audio buffer must also have been re-pointed at the new simulator.
    EXPECT_EQ(audioBuffer_->currentSimulator, newRaw);
}

// 2. Drivetrain state is transferred from the old simulator to the new one.
TEST_F(HandoverTest, HandoverTransfersDrivetrainState) {
    BridgeSimulator* seedRaw = nullptr;
    auto seed = buildSineBridge("Seed", seedRaw);

    auto session = createSession(config_, "seed.mr", std::move(seed), deps_,
                                 nullptr, nullProvider());
    ASSERT_NE(session, nullptr);

    // Configure a known drivetrain state on the OLD simulator via its exposed
    // sine vehicle body (test seam: getInternalSimulator()).
    // NOTE: we keep the transmission in NEUTRAL here. restoreDrivetrainState()
    // only calls Transmission::changeGear when snapshot.gear >= 0, and a forward
    // gear change recomputes v_theta from energy conservation (clobbering it).
    // Neutral therefore lets the seeded v_theta survive the transfer untouched —
    // the clean way to assert pure v_theta carry-over. (Gear carry-over is
    // asserted separately in HandoverTransfersDrivetrainGear.)
    BridgeSimulator* seedBridge = static_cast<BridgeSimulator*>(session->getSimulator());
    ASSERT_NE(seedBridge, nullptr);
    const double knownVtheta = 12.5;
    seedBridge->getInternalSimulator()->getVehicleMassBody()->v_theta = knownVtheta;

    // Capture the value the OLD simulator reports (pre-swap baseline).
    const double oldVtheta = seedBridge->captureDrivetrainState().vehicleMassVtheta;
    EXPECT_DOUBLE_EQ(oldVtheta, knownVtheta);

    // Hot-swap to a fresh simulator.
    BridgeSimulator* newRaw = nullptr;
    auto incoming = buildSineBridge("Incoming", newRaw);
    auto swapped = createSession(config_, "new.mr", std::move(incoming), deps_,
                                 std::move(session), nullProvider());
    ASSERT_NE(swapped, nullptr);

    BridgeSimulator* newBridge = static_cast<BridgeSimulator*>(swapped->getSimulator());
    ASSERT_EQ(newBridge, newRaw);

    // The NEW simulator must carry the transferred v_theta.
    const double newVtheta = newBridge->captureDrivetrainState().vehicleMassVtheta;
    EXPECT_DOUBLE_EQ(newVtheta, knownVtheta);
}

// 2b. Drivetrain GEAR carry-over (reviewer item #3). Seeding a forward gear and
//     asserting the new sim reports the SAME gear proves restoreDrivetrainState
//     re-applies the captured gear onto the new transmission. We seed v_theta=0
//     because a forward-gear restore recomputes v_theta from energy
//     conservation — so v_theta is intentionally NOT asserted here (it is in the
//     neutral-gear test above).
TEST_F(HandoverTest, HandoverTransfersDrivetrainGear) {
    BridgeSimulator* seedRaw = nullptr;
    auto seed = buildSineBridge("Seed", seedRaw);

    auto session = createSession(config_, "seed.mr", std::move(seed), deps_,
                                 nullptr, nullProvider());
    ASSERT_NE(session, nullptr);

    BridgeSimulator* seedBridge = static_cast<BridgeSimulator*>(session->getSimulator());
    ASSERT_NE(seedBridge, nullptr);
    seedBridge->setGear(1);  // forward gear
    const int knownGear = seedBridge->getGear();
    ASSERT_EQ(knownGear, 1);  // sanity: bridge convention forward = 1

    // Hot-swap to a fresh simulator.
    BridgeSimulator* newRaw = nullptr;
    auto incoming = buildSineBridge("Incoming", newRaw);
    auto swapped = createSession(config_, "new.mr", std::move(incoming), deps_,
                                 std::move(session), nullProvider());
    ASSERT_NE(swapped, nullptr);

    BridgeSimulator* newBridge = static_cast<BridgeSimulator*>(swapped->getSimulator());
    ASSERT_EQ(newBridge, newRaw);

    // The NEW simulator must report the SAME gear as the seed (carried over).
    EXPECT_EQ(newBridge->getGear(), knownGear);
}

// 3. handoverSession is idempotent — two consecutive swaps complete cleanly.
TEST_F(HandoverTest, HandoverIsIdempotentAcrossMultipleSwaps) {
    BridgeSimulator* seedRaw = nullptr;
    auto seed = buildSineBridge("Seed", seedRaw);
    auto session = createSession(config_, "seed.mr", std::move(seed), deps_,
                                 nullptr, nullProvider());
    ASSERT_NE(session, nullptr);
    ASSERT_EQ(session->getSimulator(), seedRaw);

    // First swap → A.
    BridgeSimulator* rawA = nullptr;
    auto simA = buildSineBridge("SimA", rawA);
    auto afterA = createSession(config_, "a.mr", std::move(simA), deps_,
                                std::move(session), nullProvider());
    ASSERT_NE(afterA, nullptr);
    EXPECT_EQ(afterA->getSimulator(), rawA);

    // Second swap → B (carried on from A's session).
    BridgeSimulator* rawB = nullptr;
    auto simB = buildSineBridge("SimB", rawB);
    auto afterB = createSession(config_, "b.mr", std::move(simB), deps_,
                                std::move(afterA), nullProvider());
    ASSERT_NE(afterB, nullptr);
    EXPECT_EQ(afterB->getSimulator(), rawB);
    EXPECT_NE(afterB->getSimulator(), rawA);
}

// 4. createSession with a null existingSession builds a fresh session
//    (first-run path — injection didn't break normal startup).
TEST_F(HandoverTest, CreateSessionWithoutExistingBuildsFreshSession) {
    BridgeSimulator* seedRaw = nullptr;
    auto seed = buildSineBridge("Seed", seedRaw);

    // Null existingSession (default) → first-run branch.
    auto session = createSession(config_, "seed.mr", std::move(seed), deps_,
                                 nullptr, nullProvider());

    ASSERT_NE(session, nullptr);
    // Fresh session exposes exactly the simulator we handed in (first-run
    // branch created the session around it rather than hot-swapping).
    EXPECT_EQ(session->getSimulator(), seedRaw);
}

// 5. Re-engagement branch: old simulator at Rollover with drivetrain momentum.
//    handoverSession must re-apply the Rollover phase to the new simulator
//    (transferDrivetrainState's hasDrivetrainMomentum → Rollover branch).
TEST_F(HandoverTest, HandoverReEngagesRolloverPhaseWhenMomentumPresent) {
    BridgeSimulator* seedRaw = nullptr;
    auto seed = buildSineBridge("Seed", seedRaw);

    auto session = createSession(config_, "seed.mr", std::move(seed), deps_,
                                 nullptr, nullProvider());
    ASSERT_NE(session, nullptr);
    BridgeSimulator* seedBridge = static_cast<BridgeSimulator*>(session->getSimulator());
    ASSERT_NE(seedBridge, nullptr);

    // Old sim: Rollover + non-zero road speed (momentum present).
    // NOTE: momentum requires snapshot.gear >= 0, so we seed a forward gear.
    // The sine transmission starts in neutral (gear=-1), which would otherwise
    // make hasDrivetrainMomentum() false and send us down the Cranking branch.
    // v_theta is seeded AFTER setGear because a forward-gear change recomputes
    // v_theta from energy conservation (clobbering it).
    seedBridge->setGear(1);
    seedPhaseVtheta(*seedBridge, EnginePhase::Rollover, /*vtheta*/ 12.5);
    ASSERT_EQ(seedBridge->getEnginePhase(), EnginePhase::Rollover);

    // Self-documenting invariant guard: confirm the seed actually carries
    // momentum BEFORE the swap. A forward-gear change recomputes v_theta from
    // energy conservation, so if setGear/seedPhaseVtheta are ever reordered
    // this fails loudly on the seed side (not the swap side), making the
    // Rollover-branch dependency on gear>=0 + v_theta>1.0 explicit.
    EXPECT_GT(seedBridge->captureDrivetrainState().vehicleMassVtheta, 1.0);
    EXPECT_GE(seedBridge->captureDrivetrainState().gear, 0);

    // Hot-swap to a fresh simulator.
    BridgeSimulator* newRaw = nullptr;
    auto incoming = buildSineBridge("Incoming", newRaw);
    auto swapped = createSession(config_, "new.mr", std::move(incoming), deps_,
                                 std::move(session), nullProvider());
    ASSERT_NE(swapped, nullptr);
    BridgeSimulator* newBridge = static_cast<BridgeSimulator*>(swapped->getSimulator());
    ASSERT_EQ(newBridge, newRaw);

    // New sim must carry the Rollover phase (re-engagement, not Stopped).
    EXPECT_EQ(newBridge->getEnginePhase(), EnginePhase::Rollover);
}

// 6. Re-engagement branch: old simulator at Cranking with NO momentum (v_theta=0).
//    handoverSession must re-apply the Cranking phase to the new simulator
//    (transferDrivetrainState's neutral/no-momentum → Cranking branch).
TEST_F(HandoverTest, HandoverReEngagesCrankingPhaseWhenNoMomentum) {
    BridgeSimulator* seedRaw = nullptr;
    auto seed = buildSineBridge("Seed", seedRaw);

    auto session = createSession(config_, "seed.mr", std::move(seed), deps_,
                                 nullptr, nullProvider());
    ASSERT_NE(session, nullptr);
    BridgeSimulator* seedBridge = static_cast<BridgeSimulator*>(session->getSimulator());
    ASSERT_NE(seedBridge, nullptr);

    // Old sim: Cranking + standstill (no momentum).
    seedPhaseVtheta(*seedBridge, EnginePhase::Cranking, /*vtheta*/ 0.0);
    ASSERT_EQ(seedBridge->getEnginePhase(), EnginePhase::Cranking);

    // Hot-swap to a fresh simulator.
    BridgeSimulator* newRaw = nullptr;
    auto incoming = buildSineBridge("Incoming", newRaw);
    auto swapped = createSession(config_, "new.mr", std::move(incoming), deps_,
                                 std::move(session), nullProvider());
    ASSERT_NE(swapped, nullptr);
    BridgeSimulator* newBridge = static_cast<BridgeSimulator*>(swapped->getSimulator());
    ASSERT_EQ(newBridge, newRaw);

    // New sim must carry the Cranking phase (starter re-engaged on the new engine).
    EXPECT_EQ(newBridge->getEnginePhase(), EnginePhase::Cranking);
}

}  // namespace
