// SimulatorFactoryBoundaryTests.cpp - configureLoadTorque boundary contracts
//
// Targets the genuinely-uncovered configureLoadTorque region (per lcov,
// SimulatorFactory.cpp:240-252): the loadFraction boundary (<=0 -> false,
// >0 -> true) and the "log only on success + when a logger is present" contract.
//
// Construction goes through the REAL SimulatorFactory::create(SineWave,...) path
// (production code), so configureLoadTorque's internal dynamic_cast<BridgeSimulator*>
// receives a genuine BridgeSimulator. The ILogging seam is a real DI boundary, so
// a minimal recording fake is appropriate (not a mock of our own classes).
//
// NOTE — CONFIRMED INIT BUG (escalated to impl via team-lead): createSineWaveSimulator's
// simulationFrequency<=0 fallback (lines 54-56) is NOT tested here because it is not
// observable through the returned simulator's public API. Root cause: SimulatorFactory::create()
// constructs the BridgeSimulator but never calls bridgeSim->create() / initAudioConfig() — the
// only place engineConfig_ is populated (BridgeSimulator.cpp:496) and the audio buffer sized
// (L504). The ctor (L16-19) sets only m_simulator + name_, leaving engineConfig_ at its default
// (simulationFrequency == 0). The fallback at 54-56 DOES run and sets the inner SineSimulator's
// frequency via initSimulator->setSimulationFrequency, but that's invisible through the returned
// ISimulator* (no inner-sim getter). Verified: ZERO callers of BridgeSimulator::create() exist in
// the app, CLI, or tests, so every factory-produced sim reports getSimulationFrequency()==0 and
// has an unsized audio buffer. Once impl fixes create() to init engineConfig_, three tests become
// pinnable (parked here as TDD red for post-fix): honors explicit positive frequency, falls back
// to SIMULATION_FREQUENCY on <=0, both observable via getSimulationFrequency(). We deliberately do
// NOT assert the current buggy 0-return — that would pin the bug as specified behavior.

#include "simulator/SimulatorFactory.h"
#include "simulator/EngineSimTypes.h"
#include "simulator/ISimulator.h"
#include "common/ILogging.h"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

namespace {

// Minimal ILogging fake: records whether _write was invoked and the mask it was
// called with. This is a genuine external boundary (DI logger), not a mock of the
// system under test.
class RecordingLogger : public ILogging {
public:
    void setMask(uint32_t mask) override { mask_ = mask; }
    uint32_t getMask() const override { return mask_; }

    int callCount() const { return callCount_; }
    uint32_t lastMask() const { return lastMask_; }

protected:
    void _write(uint32_t mask, const std::string& /*msg*/) override {
        ++callCount_;
        lastMask_ = mask;
    }

private:
    uint32_t mask_ = LogMask::ALL;
    int callCount_ = 0;
    uint32_t lastMask_ = 0;
};

// Build a real SineWave simulator via the production factory path.
std::unique_ptr<ISimulator> makeSineSimulator(double simulationFrequency) {
    ISimulatorConfig config;
    config.sampleRate = EngineSimDefaults::SAMPLE_RATE;
    config.simulationFrequency = simulationFrequency;
    config.fluidSimulationSteps = EngineSimDefaults::FLUID_SIMULATION_STEPS;
    config.targetSynthesizerLatency = EngineSimDefaults::TARGET_SYNTH_LATENCY;
    return SimulatorFactory::create(
        SimulatorType::SineWave,
        /*scriptPath*/ "",
        /*assetBasePath*/ "",
        config,
        /*logger*/ nullptr,
        /*telemetryWriter*/ nullptr);
}

}  // namespace

// --- configureLoadTorque: loadFraction boundary -------------------------------
// loadFraction <= 0 means "no load to configure" -> returns false without touching
// the simulator. Boundary contract, observable via the documented return.
TEST(SimulatorFactoryBoundaryTest, ConfigureLoadTorqueRejectsNonPositiveFraction) {
    auto sim = makeSineSimulator(EngineSimDefaults::SIMULATION_FREQUENCY);
    ASSERT_NE(sim, nullptr);

    EXPECT_FALSE(SimulatorFactory::configureLoadTorque(sim.get(), /*loadFraction*/ 0.0));
    EXPECT_FALSE(SimulatorFactory::configureLoadTorque(sim.get(), /*loadFraction*/ -0.5));
}

// loadFraction > 0 configures the dyno load -> returns true.
TEST(SimulatorFactoryBoundaryTest, ConfigureLoadTorqueAcceptsPositiveFraction) {
    auto sim = makeSineSimulator(EngineSimDefaults::SIMULATION_FREQUENCY);
    ASSERT_NE(sim, nullptr);

    EXPECT_TRUE(SimulatorFactory::configureLoadTorque(sim.get(), /*loadFraction*/ 0.5));
}

// --- configureLoadTorque: log-only-on-success contract ------------------------
// On a successful configure WITH a logger, exactly one info line is written under
// the BRIDGE category. On a declined configure (loadFraction<=0), nothing is
// logged. Pins the "don't log on failure" branch honestly via the DI seam.
TEST(SimulatorFactoryBoundaryTest, ConfigureLoadTorqueLogsOnceOnSuccessOnly) {
    auto sim = makeSineSimulator(EngineSimDefaults::SIMULATION_FREQUENCY);
    ASSERT_NE(sim, nullptr);
    RecordingLogger logger;

    ASSERT_TRUE(SimulatorFactory::configureLoadTorque(sim.get(), /*loadFraction*/ 0.5, &logger));
    EXPECT_EQ(logger.callCount(), 1);
    EXPECT_NE(logger.lastMask() & LogMask::BRIDGE, 0u);   // BRIDGE category set
    EXPECT_NE(logger.lastMask() & LogMask::INFO, 0u);     // INFO level set
}

TEST(SimulatorFactoryBoundaryTest, ConfigureLoadTorqueDoesNotLogOnDecline) {
    auto sim = makeSineSimulator(EngineSimDefaults::SIMULATION_FREQUENCY);
    ASSERT_NE(sim, nullptr);
    RecordingLogger logger;

    EXPECT_FALSE(SimulatorFactory::configureLoadTorque(sim.get(), /*loadFraction*/ 0.0, &logger));
    EXPECT_EQ(logger.callCount(), 0);
}
