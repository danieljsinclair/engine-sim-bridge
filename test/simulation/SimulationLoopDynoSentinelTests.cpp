// SimulationLoopDynoSentinelTests.cpp - Owner-decreed contract test for the
// dynoTorqueScale == -1.0 sentinel.
//
// Contract: -1.0 is a sentinal meaning "no change". It MUST be swallowed at the
// SimulationLoop layer (applyDynoControl, SimulationLoop.cpp:155) BEFORE reaching
// BridgeSimulator::setDynoTorqueScale, so calling with -1.0 is a guaranteed
// NO-OP: setDynoTorqueScale is never invoked and the last-applied scale is
// preserved. This enforces the (intentionally crappy) sentinel contract and
// guards against regressions where the sentinel leaks through to the simulator.
//
// Driven deterministically via step() with a RecordingSimulator, mirroring
// SimulationLoopVehicleControlsTests. No production code is modified, and the
// test does NOT depend on the BridgeSimulator::setDynoTorqueScale assert change
// (the -1.0 never reaches that method).

#include "simulation/SimulationLoop.h"
#include "simulator/BridgeSimulator.h"
#include "simulator/SineSimulator.h"
#include "simulator/SineEngine.h"
#include "simulator/SineVehicle.h"
#include "simulator/SineTransmission.h"
#include "simulator/EngineSimTypes.h"
#include "simulator/ISimulator.h"
#include "simulator/ICombustionEngine.h"
#include "io/IInputProvider.h"
#include "io/IPresentation.h"
#include "telemetry/ITelemetryProvider.h"
#include "strategy/IAudioBuffer.h"
#include "common/ILogging.h"
#include "simulation/CrankingController.h"

#include <gtest/gtest.h>

#include <atomic>
#include <cstring>
#include <memory>
#include <string>

namespace {

using namespace input;
using namespace telemetry;

// Recording BridgeSimulator: intercepts setDynoTorqueScale so the dyno sentinel
// fork inside applyDynoControl becomes observable. Engine audio/sim behaviour is
// delegated to the real Sine-backed base.
class RecordingSimulator : public BridgeSimulator {
public:
    struct Calls {
        int setDynoTorqueScaleCount = 0;
        double lastDynoScale = -2.0;
    };

    RecordingSimulator(std::unique_ptr<Simulator> sim, const std::string& name, Calls* calls)
        : BridgeSimulator(std::move(sim), name), calls_(calls) {}

    void setDynoTorqueScale(double scale) override {
        calls_->setDynoTorqueScaleCount++;
        calls_->lastDynoScale = scale;
        BridgeSimulator::setDynoTorqueScale(scale);
    }

private:
    Calls* calls_;
};

// ---- Minimal fakes (same shape as the step/vehicle-controls scaffolds) ----

class FakeAudioBuffer : public IAudioBuffer {
public:
    const char* getName() const override { return "FakeAudioBuffer"; }
    bool isEnabled() const override { return true; }
    bool isPlaying() const override { return false; }
    bool render(AudioBufferView&) override { return true; }
    bool AddFrames(float*, int) override { return true; }
    bool initialize(const AudioBufferConfig&, int) override { return true; }
    void prepareBuffer() override {}
    bool startPlayback(ISimulator*) override { return true; }
    void stopPlayback(ISimulator*) override {}
    void swapSimulator(ISimulator*) override {}
    void resetBufferAfterWarmup() override {}
    bool shouldDrainDuringWarmup() const override { return false; }
    void updateSimulation(ISimulator*, double) override {}
    void fillBufferFromEngine(ISimulator*, int) override {}
    void reset() override {}
    std::string getModeString() const override { return "FakeAudioBuffer"; }
};

class FakeTelemetryWriter : public ITelemetryWriter {
public:
    void writeEngineState(const EngineStateTelemetry&) override {}
    void writeFramePerformance(const FramePerformanceTelemetry&) override {}
    void writeAudioDiagnostics(const AudioDiagnosticsTelemetry&) override {}
    void writeAudioTiming(const AudioTimingTelemetry&) override {}
    void writeVehicleInputs(const VehicleInputsTelemetry&) override {}
    void writeSimulatorMetrics(const SimulatorMetricsTelemetry&) override {}
    void reset() override {}
    const char* getName() const override { return "FakeTelemetryWriter"; }
};

class FakeTelemetryReader : public ITelemetryReader {
public:
    AudioTimingTelemetry getAudioTiming() const override { return AudioTimingTelemetry{}; }
    EngineStateTelemetry getEngineState() const override { return EngineStateTelemetry{}; }
    FramePerformanceTelemetry getFramePerformance() const override { return FramePerformanceTelemetry{}; }
    AudioDiagnosticsTelemetry getAudioDiagnostics() const override { return AudioDiagnosticsTelemetry{}; }
    VehicleInputsTelemetry getVehicleInputs() const override { return VehicleInputsTelemetry{}; }
    SimulatorMetricsTelemetry getSimulatorMetrics() const override { return SimulatorMetricsTelemetry{}; }
    const char* getName() const override { return "FakeTelemetryReader"; }
};

class FakePresentation : public presentation::IPresentation {
public:
    bool Initialize(const presentation::PresentationConfig&) override { return true; }
    void Shutdown() override {}
    void ShowMessage(const std::string&) override {}
    void ShowError(const std::string&) override {}
    void ShowProgress(double, double) override {}
    void Update(double) override {}
    void ShowSimulatorStates(const presentation::EngineState&) override {}
};

class FakeLogger : public ILogging {
public:
    void setMask(uint32_t) override {}
    uint32_t getMask() const override { return 0; }
protected:
    void _write(uint32_t, const std::string&) override {}
};

class FakeInputProvider : public IInputProvider {
public:
    EngineInput OnUpdateSimulation(double) override { return EngineInput{}; }
    void provideFeedback(const EngineSimStats&) override {}
    bool Initialize() override { return true; }
    void Shutdown() override {}
    bool IsConnected() const override { return true; }
    std::string GetProviderName() const override { return "FakeInputProvider"; }
    std::string GetLastError() const override { return ""; }
};

// ============================================================================
// Fixture
// ============================================================================

class DynoSentinelTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto sineSim = std::make_unique<SineSimulator>();
        Simulator::Parameters simParams;
        simParams.systemType = Simulator::SystemType::NsvOptimized;
        sineSim->initialize(simParams);
        sineSim->setSimulationFrequency(EngineSimDefaults::SIMULATION_FREQUENCY);
        sineSim->setFluidSimulationSteps(EngineSimDefaults::FLUID_SIMULATION_STEPS);
        sineSim->setTargetSynthesizerLatency(EngineSimDefaults::TARGET_SYNTH_LATENCY);
        sineSim->loadSimulation(new SineEngine(), new SineVehicle(), new SineTransmission());

        calls_ = std::make_unique<RecordingSimulator::Calls>();
        simulator_ = std::make_unique<RecordingSimulator>(
            std::move(sineSim), "RecordSim", calls_.get());

        ISimulatorConfig config;
        config.sampleRate = EngineSimDefaults::SAMPLE_RATE;
        config.simulationFrequency = EngineSimDefaults::SIMULATION_FREQUENCY;
        config.fluidSimulationSteps = EngineSimDefaults::FLUID_SIMULATION_STEPS;
        config.targetSynthesizerLatency = EngineSimDefaults::TARGET_SYNTH_LATENCY;
        ASSERT_TRUE(simulator_->create(config, nullptr, nullptr));

        audioBuffer_ = std::make_unique<FakeAudioBuffer>();
        telemetryWriter_ = std::make_unique<FakeTelemetryWriter>();
        telemetryReader_ = std::make_unique<FakeTelemetryReader>();
        presentation_ = std::make_unique<FakePresentation>();
        logger_ = std::make_unique<FakeLogger>();
        crankingController_ = std::make_unique<CrankingController>();
        stopRequested_ = std::make_unique<std::atomic<bool>>(false);
        inputProvider_ = std::make_unique<FakeInputProvider>();

        simConfig_ = SimulationConfig{};
        simConfig_.duration = 1.0;
        simConfig_.simulatorLabel = "RecordSim";
    }

    SessionDependencies buildDeps() {
        return SessionDependencies{
            audioBuffer_.get(),
            crankingController_.get(),
            stopRequested_.get(),
            inputProvider_.get(),
            presentation_.get(),
            telemetryWriter_.get(),
            telemetryReader_.get(),
            logger_.get()
        };
    }

    // Build a LoopState for a deterministic tick with the given input.
    LoopState makeState(input::EngineInput input) {
        LoopState state;
        state.currentTime = 0.0;
        state.isFirstTick = true;
        state.combustionEngine = dynamic_cast<ICombustionEngine*>(simulator_.get());
        state.engineInput = input;
        return state;
    }

    std::unique_ptr<RecordingSimulator::Calls> calls_;
    std::unique_ptr<RecordingSimulator> simulator_;
    std::unique_ptr<FakeAudioBuffer> audioBuffer_;
    std::unique_ptr<FakeTelemetryWriter> telemetryWriter_;
    std::unique_ptr<FakeTelemetryReader> telemetryReader_;
    std::unique_ptr<FakePresentation> presentation_;
    std::unique_ptr<FakeLogger> logger_;
    std::unique_ptr<CrankingController> crankingController_;
    std::unique_ptr<std::atomic<bool>> stopRequested_;
    std::unique_ptr<FakeInputProvider> inputProvider_;
    SimulationConfig simConfig_;
};

// After a real scale (0.5) is applied, sending -1.0 must NOT call
// setDynoTorqueScale again and must preserve the last-applied scale (0.5).
// This exercises the `scale < 0.0` sentinel guard in applyDynoControl.
TEST_F(DynoSentinelTest, NegativeOneScaleIsSwallowedAsNoOp) {
    input::EngineInput input;
    input.ignition = true;
    input.dynoTorqueScale = 0.5;  // a real change, applied on the first tick

    SimulationLoop loop(*simulator_, simConfig_, buildDeps());
    LoopState state = makeState(input);
    loop.step(state);  // applies 0.5; lastDynoTorqueScale -> 0.5

    ASSERT_EQ(calls_->setDynoTorqueScaleCount, 1);
    ASSERT_DOUBLE_EQ(calls_->lastDynoScale, 0.5);

    // Now send the -1.0 sentinal on the next tick (lastDynoTorqueScale carries 0.5).
    input.dynoTorqueScale = -1.0;
    state.engineInput = input;
    loop.step(state);  // must swallow the sentinal, not propagate it

    EXPECT_EQ(calls_->setDynoTorqueScaleCount, 1)
        << "setDynoTorqueScale must NOT be called for the -1.0 sentinal";
    EXPECT_DOUBLE_EQ(calls_->lastDynoScale, 0.5)
        << "last applied dyno scale must be preserved across the sentinal";
}

// On a fresh state (lastDynoTorqueScale starts at -1.0), a -1.0 input is also a
// no-op: setDynoTorqueScale is never called. This guards the unchanged-sentinel
// guard (scale == lastScale) from ever leaking a -1.0 downstream.
TEST_F(DynoSentinelTest, NegativeOneFromFreshStateIsNoOp) {
    input::EngineInput input;
    input.ignition = true;
    input.dynoTorqueScale = -1.0;  // equals the initial lastDynoTorqueScale (-1.0)

    SimulationLoop loop(*simulator_, simConfig_, buildDeps());
    LoopState state = makeState(input);
    loop.step(state);

    EXPECT_EQ(calls_->setDynoTorqueScaleCount, 0)
        << "-1.0 from a fresh state must not call setDynoTorqueScale";
}

}  // namespace
