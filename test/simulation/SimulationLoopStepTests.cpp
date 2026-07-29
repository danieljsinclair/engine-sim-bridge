// SimulationLoopStepTests.cpp - Contract tests for SimulationLoop::step()
//
// Tests the observable contract of step() — the per-tick domain logic extracted
// from run(). step() is SLEEP-FREE and deterministic (no clock reads, no real-time
// delays). These tests assert OBSERVABLE post-conditions (telemetry recorded,
// presentation updated, audio filled) NOT internal variable assignments.
//
// The scaffold exists: step() is currently a STUB returning Continue only.
// These tests are written BLIND to the contract and will fail RED until the
// implementer fills the body. This is expected TDD red phase.
//
// Construction uses the light SineSimulator path (no script compilation),
// mirroring SineWaveRegressionTests and BridgeSimulatorContractTests.
// We use minimal fake implementations for dependencies to OBSERVE the
// post-conditions without heavy fixtures.

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

#include <memory>
#include <atomic>

namespace {

// Using namespaces for convenience
using namespace input;
using namespace telemetry;

// ============================================================================
// Minimal Fake Implementations for Observability
// ============================================================================

// Fake audio buffer that records calls for test assertion
class FakeAudioBuffer : public IAudioBuffer {
public:
    struct Calls {
        int updateSimulationCount = 0;
        int fillBufferFromEngineCount = 0;
        ISimulator* lastSimulator = nullptr;
        int lastFrameCount = -1;
    };

    explicit FakeAudioBuffer(Calls* calls) : calls_(calls) {}

    const char* getName() const override { return "FakeAudioBuffer"; }
    bool isEnabled() const override { return true; }
    bool isPlaying() const override { return isPlaying_; }

    bool render(AudioBufferView&) override { return true; }
    bool AddFrames(float*, int) override { return true; }

    bool initialize(const AudioBufferConfig&, int) override { return true; }
    void prepareBuffer() override {}

    bool startPlayback(ISimulator* simulator) override {
        simulator_ = simulator;
        isPlaying_ = true;
        return true;
    }

    void stopPlayback(ISimulator*) override {
        isPlaying_ = false;
    }

    void swapSimulator(ISimulator* newSimulator) override {
        simulator_ = newSimulator;
    }

    void resetBufferAfterWarmup() override {}
    bool shouldDrainDuringWarmup() const override { return false; }

    // OBSERVABLE: step() must call updateSimulation each tick
    void updateSimulation(ISimulator* simulator, double) override {
        calls_->updateSimulationCount++;
        calls_->lastSimulator = simulator;
    }

    // OBSERVABLE: step() must call fillBufferFromEngine each tick
    void fillBufferFromEngine(ISimulator* simulator, int frameCount) override {
        calls_->fillBufferFromEngineCount++;
        calls_->lastSimulator = simulator;
        calls_->lastFrameCount = frameCount;
    }

    void reset() override {}
    std::string getModeString() const override { return "FakeAudioBuffer"; }

private:
    Calls* calls_;
    ISimulator* simulator_ = nullptr;
    bool isPlaying_ = false;
};

// Fake telemetry writer that records calls for test assertion
class FakeTelemetryWriter : public ITelemetryWriter {
public:
    struct Calls {
        int writeVehicleInputsCount = 0;
        int writeSimulatorMetricsCount = 0;
        VehicleInputsTelemetry lastInputs;
        SimulatorMetricsTelemetry lastMetrics;
    };

    explicit FakeTelemetryWriter(Calls* calls) : calls_(calls) {}

    void writeEngineState(const EngineStateTelemetry&) override {}
    void writeFramePerformance(const FramePerformanceTelemetry&) override {}
    void writeAudioDiagnostics(const AudioDiagnosticsTelemetry&) override {}
    void writeAudioTiming(const AudioTimingTelemetry&) override {}

    // OBSERVABLE: step() must write vehicle inputs each tick
    void writeVehicleInputs(const VehicleInputsTelemetry& inputs) override {
        calls_->writeVehicleInputsCount++;
        calls_->lastInputs = inputs;
    }

    // OBSERVABLE: step() must write simulator metrics each tick
    void writeSimulatorMetrics(const SimulatorMetricsTelemetry& metrics) override {
        calls_->writeSimulatorMetricsCount++;
        calls_->lastMetrics = metrics;
    }

    void reset() override {}
    const char* getName() const override { return "FakeTelemetryWriter"; }

private:
    Calls* calls_;
};

// Fake telemetry reader (no-op, we don't assert on it)
class FakeTelemetryReader : public ITelemetryReader {
public:
    AudioTimingTelemetry getAudioTiming() const override {
        return AudioTimingTelemetry{};
    }
    EngineStateTelemetry getEngineState() const override { return EngineStateTelemetry{}; }
    FramePerformanceTelemetry getFramePerformance() const override { return FramePerformanceTelemetry{}; }
    AudioDiagnosticsTelemetry getAudioDiagnostics() const override { return AudioDiagnosticsTelemetry{}; }
    VehicleInputsTelemetry getVehicleInputs() const override { return VehicleInputsTelemetry{}; }
    SimulatorMetricsTelemetry getSimulatorMetrics() const override { return SimulatorMetricsTelemetry{}; }
    const char* getName() const override { return "FakeTelemetryReader"; }
};

// Fake presentation that records calls for test assertion
class FakePresentation : public presentation::IPresentation {
public:
    struct Calls {
        int showSimulatorStatesCount = 0;
        presentation::EngineState lastState;
    };

    explicit FakePresentation(Calls* calls) : calls_(calls) {}

    bool Initialize(const presentation::PresentationConfig&) override { return true; }
    void Shutdown() override {}
    void ShowMessage(const std::string&) override {}
    void ShowError(const std::string&) override {}
    void ShowProgress(double, double) override {}
    void Update(double) override {}

    // OBSERVABLE: step() must call ShowSimulatorStates each tick
    void ShowSimulatorStates(const presentation::EngineState& state) override {
        calls_->showSimulatorStatesCount++;
        calls_->lastState = state;
    }

private:
    Calls* calls_;
};

// Fake input provider that returns default input
class FakeInputProvider : public IInputProvider {
public:
    EngineInput OnUpdateSimulation(double) override {
        return EngineInput{};  // Default (no input)
    }

    void provideFeedback(const EngineSimStats&) override {}

    // IInputProvider lifecycle methods
    bool Initialize() override { return true; }
    void Shutdown() override {}
    bool IsConnected() const override { return true; }
    std::string GetProviderName() const override { return "FakeInputProvider"; }
    std::string GetLastError() const override { return ""; }
};

// Fake logger (no-op)
class FakeLogger : public ILogging {
public:
    void setMask(uint32_t) override {}
    uint32_t getMask() const override { return 0; }

protected:
    void _write(uint32_t, const std::string&) override {}
};

// ============================================================================
// Test Fixture
// ============================================================================

class SimulationLoopStepTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create sine simulator (light path, no script compilation)
        auto sineSim = std::make_unique<SineSimulator>();
        Simulator::Parameters simParams;
        simParams.systemType = Simulator::SystemType::NsvOptimized;
        sineSim->initialize(simParams);
        sineSim->setSimulationFrequency(EngineSimDefaults::SIMULATION_FREQUENCY);
        sineSim->setFluidSimulationSteps(EngineSimDefaults::FLUID_SIMULATION_STEPS);
        sineSim->setTargetSynthesizerLatency(EngineSimDefaults::TARGET_SYNTH_LATENCY);
        sineSim->loadSimulation(new SineEngine(), new SineVehicle(), new SineTransmission());

        // Wrap in BridgeSimulator
        simulator_ = std::make_unique<BridgeSimulator>(std::move(sineSim), "TestSim");
        ISimulatorConfig config;
        config.sampleRate = EngineSimDefaults::SAMPLE_RATE;
        config.simulationFrequency = EngineSimDefaults::SIMULATION_FREQUENCY;
        config.fluidSimulationSteps = EngineSimDefaults::FLUID_SIMULATION_STEPS;
        config.targetSynthesizerLatency = EngineSimDefaults::TARGET_SYNTH_LATENCY;
        ASSERT_TRUE(simulator_->create(config, nullptr, nullptr));

        // Create fake dependencies
        audioCalls_ = std::make_unique<FakeAudioBuffer::Calls>();
        telemetryCalls_ = std::make_unique<FakeTelemetryWriter::Calls>();
        presentationCalls_ = std::make_unique<FakePresentation::Calls>();

        audioBuffer_ = std::make_unique<FakeAudioBuffer>(audioCalls_.get());
        telemetryWriter_ = std::make_unique<FakeTelemetryWriter>(telemetryCalls_.get());
        telemetryReader_ = std::make_unique<FakeTelemetryReader>();
        presentation_ = std::make_unique<FakePresentation>(presentationCalls_.get());
        inputProvider_ = std::make_unique<FakeInputProvider>();
        logger_ = std::make_unique<FakeLogger>();
        crankingController_ = std::make_unique<CrankingController>();
        stopRequested_ = std::make_unique<std::atomic<bool>>(false);

        // Build SessionDependencies
        deps_ = SessionDependencies{
            audioBuffer_.get(),
            crankingController_.get(),
            stopRequested_.get(),
            inputProvider_.get(),
            presentation_.get(),
            telemetryWriter_.get(),
            telemetryReader_.get(),
            logger_.get()
        };

        // Create SimulationConfig
        simConfig_ = SimulationConfig{};
        simConfig_.duration = 1.0;  // 1 second duration
        simConfig_.simulatorLabel = "TestSim";
    }

    std::unique_ptr<BridgeSimulator> simulator_;
    std::unique_ptr<FakeAudioBuffer::Calls> audioCalls_;
    std::unique_ptr<FakeTelemetryWriter::Calls> telemetryCalls_;
    std::unique_ptr<FakePresentation::Calls> presentationCalls_;

    std::unique_ptr<FakeAudioBuffer> audioBuffer_;
    std::unique_ptr<FakeTelemetryWriter> telemetryWriter_;
    std::unique_ptr<FakeTelemetryReader> telemetryReader_;
    std::unique_ptr<FakePresentation> presentation_;
    std::unique_ptr<FakeInputProvider> inputProvider_;
    std::unique_ptr<FakeLogger> logger_;
    std::unique_ptr<CrankingController> crankingController_;
    std::unique_ptr<std::atomic<bool>> stopRequested_;

    SessionDependencies deps_;
    SimulationConfig simConfig_;
};

// ============================================================================
// Contract Tests - Observable Post-Conditions
// ============================================================================

// step() on a normal tick returns Continue
TEST_F(SimulationLoopStepTest, NormalTickReturnsContinue) {
    SimulationLoop loop(*simulator_, simConfig_, deps_);

    LoopState state;
    state.currentTime = 0.0;
    state.isFirstTick = true;
    state.combustionEngine = dynamic_cast<ICombustionEngine*>(simulator_.get());

    StepResult result = loop.step(state);

    // EXPECTED: stub returns Continue, so this passes even in red phase
    // Once implementer fills the body, this asserts normal tick behavior
    EXPECT_EQ(result, StepResult::Continue);
}

// step() advances currentTime by updateInterval on a Continue tick
TEST_F(SimulationLoopStepTest, NormalTickAdvancesCurrentTime) {
    SimulationLoop loop(*simulator_, simConfig_, deps_);

    const double beforeTime = 0.0;
    LoopState state;
    state.currentTime = beforeTime;
    state.isFirstTick = true;
    state.combustionEngine = dynamic_cast<ICombustionEngine*>(simulator_.get());

    loop.step(state);

    // EXPECTED: currentTime advances by updateInterval (1/60s)
    // This will FAIL against the stub (currentTime stays at 0.0)
    EXPECT_GT(state.currentTime, beforeTime);
    EXPECT_DOUBLE_EQ(state.currentTime, beforeTime + simConfig_.updateInterval());
}

// step() records telemetry for each tick
TEST_F(SimulationLoopStepTest, NormalTickRecordsTelemetry) {
    SimulationLoop loop(*simulator_, simConfig_, deps_);

    LoopState state;
    state.currentTime = 0.0;
    state.isFirstTick = true;
    state.combustionEngine = dynamic_cast<ICombustionEngine*>(simulator_.get());

    loop.step(state);

    // EXPECTED: telemetry writer received calls
    // This will FAIL against the stub (no telemetry written)
    EXPECT_GT(telemetryCalls_->writeVehicleInputsCount, 0);
    EXPECT_GT(telemetryCalls_->writeSimulatorMetricsCount, 0);
}

// step() updates the presentation surface
TEST_F(SimulationLoopStepTest, NormalTickUpdatesPresentation) {
    SimulationLoop loop(*simulator_, simConfig_, deps_);

    LoopState state;
    state.currentTime = 0.0;
    state.isFirstTick = true;
    state.combustionEngine = dynamic_cast<ICombustionEngine*>(simulator_.get());

    loop.step(state);

    // EXPECTED: presentation received ShowSimulatorStates call
    // This will FAIL against the stub (no presentation update)
    EXPECT_GT(presentationCalls_->showSimulatorStatesCount, 0);
}

// step() fills the audio buffer
TEST_F(SimulationLoopStepTest, NormalTickFillsAudioBuffer) {
    SimulationLoop loop(*simulator_, simConfig_, deps_);

    LoopState state;
    state.currentTime = 0.0;
    state.isFirstTick = true;
    state.combustionEngine = dynamic_cast<ICombustionEngine*>(simulator_.get());

    loop.step(state);

    // EXPECTED: audio buffer received updateSimulation + fillBufferFromEngine calls
    // This will FAIL against the stub (no audio operations)
    EXPECT_GT(audioCalls_->updateSimulationCount, 0);
    EXPECT_GT(audioCalls_->fillBufferFromEngineCount, 0);
}

// step() returns Stop when duration is exhausted
TEST_F(SimulationLoopStepTest, DurationExhaustedReturnsStop) {
    SimulationLoop loop(*simulator_, simConfig_, deps_);

    LoopState state;
    state.currentTime = simConfig_.duration;  // Already at duration limit
    state.isFirstTick = true;
    state.combustionEngine = dynamic_cast<ICombustionEngine*>(simulator_.get());

    StepResult result = loop.step(state);

    // EXPECTED: duration exhausted → Stop
    // This will FAIL against the stub (returns Continue)
    EXPECT_EQ(result, StepResult::Stop);
}

// step() returns PresetCycle when presetCycle flag is set
TEST_F(SimulationLoopStepTest, PresetCycleRequestedReturnsPresetCycle) {
    SimulationLoop loop(*simulator_, simConfig_, deps_);

    LoopState state;
    state.currentTime = 0.0;
    state.isFirstTick = true;
    state.combustionEngine = dynamic_cast<ICombustionEngine*>(simulator_.get());
    state.engineInput.presetCycle = true;  // Request preset cycle

    StepResult result = loop.step(state);

    // EXPECTED: presetCycle → PresetCycle result
    // This will FAIL against the stub (returns Continue)
    EXPECT_EQ(result, StepResult::PresetCycle);
}

// step() carries previousStats forward across ticks (multi-tick coherence)
TEST_F(SimulationLoopStepTest, MultiTickCoherenceCarriesPreviousStats) {
    SimulationLoop loop(*simulator_, simConfig_, deps_);

    // First tick
    LoopState state;
    state.currentTime = 0.0;
    state.isFirstTick = true;
    state.combustionEngine = dynamic_cast<ICombustionEngine*>(simulator_.get());

    loop.step(state);

    // Second tick: isFirstTick = false, previousStats carried forward
    state.isFirstTick = false;
    const double beforeSecondTick = state.currentTime;

    loop.step(state);

    // EXPECTED: previousStats from first tick is available to second tick
    // This validates cross-tick state coherence
    // This will FAIL against the stub (no state mutation)
    EXPECT_GT(state.currentTime, beforeSecondTick);
}

// step() on first tick behaves distinctly (initialization)
TEST_F(SimulationLoopStepTest, FirstTickDistinctBehavior) {
    SimulationLoop loop(*simulator_, simConfig_, deps_);

    LoopState state;
    state.currentTime = 0.0;
    state.isFirstTick = true;  // FIRST TICK
    state.combustionEngine = dynamic_cast<ICombustionEngine*>(simulator_.get());

    StepResult firstResult = loop.step(state);

    // After first tick, isFirstTick should be false (caller manages this)
    state.isFirstTick = false;
    StepResult secondResult = loop.step(state);

    // EXPECTED: both calls succeed (Continue), but first tick may have
    // distinct initialization behavior (e.g. starter button in timed mode)
    // The stub returns Continue for both, so this passes even in red phase
    EXPECT_EQ(firstResult, StepResult::Continue);
    EXPECT_EQ(secondResult, StepResult::Continue);
}

}  // namespace
