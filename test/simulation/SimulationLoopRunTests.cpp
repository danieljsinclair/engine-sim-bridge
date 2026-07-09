// SimulationLoopRunTests.cpp - Contract tests for SimulationLoop::run()
//
// These cover the run() control surface (currently 0% covered branches): the
// loop pacing is driven deterministically by an injected FakeLoopClock (no-op
// waitUntilNextTick) so run() is host-driven and completes without real-time
// sleeps. Construction mirrors SimulationLoopStepTests.cpp using the light
// SineSimulator path. We additionally drive:
//   - run() with a NON-combustion ISimulator mock: exercises the
//     applyCrankingDecision combustionEngine==null branch and returns 0 (Stop)
//     on duration.
//   - run() with engineInput.presetCycle requested mid-run -> EXIT_BUT_CONTINUE_NEXT (2)
//   - run() reaching duration -> returns 0 (Stop)
//   - run() with stopRequested_ set -> returns 0 (Stop)

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
#include "simulation/ILoopClock.h"

#include <gtest/gtest.h>

#include <atomic>
#include <cstring>
#include <memory>
#include <string>

namespace {

using namespace input;
using namespace telemetry;

// Minimal non-combustion ISimulator: returns a null combustion engine via the
// dynamic_cast in run(), exercising the combustionEngine==null cranking branch.
class NonCombustionSimulator : public ISimulator {
public:
    bool create(const ISimulatorConfig&, ILogging*, telemetry::ITelemetryWriter*) override { return true; }
    void destroy() override {}
    std::string getLastError() const override { return ""; }
    const char* getName() const override { return "NonCombustionSim"; }
    void update(double) override {}
    EngineSimStats getStats() const override { return EngineSimStats{}; }
    void setThrottle(double) override {}
    bool renderOnDemand(float*, int32_t, int32_t*) override { return false; }
    bool readAudioBuffer(float*, int32_t, int32_t*) override { return false; }
    bool start() override { return true; }
    void stop() override {}
    int getSimulationFrequency() const override { return 10000; }
};

// Input provider that can be toggled to request a preset cycle on the first tick.
class PresetCycleInputProvider : public IInputProvider {
public:
    EngineInput OnUpdateSimulation(double) override {
        EngineInput input;
        if (!fired_) {
            input.presetCycle = true;
            fired_ = true;
        }
        return input;
    }
    void provideFeedback(const EngineSimStats&) override {}
    bool Initialize() override { return true; }
    void Shutdown() override {}
    bool IsConnected() const override { return true; }
    std::string GetProviderName() const override { return "PresetCycleInputProvider"; }
    std::string GetLastError() const override { return ""; }
private:
    bool fired_ = false;
};

// ---- Fakes reused from the step-test scaffold (kept local & minimal) ----

class FakeAudioBuffer : public IAudioBuffer {
public:
    struct Calls { int updateSimulationCount = 0; int fillBufferFromEngineCount = 0; };
    explicit FakeAudioBuffer(Calls* c) : calls_(c) {}
    const char* getName() const override { return "FakeAudioBuffer"; }
    bool isEnabled() const override { return true; }
    bool isPlaying() const override { return isPlaying_; }
    bool render(AudioBufferView&) override { return true; }
    bool AddFrames(float*, int) override { return true; }
    bool initialize(const AudioBufferConfig&, int) override { return true; }
    void prepareBuffer() override {}
    bool startPlayback(ISimulator* s) override { sim_ = s; isPlaying_ = true; return true; }
    void stopPlayback(ISimulator*) override { isPlaying_ = false; }
    void swapSimulator(ISimulator* s) override { sim_ = s; }
    void resetBufferAfterWarmup() override {}
    bool shouldDrainDuringWarmup() const override { return false; }
    void updateSimulation(ISimulator*, double) override { calls_->updateSimulationCount++; }
    void fillBufferFromEngine(ISimulator*, int) override { calls_->fillBufferFromEngineCount++; }
    void reset() override {}
    std::string getModeString() const override { return "FakeAudioBuffer"; }
private:
    Calls* calls_; ISimulator* sim_ = nullptr; bool isPlaying_ = false;
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

// ============================================================================
// Fixture — builds a real BridgeSimulator (combustion) by default, plus shared
// deps and an injected FakeLoopClock.
// ============================================================================

class SimulationLoopRunTest : public ::testing::Test {
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

        simulator_ = std::make_unique<BridgeSimulator>(std::move(sineSim), "TestSim");
        ISimulatorConfig config;
        config.sampleRate = EngineSimDefaults::SAMPLE_RATE;
        config.simulationFrequency = EngineSimDefaults::SIMULATION_FREQUENCY;
        config.fluidSimulationSteps = EngineSimDefaults::FLUID_SIMULATION_STEPS;
        config.targetSynthesizerLatency = EngineSimDefaults::TARGET_SYNTH_LATENCY;
        ASSERT_TRUE(simulator_->create(config, nullptr, nullptr));

        audioCalls_ = std::make_unique<FakeAudioBuffer::Calls>();
        audioBuffer_ = std::make_unique<FakeAudioBuffer>(audioCalls_.get());
        telemetryWriter_ = std::make_unique<FakeTelemetryWriter>();
        telemetryReader_ = std::make_unique<FakeTelemetryReader>();
        presentation_ = std::make_unique<FakePresentation>();
        logger_ = std::make_unique<FakeLogger>();
        crankingController_ = std::make_unique<CrankingController>();
        stopRequested_ = std::make_unique<std::atomic<bool>>(false);
        clock_ = std::make_unique<FakeLoopClock>();

        inputProvider_ = std::make_unique<FakeInputProvider>();
    }

    // Build deps with the given input provider (allows swapping the provider per test).
    SessionDependencies buildDeps(input::IInputProvider* provider) {
        return SessionDependencies{
            audioBuffer_.get(),
            crankingController_.get(),
            stopRequested_.get(),
            provider,
            presentation_.get(),
            telemetryWriter_.get(),
            telemetryReader_.get(),
            logger_.get(),
            clock_.get()
        };
    }

    std::unique_ptr<BridgeSimulator> simulator_;
    std::unique_ptr<FakeAudioBuffer::Calls> audioCalls_;
    std::unique_ptr<FakeAudioBuffer> audioBuffer_;
    std::unique_ptr<FakeTelemetryWriter> telemetryWriter_;
    std::unique_ptr<FakeTelemetryReader> telemetryReader_;
    std::unique_ptr<FakePresentation> presentation_;
    std::unique_ptr<FakeLogger> logger_;
    std::unique_ptr<CrankingController> crankingController_;
    std::unique_ptr<std::atomic<bool>> stopRequested_;
    std::unique_ptr<FakeLoopClock> clock_;
    std::unique_ptr<input::IInputProvider> inputProvider_;

    // Default fake input provider (returns default input each tick).
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
};

// run() with a NON-combustion simulator: the combustionEngine==null branch in
// applyCrankingDecision is exercised; loop completes the duration and returns 0.
TEST_F(SimulationLoopRunTest, RunWithNonCombustionSimCompletesAndReturnsStop) {
    NonCombustionSimulator nonCombustion;
    SimulationConfig cfg;
    cfg.duration = 0.1;  // short so the test terminates quickly
    cfg.simulatorLabel = "NonCombustion";

    SimulationLoop loop(nonCombustion, cfg, buildDeps(inputProvider_.get()));
    int result = loop.run();
    EXPECT_EQ(result, 0);  // Stop
}

// run() with a preset-cycle request mid-run returns EXIT_BUT_CONTINUE_NEXT (2).
TEST_F(SimulationLoopRunTest, RunWithPresetCycleReturnsContinueNext) {
    auto presetProvider = std::make_unique<PresetCycleInputProvider>();

    SimulationConfig cfg;
    cfg.duration = 5.0;  // long, but preset cycle should exit first
    cfg.simulatorLabel = "PresetCycle";

    SimulationLoop loop(*simulator_, cfg, buildDeps(presetProvider.get()));
    int result = loop.run();
    EXPECT_EQ(result, EXIT_BUT_CONTINUE_NEXT);
    EXPECT_EQ(result, 2);
}

// run() reaching duration returns 0 (Stop). The combustion default provider
// drives a normal tick each frame until duration elapses.
TEST_F(SimulationLoopRunTest, RunReachingDurationReturnsStop) {
    SimulationConfig cfg;
    cfg.duration = 0.1;  // ~6 ticks
    cfg.simulatorLabel = "Duration";

    SimulationLoop loop(*simulator_, cfg, buildDeps(inputProvider_.get()));
    int result = loop.run();
    EXPECT_EQ(result, 0);
    EXPECT_GT(audioCalls_->updateSimulationCount, 0);  // loop actually ticked
}

// run() with stopRequested_ set returns 0 (Stop) promptly.
TEST_F(SimulationLoopRunTest, RunWithStopRequestedReturnsStop) {
    stopRequested_->store(true);  // stop before the loop body runs a tick

    SimulationConfig cfg;
    cfg.duration = 5.0;
    cfg.simulatorLabel = "StopRequested";

    SimulationLoop loop(*simulator_, cfg, buildDeps(inputProvider_.get()));
    int result = loop.run();
    EXPECT_EQ(result, 0);
}

}  // namespace
