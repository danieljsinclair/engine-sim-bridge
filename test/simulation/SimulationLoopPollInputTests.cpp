// SimulationLoopPollInputTests.cpp - Owner-decreed contract test for the
// SimulationLoop::pollInput() null-inputProvider branch (SimulationLoop.cpp:43).
//
// When inputProvider_ is null (non-interactive / headless mode), pollInput()
// generates timed input instead of delegating to a provider:
//   * throttle ramps linearly from 0.0 to FULL_THROTTLE (1.0) over
//     THROTTLE_RAMP_DURATION_SECONDS (0.5s), then holds at 1.0;
//   * the starter button is pressed on the FIRST tick (isFirstTick) so the
//     engine auto-starts in non-interactive mode.
//
// These behaviors are driven deterministically through run() with an injected
// FakeLoopClock (no real-time pacing) and NO input provider. We assert the
// OBSERVABLE downstream effects (the recorded throttle ramp and the recorded
// starter-engaged flag in telemetry), mirroring the established run/step
// scaffolds. No production code is modified.
//
// NOTE: the throttle ramp is masked by the cranking state machine when driven
// through a combustion simulator (throttle is forced to the cranking value while
// the engine is cranking), so the ramp is observed through a NON-combustion
// recording ISimulator, where applyCrankingDecision passes engineInput.throttle
// straight through to setThrottle. The first-tick starter is observed through a
// combustion recording simulator, where the starter button reaches the
// cranking controller and surfaces in telemetry.

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
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace {

using namespace input;
using namespace telemetry;

// ============================================================================
// Recording non-combustion ISimulator — records every setThrottle() so the
// timed-ramp produced by the null inputProvider is observable. The cranking
// decision passes engineInput.throttle straight through, so the ramp reaches
// setThrottle() unmasked.
// ============================================================================

class RecordingNonCombustionSimulator : public ISimulator {
public:
    std::vector<double> throttleCalls;

    bool create(const ISimulatorConfig&, ILogging*, telemetry::ITelemetryWriter*) override { return true; }
    void destroy() override {}
    std::string getLastError() const override { return ""; }
    const char* getName() const override { return "RecordingNonCombustion"; }
    void update(double) override {}
    EngineSimStats getStats() const override { return EngineSimStats{}; }
    void setThrottle(double position) override { throttleCalls.push_back(position); }
    bool renderOnDemand(float*, int32_t, int32_t*) override { return false; }
    bool readAudioBuffer(float*, int32_t, int32_t*) override { return false; }
    bool start() override { return true; }
    void stop() override {}
    int getSimulationFrequency() const override { return 10000; }
};

// ============================================================================
// Recording combustion simulator + recording telemetry writer for the
// first-tick starter observation.
// ============================================================================

class RecordingSimulator : public BridgeSimulator {
public:
    struct Calls {
        int setThrottleCount = 0;
        int setGearCount = 0;
    };

    RecordingSimulator(std::unique_ptr<Simulator> sim, const std::string& name, Calls* calls)
        : BridgeSimulator(std::move(sim), name), calls_(calls) {}

    void setThrottle(double position) override {
        calls_->setThrottleCount++;
        BridgeSimulator::setThrottle(position);
    }
    int setGear(int gear) override {
        calls_->setGearCount++;
        return BridgeSimulator::setGear(gear);
    }

private:
    Calls* calls_;
};

class RecordingTelemetryWriter : public ITelemetryWriter {
public:
    struct Calls {
        int writeVehicleInputsCount = 0;
        std::vector<bool> starterEngagedSequence;
    };

    explicit RecordingTelemetryWriter(Calls* calls) : calls_(calls) {}

    void writeEngineState(const EngineStateTelemetry&) override {}
    void writeFramePerformance(const FramePerformanceTelemetry&) override {}
    void writeAudioDiagnostics(const AudioDiagnosticsTelemetry&) override {}
    void writeAudioTiming(const AudioTimingTelemetry&) override {}
    void writeVehicleInputs(const VehicleInputsTelemetry& inputs) override {
        calls_->writeVehicleInputsCount++;
        calls_->starterEngagedSequence.push_back(inputs.starterMotorEngaged);
    }
    void writeSimulatorMetrics(const SimulatorMetricsTelemetry&) override {}
    void reset() override {}
    const char* getName() const override { return "RecordingTelemetryWriter"; }

private:
    Calls* calls_;
};

// ---- Minimal fakes ----

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

class FakeCrankingController : public CrankingController {};

// ============================================================================
// Throttle-ramp fixture (non-combustion, null inputProvider)
// ============================================================================

class PollInputRampTest : public ::testing::Test {
protected:
    void SetUp() override {
        sim_ = std::make_unique<RecordingNonCombustionSimulator>();
        audioBuffer_ = std::make_unique<FakeAudioBuffer>();
        telemetryReader_ = std::make_unique<FakeTelemetryReader>();
        presentation_ = std::make_unique<FakePresentation>();
        logger_ = std::make_unique<FakeLogger>();
        crankingController_ = std::make_unique<FakeCrankingController>();
        stopRequested_ = std::make_unique<std::atomic<bool>>(false);
        clock_ = std::make_unique<FakeLoopClock>();

        // Null input provider: leave deps.inputProvider default (nullptr).
        simConfig_ = SimulationConfig{};
        simConfig_.duration = 0.6;  // spans the full 0.5s ramp + clamp
        simConfig_.simulatorLabel = "PollInputRamp";
    }

    SessionDependencies buildDeps() {
        return SessionDependencies{
            audioBuffer_.get(),
            crankingController_.get(),
            stopRequested_.get(),
            nullptr,            // <-- null inputProvider: timed-ramp path
            presentation_.get(),
            nullptr,            // telemetryWriter optional
            telemetryReader_.get(),
            logger_.get(),
            clock_.get()
        };
    }

    std::unique_ptr<RecordingNonCombustionSimulator> sim_;
    std::unique_ptr<FakeAudioBuffer> audioBuffer_;
    std::unique_ptr<FakeTelemetryReader> telemetryReader_;
    std::unique_ptr<FakePresentation> presentation_;
    std::unique_ptr<FakeLogger> logger_;
    std::unique_ptr<FakeCrankingController> crankingController_;
    std::unique_ptr<std::atomic<bool>> stopRequested_;
    std::unique_ptr<FakeLoopClock> clock_;
    SimulationConfig simConfig_;
};

// The null-provider timed ramp: throttle starts at 0.0 (pre-poll default),
// ramps linearly to FULL_THROTTLE (1.0) over 0.5s, then holds at 1.0.
TEST_F(PollInputRampTest, NullInputProviderRampsThrottleToFull) {
    SimulationLoop loop(*sim_, simConfig_, buildDeps());
    int result = loop.run();

    // run() completes on duration.
    EXPECT_EQ(result, 0);

    const auto& t = sim_->throttleCalls;
    ASSERT_GE(t.size(), 2u) << "expected multiple ticks";

    // First tick runs on the pre-poll default input (throttle 0.0).
    EXPECT_DOUBLE_EQ(t.front(), 0.0);

    // Final throttle is clamped to FULL_THROTTLE once the ramp completes.
    EXPECT_DOUBLE_EQ(t.back(), 1.0);

    // The ramp is GRADUAL: monotonic non-decreasing and passes through
    // intermediate values (not an instant jump to full throttle).
    bool monotonic = true;
    bool sawIntermediate = false;
    for (size_t i = 1; i < t.size(); ++i) {
        if (t[i] < t[i - 1] - 1e-9) monotonic = false;
        if (t[i] > 0.0 + 1e-9 && t[i] < 1.0 - 1e-9) sawIntermediate = true;
    }
    EXPECT_TRUE(monotonic) << "throttle ramp must be non-decreasing";
    EXPECT_TRUE(sawIntermediate) << "throttle ramp must pass through intermediate values";
}

// ============================================================================
// First-tick starter fixture (combustion, null inputProvider, recording telemetry)
// ============================================================================

class PollInputStarterTest : public ::testing::Test {
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
        telemetryCalls_ = std::make_unique<RecordingTelemetryWriter::Calls>();
        telemetryWriter_ = std::make_unique<RecordingTelemetryWriter>(telemetryCalls_.get());
        telemetryReader_ = std::make_unique<FakeTelemetryReader>();
        presentation_ = std::make_unique<FakePresentation>();
        logger_ = std::make_unique<FakeLogger>();
        crankingController_ = std::make_unique<FakeCrankingController>();
        stopRequested_ = std::make_unique<std::atomic<bool>>(false);
        clock_ = std::make_unique<FakeLoopClock>();

        simConfig_ = SimulationConfig{};
        simConfig_.duration = 0.1;  // ~6 ticks: enough to see cranking starter
        simConfig_.simulatorLabel = "PollInputStarter";
    }

    SessionDependencies buildDeps() {
        return SessionDependencies{
            audioBuffer_.get(),
            crankingController_.get(),
            stopRequested_.get(),
            nullptr,            // <-- null inputProvider: first-tick starter path
            presentation_.get(),
            telemetryWriter_.get(),
            telemetryReader_.get(),
            logger_.get(),
            clock_.get()
        };
    }

    std::unique_ptr<RecordingSimulator::Calls> calls_;
    std::unique_ptr<RecordingSimulator> simulator_;
    std::unique_ptr<FakeAudioBuffer> audioBuffer_;
    std::unique_ptr<RecordingTelemetryWriter::Calls> telemetryCalls_;
    std::unique_ptr<RecordingTelemetryWriter> telemetryWriter_;
    std::unique_ptr<FakeTelemetryReader> telemetryReader_;
    std::unique_ptr<FakePresentation> presentation_;
    std::unique_ptr<FakeLogger> logger_;
    std::unique_ptr<FakeCrankingController> crankingController_;
    std::unique_ptr<std::atomic<bool>> stopRequested_;
    std::unique_ptr<FakeLoopClock> clock_;
    SimulationConfig simConfig_;
};

// The null-provider branch presses the starter on the FIRST tick (isFirstTick).
// run() executes step() before the first poll, so the starter button set by the
// first poll takes effect on the SECOND tick: the first telemetry sample has
// starterMotorEngaged == false, the next has it == true.
TEST_F(PollInputStarterTest, NullInputProviderEngagesStarterOnFirstTick) {
    SimulationLoop loop(*simulator_, simConfig_, buildDeps());
    int result = loop.run();

    EXPECT_EQ(result, 0);
    ASSERT_GE(telemetryCalls_->writeVehicleInputsCount, 2)
        << "expected at least two ticks of telemetry";

    const auto& seq = telemetryCalls_->starterEngagedSequence;
    ASSERT_EQ(seq.size(), static_cast<size_t>(telemetryCalls_->writeVehicleInputsCount));

    // First tick runs on the pre-poll default input: no starter yet.
    EXPECT_FALSE(seq[0]) << "first tick (pre-poll) must not have starter engaged";

    // After the first poll sets starterButton=true, the next tick engages it.
    EXPECT_TRUE(seq[1]) << "starter must be engaged on the tick after the first poll";
}

}  // namespace
