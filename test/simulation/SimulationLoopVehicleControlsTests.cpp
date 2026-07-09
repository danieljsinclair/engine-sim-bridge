// SimulationLoopVehicleControlsTests.cpp - Contract tests for the per-tick forks
// inside SimulationLoop::applyVehicleControls() and the presentation/telemetry
// present-vs-absent forks inside updatePresentation()/writeTelemetry().
//
// applyVehicleControls is private and applyCrankingDecision's combustionEngine
// branch and the speed-target forks use a dynamic_cast<BridgeSimulator*>. To make
// every fork OBSERVABLE we drive step() (deterministic, no clock needed) with a
// RecordingSimulator — a thin BridgeSimulator subclass that records which control
// methods are invoked. Because it IS-A BridgeSimulator, the internal dynamic_cast
// succeeds and the speed-target forks are exercised for real.
//
// We do NOT modify production code; the recording subclass is a test double.

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

// Recording BridgeSimulator: overrides the control surface so the forks inside
// applyVehicleControls become observable. Engine audio/sim behaviour is delegated
// to the real Sine-backed base; only the control methods are intercepted.
class RecordingSimulator : public BridgeSimulator {
public:
    struct Calls {
        int setThrottleCount = 0;
        double lastThrottle = 0.0;
        int setGearCount = 0;
        int lastGear = -1;
        int changeGearCount = 0;
        int lastGearDelta = 0;
        int setClutchPressureCount = 0;
        int setBrakePressureCount = 0;
        double lastBrake = 0.0;
        int setDynoTorqueScaleCount = 0;
        double lastDynoScale = -2.0;
        int setIgnitionCount = 0;
    };

    RecordingSimulator(std::unique_ptr<Simulator> sim, const std::string& name, Calls* calls)
        : BridgeSimulator(std::move(sim), name), calls_(calls) {}

    void setThrottle(double position) override {
        calls_->setThrottleCount++;
        calls_->lastThrottle = position;
        BridgeSimulator::setThrottle(position);
    }
    int setGear(int gear) override {
        calls_->setGearCount++;
        calls_->lastGear = gear;
        return BridgeSimulator::setGear(gear);
    }
    bool changeGear(int gearDelta) override {
        calls_->changeGearCount++;
        calls_->lastGearDelta = gearDelta;
        return BridgeSimulator::changeGear(gearDelta);
    }
    void setClutchPressure(double pressure) override {
        calls_->setClutchPressureCount++;
        BridgeSimulator::setClutchPressure(pressure);
    }
    void setBrakePressure(double pressure) override {
        calls_->setBrakePressureCount++;
        calls_->lastBrake = pressure;
        BridgeSimulator::setBrakePressure(pressure);
    }
    void setDynoTorqueScale(double scale) override {
        calls_->setDynoTorqueScaleCount++;
        calls_->lastDynoScale = scale;
        BridgeSimulator::setDynoTorqueScale(scale);
    }
    void setIgnition(bool on) override {
        calls_->setIgnitionCount++;
        BridgeSimulator::setIgnition(on);
    }

private:
    Calls* calls_;
};

// ---- Fakes (minimal, same shape as the step/run scaffolds) ----

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

class SimulationLoopVehicleControlsTest : public ::testing::Test {
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

    // Build a LoopState for a single deterministic tick with the given input.
    LoopState makeState(input::EngineInput input) {
        LoopState state;
        state.currentTime = 0.0;
        state.isFirstTick = true;
        state.combustionEngine = dynamic_cast<ICombustionEngine*>(simulator_.get());
        state.engineInput = input;
        return state;
    }
    // step() takes a non-const reference, so build into a member slot.
    LoopState stateSlot_;
    LoopState& makeStateRef(input::EngineInput input) {
        stateSlot_ = makeState(input);
        return stateSlot_;
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

// ---------------------------------------------------------------------------
// gearAbsolute fork: >=0 calls setGear (not changeGear)
// ---------------------------------------------------------------------------

TEST_F(SimulationLoopVehicleControlsTest, GearAbsoluteSetsGear) {
    input::EngineInput input;
    input.gearAbsolute = 2;
    input.ignition = true;

    SimulationLoop loop(*simulator_, simConfig_, buildDeps());
    loop.step(makeStateRef(input));

    EXPECT_EQ(calls_->setGearCount, 1);
    EXPECT_EQ(calls_->lastGear, 2);
    EXPECT_EQ(calls_->changeGearCount, 0);  // gearDelta path not taken
}

// ---------------------------------------------------------------------------
// gearDelta fork: gearAbsolute < 0 calls changeGear
// ---------------------------------------------------------------------------

TEST_F(SimulationLoopVehicleControlsTest, GearDeltaChangesGear) {
    input::EngineInput input;
    input.gearAbsolute = -1;  // default: use gearDelta logic
    input.gearDelta = 1;
    input.ignition = true;

    SimulationLoop loop(*simulator_, simConfig_, buildDeps());
    loop.step(makeStateRef(input));

    EXPECT_EQ(calls_->changeGearCount, 1);
    EXPECT_EQ(calls_->lastGearDelta, 1);
    EXPECT_EQ(calls_->setGearCount, 0);  // gearAbsolute path not taken
}

// ---------------------------------------------------------------------------
// Vehicle speed target fork (Spike-A inverse path)
// ---------------------------------------------------------------------------

TEST_F(SimulationLoopVehicleControlsTest, VehicleSpeedTargetForcesDynoOff) {
    // Engage a forward gear so the speed forks take effect (neutral disables both).
    simulator_->setGear(1);

    input::EngineInput input;
    input.ignition = true;
    input.gearAbsolute = -1;
    input.vehicleSpeedTargetKmh = 60.0;  // >= 0 -> setVehicleSpeedTarget (Spike-A)

    SimulationLoop loop(*simulator_, simConfig_, buildDeps());
    loop.step(makeStateRef(input));

    // Spike-A inverse path drives the wheels and forces the DYNO OFF, so no
    // road-speed tracking dyno target is configured.
    EXPECT_DOUBLE_EQ(simulator_->getStats().dynoTargetRPM, 0.0);
}

// ---------------------------------------------------------------------------
// Road-speed fallback fork (legacy dyno path): vehicleSpeedTarget < 0 (default),
// manual mode, roadSpeed >= 0 -> setSpeedTrackingTarget
// ---------------------------------------------------------------------------

TEST_F(SimulationLoopVehicleControlsTest, RoadSpeedFallbackEnablesDynoTracking) {
    simulator_->setGear(1);  // forward gear so the dyno path engages

    input::EngineInput input;
    input.ignition = true;
    input.gearAbsolute = -1;
    input.vehicleSpeedTargetKmh = -1.0;  // default: untouched
    input.roadSpeedKmh = 40.0;           // >= 0
    input.gearAutoMode = false;          // manual

    SimulationLoop loop(*simulator_, simConfig_, buildDeps());
    loop.step(makeStateRef(input));

    // Legacy path enables the speed-tracking DYNO (hold mode) targeting a
    // positive road-speed RPM, and must NOT take the vehicle-speed fork.
    EXPECT_GT(simulator_->getStats().dynoTargetRPM, 0.0);
}

// ---------------------------------------------------------------------------
// Dyno torque scale fork
// ---------------------------------------------------------------------------

// A non-negative scale != lastScale calls setDynoTorqueScale.
TEST_F(SimulationLoopVehicleControlsTest, DynoScaleChangeApplies) {
    input::EngineInput input;
    input.ignition = true;
    input.dynoTorqueScale = 0.5;  // default -1, so this is a change

    SimulationLoop loop(*simulator_, simConfig_, buildDeps());
    loop.step(makeStateRef(input));

    EXPECT_EQ(calls_->setDynoTorqueScaleCount, 1);
    EXPECT_DOUBLE_EQ(calls_->lastDynoScale, 0.5);
}

// Same scale on consecutive ticks is a no-op after the first.
TEST_F(SimulationLoopVehicleControlsTest, DynoScaleUnchangedIsNoOp) {
    input::EngineInput input;
    input.ignition = true;
    input.dynoTorqueScale = 0.5;

    SimulationLoop loop(*simulator_, simConfig_, buildDeps());
    LoopState state = makeState(input);
    loop.step(state);  // first: applies 0.5
    loop.step(state);  // second: scale == lastScale -> no-op

    EXPECT_EQ(calls_->setDynoTorqueScaleCount, 1);
}

// ---------------------------------------------------------------------------
// Clutch + brake forks
// ---------------------------------------------------------------------------

TEST_F(SimulationLoopVehicleControlsTest, ClutchPressureAppliedWhenNonNegative) {
    input::EngineInput input;
    input.ignition = true;
    input.clutchPressure = 0.7;

    SimulationLoop loop(*simulator_, simConfig_, buildDeps());
    loop.step(makeStateRef(input));

    EXPECT_EQ(calls_->setClutchPressureCount, 1);
}

TEST_F(SimulationLoopVehicleControlsTest, BrakePressureForwarded) {
    input::EngineInput input;
    input.ignition = true;
    input.brakeLevel = 0.4;

    SimulationLoop loop(*simulator_, simConfig_, buildDeps());
    loop.step(makeStateRef(input));

    EXPECT_EQ(calls_->setBrakePressureCount, 1);
    EXPECT_DOUBLE_EQ(calls_->lastBrake, 0.4);
}

// ---------------------------------------------------------------------------
// Presentation / telemetry present vs absent forks
// ---------------------------------------------------------------------------

// With presentation present, updatePresentation proceeds (no early return). We
// exercise this by simply running a step with a real presentation set (already
// the default) — the absence test below contrasts it.
TEST_F(SimulationLoopVehicleControlsTest, PresentationPresentAdvancesTick) {
    // presentation_ is set (non-null) by default.
    input::EngineInput input;
    input.ignition = true;

    SimulationLoop loop(*simulator_, simConfig_, buildDeps());
    StepResult result = loop.step(makeStateRef(input));
    EXPECT_EQ(result, StepResult::Continue);
    EXPECT_EQ(calls_->setThrottleCount, 1);  // control surface was driven
}

// With presentation AND telemetryWriter AND telemetryReader all null, the
// updatePresentation/writeTelemetry forks take their early-return/absent paths.
TEST_F(SimulationLoopVehicleControlsTest, PresentationAbsentStillTicks) {
    SessionDependencies deps{
        audioBuffer_.get(),
        crankingController_.get(),
        stopRequested_.get(),
        inputProvider_.get(),
        nullptr,          // presentation_ absent
        nullptr,          // telemetryWriter_ absent
        nullptr,          // telemetryReader_ absent
        logger_.get()
    };

    input::EngineInput input;
    input.ignition = true;

    SimulationLoop loop(*simulator_, simConfig_, deps);
    StepResult result = loop.step(makeStateRef(input));
    EXPECT_EQ(result, StepResult::Continue);
    // Control surface is still driven even with presentation/telemetry absent.
    EXPECT_EQ(calls_->setThrottleCount, 1);
}

}  // namespace
