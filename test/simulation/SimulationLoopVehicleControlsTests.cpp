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
#include "simulator/GearConventions.h"
#include "input/LiveTelemetryProvider.h"
#include "twin/IceVehicleProfile.h"

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
        bool lastIgnition = true;
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
        calls_->lastIgnition = on;
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
    void ShowSimulatorStates(const presentation::EngineState& state) override {
        lastControls = state.controls;
    }

    presentation::EngineState::Controls lastControls;
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

// Brake-light assembly: with no telemetry value, the local brake level (the
// keyboard 'B' key's only output) derives the canonical light signal that the
// display consumes.
TEST_F(SimulationLoopVehicleControlsTest, BrakeLevel_DerivesBrakeLightWhenTelemetryAbsent) {
    input::EngineInput input;
    input.ignition = true;
    input.brakeLevel = 1.0;   // keyboard 'B' held
    // brakeLight left nullopt — no telemetry on this path

    SimulationLoop loop(*simulator_, simConfig_, buildDeps());
    loop.step(makeStateRef(input));

    EXPECT_EQ(calls_->setBrakePressureCount, 1);
    EXPECT_DOUBLE_EQ(calls_->lastBrake, 1.0);
    ASSERT_TRUE(presentation_->lastControls.brakeLight.has_value());
    EXPECT_TRUE(*presentation_->lastControls.brakeLight)
        << "Keyboard brake level must derive brakeLight=true for display";
}

// Brake-light assembly: telemetry (CSV brake_light=1) supplies the light
// directly. It is an indicator, not a pedal — it must NOT drive brake physics.
TEST_F(SimulationLoopVehicleControlsTest, TelemetryBrakeLight_DoesNotTouchBrakePhysics) {
    input::EngineInput input;
    input.ignition = true;
    input.brakeLight = true;  // CSV brake_light=1 row
    // brakeLevel left at default 0.0 — nothing but the 'B' key writes it

    SimulationLoop loop(*simulator_, simConfig_, buildDeps());
    loop.step(makeStateRef(input));

    ASSERT_TRUE(presentation_->lastControls.brakeLight.has_value());
    EXPECT_TRUE(*presentation_->lastControls.brakeLight)
        << "CSV brake light must reach the display unchanged";
    EXPECT_EQ(calls_->setBrakePressureCount, 1);
    EXPECT_DOUBLE_EQ(calls_->lastBrake, 0.0)
        << "CSV brake light is an indicator — it must not apply brake pressure";
}

// Equivalence on the CANONICAL signal only: a keyboard 'B' press (level 1.0,
// no telemetry light) and a CSV brake_light=1 row (light true, level 0) must
// be indistinguishable to the controller/display view (brakeLight) — while
// remaining distinct at the physics input (the key applies pressure, the CSV
// row does not).
TEST_F(SimulationLoopVehicleControlsTest, KeyboardAndCsv_EquivalentOnCanonicalLightOnly) {
    input::EngineInput keyboardInput;
    keyboardInput.ignition = true;
    keyboardInput.brakeLevel = 1.0;   // keyboard 'B' held

    SimulationLoop keyboardLoop(*simulator_, simConfig_, buildDeps());
    keyboardLoop.step(makeStateRef(keyboardInput));
    const auto keyboardLight = presentation_->lastControls.brakeLight;

    input::EngineInput csvInput;
    csvInput.ignition = true;
    csvInput.brakeLight = true;       // CSV brake_light=1 row
    csvInput.brakeLevel = 0.0;        // nothing but the 'B' key writes the level

    SimulationLoop csvLoop(*simulator_, simConfig_, buildDeps());
    csvLoop.step(makeStateRef(csvInput));
    const auto csvLight = presentation_->lastControls.brakeLight;

    ASSERT_TRUE(keyboardLight.has_value());
    ASSERT_TRUE(csvLight.has_value());
    EXPECT_EQ(*keyboardLight, *csvLight)
        << "Controller/display view must see the same brakeLight from both entry points";
}

// ---------------------------------------------------------------------------
// Vehicle start/stop — the single SimulationLoop decision site. The controller
// consumes the CANONICAL brakeLight (telemetry value or keyboard-level-derived)
// + gearSelector, so keyboard, CSV and live inputs are indistinguishable here.
// ---------------------------------------------------------------------------

// Interactive 'B' key: brakeLevel (the key's only output) derives the light,
// which engages the controller: starter pulse on frame 1 (Stopped->Cranking),
// ignition held off until kDefaultCrankDelayS (0.5s) of held brake accumulates.
TEST_F(SimulationLoopVehicleControlsTest, InteractiveBrakeKey_AutoStartsWithCrankDelay) {
    input::EngineInput brakeHeld;         // keyboard defaults: ignition true
    brakeHeld.brakeLevel = 1.0;           // 'B' held
    brakeHeld.gearSelector = 0;           // NEUTRAL

    SimulationLoop loop(*simulator_, simConfig_, buildDeps());

    loop.step(makeStateRef(brakeHeld));
    EXPECT_EQ(simulator_->getEnginePhase(), EnginePhase::Cranking)
        << "Brake start must fire the starter pulse on frame 1";
    EXPECT_FALSE(calls_->lastIgnition)
        << "Ignition must stay off during the crank delay";

    // Hold the brake well past the 0.5s crank delay (dt = 1/60 per step).
    for (int frame = 1; frame < 40; ++frame) {
        loop.step(makeStateRef(brakeHeld));
    }
    EXPECT_TRUE(calls_->lastIgnition)
        << "Ignition must fire after the crank delay while the brake is held";
}

// Interactive gear selection (D): instant start — ignition ON on the FIRST
// frame, with the starter pulse. No crank delay on a drive-gear start.
TEST_F(SimulationLoopVehicleControlsTest, InteractiveGearKey_StartsInstantly) {
    input::EngineInput driveSelected;
    driveSelected.gearSelector = static_cast<int>(bridge::GearSelector::DRIVE);

    SimulationLoop loop(*simulator_, simConfig_, buildDeps());
    loop.step(makeStateRef(driveSelected));

    EXPECT_EQ(simulator_->getEnginePhase(), EnginePhase::Cranking)
        << "Gear start must fire the starter pulse on frame 1";
    EXPECT_TRUE(calls_->lastIgnition)
        << "Gear start is instant — no crank delay";
}

// Brake + PARK while running stops the engine (ignition off), identically to
// the live-telemetry path.
TEST_F(SimulationLoopVehicleControlsTest, InteractiveBrakePlusPark_Stops) {
    input::EngineInput driveSelected;
    driveSelected.gearSelector = static_cast<int>(bridge::GearSelector::DRIVE);

    SimulationLoop loop(*simulator_, simConfig_, buildDeps());
    loop.step(makeStateRef(driveSelected));
    ASSERT_TRUE(calls_->lastIgnition);

    input::EngineInput brakeAndPark;
    brakeAndPark.brakeLevel = 1.0;        // 'B' held
    brakeAndPark.gearSelector = static_cast<int>(bridge::GearSelector::PARK);
    loop.step(makeStateRef(brakeAndPark));

    EXPECT_FALSE(calls_->lastIgnition)
        << "Brake + PARK while running must stop the engine";
}

// No vehicle-control signal (no telemetry light, no brake level, no D/R gear):
// the controller has NO opinion and must not touch the provider's start/stop
// fields. This is what preserves replay autoStart and --start: their frame-0
// starter pulse reaches the CrankingController unmodified.
TEST_F(SimulationLoopVehicleControlsTest, NoVehicleSignal_ProviderKeepsStartStopAuthority) {
    input::EngineInput providerStart;     // e.g. replay autoStart / --start
    providerStart.ignition = true;
    providerStart.starterButton = true;   // frame-0 pulse from the provider

    SimulationLoop loop(*simulator_, simConfig_, buildDeps());
    loop.step(makeStateRef(providerStart));

    EXPECT_EQ(simulator_->getEnginePhase(), EnginePhase::Cranking)
        << "Provider starter pulse must not be suppressed by the controller";
    EXPECT_TRUE(calls_->lastIgnition)
        << "Provider ignition must pass through when the controller has no opinion";
}

// A telemetry-REPORTED light (even false) is an opinion: the controller takes
// authority from frame 1 and holds the engine off until a start demand — the
// live-mode contract (OFF -> Cranking on brake -> Running), now mode-agnostic.
TEST_F(SimulationLoopVehicleControlsTest, TelemetryLightEngages_ControllerHoldsEngineOff) {
    input::EngineInput idleRow;           // live CSV row: brake_light=0, no gear
    idleRow.ignition = true;              // provider default must be overridden
    idleRow.brakeLight = false;           // REPORTED false (not absent)

    SimulationLoop loop(*simulator_, simConfig_, buildDeps());
    loop.step(makeStateRef(idleRow));

    EXPECT_FALSE(calls_->lastIgnition)
        << "A reported light hands start/stop authority to the controller";
    EXPECT_EQ(simulator_->getEnginePhase(), EnginePhase::Stopped)
        << "No start demand — no starter pulse";
}

// Starter-pulse contract at the loop site: a held brake keeps the controller's
// starter LEVEL high, but the flattened pulse fires once; a second pulse would
// toggle the CrankingController back to Stopped and abort the crank.
TEST_F(SimulationLoopVehicleControlsTest, HeldBrakeStarterPulseIsSingleFrame) {
    input::EngineInput brakeHeld;
    brakeHeld.brakeLevel = 1.0;
    brakeHeld.gearSelector = 0;           // NEUTRAL

    SimulationLoop loop(*simulator_, simConfig_, buildDeps());
    for (int frame = 0; frame < 3; ++frame) {
        loop.step(makeStateRef(brakeHeld));
        EXPECT_EQ(simulator_->getEnginePhase(), EnginePhase::Cranking)
            << "A held-high starter level must not toggle the crank off";
    }
    EXPECT_FALSE(calls_->lastIgnition)    // 3 frames = 0.05s < 0.5s delay
        << "Crank delay must not have elapsed yet";
}

// ---------------------------------------------------------------------------
// Live provider end-to-end: a gear-only network frame (DRIVE, no brake) must
// reach the consolidated applyStartStopDecision site with engineInput.gearSelector
// == DRIVE, so driveSelected is true and the engine starts WITHOUT a crank delay
// (ignition ON on frame 1). This is the regression guard for the live JSON path:
// the network branch of LiveTelemetryProvider must relay signal.gearSelector to
// the twin, which is the only owner of the selector the decision site reads.
// ---------------------------------------------------------------------------

// Thin adapter so the real LiveTelemetryProvider (an IInputProvider) can be fed
// to SimulationLoop, exactly as the production --live-telemetry wiring does.
class FakeStartStopProvider : public IInputProvider {
public:
    explicit FakeStartStopProvider(std::unique_ptr<input::LiveTelemetryProvider> p)
        : provider_(std::move(p)) {}

    EngineInput OnUpdateSimulation(double dt) override {
        return provider_->OnUpdateSimulation(dt);
    }
    void provideFeedback(const EngineSimStats& s) override { provider_->provideFeedback(s); }
    bool Initialize() override { return provider_->Initialize(); }
    void Shutdown() override { provider_->Shutdown(); }
    bool IsConnected() const override { return provider_->IsConnected(); }
    std::string GetProviderName() const override { return provider_->GetProviderName(); }
    std::string GetLastError() const override { return provider_->GetLastError(); }

private:
    std::unique_ptr<input::LiveTelemetryProvider> provider_;
};

TEST_F(SimulationLoopVehicleControlsTest, LiveNetworkFrame_DriveGearStartsInstantly) {
    // Build the REAL live provider (network mode: no stream), submit a DRIVE
    // gear frame, and run it through the loop's start/stop decision site.
    auto liveProvider = std::make_unique<input::LiveTelemetryProvider>(
        twin::IceVehicleProfile::zf8hp45());
    ASSERT_TRUE(liveProvider->Initialize());

    input::UpstreamSignal signal;
    signal.gearSelector = bridge::GearSelector::DRIVE;  // gear-only, NO brake
    signal.isValid = true;
    signal.throttleFraction = 0.5;
    signal.speedKmh = 30.0;
    liveProvider->submitSignal(signal);

    auto provider = std::make_unique<FakeStartStopProvider>(std::move(liveProvider));

    SessionDependencies deps{
        audioBuffer_.get(),
        crankingController_.get(),
        stopRequested_.get(),
        provider.get(),
        presentation_.get(),
        telemetryWriter_.get(),
        telemetryReader_.get(),
        logger_.get()
    };

    SimulationLoop loop(*simulator_, simConfig_, deps);

    // Frame 1 of a gear-only live frame: the decision site must see DRIVE and
    // start instantly (starter pulse + ignition ON), no 0.5s crank delay.
    LoopState state = makeState(input::EngineInput{});
    loop.step(state);

    EXPECT_EQ(simulator_->getEnginePhase(), EnginePhase::Cranking)
        << "Live DRIVE frame must fire the starter pulse on frame 1";
    EXPECT_TRUE(calls_->lastIgnition)
        << "Live DRIVE frame is an instant start — ignition must be ON, no crank delay";
}

// Contrast: a live NEUTRAL frame (gear-only, no brake) must NOT start — it must
// remain Stopped, proving the selector value genuinely flows from the signal and
// the decision site distinguishes DRIVE from NEUTRAL. Guards against the field
// being hardcoded to a start-triggering value.
TEST_F(SimulationLoopVehicleControlsTest, LiveNetworkFrame_NeutralDoesNotStart) {
    auto liveProvider = std::make_unique<input::LiveTelemetryProvider>(
        twin::IceVehicleProfile::zf8hp45());
    ASSERT_TRUE(liveProvider->Initialize());

    input::UpstreamSignal signal;
    signal.gearSelector = bridge::GearSelector::NEUTRAL;
    signal.isValid = true;
    signal.throttleFraction = 0.5;
    signal.speedKmh = 30.0;
    liveProvider->submitSignal(signal);

    auto provider = std::make_unique<FakeStartStopProvider>(std::move(liveProvider));

    SessionDependencies deps{
        audioBuffer_.get(),
        crankingController_.get(),
        stopRequested_.get(),
        provider.get(),
        presentation_.get(),
        telemetryWriter_.get(),
        telemetryReader_.get(),
        logger_.get()
    };

    SimulationLoop loop(*simulator_, simConfig_, deps);

    LoopState state = makeState(input::EngineInput{});
    loop.step(state);

    EXPECT_EQ(simulator_->getEnginePhase(), EnginePhase::Stopped)
        << "Live NEUTRAL frame must not start the engine";
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
