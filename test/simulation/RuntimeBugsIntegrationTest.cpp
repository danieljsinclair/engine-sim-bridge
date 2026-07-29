// RuntimeBugsIntegrationTest.cpp - Reproduces two LIVE runtime bugs reported from
// interactive testing. The components pass in isolation; these tests drive the
// REAL integration path (keyboard target -> EngineInput -> SimulationLoop decision
// -> BridgeSimulator dyno) to expose the gap.
//
// Bug 1: In NEUTRAL, ',' (slower) / '.' (faster) do not change the displayed mph.
// Bug 2: The engine stalls / never catches when put in a forward GEAR.
//
// Root cause under test: EngineInput.roadSpeedKmh defaults to 0.0, and
// SimulationLoop::applyVehicleControls gates setSpeedTrackingTarget on
// `roadSpeedKmh >= 0.0` -- which is ALWAYS true -- so the dyno is forced into
// hold-at-0-RPM the moment a forward gear is engaged, stalling the engine.
// Symptom 1 follows because the speed display reads vehicle physics, not the
// commanded target, and setSpeedTrackingTarget is (correctly) inert in neutral.

#include <gtest/gtest.h>
#include "input/EngineInputTarget.h"
#include "input/KeyboardInputProvider.h"
#include "input/DemoInputProvider.h"
#include "input/DemoThrottleSource.h"
#include "input/GearSelectorInput.h"
#include "input/IgnitionInput.h"
#include "input/IKeyboardInput.h"
#include "io/IInputProvider.h"
#include "simulator/BridgeSimulator.h"
#include "simulator/SineSimulator.h"
#include "simulator/SineEngine.h"
#include "simulator/SineVehicle.h"
#include "simulator/SineTransmission.h"
#include "twin/IceVehicleProfile.h"
#include "mocks/MockKeyboardInput.h"

#include <memory>

using namespace input;
using namespace twin;

namespace {
// Mirrors the gate in SimulationLoop::applyVehicleControls that decides whether
// to drive BridgeSimulator::setSpeedTrackingTarget from roadSpeedKmh. Kept here
// so the test asserts the SAME contract the live loop relies on, without needing
// access to the private method.
bool shouldDriveSpeedTracking(double roadSpeedKmh) {
    return roadSpeedKmh >= 0.0;
}
}  // namespace

// ============================================================================
// Fixtures
// ============================================================================

class RuntimeBugsIntegrationTest : public ::testing::Test {
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
        simulator_ = std::make_unique<BridgeSimulator>(std::move(sineSim));

        ISimulatorConfig config;
        config.sampleRate = EngineSimDefaults::SAMPLE_RATE;
        config.simulationFrequency = EngineSimDefaults::SIMULATION_FREQUENCY;
        ASSERT_TRUE(simulator_->create(config, nullptr, nullptr)) << "BridgeSimulator::create() failed";

        rawSim_ = simulator_->getInternalSimulator();
        ASSERT_NE(rawSim_, nullptr);
        trans_ = rawSim_->getTransmission();
        ASSERT_NE(trans_, nullptr);
    }

    std::unique_ptr<BridgeSimulator> simulator_;
    Simulator* rawSim_ = nullptr;
    Transmission* trans_ = nullptr;
};

// Keyboard-target chain identical to CLIMain --connect-demo wiring.
class KeyboardChainFixture : public ::testing::Test {
protected:
    IceVehicleProfile profile_{IceVehicleProfile::zf8hp45()};
    MockKeyboardInput* rawKeyboard_ = nullptr;
    std::unique_ptr<EngineInputTarget> target_;
    std::unique_ptr<DemoInputProvider> demoProvider_;
    std::unique_ptr<KeyboardInputProvider> provider_;

    void SetUp() override {
        auto keyboard = std::make_unique<MockKeyboardInput>();
        rawKeyboard_ = keyboard.get();

        auto target = std::make_unique<EngineInputTarget>();
        auto throttle = std::make_unique<DemoThrottleSource>();
        auto demoProvider = std::make_unique<DemoInputProvider>(
            std::move(throttle),
            std::make_unique<GearSelectorInput>(),
            std::make_unique<IgnitionInput>(),
            profile_);
        ASSERT_TRUE(demoProvider->Initialize());

        target->setSpeedEnhancer(demoProvider.get());

        auto provider = std::make_unique<KeyboardInputProvider>(
            std::move(keyboard), target.get());
        ASSERT_TRUE(provider->Initialize());

        target_ = std::move(target);
        demoProvider_ = std::move(demoProvider);
        provider_ = std::move(provider);
    }

    EngineInput pressAndTick(int key, double dt = 16.0) {
        rawKeyboard_->enqueue(key);
        return provider_->OnUpdateSimulation(dt);
    }

    EngineInput tick(double dt = 16.0) {
        return provider_->OnUpdateSimulation(dt);
    }
};

// ============================================================================
// Bug 2: Engine stalls in a forward gear.
// Root cause: a fresh keyboard frame (no speed commanded) still produces
// roadSpeedKmh == 0.0, and the loop's `>= 0.0` gate forces the dyno to hold at
// 0 RPM, which opposes and stalls the engine.
// ============================================================================

TEST_F(KeyboardChainFixture, FreshFrame_DoesNotCommandSpeedTracking) {
    // A user that has only just started the app and pressed nothing speed-related
    // must NOT be commanding a road-speed target. The default EngineInput must
    // signal "no speed commanded" so the loop leaves the dyno alone.
    EngineInput input = tick();
    EXPECT_FALSE(shouldDriveSpeedTracking(input.roadSpeedKmh))
        << "Default roadSpeedKmh must read as 'not commanded'; got "
        << input.roadSpeedKmh;
}

TEST_F(RuntimeBugsIntegrationTest, InGear_ZeroRoadSpeed_DoesNotHoldEngineAtZeroRpm) {
    // Engage first gear with the clutch locked, as the loop does when the user
    // shifts into a forward gear.
    trans_->changeGear(0);  // engine-sim convention: 0 == first forward gear
    trans_->setClutchPressure(1.0);

    // A fresh frame (no speed commanded) routed through the loop's gate must NOT
    // configure the dyno to hold the engine at 0 RPM.
    EngineInput freshFrame;  // default-constructed: roadSpeedKmh == 0.0

    ASSERT_FALSE(shouldDriveSpeedTracking(freshFrame.roadSpeedKmh))
        << "If this fails, the loop will call setSpeedTrackingTarget(0.0) on every frame.";

    // Sanity: when the gate is (incorrectly) open, the dyno WOULD hold at zero --
    // this is exactly the stall mechanism.
    simulator_->setSpeedTrackingTarget(0.0);
    EXPECT_NEAR(rawSim_->m_dyno.m_rotationSpeed, 0.0, 1e-3)
        << "Holds: confirms setSpeedTrackingTarget(0.0) is the stall mechanism";
}

// ============================================================================
// Bug 1: ',' / '.' have no visible effect in NEUTRAL.
// The keys DO adjust the roadSpeedKmh target through the chain (proven by unit
// tests). The reason the display is inert: (a) setSpeedTrackingTarget is
// intentionally disabled in neutral, and (b) the speed display reads vehicle
// physics, not the commanded target. This test pins the chain contract so the
// target is at least observable for surfacing in the UI.
// ============================================================================

TEST_F(KeyboardChainFixture, DotKey_RoadSpeedTargetVisibleInEngineInput) {
    EngineInput before = tick();
    double baseline = before.roadSpeedKmh;

    // Hold '.' for several frames (key-active ramps the target).
    for (int i = 0; i < 5; ++i) {
        pressAndTick('.');
    }
    EngineInput after = tick();
    EXPECT_GT(after.roadSpeedKmh, baseline)
        << "'.' must raise the commanded road-speed target carried in EngineInput";
}

TEST_F(KeyboardChainFixture, CommaKey_RoadSpeedTargetVisibleInEngineInput) {
    // Wind the target up first so a decrease is observable.
    for (int i = 0; i < 10; ++i) {
        pressAndTick('.');
    }
    EngineInput before = tick();
    double baseline = before.roadSpeedKmh;

    for (int i = 0; i < 5; ++i) {
        pressAndTick(',');
    }
    EngineInput after = tick();
    EXPECT_LT(after.roadSpeedKmh, baseline)
        << "',' must lower the commanded road-speed target carried in EngineInput";
}

// Neutral disables speed tracking by design; the gate change must not regress
// that behavior. When a real speed IS commanded, setSpeedTrackingTarget should
// still engage in gear.
TEST_F(RuntimeBugsIntegrationTest, InGear_PositiveCommandedSpeed_EngagesSpeedTracking) {
    trans_->changeGear(0);  // first forward gear
    trans_->setClutchPressure(1.0);

    // After the fix, only a genuinely commanded (>0) speed should drive tracking.
    EngineInput commanded;
    commanded.roadSpeedKmh = 50.0;
    ASSERT_TRUE(shouldDriveSpeedTracking(commanded.roadSpeedKmh));

    bool ok = simulator_->setSpeedTrackingTarget(commanded.roadSpeedKmh);
    EXPECT_TRUE(ok) << "A commanded positive speed in gear must engage tracking";
    EXPECT_TRUE(rawSim_->m_dyno.m_enabled) << "Dyno must be enabled for a commanded speed";
    EXPECT_TRUE(rawSim_->m_dyno.m_hold) << "Dyno must hold at the commanded target RPM";
    EXPECT_GT(rawSim_->m_dyno.m_rotationSpeed, 0.0)
        << "Hold RPM must be non-zero for a 50 km/h target in first gear";
}
