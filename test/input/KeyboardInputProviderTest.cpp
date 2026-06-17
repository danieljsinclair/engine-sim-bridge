// KeyboardInputProviderTest.cpp - TDD RED phase tests for consolidated keyboard input
//
// Tests verify the KeyboardInputProvider dispatches key actions to IKeyActionTarget
// correctly, using KeyHoldBridge for key state tracking.
//
// Tests 1-23: Key dispatch via consolidated KeyboardInputProvider + MockKeyActionTarget
// Tests 24-31: EngineInputTarget state management

#include "input/KeyboardInputProvider.h"
#include "input/EngineInputTarget.h"
#include "input/IKeyActionTarget.h"
#include "input/IKeyboardInput.h"
#include "input/KeyHoldBridge.h"
#include "mocks/MockKeyActionTarget.h"
#include "mocks/MockKeyboardInput.h"
#include "input/IDemoSpeedEnhancer.h"

#include <gtest/gtest.h>
#include <memory>
#include <algorithm>

using namespace input;

// ============================================================================
// Test fixture: creates a fresh provider with mock keyboard + mock target
// for each test. Tests are independent.
// ============================================================================

class KeyboardDispatchTest : public ::testing::Test {
protected:
    void SetUp() override {
        mockKeyboard = std::make_unique<MockKeyboardInput>();
        mockKeyboardPtr = mockKeyboard.get();
        mockTarget = std::make_unique<MockKeyActionTarget>();
        mockTargetPtr = mockTarget.get();
    }

    void createProvider() {
        provider = std::make_unique<KeyboardInputProvider>(
            std::move(mockKeyboard), mockTargetPtr);
        provider->Initialize();
    }

    void drainFrame(double dt = 16.0) {
        provider->OnUpdateSimulation(dt);
    }

    // Convenience: enqueue a single key and drain one frame
    void pressKey(int key, double dt = 16.0) {
        mockKeyboardPtr->enqueue(key);
        drainFrame(dt);
    }

    // Convenience: enqueue multiple keys (one per frame) and drain all
    void pressKeys(const std::vector<int>& keys, double dt = 16.0) {
        for (int key : keys) {
            mockKeyboardPtr->enqueue(key);
            drainFrame(dt);
        }
    }

    std::unique_ptr<MockKeyboardInput> mockKeyboard;
    MockKeyboardInput* mockKeyboardPtr = nullptr;
    std::unique_ptr<MockKeyActionTarget> mockTarget;
    MockKeyActionTarget* mockTargetPtr = nullptr;
    std::unique_ptr<KeyboardInputProvider> provider;
};

// ============================================================================
// Tests 1-2: Throttle ramp via adjustThrottle
// ============================================================================

TEST_F(KeyboardDispatchTest, WKey_AdjustsThrottleUp) {
    createProvider();
    pressKey('w');
    ASSERT_FALSE(mockTargetPtr->calls.empty());
    EXPECT_EQ(mockTargetPtr->calls.back(), "adjustThrottle(0.050000)");
}

TEST_F(KeyboardDispatchTest, ZKey_AdjustsThrottleDown) {
    createProvider();
    pressKey('z');
    ASSERT_FALSE(mockTargetPtr->calls.empty());
    EXPECT_EQ(mockTargetPtr->calls.back(), "adjustThrottle(-0.050000)");
}

// ============================================================================
// Test 3: W key held via OS repeat fires again
// ============================================================================

TEST_F(KeyboardDispatchTest, WKeyHeld_OSRepeat_FiresAgain) {
    createProvider();
    // First press
    mockKeyboardPtr->enqueue('w');
    drainFrame();
    // OS repeat (same key arriving again next frame)
    mockKeyboardPtr->enqueue('w');
    drainFrame();

    int adjustCount = 0;
    for (const auto& call : mockTargetPtr->calls) {
        if (call.find("adjustThrottle(0.050000)") != std::string::npos) {
            adjustCount++;
        }
    }
    EXPECT_GE(adjustCount, 2) << "W key held via OS repeat should fire adjustThrottle again";
}

// ============================================================================
// Tests 4: Discrete throttle via number keys 0-9
// ============================================================================

TEST_F(KeyboardDispatchTest, NumberKey0_SetsThrottleTo100) {
    createProvider();
    pressKey('0');
    ASSERT_FALSE(mockTargetPtr->calls.empty());
    EXPECT_DOUBLE_EQ(mockTargetPtr->lastThrottleLevel, 1.0);
}

TEST_F(KeyboardDispatchTest, NumberKey1_SetsThrottleTo10) {
    createProvider();
    pressKey('1');
    ASSERT_FALSE(mockTargetPtr->calls.empty());
    EXPECT_DOUBLE_EQ(mockTargetPtr->lastThrottleLevel, 0.1);
}

TEST_F(KeyboardDispatchTest, NumberKey5_SetsThrottleTo50) {
    createProvider();
    pressKey('5');
    ASSERT_FALSE(mockTargetPtr->calls.empty());
    EXPECT_DOUBLE_EQ(mockTargetPtr->lastThrottleLevel, 0.5);
}

TEST_F(KeyboardDispatchTest, NumberKey9_SetsThrottleTo90) {
    createProvider();
    pressKey('9');
    ASSERT_FALSE(mockTargetPtr->calls.empty());
    EXPECT_DOUBLE_EQ(mockTargetPtr->lastThrottleLevel, 0.9);
}

// ============================================================================
// Test 5: Space zeroes throttle
// ============================================================================

TEST_F(KeyboardDispatchTest, SpaceKey_SetsThrottleToZero) {
    createProvider();
    pressKey(' ');
    ASSERT_FALSE(mockTargetPtr->calls.empty());
    EXPECT_DOUBLE_EQ(mockTargetPtr->lastThrottleLevel, 0.0);
}

// ============================================================================
// Test 6: R sets 20% throttle
// ============================================================================

TEST_F(KeyboardDispatchTest, RKey_SetsThrottleTo20Percent) {
    createProvider();
    pressKey('r');
    ASSERT_FALSE(mockTargetPtr->calls.empty());
    EXPECT_DOUBLE_EQ(mockTargetPtr->lastThrottleLevel, 0.2);
}

// ============================================================================
// Tests 7-8: Gear shift
// ============================================================================

TEST_F(KeyboardDispatchTest, CloseBracket_ShiftsUp) {
    createProvider();
    pressKey(']');
    EXPECT_EQ(mockTargetPtr->shiftUpCount, 1);
}

TEST_F(KeyboardDispatchTest, OpenBracket_ShiftsDown) {
    createProvider();
    pressKey('[');
    EXPECT_EQ(mockTargetPtr->shiftDownCount, 1);
}

// ============================================================================
// Test 9: Ignition toggle
// ============================================================================

TEST_F(KeyboardDispatchTest, IKey_TogglesIgnition) {
    createProvider();
    pressKey('i');
    EXPECT_TRUE(mockTargetPtr->ignitionToggled);
}

// ============================================================================
// Test 10: Starter (momentary)
// ============================================================================

TEST_F(KeyboardDispatchTest, SKey_SetsStarter) {
    createProvider();
    pressKey('s');
    EXPECT_TRUE(mockTargetPtr->starterCalled);
}

// ============================================================================
// Tests 11-13: Brake (level-triggered)
// ============================================================================

TEST_F(KeyboardDispatchTest, BKey_SetsBrakeToFull) {
    createProvider();
    pressKey('b');
    ASSERT_FALSE(mockTargetPtr->calls.empty());
    EXPECT_DOUBLE_EQ(mockTargetPtr->lastBrakeLevel, 1.0);
}

TEST_F(KeyboardDispatchTest, BKeyReleased_SetsBrakeToZero) {
    createProvider();
    // Press B then wait for release (no more B events)
    mockKeyboardPtr->enqueue('b');
    drainFrame();
    // KeyHoldBridge times out after no repeat, releasing the key
    // Drain several frames with no key to simulate release
    for (int i = 0; i < 10; i++) {
        drainFrame(50.0);  // Fast timeout to trigger release
    }
    // After release, brake should be set to 0
    bool brakeReleaseFound = false;
    for (const auto& call : mockTargetPtr->calls) {
        if (call.find("setBrake(0.000000)") != std::string::npos) {
            brakeReleaseFound = true;
            break;
        }
    }
    EXPECT_TRUE(brakeReleaseFound) << "Brake should be released when B key times out";
}

TEST_F(KeyboardDispatchTest, BKeyHeld_SetsBrakeEachFrame) {
    createProvider();
    // Simulate held key: B arriving every frame
    mockKeyboardPtr->enqueue('b');
    drainFrame();
    mockKeyboardPtr->enqueue('b');
    drainFrame();
    mockKeyboardPtr->enqueue('b');
    drainFrame();

    int brakeCount = 0;
    for (const auto& call : mockTargetPtr->calls) {
        if (call.find("setBrake(1.000000)") != std::string::npos) {
            brakeCount++;
        }
    }
    EXPECT_GE(brakeCount, 2) << "Held B key should set brake each frame";
}

// ============================================================================
// Tests 14-17: Speed control (comma and dot keys)
// ============================================================================

TEST_F(KeyboardDispatchTest, CommaKey_DecreasesSpeed) {
    createProvider();
    pressKey(',');
    ASSERT_FALSE(mockTargetPtr->calls.empty());
    EXPECT_DOUBLE_EQ(mockTargetPtr->lastSpeedDelta, -2.0);
}

TEST_F(KeyboardDispatchTest, DotKey_IncreasesSpeed) {
    createProvider();
    pressKey('.');
    ASSERT_FALSE(mockTargetPtr->calls.empty());
    EXPECT_DOUBLE_EQ(mockTargetPtr->lastSpeedDelta, 2.0);
}

TEST_F(KeyboardDispatchTest, CommaKeyHeld_RampsSpeed) {
    createProvider();
    // First press
    mockKeyboardPtr->enqueue(',');
    drainFrame();
    // OS repeat (same key arriving again next frame)
    mockKeyboardPtr->enqueue(',');
    drainFrame();

    EXPECT_GE(mockTargetPtr->speedAdjustCount, 2) << "Comma key held via OS repeat should fire multiple times";
}

// ============================================================================
// Tests 18-21: Dyno torque
// ============================================================================

TEST_F(KeyboardDispatchTest, EKey_DecreasesDynoTorque) {
    createProvider();
    pressKey('e');
    ASSERT_FALSE(mockTargetPtr->calls.empty());
    EXPECT_DOUBLE_EQ(mockTargetPtr->lastDynoDelta, -0.1);
}

TEST_F(KeyboardDispatchTest, DKey_IncreasesDynoTorque) {
    createProvider();
    pressKey('d');
    ASSERT_FALSE(mockTargetPtr->calls.empty());
    EXPECT_DOUBLE_EQ(mockTargetPtr->lastDynoDelta, 0.1);
}

TEST_F(KeyboardDispatchTest, CKey_ReleasesDynoTorque) {
    createProvider();
    pressKey('c');
    EXPECT_TRUE(mockTargetPtr->dynoReleased);
}

// ============================================================================
// Test 22: Preset cycle
// ============================================================================

TEST_F(KeyboardDispatchTest, PKey_CyclesPreset) {
    createProvider();
    pressKey('p');
    EXPECT_TRUE(mockTargetPtr->presetCalled);
}

// ============================================================================
// Test 23: Quit
// ============================================================================

TEST_F(KeyboardDispatchTest, QKey_TriggersQuit) {
    createProvider();
    pressKey('q');
    EXPECT_TRUE(mockTargetPtr->quitCalled);
}

TEST_F(KeyboardDispatchTest, EscapeKey_TriggersQuit) {
    createProvider();
    pressKey(27);
    EXPECT_TRUE(mockTargetPtr->quitCalled);
}

// ============================================================================
// Test 24: No key = nothing called
// ============================================================================

TEST_F(KeyboardDispatchTest, NoKey_NothingCalled) {
    createProvider();
    drainFrame();
    EXPECT_TRUE(mockTargetPtr->calls.empty());
}

// ============================================================================
// Test 25: Edge-triggered shift (repeat does NOT shift again)
// ============================================================================

TEST_F(KeyboardDispatchTest, ShiftRepeat_DoesNotShiftAgain) {
    createProvider();
    // First press
    mockKeyboardPtr->enqueue(']');
    drainFrame();
    EXPECT_EQ(mockTargetPtr->shiftUpCount, 1);

    // Same key arriving again (edge-triggered: should NOT shift again)
    mockKeyboardPtr->enqueue(']');
    drainFrame();
    EXPECT_EQ(mockTargetPtr->shiftUpCount, 1)
        << "Shift should be edge-triggered: repeat of ] must NOT shift again";
}

// ============================================================================
// Test fixture for EngineInputTarget state tests
// ============================================================================

class EngineInputTargetTest : public ::testing::Test {
protected:
    void SetUp() override {
        target = std::make_unique<EngineInputTarget>();
    }

    std::unique_ptr<EngineInputTarget> target;
};

// ============================================================================
// Test 26: adjustThrottle increases by delta
// ============================================================================

TEST_F(EngineInputTargetTest, AdjustThrottleUp_IncreasesBy005) {
    // Get initial throttle value (whatever it is)
    double initial = target->buildInput().throttle;

    target->adjustThrottle(0.05);
    EngineInput input = target->buildInput();

    // Test the delta, not the absolute value
    EXPECT_DOUBLE_EQ(input.throttle, initial + 0.05);
}

// ============================================================================
// Test 27: adjustThrottle decreases by delta
// ============================================================================

TEST_F(EngineInputTargetTest, AdjustThrottleDown_DecreasesBy005) {
    target->setThrottle(0.5);
    double before = target->buildInput().throttle;

    target->adjustThrottle(-0.05);
    EngineInput input = target->buildInput();

    // Test the delta behavior
    EXPECT_NEAR(input.throttle, before - 0.05, 0.001);
}

// ============================================================================
// Test 28: setThrottle absolute
// ============================================================================

TEST_F(EngineInputTargetTest, SetThrottle_SetsAbsolute) {
    target->setThrottle(0.5);
    EngineInput input = target->buildInput();
    EXPECT_DOUBLE_EQ(input.throttle, 0.5);
}

// ============================================================================
// Test 29: shiftUp increments gearSelector and sets gearDelta
// ============================================================================

TEST_F(EngineInputTargetTest, ShiftUp_IncrementsGearSelectorAndSetsDelta) {
    target->shiftUp();
    EngineInput input = target->buildInput();
    EXPECT_EQ(input.gearDelta, 1);
    EXPECT_EQ(input.gearSelector, 1);
}

// ============================================================================
// Manual gear counter clamps to the valid selector range (REVERSE=-1 .. EIGHTH=8)
// so the display can never render a stray 'P' (PARK=-2) or '?' (>8).
// ============================================================================

TEST_F(EngineInputTargetTest, ShiftUp_ClampsAtEighthGear) {
    for (int i = 0; i < 20; ++i) target->shiftUp();
    EXPECT_EQ(target->buildInput().gearSelector, 8);
}

TEST_F(EngineInputTargetTest, ShiftDown_ClampsAtReverse) {
    for (int i = 0; i < 20; ++i) target->shiftDown();
    EXPECT_EQ(target->buildInput().gearSelector, -1);
}

// ============================================================================
// Feedback loop: provideFeedback must route simulator stats (RPM/speed/torque)
// to the speed enhancer (twin/gearbox). Regression for the "redlines but never
// upshifts" bug — KeyboardInputProvider had no provideFeedback override, so the
// gearbox's rpmFeedback_ stayed 0 and the redline safety never fired.
// ============================================================================
namespace {
class RecordingEnhancer : public input::IDemoSpeedEnhancer {
public:
    int feedbackCount = 0;
    double lastRpm = -1.0;
    input::EngineInput enhanceInput(const input::EngineInput& base, double) override { return base; }
    void provideFeedback(const EngineSimStats& stats) override {
        ++feedbackCount;
        lastRpm = stats.currentRPM;
    }
};
} // namespace

TEST_F(EngineInputTargetTest, ProvideFeedback_ForwardsToSpeedEnhancer) {
    RecordingEnhancer enh;
    target->setSpeedEnhancer(&enh);
    EngineSimStats stats{};
    stats.currentRPM = 5000.0;
    target->provideFeedback(stats);
    EXPECT_EQ(enh.feedbackCount, 1);
    EXPECT_DOUBLE_EQ(enh.lastRpm, 5000.0);
}

TEST_F(EngineInputTargetTest, ProvideFeedback_NoEnhancer_IsSafe) {
    EngineSimStats stats{};
    target->provideFeedback(stats);  // no enhancer set — must not crash
    SUCCEED();
}

TEST_F(EngineInputTargetTest, ProvideFeedback_ChainsKeyboardProviderToEnhancer) {
    RecordingEnhancer enh;
    target->setSpeedEnhancer(&enh);
    auto kb = std::make_unique<MockKeyboardInput>();
    input::KeyboardInputProvider provider(std::move(kb), target.get());
    EngineSimStats stats{};
    stats.currentRPM = 6500.0;
    provider.provideFeedback(stats);
    EXPECT_EQ(enh.feedbackCount, 1);
    EXPECT_DOUBLE_EQ(enh.lastRpm, 6500.0);
}

// ============================================================================
// Test 30: toggleIgnition flips state
// ============================================================================

TEST_F(EngineInputTargetTest, ToggleIgnition_FlipsState) {
    EngineInput before = target->buildInput();
    EXPECT_TRUE(before.ignition);  // default on

    target->toggleIgnition();
    EngineInput after = target->buildInput();
    EXPECT_FALSE(after.ignition);

    target->toggleIgnition();
    EngineInput again = target->buildInput();
    EXPECT_TRUE(again.ignition);
}

// ============================================================================
// Test 31: setStarter is momentary
// ============================================================================

TEST_F(EngineInputTargetTest, SetStarter_SetsMomentaryFlag) {
    target->setStarter();
    EngineInput input = target->buildInput();
    EXPECT_TRUE(input.starterButton);

    // Second build should have starter reset (momentary)
    EngineInput input2 = target->buildInput();
    EXPECT_FALSE(input2.starterButton);
}

// ============================================================================
// Test 32: setBrake sets brakeLevel
// ============================================================================

TEST_F(EngineInputTargetTest, SetBrake_SetsLevel) {
    target->setBrake(1.0);
    EngineInput input = target->buildInput();
    EXPECT_DOUBLE_EQ(input.brakeLevel, 1.0);
}

// ============================================================================
// Test 33: buildInput returns correct EngineInput
// ============================================================================

TEST_F(EngineInputTargetTest, BuildInput_ReturnsCorrectEngineInput) {
    target->setThrottle(0.7);
    target->shiftUp();
    target->setBrake(0.5);
    target->cyclePreset();

    EngineInput input = target->buildInput();

    EXPECT_DOUBLE_EQ(input.throttle, 0.7);
    EXPECT_EQ(input.gearDelta, 1);
    EXPECT_EQ(input.gearSelector, 1);
    EXPECT_DOUBLE_EQ(input.brakeLevel, 0.5);
    EXPECT_TRUE(input.presetCycle);
    EXPECT_TRUE(input.ignition);  // unchanged

    // Verify one-shot flags are consumed
    EngineInput input2 = target->buildInput();
    EXPECT_EQ(input2.gearDelta, 0);
    EXPECT_FALSE(input2.presetCycle);
}

// ============================================================================
// Tests 34-40: Throttle latch vs momentary semantics
// ============================================================================

// Latched keys: W/Z/R/Space -- set and forget, NO decay

TEST_F(EngineInputTargetTest, AdjustThrottleUp_LatchesAndStays) {
    target->adjustThrottle(0.05);  // W key adds 0.05
    auto input1 = target->buildInput();
    double latchedThrottle = input1.throttle;

    // Multiple frames with no key -- should NOT decay (latching behavior)
    for (int i = 0; i < 100; i++) {
        auto input = target->buildInput();
        EXPECT_DOUBLE_EQ(input.throttle, latchedThrottle) << "Latched throttle should not decay at frame " << i;
    }
}

TEST_F(EngineInputTargetTest, AdjustThrottleDown_LatchesAndStays) {
    // Set a known baseline
    target->setThrottle(0.1);
    double before = target->buildInput().throttle;

    target->adjustThrottle(-0.05);  // Z key subtracts 0.05
    auto input1 = target->buildInput();
    EXPECT_DOUBLE_EQ(input1.throttle, before - 0.05);

    // Latching: no decay across frames
    auto input2 = target->buildInput();
    EXPECT_DOUBLE_EQ(input2.throttle, input1.throttle);
}

TEST_F(EngineInputTargetTest, SetThrottle20_LatchesAndStays) {
    target->setThrottle(0.2);  // R key
    auto input = target->buildInput();
    EXPECT_DOUBLE_EQ(input.throttle, 0.2);

    // No decay over many frames
    for (int i = 0; i < 50; i++) {
        auto input = target->buildInput();
        EXPECT_DOUBLE_EQ(input.throttle, 0.2) << "R throttle should not decay at frame " << i;
    }
}

TEST_F(EngineInputTargetTest, SetThrottleZero_LatchesAndStays) {
    target->setThrottle(0.0);  // Space
    auto input = target->buildInput();
    EXPECT_DOUBLE_EQ(input.throttle, 0.0);

    auto input2 = target->buildInput();
    EXPECT_DOUBLE_EQ(input2.throttle, 0.0);
}

// Momentary keys: 0-9 -- held only, smooth decay back to latched baseline on release

TEST_F(EngineInputTargetTest, MomentaryThrottle_HeldStays_DecaysToBaseline) {
    // Set a latched baseline first (establishes the decay target)
    target->adjustThrottle(0.05);  // W key: establishes baseline
    target->buildInput();  // consume one frame to latch
    double baselineThrottle = target->buildInput().throttle;

    // Press 7 (momentary 70%)
    target->setThrottleMomentary(0.7);
    auto input1 = target->buildInput();
    EXPECT_DOUBLE_EQ(input1.throttle, 0.7);

    // Release (no key next frame) -- throttle should decay toward baseline
    // First frame after release: moves toward baseline (not instant)
    auto input2 = target->buildInput();
    EXPECT_LT(input2.throttle, 0.7);
    EXPECT_GT(input2.throttle, baselineThrottle);

    // After enough frames, converges to baseline (test decay behavior)
    double lastThrottle = input2.throttle;
    for (int i = 0; i < 100; i++) {
        auto input = target->buildInput();
        lastThrottle = input.throttle;
    }
    EXPECT_NEAR(lastThrottle, baselineThrottle, 0.005);
}

TEST_F(EngineInputTargetTest, MomentaryThrottle_NoBaseline_DecaysToZero) {
    // Press Space to set latched baseline to 0
    target->setThrottle(0.0);
    target->buildInput();

    // Press 5 (momentary 50%)
    target->setThrottleMomentary(0.5);
    auto input1 = target->buildInput();
    EXPECT_DOUBLE_EQ(input1.throttle, 0.5);

    // Release -- decays toward baseline (0)
    auto input2 = target->buildInput();
    EXPECT_LT(input2.throttle, 0.5);
    EXPECT_GT(input2.throttle, 0.0);

    // Converges to baseline
    double lastThrottle = input2.throttle;
    for (int i = 0; i < 100; i++) {
        auto input = target->buildInput();
        lastThrottle = input.throttle;
    }
    EXPECT_NEAR(lastThrottle, 0.0, 0.005);
}

TEST_F(EngineInputTargetTest, MomentaryThrottle_HeldMultipleFrames_ThenDecaysToBaseline) {
    target->setThrottle(0.2);  // R: baseline = 0.2
    target->buildInput();

    // Hold 9 (momentary 90%) for 10 frames
    for (int i = 0; i < 10; i++) {
        target->setThrottleMomentary(0.9);
        auto input = target->buildInput();
        EXPECT_DOUBLE_EQ(input.throttle, 0.9) << "Momentary should hold at frame " << i;
    }

    // Release -- decays toward baseline (not instant)
    auto input = target->buildInput();
    EXPECT_LT(input.throttle, 0.9);
    EXPECT_GT(input.throttle, 0.2);

    // Converges to baseline
    double lastThrottle = input.throttle;
    for (int i = 0; i < 100; i++) {
        auto input = target->buildInput();
        lastThrottle = input.throttle;
    }
    EXPECT_NEAR(lastThrottle, 0.2, 0.005);
}

// ============================================================================
// Tests 41-46: Speed control state
// ============================================================================

TEST_F(EngineInputTargetTest, DefaultRoadSpeed_IsUncommandedSentinel) {
    // The default must signal "no speed commanded" so the simulation loop does
    // not force the dyno to hold the engine at 0 RPM (which stalls it in gear).
    EngineInput input = target->buildInput();
    EXPECT_LT(input.roadSpeedKmh, 0.0)
        << "Uncommanded roadSpeedKmh must be negative; got " << input.roadSpeedKmh;
}

TEST_F(EngineInputTargetTest, AdjustSpeedUp_IncreasesByDelta) {
    // Establish a 0 km/h baseline from the uncommanded sentinel first.
    target->adjustSpeed(0.0);
    target->adjustSpeed(10.0);
    EngineInput input = target->buildInput();
    EXPECT_DOUBLE_EQ(input.roadSpeedKmh, 10.0);
}

TEST_F(EngineInputTargetTest, AdjustSpeedDown_DecreasesByDelta) {
    target->adjustSpeed(0.0);
    target->adjustSpeed(50.0);
    target->adjustSpeed(-10.0);
    EngineInput input = target->buildInput();
    EXPECT_DOUBLE_EQ(input.roadSpeedKmh, 40.0);
}

TEST_F(EngineInputTargetTest, AdjustSpeed_ClampsToZero) {
    target->adjustSpeed(0.0);
    target->adjustSpeed(50.0);
    target->adjustSpeed(-100.0);  // Try to go below zero
    EngineInput input = target->buildInput();
    EXPECT_DOUBLE_EQ(input.roadSpeedKmh, 0.0);
}

TEST_F(EngineInputTargetTest, AdjustSpeed_ClampsToMax) {
    target->adjustSpeed(0.0);
    target->adjustSpeed(200.0);
    target->adjustSpeed(150.0);  // Try to exceed 300 km/h
    EngineInput input = target->buildInput();
    EXPECT_DOUBLE_EQ(input.roadSpeedKmh, 300.0);
}

TEST_F(EngineInputTargetTest, BuildInput_IncludesRoadSpeed) {
    target->adjustSpeed(0.0);
    target->adjustSpeed(100.0);
    EngineInput input = target->buildInput();
    EXPECT_DOUBLE_EQ(input.roadSpeedKmh, 100.0);
}

// ============================================================================
// Gearbox-mode propagation (AC1/AC2): --auto must engage the automatic box
// and disable manual ]/[ shifting; default stays manual.
// ============================================================================

TEST_F(EngineInputTargetTest, DefaultMode_IsManual) {
    // AC2: without --auto, mode is manual (M)
    EngineInput input = target->buildInput();
    EXPECT_FALSE(input.gearAutoMode);
}

TEST_F(EngineInputTargetTest, ManualMode_ShiftKeysChangeGear) {
    // AC2: ]/[ shift in manual mode
    target->shiftUp();
    EngineInput input = target->buildInput();
    EXPECT_EQ(input.gearDelta, 1);
    EXPECT_NE(input.gearSelector, 0);
}

TEST_F(EngineInputTargetTest, AutoMode_ReportsGearAutoModeTrue) {
    // AC1: with --auto, gearAutoMode is true (display char A)
    target->setGearAutoMode(true);
    EngineInput input = target->buildInput();
    EXPECT_TRUE(input.gearAutoMode);
}

TEST_F(EngineInputTargetTest, AutoMode_ShiftKeysDoNotChangeGear) {
    // AC1: in auto mode the box shifts itself; manual ]/[ must NOT change gear.
    // gearDelta stays 0 and the gear selector/number is untouched.
    target->setGearAutoMode(true);
    target->shiftUp();
    target->shiftDown();
    EngineInput input = target->buildInput();
    EXPECT_EQ(input.gearDelta, 0)
        << "Manual shift keys must not request a gear change in auto mode";
    EXPECT_EQ(input.gearSelector, 0)
        << "Manual shift keys must not move the selector in auto mode";
}
