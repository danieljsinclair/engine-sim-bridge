// KeyboardInputProviderTest.cpp - TDD RED phase tests for consolidated keyboard input
//
// Tests verify the KeyboardInputProvider dispatches key actions to IKeyActionTarget
// correctly, using KeyHoldBridge for key state tracking.
//
// Tests 1-20: Key dispatch via consolidated KeyboardInputProvider + MockKeyActionTarget
// Tests 21-28: EngineInputTarget state management

#include "input/KeyboardInputProvider.h"
#include "input/EngineInputTarget.h"
#include "input/IKeyActionTarget.h"
#include "input/IKeyboardInput.h"
#include "input/KeyHoldBridge.h"
#include "mocks/MockKeyActionTarget.h"
#include "mocks/MockKeyboardInput.h"

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
// Tests 14-16: Dyno torque
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
// Test 17: Preset cycle
// ============================================================================

TEST_F(KeyboardDispatchTest, PKey_CyclesPreset) {
    createProvider();
    pressKey('p');
    EXPECT_TRUE(mockTargetPtr->presetCalled);
}

// ============================================================================
// Test 18: Quit
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
// Test 19: No key = nothing called
// ============================================================================

TEST_F(KeyboardDispatchTest, NoKey_NothingCalled) {
    createProvider();
    drainFrame();
    EXPECT_TRUE(mockTargetPtr->calls.empty());
}

// ============================================================================
// Test 20: Edge-triggered shift (repeat does NOT shift again)
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
// Test 21: adjustThrottle increases
// ============================================================================

TEST_F(EngineInputTargetTest, AdjustThrottleUp_IncreasesBy005) {
    target->adjustThrottle(0.05);
    EngineInput input = target->buildInput();
    EXPECT_DOUBLE_EQ(input.throttle, 0.15);  // starts at 0.1
}

// ============================================================================
// Test 22: adjustThrottle decreases
// ============================================================================

TEST_F(EngineInputTargetTest, AdjustThrottleDown_DecreasesBy005) {
    target->setThrottle(0.5);
    target->adjustThrottle(-0.05);
    EngineInput input = target->buildInput();
    EXPECT_NEAR(input.throttle, 0.45, 0.001);
}

// ============================================================================
// Test 23: setThrottle absolute
// ============================================================================

TEST_F(EngineInputTargetTest, SetThrottle_SetsAbsolute) {
    target->setThrottle(0.5);
    EngineInput input = target->buildInput();
    EXPECT_DOUBLE_EQ(input.throttle, 0.5);
}

// ============================================================================
// Test 24: shiftUp increments gearSelector and sets gearDelta
// ============================================================================

TEST_F(EngineInputTargetTest, ShiftUp_IncrementsGearSelectorAndSetsDelta) {
    target->shiftUp();
    EngineInput input = target->buildInput();
    EXPECT_EQ(input.gearDelta, 1);
    EXPECT_EQ(input.gearSelector, 1);
}

// ============================================================================
// Test 25: toggleIgnition flips state
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
// Test 26: setStarter is momentary
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
// Test 27: setBrake sets brakeLevel
// ============================================================================

TEST_F(EngineInputTargetTest, SetBrake_SetsLevel) {
    target->setBrake(1.0);
    EngineInput input = target->buildInput();
    EXPECT_DOUBLE_EQ(input.brakeLevel, 1.0);
}

// ============================================================================
// Test 28: buildInput returns correct EngineInput
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
