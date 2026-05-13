#include <gtest/gtest.h>
#include <input/KeyboardDemoThrottleSource.h>
#include <simulator/GearConventions.h>
#include <memory>

using namespace input;

class MockKeyboardInput : public input::IKeyboardInput {
public:
    explicit MockKeyboardInput(int key = -1) : key_(key) {}
    int getKey() override {
        int k = key_;
        key_ = -1;  // Auto-reset like real keyboard (buffered key consumed)
        return k;
    }
    void setKey(int key) { key_ = key; }
private:
    int key_;
};

class KeyboardDemoThrottleSourceTest : public ::testing::Test {
protected:
    MockKeyboardInput mockKeyboard_{-1};
    std::unique_ptr<KeyboardDemoThrottleSource> throttleSource_;

    void SetUp() override {
        throttleSource_ = std::make_unique<KeyboardDemoThrottleSource>(mockKeyboard_);
    }
};

// ============================================================================
// Throttle keys 1-9 → throttle 0.1-0.9
// ============================================================================

TEST_F(KeyboardDemoThrottleSourceTest, Key1_ReturnsThrottle0_1) {
    mockKeyboard_.setKey('1');
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.1);
}

TEST_F(KeyboardDemoThrottleSourceTest, Key5_ReturnsThrottle0_5) {
    mockKeyboard_.setKey('5');
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.5);
}

TEST_F(KeyboardDemoThrottleSourceTest, Key9_ReturnsThrottle0_9) {
    mockKeyboard_.setKey('9');
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.9);
}

TEST_F(KeyboardDemoThrottleSourceTest, Key0_ReturnsThrottle1_0) {
    mockKeyboard_.setKey('0');
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 1.0);
}

// ============================================================================
// Throttle snaps to 0 when no key held
// ============================================================================

TEST_F(KeyboardDemoThrottleSourceTest, NoKey_HoldsThenSnapsToZero) {
    mockKeyboard_.setKey('5');
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.5);

    // Hold frame: throttle persists for THROTTLE_HOLD_FRAMES after key release
    for (int i = 0; i < KeyboardDemoThrottleSource::THROTTLE_HOLD_FRAMES; ++i) {
        EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.5)
            << "Throttle should hold at 0.5 for " << KeyboardDemoThrottleSource::THROTTLE_HOLD_FRAMES << " frames";
    }

    // After hold window expires, snaps to 0.0
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.0);
}

TEST_F(KeyboardDemoThrottleSourceTest, ThrottleKeyReturnsCorrectValue) {
    mockKeyboard_.setKey('7');
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.7);

    // Drain hold window
    for (int i = 0; i <= KeyboardDemoThrottleSource::THROTTLE_HOLD_FRAMES; ++i) {
        throttleSource_->pollThrottle();
    }
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.0);

    mockKeyboard_.setKey('3');
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.3);

    for (int i = 0; i <= KeyboardDemoThrottleSource::THROTTLE_HOLD_FRAMES; ++i) {
        throttleSource_->pollThrottle();
    }
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.0);
}

// ============================================================================
// Q/ESC → shouldContinue = false
// ============================================================================

TEST_F(KeyboardDemoThrottleSourceTest, KeyQ_SetsShouldContinueFalse) {
    mockKeyboard_.setKey('Q');
    throttleSource_->pollThrottle();
    EXPECT_FALSE(throttleSource_->shouldContinue());
}

TEST_F(KeyboardDemoThrottleSourceTest, EscapeKey_SetsShouldContinueFalse) {
    mockKeyboard_.setKey(27);
    throttleSource_->pollThrottle();
    EXPECT_FALSE(throttleSource_->shouldContinue());
}

TEST_F(KeyboardDemoThrottleSourceTest, NonExitKeys_KeepShouldContinueTrue) {
    mockKeyboard_.setKey('5');
    throttleSource_->pollThrottle();
    EXPECT_TRUE(throttleSource_->shouldContinue());
}

// ============================================================================
// Gear selector: initial state is NEUTRAL
// ============================================================================

TEST_F(KeyboardDemoThrottleSourceTest, InitialSelector_IsNeutral) {
    EXPECT_EQ(throttleSource_->getGearSelector(),
              static_cast<int>(bridge::GearSelector::NEUTRAL));
}

// ============================================================================
// Direct selector keys (P/R/N/D)
// ============================================================================

TEST_F(KeyboardDemoThrottleSourceTest, KeyP_SelectsPark) {
    mockKeyboard_.setKey('P');
    throttleSource_->pollThrottle();
    EXPECT_EQ(throttleSource_->getGearSelector(),
              static_cast<int>(bridge::GearSelector::PARK));
}

TEST_F(KeyboardDemoThrottleSourceTest, KeyR_SelectsReverse) {
    mockKeyboard_.setKey('R');
    throttleSource_->pollThrottle();
    EXPECT_EQ(throttleSource_->getGearSelector(),
              static_cast<int>(bridge::GearSelector::REVERSE));
}

TEST_F(KeyboardDemoThrottleSourceTest, KeyN_SelectsNeutral) {
    mockKeyboard_.setKey('D');
    throttleSource_->pollThrottle();
    ASSERT_EQ(throttleSource_->getGearSelector(),
              static_cast<int>(bridge::GearSelector::DRIVE));

    mockKeyboard_.setKey('N');
    throttleSource_->pollThrottle();
    EXPECT_EQ(throttleSource_->getGearSelector(),
              static_cast<int>(bridge::GearSelector::NEUTRAL));
}

TEST_F(KeyboardDemoThrottleSourceTest, KeyD_SelectsDrive) {
    mockKeyboard_.setKey('D');
    throttleSource_->pollThrottle();
    EXPECT_EQ(throttleSource_->getGearSelector(),
              static_cast<int>(bridge::GearSelector::DRIVE));
}

// ============================================================================
// PRNDL cycling with [ and ] — clamping at ends
// ============================================================================

TEST_F(KeyboardDemoThrottleSourceTest, ShiftUp_FromNeutral_GoesToDrive) {
    ASSERT_EQ(throttleSource_->getGearSelector(),
              static_cast<int>(bridge::GearSelector::NEUTRAL));

    mockKeyboard_.setKey(']');
    throttleSource_->pollThrottle();
    EXPECT_EQ(throttleSource_->getGearSelector(),
              static_cast<int>(bridge::GearSelector::DRIVE));
}

TEST_F(KeyboardDemoThrottleSourceTest, ShiftDown_FromDrive_GoesToNeutral) {
    mockKeyboard_.setKey('D');
    throttleSource_->pollThrottle();
    ASSERT_EQ(throttleSource_->getGearSelector(),
              static_cast<int>(bridge::GearSelector::DRIVE));

    mockKeyboard_.setKey('[');
    throttleSource_->pollThrottle();
    EXPECT_EQ(throttleSource_->getGearSelector(),
              static_cast<int>(bridge::GearSelector::NEUTRAL));
}

TEST_F(KeyboardDemoThrottleSourceTest, ShiftUp_FromDrive_ClampsAtDrive) {
    mockKeyboard_.setKey('D');
    throttleSource_->pollThrottle();

    // Already at DRIVE — shiftUp should clamp
    mockKeyboard_.setKey(']');
    throttleSource_->pollThrottle();
    EXPECT_EQ(throttleSource_->getGearSelector(),
              static_cast<int>(bridge::GearSelector::DRIVE));
}

TEST_F(KeyboardDemoThrottleSourceTest, ShiftDown_FromPark_ClampsAtPark) {
    mockKeyboard_.setKey('P');
    throttleSource_->pollThrottle();

    // Already at PARK — shiftDown should clamp
    mockKeyboard_.setKey('[');
    throttleSource_->pollThrottle();
    EXPECT_EQ(throttleSource_->getGearSelector(),
              static_cast<int>(bridge::GearSelector::PARK));
}

TEST_F(KeyboardDemoThrottleSourceTest, ShiftUp_CyclesP_R_N_D) {
    mockKeyboard_.setKey('P');
    throttleSource_->pollThrottle();

    mockKeyboard_.setKey(']');
    throttleSource_->pollThrottle();
    EXPECT_EQ(throttleSource_->getGearSelector(),
              static_cast<int>(bridge::GearSelector::REVERSE));

    mockKeyboard_.setKey(']');
    throttleSource_->pollThrottle();
    EXPECT_EQ(throttleSource_->getGearSelector(),
              static_cast<int>(bridge::GearSelector::NEUTRAL));

    mockKeyboard_.setKey(']');
    throttleSource_->pollThrottle();
    EXPECT_EQ(throttleSource_->getGearSelector(),
              static_cast<int>(bridge::GearSelector::DRIVE));
}

TEST_F(KeyboardDemoThrottleSourceTest, ShiftDown_CyclesD_N_R_P) {
    mockKeyboard_.setKey('D');
    throttleSource_->pollThrottle();

    mockKeyboard_.setKey('[');
    throttleSource_->pollThrottle();
    EXPECT_EQ(throttleSource_->getGearSelector(),
              static_cast<int>(bridge::GearSelector::NEUTRAL));

    mockKeyboard_.setKey('[');
    throttleSource_->pollThrottle();
    EXPECT_EQ(throttleSource_->getGearSelector(),
              static_cast<int>(bridge::GearSelector::REVERSE));

    mockKeyboard_.setKey('[');
    throttleSource_->pollThrottle();
    EXPECT_EQ(throttleSource_->getGearSelector(),
              static_cast<int>(bridge::GearSelector::PARK));
}

// ============================================================================
// Selector keys don't affect throttle
// ============================================================================

TEST_F(KeyboardDemoThrottleSourceTest, SelectorKeys_DontInterruptThrottleHold) {
    mockKeyboard_.setKey('5');
    throttleSource_->pollThrottle(); // throttle = 0.5, hold starts

    // Gear selector key doesn't interrupt throttle hold
    mockKeyboard_.setKey(']');
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.5)
        << "Selector key should not interrupt throttle hold";

    // Drain hold window
    for (int i = 0; i < KeyboardDemoThrottleSource::THROTTLE_HOLD_FRAMES; ++i) {
        throttleSource_->pollThrottle();
    }
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.0);
}
