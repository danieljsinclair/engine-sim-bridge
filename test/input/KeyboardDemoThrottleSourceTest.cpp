#include <gtest/gtest.h>
#include <input/KeyboardDemoThrottleSource.h>
#include <memory>

using namespace input;

// Mock KeyboardInput for testing
class MockKeyboardInput : public input::IKeyboardInput {
public:
    explicit MockKeyboardInput(int key = -1) : key_(key) {}

    int getKey() override { return key_; }

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
// Key 1-9 → throttle 0.1-0.9
// ============================================================================

TEST_F(KeyboardDemoThrottleSourceTest, Key1_ReturnsThrottle0_1) {
    mockKeyboard_.setKey('1');
    double throttle = throttleSource_->pollThrottle();
    EXPECT_DOUBLE_EQ(throttle, 0.1);
}

TEST_F(KeyboardDemoThrottleSourceTest, Key2_ReturnsThrottle0_2) {
    mockKeyboard_.setKey('2');
    double throttle = throttleSource_->pollThrottle();
    EXPECT_DOUBLE_EQ(throttle, 0.2);
}

TEST_F(KeyboardDemoThrottleSourceTest, Key3_ReturnsThrottle0_3) {
    mockKeyboard_.setKey('3');
    double throttle = throttleSource_->pollThrottle();
    EXPECT_DOUBLE_EQ(throttle, 0.3);
}

TEST_F(KeyboardDemoThrottleSourceTest, Key4_ReturnsThrottle0_4) {
    mockKeyboard_.setKey('4');
    double throttle = throttleSource_->pollThrottle();
    EXPECT_DOUBLE_EQ(throttle, 0.4);
}

TEST_F(KeyboardDemoThrottleSourceTest, Key5_ReturnsThrottle0_5) {
    mockKeyboard_.setKey('5');
    double throttle = throttleSource_->pollThrottle();
    EXPECT_DOUBLE_EQ(throttle, 0.5);
}

TEST_F(KeyboardDemoThrottleSourceTest, Key6_ReturnsThrottle0_6) {
    mockKeyboard_.setKey('6');
    double throttle = throttleSource_->pollThrottle();
    EXPECT_DOUBLE_EQ(throttle, 0.6);
}

TEST_F(KeyboardDemoThrottleSourceTest, Key7_ReturnsThrottle0_7) {
    mockKeyboard_.setKey('7');
    double throttle = throttleSource_->pollThrottle();
    EXPECT_DOUBLE_EQ(throttle, 0.7);
}

TEST_F(KeyboardDemoThrottleSourceTest, Key8_ReturnsThrottle0_8) {
    mockKeyboard_.setKey('8');
    double throttle = throttleSource_->pollThrottle();
    EXPECT_DOUBLE_EQ(throttle, 0.8);
}

TEST_F(KeyboardDemoThrottleSourceTest, Key9_ReturnsThrottle0_9) {
    mockKeyboard_.setKey('9');
    double throttle = throttleSource_->pollThrottle();
    EXPECT_DOUBLE_EQ(throttle, 0.9);
}

// ============================================================================
// Key 0 → throttle 1.0
// ============================================================================

TEST_F(KeyboardDemoThrottleSourceTest, Key0_ReturnsThrottle1_0) {
    mockKeyboard_.setKey('0');
    double throttle = throttleSource_->pollThrottle();
    EXPECT_DOUBLE_EQ(throttle, 1.0);
}

// ============================================================================
// No key pressed (-1) → throttle snaps to 0.0
// ============================================================================

TEST_F(KeyboardDemoThrottleSourceTest, NoKey_SnapsThrottleTo0_0) {
    mockKeyboard_.setKey('5');
    throttleSource_->pollThrottle(); // Set to 0.5

    mockKeyboard_.setKey(-1);
    double throttle = throttleSource_->pollThrottle();
    EXPECT_DOUBLE_EQ(throttle, 0.0);
}

TEST_F(KeyboardDemoThrottleSourceTest, NoKeyRepeated_KeepsThrottleAt0_0) {
    mockKeyboard_.setKey(-1);
    throttleSource_->pollThrottle();

    double throttle = throttleSource_->pollThrottle();
    EXPECT_DOUBLE_EQ(throttle, 0.0);
}

// ============================================================================
// Space → throttle 0.0
// ============================================================================

TEST_F(KeyboardDemoThrottleSourceTest, SpaceKey_SetsThrottleTo0_0) {
    mockKeyboard_.setKey('5');
    throttleSource_->pollThrottle(); // Set to 0.5

    mockKeyboard_.setKey(' ');
    double throttle = throttleSource_->pollThrottle();
    EXPECT_DOUBLE_EQ(throttle, 0.0);
}

// ============================================================================
// Q/ESC → shouldContinue = false
// ============================================================================

TEST_F(KeyboardDemoThrottleSourceTest, KeyQ_SetsShouldContinueToFalse) {
    mockKeyboard_.setKey('Q');
    throttleSource_->pollThrottle();
    EXPECT_FALSE(throttleSource_->shouldContinue());
}

TEST_F(KeyboardDemoThrottleSourceTest, Keyq_Lowercase_SetsShouldContinueToFalse) {
    mockKeyboard_.setKey('q');
    throttleSource_->pollThrottle();
    EXPECT_FALSE(throttleSource_->shouldContinue());
}

TEST_F(KeyboardDemoThrottleSourceTest, EscapeKey_SetsShouldContinueToFalse) {
    mockKeyboard_.setKey(27); // ASCII 27 = ESC
    throttleSource_->pollThrottle();
    EXPECT_FALSE(throttleSource_->shouldContinue());
}

// ============================================================================
// ShouldContinue remains true for non-exit keys
// ============================================================================

TEST_F(KeyboardDemoThrottleSourceTest, ThrottleKeys_KeepShouldContinueTrue) {
    for (char key = '0'; key <= '9'; ++key) {
        mockKeyboard_.setKey(key);
        throttleSource_->pollThrottle();
        EXPECT_TRUE(throttleSource_->shouldContinue())
            << "Should continue after key '" << key << "'";
    }
}

TEST_F(KeyboardDemoThrottleSourceTest, SpaceKey_KeepsShouldContinueTrue) {
    mockKeyboard_.setKey(' ');
    throttleSource_->pollThrottle();
    EXPECT_TRUE(throttleSource_->shouldContinue());
}

TEST_F(KeyboardDemoThrottleSourceTest, NoKey_KeepsShouldContinueTrue) {
    mockKeyboard_.setKey(-1);
    throttleSource_->pollThrottle();
    EXPECT_TRUE(throttleSource_->shouldContinue());
}
