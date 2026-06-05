#include <gtest/gtest.h>
#include <input/DemoThrottleSource.h>
#include <memory>

using namespace input;

class DemoThrottleSourceTest : public ::testing::Test {
protected:
    // Use holdFrames=4 for deterministic tests
    std::unique_ptr<DemoThrottleSource> throttleSource_;

    void SetUp() override {
        throttleSource_ = std::make_unique<DemoThrottleSource>(4);
    }
};

TEST_F(DemoThrottleSourceTest, DefaultThrottleIsZero) {
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.0);
}

TEST_F(DemoThrottleSourceTest, SetThrottleLevel_ReturnsOnNextPoll) {
    throttleSource_->setThrottleLevel(0.5);
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.5);
}

TEST_F(DemoThrottleSourceTest, SetThrottleLevel0_1_ReturnsOnNextPoll) {
    throttleSource_->setThrottleLevel(0.1);
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.1);
}

TEST_F(DemoThrottleSourceTest, SetThrottleLevel1_0_ReturnsOnNextPoll) {
    throttleSource_->setThrottleLevel(1.0);
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 1.0);
}

TEST_F(DemoThrottleSourceTest, ThrottleDecays_AfterHoldFrames) {
    throttleSource_->setThrottleLevel(0.7);

    // Hold for holdFrames polls — should return the set value
    for (int i = 0; i < 4; ++i) {
        EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.7);
    }

    // Next poll — decayed to zero
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.0);
}

TEST_F(DemoThrottleSourceTest, SetThrottle_ResetsHoldCounter) {
    throttleSource_->setThrottleLevel(0.6);

    // Poll 2 of 4 hold frames
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.6);
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.6);

    // Re-set before decay — counter resets
    throttleSource_->setThrottleLevel(0.8);

    // Should hold for another full 4 frames
    for (int i = 0; i < 4; ++i) {
        EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.8);
    }
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.0);
}

TEST_F(DemoThrottleSourceTest, KeyRepeatSustainsThrottle) {
    // Simulates key repeat: setLevel called each frame
    for (int i = 0; i < 20; ++i) {
        throttleSource_->setThrottleLevel(0.5);
        EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.5);
    }

    // Key released — no more setLevel calls — decays after hold frames
    // Last loop iteration left framesSinceSet_ at 1, so 3 more holds remain
    for (int i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.5);
    }
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.0);
}

TEST_F(DemoThrottleSourceTest, ZeroThrottleStaysZero) {
    throttleSource_->setThrottleLevel(0.0);
    for (int i = 0; i < 6; ++i) {
        EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.0);
    }
}

