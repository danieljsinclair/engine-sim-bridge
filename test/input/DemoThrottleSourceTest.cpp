#include <gtest/gtest.h>
#include <input/DemoThrottleSource.h>
#include <memory>

using namespace input;

class DemoThrottleSourceTest : public ::testing::Test {
protected:
    std::unique_ptr<DemoThrottleSource> throttleSource_;

    void SetUp() override {
        throttleSource_ = std::make_unique<DemoThrottleSource>();
    }
};

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

TEST_F(DemoThrottleSourceTest, ThrottleHoldsForDefaultFramesThenDecays) {
    throttleSource_->setThrottleLevel(0.5);

    // Should hold for DEFAULT_HOLD_FRAMES (8) polls
    for (int i = 0; i < DemoThrottleSource::DEFAULT_HOLD_FRAMES; ++i) {
        EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.5) << "Frame " << i;
    }

    // Should decay to 0 after hold frames expire
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.0);
}

TEST_F(DemoThrottleSourceTest, ThrottleChangesOnlyWhenSet_ResetsHoldCounter) {
    throttleSource_->setThrottleLevel(0.5);
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.5);

    // Poll a few times (but not enough to expire)
    for (int i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.5);
    }

    // Explicit change to new level resets hold counter
    throttleSource_->setThrottleLevel(0.8);
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.8);

    // Should hold for DEFAULT_HOLD_FRAMES again (7 more polls after the first)
    for (int i = 0; i < DemoThrottleSource::DEFAULT_HOLD_FRAMES - 1; ++i) {
        EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.8) << "Frame " << i;
    }

    // Then decay to 0
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.0);
}

TEST_F(DemoThrottleSourceTest, RequestExit_SetsShouldContinueFalse) {
    EXPECT_TRUE(throttleSource_->shouldContinue());
    throttleSource_->requestExit();
    EXPECT_FALSE(throttleSource_->shouldContinue());
}

TEST_F(DemoThrottleSourceTest, ShouldContinue_DefaultsToTrue) {
    EXPECT_TRUE(throttleSource_->shouldContinue());
}

TEST_F(DemoThrottleSourceTest, DefaultThrottleIsZero) {
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.0);
}

TEST_F(DemoThrottleSourceTest, CustomHoldFrames) {
    auto custom = std::make_unique<DemoThrottleSource>(3);  // Hold for 3 frames
    custom->setThrottleLevel(0.5);

    EXPECT_DOUBLE_EQ(custom->pollThrottle(), 0.5);
    EXPECT_DOUBLE_EQ(custom->pollThrottle(), 0.5);
    EXPECT_DOUBLE_EQ(custom->pollThrottle(), 0.5);

    // Decays on 4th poll
    EXPECT_DOUBLE_EQ(custom->pollThrottle(), 0.0);
}

TEST_F(DemoThrottleSourceTest, ZeroLevelStaysZero) {
    throttleSource_->setThrottleLevel(0.0);

    for (int i = 0; i < 20; ++i) {
        EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.0);
    }
}