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

TEST_F(DemoThrottleSourceTest, ThrottleLatches_UntilExplicitlyChanged) {
    throttleSource_->setThrottleLevel(0.5);

    // Throttle should hold indefinitely (no snap-to-zero)
    for (int i = 0; i < 100; ++i) {
        EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.5);
    }
}

TEST_F(DemoThrottleSourceTest, ThrottleChangesOnlyWhenSet) {
    throttleSource_->setThrottleLevel(0.5);
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.5);

    // After many polls, still holds
    for (int i = 0; i < 50; ++i) {
        throttleSource_->pollThrottle();
    }
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.5);

    // Explicit change to new level
    throttleSource_->setThrottleLevel(0.8);
    EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.8);

    // Holds new level
    for (int i = 0; i < 50; ++i) {
        EXPECT_DOUBLE_EQ(throttleSource_->pollThrottle(), 0.8);
    }
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
