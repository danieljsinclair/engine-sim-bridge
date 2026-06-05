#include <gtest/gtest.h>
#include <input/BrakeInput.h>

using namespace input;

class BrakeInputTest : public ::testing::Test {
protected:
    // Use holdFrames=4 for deterministic tests
    BrakeInput brake_{4};
};

TEST_F(BrakeInputTest, DefaultLevelIsZero) {
    EXPECT_DOUBLE_EQ(brake_.pollLevel(), 0.0);
}

TEST_F(BrakeInputTest, SetLevel_ReturnsOnNextPoll) {
    brake_.setLevel(1.0);
    EXPECT_DOUBLE_EQ(brake_.pollLevel(), 1.0);
}

TEST_F(BrakeInputTest, HoldAndDecay_HoldsForHoldFramesThenDecays) {
    brake_.setLevel(1.0);

    // Hold for holdFrames polls
    for (int i = 0; i < 4; ++i) {
        EXPECT_DOUBLE_EQ(brake_.pollLevel(), 1.0);
    }

    // Decayed to zero
    EXPECT_DOUBLE_EQ(brake_.pollLevel(), 0.0);
}

TEST_F(BrakeInputTest, KeyRepeatSustainsBrake) {
    // Simulate key repeat: setLevel called each frame
    for (int i = 0; i < 20; ++i) {
        brake_.setLevel(1.0);
        EXPECT_DOUBLE_EQ(brake_.pollLevel(), 1.0);
    }

    // Key released — decays after hold frames
    for (int i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(brake_.pollLevel(), 1.0);
    }
    EXPECT_DOUBLE_EQ(brake_.pollLevel(), 0.0);
}

TEST_F(BrakeInputTest, SetLevel_ResetsHoldCounter) {
    brake_.setLevel(0.8);

    // Poll 2 of 4 hold frames
    EXPECT_DOUBLE_EQ(brake_.pollLevel(), 0.8);
    EXPECT_DOUBLE_EQ(brake_.pollLevel(), 0.8);

    // Re-set before decay — counter resets
    brake_.setLevel(0.6);

    for (int i = 0; i < 4; ++i) {
        EXPECT_DOUBLE_EQ(brake_.pollLevel(), 0.6);
    }
    EXPECT_DOUBLE_EQ(brake_.pollLevel(), 0.0);
}

TEST_F(BrakeInputTest, LevelClampedToUpperBound) {
    brake_.setLevel(1.5);
    EXPECT_DOUBLE_EQ(brake_.pollLevel(), 1.0);
}

TEST_F(BrakeInputTest, LevelClampedToLowerBound) {
    brake_.setLevel(-0.5);
    EXPECT_DOUBLE_EQ(brake_.pollLevel(), 0.0);
}

TEST_F(BrakeInputTest, DefaultConstructorUsesDefaultHoldFrames) {
    BrakeInput defaultBrake;
    defaultBrake.setLevel(1.0);

    // Should hold for DEFAULT_HOLD_FRAMES (8) then decay
    for (int i = 0; i < 8; ++i) {
        EXPECT_DOUBLE_EQ(defaultBrake.pollLevel(), 1.0);
    }
    EXPECT_DOUBLE_EQ(defaultBrake.pollLevel(), 0.0);
}
