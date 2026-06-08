#include <gtest/gtest.h>
#include <input/BrakeInput.h>

using namespace input;

class BrakeInputTest : public ::testing::Test {
protected:
    BrakeInput brake_;
};

TEST_F(BrakeInputTest, DefaultLevelIsZero) {
    EXPECT_DOUBLE_EQ(brake_.pollLevel(), 0.0);
}

TEST_F(BrakeInputTest, SetLevel_ReturnsOnPoll) {
    brake_.setLevel(1.0);
    EXPECT_DOUBLE_EQ(brake_.pollLevel(), 1.0);
}

TEST_F(BrakeInputTest, SetLevel_StaysUntilChanged) {
    brake_.setLevel(1.0);
    EXPECT_DOUBLE_EQ(brake_.pollLevel(), 1.0);
    EXPECT_DOUBLE_EQ(brake_.pollLevel(), 1.0);
    EXPECT_DOUBLE_EQ(brake_.pollLevel(), 1.0);
}

TEST_F(BrakeInputTest, SetZero_ResetsImmediately) {
    brake_.setLevel(1.0);
    EXPECT_DOUBLE_EQ(brake_.pollLevel(), 1.0);
    brake_.setLevel(0.0);
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

TEST_F(BrakeInputTest, IntermediateLevel) {
    brake_.setLevel(0.5);
    EXPECT_DOUBLE_EQ(brake_.pollLevel(), 0.5);
}
