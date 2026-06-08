#include <gtest/gtest.h>
#include <input/IgnitionInput.h>

using namespace input;

class IgnitionInputTest : public ::testing::Test {
protected:
    IgnitionInput ignition_;
};

TEST_F(IgnitionInputTest, Default_IsOn) {
    EXPECT_TRUE(ignition_.isOn());
}

TEST_F(IgnitionInputTest, SetOff_MakesIsOnFalse) {
    ignition_.setOn(false);
    EXPECT_FALSE(ignition_.isOn());
}

TEST_F(IgnitionInputTest, SetOn_MakesIsOnTrue) {
    ignition_.setOn(false);
    ignition_.setOn(true);
    EXPECT_TRUE(ignition_.isOn());
}

TEST_F(IgnitionInputTest, Toggle_FlipsState) {
    bool initial = ignition_.isOn();
    ignition_.toggle();
    bool afterToggle = ignition_.isOn();
    EXPECT_NE(initial, afterToggle);
}

TEST_F(IgnitionInputTest, MultipleToggles_AlternatesState) {
    bool state1 = ignition_.isOn();
    ignition_.toggle();
    bool state2 = ignition_.isOn();
    ignition_.toggle();
    bool state3 = ignition_.isOn();
    ignition_.toggle();
    bool state4 = ignition_.isOn();

    EXPECT_EQ(state1, state3);
    EXPECT_EQ(state2, state4);
    EXPECT_NE(state1, state2);
}

TEST_F(IgnitionInputTest, SetOnThenToggle_GoesToOff) {
    ignition_.setOn(true);
    EXPECT_TRUE(ignition_.isOn());
    ignition_.toggle();
    EXPECT_FALSE(ignition_.isOn());
}

TEST_F(IgnitionInputTest, SetOffThenToggle_GoesToOn) {
    ignition_.setOn(false);
    EXPECT_FALSE(ignition_.isOn());
    ignition_.toggle();
    EXPECT_TRUE(ignition_.isOn());
}

TEST_F(IgnitionInputTest, ConsecutiveSetSameState_NoChange) {
    ignition_.setOn(true);
    EXPECT_TRUE(ignition_.isOn());
    ignition_.setOn(true);
    EXPECT_TRUE(ignition_.isOn());

    ignition_.setOn(false);
    EXPECT_FALSE(ignition_.isOn());
    ignition_.setOn(false);
    EXPECT_FALSE(ignition_.isOn());
}