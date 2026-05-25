#include <gtest/gtest.h>
#include <input/GearSelectorInput.h>

using namespace input;

class GearSelectorInputTest : public ::testing::Test {
protected:
    GearSelectorInput gear_;

    void SetUp() override {
        // Default state is NEUTRAL
    }
};

TEST_F(GearSelectorInputTest, DefaultState_IsNeutral) {
    int state = gear_.getState();
    EXPECT_EQ(state, static_cast<int>(bridge::GearSelector::NEUTRAL));
}

TEST_F(GearSelectorInputTest, ShiftUp_NeutralToDrive) {
    int initial = gear_.getState();
    gear_.shiftUp();
    int after = gear_.getState();
    EXPECT_NE(initial, after);
}

TEST_F(GearSelectorInputTest, ShiftUp_ClampsAtDrive) {
    // Shift up multiple times to reach DRIVE
    for (int i = 0; i < 10; ++i) {
        gear_.shiftUp();
    }
    int driveState = gear_.getState();
    EXPECT_EQ(driveState, static_cast<int>(bridge::GearSelector::DRIVE));

    // Try to shift up again
    gear_.shiftUp();
    int stillDrive = gear_.getState();
    EXPECT_EQ(stillDrive, static_cast<int>(bridge::GearSelector::DRIVE));
}

TEST_F(GearSelectorInputTest, ShiftDown_ClampsAtPark) {
    // Shift down multiple times to reach PARK
    for (int i = 0; i < 10; ++i) {
        gear_.shiftDown();
    }
    int parkState = gear_.getState();
    EXPECT_EQ(parkState, static_cast<int>(bridge::GearSelector::PARK));

    // Try to shift down again
    gear_.shiftDown();
    int stillPark = gear_.getState();
    EXPECT_EQ(stillPark, static_cast<int>(bridge::GearSelector::PARK));
}

TEST_F(GearSelectorInputTest, ShiftUpThenShiftDown_Reverses) {
    gear_.shiftUp();
    int afterUp = gear_.getState();
    gear_.shiftDown();
    int afterDown = gear_.getState();
    EXPECT_NE(afterUp, afterDown);
}

TEST_F(GearSelectorInputTest, FullPRNDLCycle) {
    // Start at NEUTRAL, go to PARK (two shift downs: N→R→P)
    gear_.shiftDown();
    EXPECT_EQ(gear_.getState(), static_cast<int>(bridge::GearSelector::REVERSE));
    gear_.shiftDown();
    EXPECT_EQ(gear_.getState(), static_cast<int>(bridge::GearSelector::PARK));

    // P → R → N → D
    gear_.shiftUp();
    EXPECT_EQ(gear_.getState(), static_cast<int>(bridge::GearSelector::REVERSE));

    gear_.shiftUp();
    EXPECT_EQ(gear_.getState(), static_cast<int>(bridge::GearSelector::NEUTRAL));

    gear_.shiftUp();
    EXPECT_EQ(gear_.getState(), static_cast<int>(bridge::GearSelector::DRIVE));
}

TEST_F(GearSelectorInputTest, MultipleShiftUps_PToD) {
    // Go to PARK
    while (gear_.getState() != static_cast<int>(bridge::GearSelector::PARK)) {
        gear_.shiftDown();
    }

    // P → R → N → D
    gear_.shiftUp();
    EXPECT_EQ(gear_.getState(), static_cast<int>(bridge::GearSelector::REVERSE));

    gear_.shiftUp();
    EXPECT_EQ(gear_.getState(), static_cast<int>(bridge::GearSelector::NEUTRAL));

    gear_.shiftUp();
    EXPECT_EQ(gear_.getState(), static_cast<int>(bridge::GearSelector::DRIVE));
}

TEST_F(GearSelectorInputTest, GetState_ReturnsConsistentValue) {
    int state1 = gear_.getState();
    int state2 = gear_.getState();
    EXPECT_EQ(state1, state2);
}