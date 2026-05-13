#include <gtest/gtest.h>
#include <twin/ManualTwin.h>
#include <simulator/GearConventions.h>

using namespace twin;

class ManualTwinTest : public ::testing::Test {
protected:
    ManualTwin twin_;

    TwinFeedback makeFeedback(double rpm, bool valid = true) {
        TwinFeedback fb;
        fb.engineRpm = rpm;
        fb.isValid = valid;
        return fb;
    }

    // Advance twin to CRANKING state via ignition + starter
    void advanceToCranking() {
        twin_.setIgnition(true);
        twin_.setStarterMotor(true);
        twin_.update(0.016, makeFeedback(0.0));
    }

    // Advance twin through CRANKING to RUNNING
    void advanceToRunning() {
        advanceToCranking();
        twin_.update(0.016, makeFeedback(450.0));
    }
};

TEST_F(ManualTwinTest, StartsInOffState) {
    EXPECT_EQ(twin_.getState(), TwinState::OFF);
}

TEST_F(ManualTwinTest, OffStaysOffWithoutIgnitionAndStarter) {
    auto output = twin_.update(0.016, makeFeedback(0.0));
    EXPECT_EQ(twin_.getState(), TwinState::OFF);
    EXPECT_FALSE(output.starterMotor);
    EXPECT_FALSE(output.ignition);
}

TEST_F(ManualTwinTest, OffToCrankingWithIgnitionAndStarter) {
    twin_.setIgnition(true);
    twin_.setStarterMotor(true);
    auto output = twin_.update(0.016, makeFeedback(0.0));
    EXPECT_EQ(twin_.getState(), TwinState::CRANKING);
    EXPECT_TRUE(output.starterMotor);
    EXPECT_TRUE(output.ignition);
    EXPECT_EQ(output.gear, static_cast<int>(bridge::BridgeGear::NEUTRAL));
    EXPECT_DOUBLE_EQ(output.clutchPressure, 0.0);
}

TEST_F(ManualTwinTest, CrankingThrottlePassthrough) {
    advanceToCranking();
    ASSERT_EQ(twin_.getState(), TwinState::CRANKING);

    twin_.setThrottle(0.3);
    auto output = twin_.update(0.016, makeFeedback(200.0));
    EXPECT_EQ(twin_.getState(), TwinState::CRANKING);
    EXPECT_DOUBLE_EQ(output.throttle, 0.3);
}

TEST_F(ManualTwinTest, CrankingToRunningWhenRpmExceedsThreshold) {
    advanceToCranking();
    ASSERT_EQ(twin_.getState(), TwinState::CRANKING);

    auto output = twin_.update(0.016, makeFeedback(450.0));
    EXPECT_EQ(twin_.getState(), TwinState::RUNNING);
    EXPECT_FALSE(output.starterMotor);
}

TEST_F(ManualTwinTest, CrankingStaysCrankingBelowThreshold) {
    advanceToCranking();
    ASSERT_EQ(twin_.getState(), TwinState::CRANKING);

    auto output = twin_.update(0.016, makeFeedback(200.0));
    EXPECT_EQ(twin_.getState(), TwinState::CRANKING);
    EXPECT_TRUE(output.starterMotor);
}

TEST_F(ManualTwinTest, CrankingBackToOffWhenIgnitionOff) {
    advanceToCranking();
    ASSERT_EQ(twin_.getState(), TwinState::CRANKING);

    twin_.setIgnition(false);
    twin_.update(0.016, makeFeedback(200.0));
    EXPECT_EQ(twin_.getState(), TwinState::OFF);
}

TEST_F(ManualTwinTest, RunningStaysRunningWithValidFeedback) {
    advanceToRunning();
    ASSERT_EQ(twin_.getState(), TwinState::RUNNING);

    twin_.setThrottle(0.5);
    auto output = twin_.update(0.016, makeFeedback(800.0));
    EXPECT_EQ(twin_.getState(), TwinState::RUNNING);
    EXPECT_DOUBLE_EQ(output.throttle, 0.5);
}

TEST_F(ManualTwinTest, RunningToOffWhenIgnitionTurnedOff) {
    advanceToRunning();
    ASSERT_EQ(twin_.getState(), TwinState::RUNNING);

    twin_.setIgnition(false);
    twin_.update(0.016, makeFeedback(800.0));
    EXPECT_EQ(twin_.getState(), TwinState::OFF);
}

TEST_F(ManualTwinTest, RunningToOffAfter5sInvalidFeedback) {
    advanceToRunning();
    ASSERT_EQ(twin_.getState(), TwinState::RUNNING);

    for (int i = 0; i < 315; ++i) {
        twin_.update(0.016, makeFeedback(0.0, false));
    }
    EXPECT_EQ(twin_.getState(), TwinState::OFF);
}

TEST_F(ManualTwinTest, ThrottlePassthroughInRunningState) {
    advanceToRunning();

    twin_.setThrottle(0.8);
    auto output = twin_.update(0.016, makeFeedback(800.0));
    EXPECT_DOUBLE_EQ(output.throttle, 0.8);
}

TEST_F(ManualTwinTest, ThrottleClampedToZeroToOne) {
    advanceToRunning();

    twin_.setThrottle(1.5);
    auto output = twin_.update(0.016, makeFeedback(800.0));
    EXPECT_DOUBLE_EQ(output.throttle, 1.0);

    twin_.setThrottle(-0.5);
    output = twin_.update(0.016, makeFeedback(800.0));
    EXPECT_DOUBLE_EQ(output.throttle, 0.0);
}

TEST_F(ManualTwinTest, GearUpFromNeutral) {
    advanceToRunning();

    twin_.requestGearUp();
    auto output = twin_.update(0.016, makeFeedback(800.0));
    EXPECT_EQ(output.gear, static_cast<int>(bridge::BridgeGear::FIRST));
}

TEST_F(ManualTwinTest, GearDownFromFirst) {
    advanceToRunning();

    twin_.requestGearUp();
    twin_.update(0.016, makeFeedback(800.0));
    ASSERT_EQ(twin_.getCurrentGear(), static_cast<int>(bridge::BridgeGear::FIRST));

    twin_.requestGearDown();
    auto output = twin_.update(0.016, makeFeedback(800.0));
    EXPECT_EQ(output.gear, static_cast<int>(bridge::BridgeGear::NEUTRAL));
}

TEST_F(ManualTwinTest, GearDownClampsAtNeutral) {
    advanceToRunning();

    twin_.requestGearDown();
    auto output = twin_.update(0.016, makeFeedback(800.0));
    EXPECT_EQ(output.gear, static_cast<int>(bridge::BridgeGear::NEUTRAL));
}

TEST_F(ManualTwinTest, GearUpClampsAtEighth) {
    advanceToRunning();

    for (int i = 0; i < 10; ++i) {
        twin_.requestGearUp();
        twin_.update(0.016, makeFeedback(800.0));
    }
    EXPECT_EQ(twin_.getCurrentGear(), static_cast<int>(bridge::BridgeGear::EIGHTH));
}
