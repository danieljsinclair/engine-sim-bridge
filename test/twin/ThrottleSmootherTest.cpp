#include <gtest/gtest.h>
#include <twin/ThrottleSmoother.h>
#include <cmath>

using namespace twin;

class ThrottleSmootherTest : public ::testing::Test {
protected:
    void SetUp() override {
        smoother_ = std::make_unique<ThrottleSmoother>(50.0);
    }

    std::unique_ptr<ThrottleSmoother> smoother_;
};

TEST_F(ThrottleSmootherTest, Reaches63PercentWithinTau_AC09_1) {
    smoother_->reset(0.0);
    double result = smoother_->update(0.050, 1.0);
    EXPECT_NEAR(result, 0.632, 0.01) << "Filtered throttle should reach ~63.2% of step within TAU=50ms";
}

TEST_F(ThrottleSmootherTest, Reaches95PercentWithin3Tau_AC09_2) {
    smoother_->reset(0.0);
    smoother_->update(0.050, 1.0);
    smoother_->update(0.050, 1.0);
    smoother_->update(0.050, 1.0);
    double result = smoother_->getCurrentValue();
    EXPECT_NEAR(result, 0.95, 0.02) << "Filtered throttle should reach ~95% within 3×TAU=150ms";
}

TEST_F(ThrottleSmootherTest, ZeroSteadyStateErrorAfter200ms_AC09_3) {
    smoother_->reset(0.0);
    for (int i = 0; i < 4; ++i) {
        smoother_->update(0.050, 1.0);
    }
    double result = smoother_->getCurrentValue();
    EXPECT_NEAR(result, 1.0, 0.02) << "Steady-state error should be ~0 after 200ms (exponential asymptote)";
}

TEST_F(ThrottleSmootherTest, NoOvershoot_AC09_4) {
    smoother_->reset(0.0);
    for (int i = 0; i < 10; ++i) {
        smoother_->update(0.010, 1.0);
        double val = smoother_->getCurrentValue();
        EXPECT_LE(val, 1.0) << "Filtered throttle should never exceed raw throttle (no overshoot)";
    }
}

TEST_F(ThrottleSmootherTest, ExponentialResponseCharacteristic_AC09_5) {
    smoother_->reset(0.0);
    double prev = 0.0;
    double rates[5];
    for (int i = 0; i < 5; ++i) {
        smoother_->update(0.010, 1.0);
        double curr = smoother_->getCurrentValue();
        rates[i] = curr - prev;
        prev = curr;
    }

    for (int i = 1; i < 5; ++i) {
        EXPECT_LT(rates[i], rates[i - 1]) << "Rate should decrease monotonically (exponential decay, no oscillation)";
    }
}

TEST_F(ThrottleSmootherTest, HandlesNegativeTauGracefully) {
    ThrottleSmoother badSmoother(-1.0);
    badSmoother.reset(0.0);
    double result = badSmoother.update(0.01, 1.0);
    EXPECT_EQ(result, 1.0) << "Negative TAU should bypass filter and return raw value";
}

TEST_F(ThrottleSmootherTest, StepDownAlsoSmoothed) {
    smoother_->reset(1.0);
    smoother_->update(0.050, 0.0);
    double result = smoother_->getCurrentValue();
    EXPECT_GT(result, 0.3) << "Step down should also be smoothed (not instant)";
    EXPECT_LT(result, 1.0) << "Step down should start moving toward 0";
}

TEST_F(ThrottleSmootherTest, CustomTauAdjustsResponseSpeed) {
    ThrottleSmoother fastSmoother(10.0);
    fastSmoother.reset(0.0);
    fastSmoother.update(0.010, 1.0);
    double fastResult = fastSmoother.getCurrentValue();

    smoother_->reset(0.0);
    smoother_->update(0.010, 1.0);
    double slowResult = smoother_->getCurrentValue();

    EXPECT_GT(fastResult, slowResult) << "Smaller TAU should produce faster response";
}
