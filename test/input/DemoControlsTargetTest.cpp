// DemoControlsTargetTest.cpp - Contract tests for DemoControlsTarget
//
// DemoControlsTarget.cpp is a dead-stripped adapter (63 LOC, 0% coverage) that
// maps key action targets onto IDemoControls and builds EngineInput from an
// optional demo IInputProvider. These tests assert the OBSERVABLE contract:
//   - throttle clamps to [0,1] cumulatively
//   - setStarter() latches a one-shot starter press consumed by buildEngineInput
//   - buildEngineInput merges the demo provider's output with the latched starter
//   - key actions forward to the IDemoControls surface (recorded by a fake)
//
// No IDemoControls production type is suitable as a fake, so we implement a
// minimal recording stub of the interface. This is a test double, not production
// code — it exists only to make the forwarding contract observable.

#include "input/DemoControlsTarget.h"
#include "input/IDemoControls.h"
#include "io/IInputProvider.h"
#include "io/UpstreamSignal.h"

#include <gtest/gtest.h>

#include <string>

namespace {

// Recording fake for IDemoControls — captures which control actions were invoked
// and with what arguments. This is the OBSERVABLE seam for the forwarding tests.
class FakeDemoControls : public input::IDemoControls {
public:
    struct Calls {
        bool requestExit = false;
        int setThrottleCount = 0;
        double lastThrottle = 0.0;
        int shiftUpCount = 0;
        int shiftDownCount = 0;
        bool toggleIgnition = false;
        int setIgnitionCount = 0;
        bool lastIgnition = false;
        int setBrakeCount = 0;
        double lastBrake = 0.0;
    };

    explicit FakeDemoControls(Calls* calls) : calls_(calls) {}

    void setThrottle(double level) override {
        calls_->setThrottleCount++;
        calls_->lastThrottle = level;
    }
    void shiftUp() override { calls_->shiftUpCount++; }
    void shiftDown() override { calls_->shiftDownCount++; }
    int getGearSelectorState() const override { return 0; }
    void setIgnition(bool on) override {
        calls_->setIgnitionCount++;
        calls_->lastIgnition = on;
    }
    void toggleIgnition() override { calls_->toggleIgnition = true; }
    bool isIgnitionOn() const override { return false; }
    void requestExit() override { calls_->requestExit = true; }
    void setBrake(double level) override {
        calls_->setBrakeCount++;
        calls_->lastBrake = level;
    }

private:
    Calls* calls_;
};

// Fake demo provider returning a fixed EngineInput so we can observe the merge
// with the latched starter press.
class FakeDemoProvider : public input::IInputProvider {
public:
    bool Initialize() override { return true; }
    void Shutdown() override {}
    bool IsConnected() const override { return true; }
    std::string GetProviderName() const override { return "FakeDemoProvider"; }
    std::string GetLastError() const override { return ""; }

    input::EngineInput OnUpdateSimulation(double) override {
        input::EngineInput input;
        input.throttle = 0.42;
        input.gearAbsolute = 3;
        return input;
    }
    void provideFeedback(const EngineSimStats&) override {}
};

// ============================================================================
// Fixture
// ============================================================================

class DemoControlsTargetTest : public ::testing::Test {
protected:
    void SetUp() override {
        calls_ = std::make_unique<FakeDemoControls::Calls>();
        controls_ = std::make_unique<FakeDemoControls>(calls_.get());
        target_ = std::make_unique<input::DemoControlsTarget>(controls_.get());
    }

    std::unique_ptr<FakeDemoControls::Calls> calls_;
    std::unique_ptr<FakeDemoControls> controls_;
    std::unique_ptr<input::DemoControlsTarget> target_;
};

// ---------------------------------------------------------------------------
// Throttle clamping
// ---------------------------------------------------------------------------

// adjustThrottle accumulates then clamps at the 1.0 ceiling.
TEST_F(DemoControlsTargetTest, AdjustThrottleClampsAtUpperBound) {
    target_->adjustThrottle(0.6);
    target_->adjustThrottle(0.6);  // 1.2 -> clamp to 1.0
    EXPECT_DOUBLE_EQ(calls_->lastThrottle, 1.0);
    EXPECT_EQ(calls_->setThrottleCount, 2);
}

// adjustThrottle accumulates then clamps at the 0.0 floor.
TEST_F(DemoControlsTargetTest, AdjustThrottleClampsAtLowerBound) {
    target_->adjustThrottle(0.5);
    target_->adjustThrottle(-2.0);  // -1.5 -> clamp to 0.0
    EXPECT_DOUBLE_EQ(calls_->lastThrottle, 0.0);
}

// adjustThrottle within range accumulates without clamping.
TEST_F(DemoControlsTargetTest, AdjustThrottleAccumulatesInRange) {
    target_->adjustThrottle(0.25);
    target_->adjustThrottle(0.25);  // 0.5
    EXPECT_DOUBLE_EQ(calls_->lastThrottle, 0.5);
}

// setThrottle forwards the absolute level verbatim (clamping is the job of
// adjustThrottle, not setThrottle).
TEST_F(DemoControlsTargetTest, SetThrottleForwardsLevel) {
    target_->setThrottle(0.75);
    EXPECT_DOUBLE_EQ(calls_->lastThrottle, 0.75);
    target_->setThrottle(5.0);  // forwarded as-is, no clamp
    EXPECT_DOUBLE_EQ(calls_->lastThrottle, 5.0);
}

// ---------------------------------------------------------------------------
// Starter latching
// ---------------------------------------------------------------------------

// setStarter latches a press. NOTE: the latched starter is only OR'd into the
// built EngineInput (and cleared) when a demo provider is set — without one,
// buildEngineInput returns the default EngineInput and does not surface it.
// This test captures that no-provider contract.
TEST_F(DemoControlsTargetTest, StarterLatchNotSurfacedWithoutProvider) {
    target_->setStarter();

    input::EngineInput input = target_->buildEngineInput(0.016);
    EXPECT_FALSE(input.starterButton);  // default input, starter not surfaced
}

// buildEngineInput with no demo provider and no starter returns default input.
TEST_F(DemoControlsTargetTest, BuildWithoutProviderReturnsDefault) {
    input::EngineInput input = target_->buildEngineInput(0.016);
    EXPECT_FALSE(input.starterButton);
    EXPECT_EQ(input.throttle, 0.0);
    EXPECT_EQ(input.gearAbsolute, -1);
}

// ---------------------------------------------------------------------------
// buildEngineInput merges demo provider output with latched starter
// ---------------------------------------------------------------------------

// With a demo provider set, buildEngineInput returns the provider's output with
// the latched starter OR'd in.
TEST_F(DemoControlsTargetTest, BuildWithProviderMergesStarter) {
    auto provider = std::make_unique<FakeDemoProvider>();
    target_->setDemoProvider(provider.get());
    target_->setStarter();

    input::EngineInput input = target_->buildEngineInput(0.016);
    EXPECT_TRUE(input.starterButton);
    EXPECT_DOUBLE_EQ(input.throttle, 0.42);
    EXPECT_EQ(input.gearAbsolute, 3);
}

// The merged starter is cleared after the first merge even with a provider set.
TEST_F(DemoControlsTargetTest, BuildWithProviderClearsStarterAfterMerge) {
    auto provider = std::make_unique<FakeDemoProvider>();
    target_->setDemoProvider(provider.get());
    target_->setStarter();

    target_->buildEngineInput(0.016);
    input::EngineInput second = target_->buildEngineInput(0.016);
    EXPECT_FALSE(second.starterButton);
}

// ---------------------------------------------------------------------------
// Key actions forward to IDemoControls
// ---------------------------------------------------------------------------

TEST_F(DemoControlsTargetTest, QuitForwardsToRequestExit) {
    target_->quit();
    EXPECT_TRUE(calls_->requestExit);
}

TEST_F(DemoControlsTargetTest, ShiftUpForwards) {
    target_->shiftUp();
    EXPECT_EQ(calls_->shiftUpCount, 1);
}

TEST_F(DemoControlsTargetTest, ShiftDownForwards) {
    target_->shiftDown();
    EXPECT_EQ(calls_->shiftDownCount, 1);
}

TEST_F(DemoControlsTargetTest, ToggleIgnitionForwards) {
    target_->toggleIgnition();
    EXPECT_TRUE(calls_->toggleIgnition);
}

TEST_F(DemoControlsTargetTest, SetBrakeForwards) {
    target_->setBrake(0.3);
    EXPECT_EQ(calls_->setBrakeCount, 1);
    EXPECT_DOUBLE_EQ(calls_->lastBrake, 0.3);
}

}  // namespace
