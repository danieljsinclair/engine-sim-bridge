// DemoChainIntegrationTest.cpp - End-to-end unified chain wiring test
// Proves: MockKeyboard -> KeyboardInputProvider -> EngineInputTarget (+ DemoInputProvider enhancer)
// Verifies key actions propagate through the full unified chain to EngineInput.

#include "input/KeyboardInputProvider.h"
#include "input/DemoInputProvider.h"
#include "input/EngineInputTarget.h"
#include "input/DemoThrottleSource.h"
#include "input/GearSelectorInput.h"
#include "input/IgnitionInput.h"
#include "input/IDemoSpeedEnhancer.h"
#include "twin/IceVehicleProfile.h"
#include "mocks/MockKeyboardInput.h"

#include <gtest/gtest.h>
#include <memory>

using namespace input;
using namespace twin;

class DemoChainIntegrationTest : public ::testing::Test {
protected:
    IceVehicleProfile profile_{IceVehicleProfile::zf8hp45()};
    MockKeyboardInput* rawKeyboard_ = nullptr;
    std::unique_ptr<KeyboardInputProvider> provider_;
    std::unique_ptr<EngineInputTarget> target_;
    std::unique_ptr<DemoInputProvider> demoProvider_;

    void SetUp() override {
        auto keyboard = std::make_unique<MockKeyboardInput>();
        rawKeyboard_ = keyboard.get();

        auto target = std::make_unique<EngineInputTarget>();

        auto throttle = std::make_unique<DemoThrottleSource>();
        auto demoProvider = std::make_unique<DemoInputProvider>(
            std::move(throttle),
            std::make_unique<GearSelectorInput>(),
            std::make_unique<IgnitionInput>(),
            profile_
        );
        ASSERT_TRUE(demoProvider->Initialize());

        // Wire demo provider as speed enhancer (same as CLIMain --connect-demo)
        target->setSpeedEnhancer(demoProvider.get());

        auto provider = std::make_unique<KeyboardInputProvider>(
            std::move(keyboard), target.get());
        ASSERT_TRUE(provider->Initialize());

        target_ = std::move(target);
        demoProvider_ = std::move(demoProvider);
        provider_ = std::move(provider);
    }

    EngineInput pressAndTick(int key, double dt = 16.0) {
        rawKeyboard_->enqueue(key);
        return provider_->OnUpdateSimulation(dt);
    }

    EngineInput tick(double dt = 16.0) {
        return provider_->OnUpdateSimulation(dt);
    }
};

// PROVE: brake key propagates through chain to brakeLevel
TEST_F(DemoChainIntegrationTest, BrakeKey_PropagatesThroughChain) {
    EngineInput input = pressAndTick('b');
    EXPECT_DOUBLE_EQ(input.brakeLevel, 1.0);
}

// PROVE: brake clears after key release + timeout
TEST_F(DemoChainIntegrationTest, BrakeReleased_ClearsAfterTimeout) {
    pressAndTick('b');

    double elapsedMs = 0.0;
    while (elapsedMs < 300.0) {
        tick();
        elapsedMs += 16.0;
    }

    EngineInput input = tick();
    EXPECT_DOUBLE_EQ(input.brakeLevel, 0.0);
}

// PROVE: keyboard throttle is PRESERVED through the enhancer chain
// (the enhancer only adds gear/clutch, never overwrites throttle)
TEST_F(DemoChainIntegrationTest, Throttle_PreservedThroughEnhancer) {
    // Get baseline throttle (whatever the default is)
    EngineInput baseline = tick();
    double baselineThrottle = baseline.throttle;

    // Press 'w' which should add 0.05 to throttle
    EngineInput input = pressAndTick('w');
    EXPECT_DOUBLE_EQ(input.throttle, baselineThrottle + 0.05);
}

// PROVE: ignition toggle works through the unified chain
TEST_F(DemoChainIntegrationTest, IgnitionToggle_PropagatesThroughChain) {
    EngineInput input1 = tick();
    EXPECT_TRUE(input1.ignition);

    EngineInput input2 = pressAndTick('i');
    EXPECT_FALSE(input2.ignition);
}

// PROVE: gear shift through keyboard produces gearDelta in output
TEST_F(DemoChainIntegrationTest, ShiftUp_ProducesGearDelta) {
    EngineInput input = pressAndTick(']');
    // The enhancer may modify gearAbsolute, but gearDelta should be non-zero
    EXPECT_EQ(input.gearDelta, 1);
}

// PROVE: no keys produces stable default EngineInput
TEST_F(DemoChainIntegrationTest, NoKeys_ProducesStableDefault) {
    EngineInput input1 = tick();
    EngineInput input2 = tick();

    // All values should be stable across multiple ticks
    EXPECT_DOUBLE_EQ(input1.brakeLevel, 0.0);
    EXPECT_FALSE(input1.starterButton);
    EXPECT_EQ(input1.gearDelta, 0);

    // Throttle should be stable (whatever the default is)
    EXPECT_DOUBLE_EQ(input1.throttle, input2.throttle);
}
