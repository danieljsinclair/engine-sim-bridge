#include <gtest/gtest.h>
#include <input/KeyHoldBridge.h>
#include <input/DemoInputProvider.h>
#include <input/DemoThrottleSource.h>
#include <input/GearSelectorInput.h>
#include <input/IgnitionInput.h>
#include <input/IDemoControls.h>
#include <twin/IceVehicleProfile.h>
#include <queue>

// Proves the full chain: KeyHoldBridge → IDemoControls::setBrake → DemoInputProvider → EngineInput.brakeLevel
// This reproduces exactly what DemoKeyboardInputProvider does, without the real KeyboardInput dependency.

using namespace input;
using namespace twin;

namespace {
class MockKeyReader {
public:
    void enqueue(int key) { keys_.push(key); }
    int read() {
        if (keys_.empty()) return -1;
        int k = keys_.front();
        keys_.pop();
        return k;
    }
private:
    std::queue<int> keys_;
};
}

class BrakeChainTest : public ::testing::Test {
protected:
    IceVehicleProfile profile_{IceVehicleProfile::zf8hp45()};
    KeyHoldBridge keyState_;
    MockKeyReader reader_;

    std::unique_ptr<DemoThrottleSource> throttle_{std::make_unique<DemoThrottleSource>()};
    std::unique_ptr<DemoInputProvider> provider_;

    void SetUp() override {
        provider_ = std::make_unique<DemoInputProvider>(
            std::move(throttle_),
            std::make_unique<GearSelectorInput>(),
            std::make_unique<IgnitionInput>(),
            profile_
        );
        ASSERT_TRUE(provider_->Initialize());
    }

    void drainAndDispatch() {
        keyState_.drainInput([this]() { return reader_.read(); }, 16.0);

        // Simulate what DemoKeyboardInputProvider::dispatchKeys() does for brake
        if (keyState_.isKeyDown('b') || keyState_.isKeyDown('B')) {
            provider_->setBrake(1.0);
        } else {
            provider_->setBrake(0.0);
        }
    }

    EngineInput tick() {
        drainAndDispatch();
        return provider_->OnUpdateSimulation(0.016);
    }
};

// PROVE: single 'b' key event → brakeLevel = 1.0 for that frame
TEST_F(BrakeChainTest, SingleKeyPress_ShowsBrake) {
    reader_.enqueue('b');
    EngineInput input = tick();
    EXPECT_DOUBLE_EQ(input.brakeLevel, 1.0)
        << "Single 'b' press should set brakeLevel to 1.0";
}

// PROVE: held 'b' (OS key repeat) → brakeLevel stays 1.0
TEST_F(BrakeChainTest, KeyHeld_BrakeStaysOn) {
    for (int i = 0; i < 30; ++i) {
        reader_.enqueue('b');  // OS key repeat
        EngineInput input = tick();
        EXPECT_DOUBLE_EQ(input.brakeLevel, 1.0)
            << "brakeLevel should be 1.0 on frame " << (i + 1) << " while key held";
    }
}

// PROVE: after key released (no more events), brake stays for REPEAT_TIMEOUT_MS then goes 0
TEST_F(BrakeChainTest, KeyReleased_BrakeStaysDuringTimeout) {
    // Hold for a few frames (enters repeat mode after first repeat)
    for (int i = 0; i < 5; ++i) {
        reader_.enqueue('b');
        tick();
    }

    // Release — no more 'b' events
    // After 5 frames, we're in repeat mode, so use REPEAT_TIMEOUT_MS
    double elapsedMs = 0.0;
    while (elapsedMs < KeyHoldBridge::REPEAT_TIMEOUT_MS) {
        EngineInput input = tick();
        elapsedMs += 16.0;  // Simulate 60Hz frame time
        if (elapsedMs < KeyHoldBridge::REPEAT_TIMEOUT_MS) {
            EXPECT_DOUBLE_EQ(input.brakeLevel, 1.0)
                << "brakeLevel should be 1.0 during timeout at " << elapsedMs << "ms";
        }
    }

    // On the frame that exceeds REPEAT_TIMEOUT_MS, brake should be 0
    EngineInput input = tick();
    EXPECT_DOUBLE_EQ(input.brakeLevel, 0.0)
        << "brakeLevel should be 0.0 after timeout expires";
}

// PROVE: default state is 0.0
TEST_F(BrakeChainTest, NoKey_BrakeIsZero) {
    EngineInput input = tick();
    EXPECT_DOUBLE_EQ(input.brakeLevel, 0.0);
}

// PROVE: brake and throttle are independent
TEST_F(BrakeChainTest, BrakeAndThrottle_Independent) {
    reader_.enqueue('b');
    reader_.enqueue('5');  // throttle to 0.5
    EngineInput input = tick();
    EXPECT_DOUBLE_EQ(input.brakeLevel, 1.0);
    // Throttle may vary through twin, but brake should always be 1.0
}

// PROVE: rapid press/release cycles
TEST_F(BrakeChainTest, RapidPressRelease) {
    // Press (initial mode, no repeat yet)
    reader_.enqueue('b');
    EngineInput input = tick();
    EXPECT_DOUBLE_EQ(input.brakeLevel, 1.0);

    // Release (timeout in initial mode)
    double elapsedMs = 0.0;
    while (elapsedMs <= KeyHoldBridge::INITIAL_TIMEOUT_MS + 16.0) {
        tick();
        elapsedMs += 16.0;
    }
    input = tick();
    EXPECT_DOUBLE_EQ(input.brakeLevel, 0.0);

    // Press again immediately
    reader_.enqueue('b');
    input = tick();
    EXPECT_DOUBLE_EQ(input.brakeLevel, 1.0);
}
