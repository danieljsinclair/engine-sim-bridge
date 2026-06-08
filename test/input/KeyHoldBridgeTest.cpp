#include <gtest/gtest.h>
#include <input/KeyHoldBridge.h>
#include <queue>

using namespace input;

namespace {

// Mock key reader — returns queued keys, then -1
class MockReader {
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

} // namespace

class KeyHoldBridgeTest : public ::testing::Test {
protected:
    KeyHoldBridge bridge_;
    MockReader reader_;

    void drain(double deltaTimeMs = 16.0) {
        bridge_.drainInput([this]() { return reader_.read(); }, deltaTimeMs);
    }
};

TEST_F(KeyHoldBridgeTest, NoInput_NothingDown) {
    drain();
    EXPECT_FALSE(bridge_.isKeyDown('b'));
    EXPECT_FALSE(bridge_.isKeyPressed('b'));
    EXPECT_FALSE(bridge_.isKeyReleased('b'));
}

TEST_F(KeyHoldBridgeTest, SingleKey_IsDown) {
    reader_.enqueue('b');
    drain();
    EXPECT_TRUE(bridge_.isKeyDown('b'));
    EXPECT_TRUE(bridge_.isKeyPressed('b'));
    EXPECT_FALSE(bridge_.isKeyReleased('b'));
}

TEST_F(KeyHoldBridgeTest, KeyHeld_StaysDownViaRepeat) {
    // Frame 1: key arrives
    reader_.enqueue('b');
    drain();
    EXPECT_TRUE(bridge_.isKeyDown('b'));
    EXPECT_TRUE(bridge_.isKeyPressed('b'));

    // Frame 2: OS key repeat sustains it
    reader_.enqueue('b');
    drain();
    EXPECT_TRUE(bridge_.isKeyDown('b'));
    EXPECT_FALSE(bridge_.isKeyPressed('b'));  // not an edge — already down
}

TEST_F(KeyHoldBridgeTest, KeyHeld_20Frames_AllDown) {
    // Simulate 20 frames of OS key repeat
    for (int i = 0; i < 20; ++i) {
        reader_.enqueue('b');
        drain();
        EXPECT_TRUE(bridge_.isKeyDown('b'))
            << "key should be down on frame " << (i + 1);
    }
}

TEST_F(KeyHoldBridgeTest, KeyReleased_AfterTimeoutFrames) {
    // Key goes down (initial mode, no repeat yet)
    reader_.enqueue('b');
    drain();
    EXPECT_TRUE(bridge_.isKeyDown('b'));

    // No more events for INITIAL_TIMEOUT_MS
    double elapsedMs = 0.0;
    while (elapsedMs < KeyHoldBridge::INITIAL_TIMEOUT_MS) {
        drain(16.0);  // Simulate 60Hz frame time
        elapsedMs += 16.0;
        if (elapsedMs < KeyHoldBridge::INITIAL_TIMEOUT_MS) {
            EXPECT_TRUE(bridge_.isKeyDown('b'))
                << "key should still be held at " << elapsedMs << "ms";
        }
    }

    // On the frame that exceeds INITIAL_TIMEOUT_MS, key is released
    EXPECT_FALSE(bridge_.isKeyDown('b'));
    EXPECT_TRUE(bridge_.isKeyReleased('b'));
}

TEST_F(KeyHoldBridgeTest, MultipleKeys_Independent) {
    reader_.enqueue('b');
    reader_.enqueue(']');
    drain();

    EXPECT_TRUE(bridge_.isKeyDown('b'));
    EXPECT_TRUE(bridge_.isKeyDown(']'));
    EXPECT_TRUE(bridge_.isKeyPressed('b'));
    EXPECT_TRUE(bridge_.isKeyPressed(']'));

    // Next frame: only 'b' repeats
    reader_.enqueue('b');
    drain();

    EXPECT_TRUE(bridge_.isKeyDown('b'));
    EXPECT_TRUE(bridge_.isKeyDown(']'));  // still held (within timeout)

    // After timeout without ']' event — ']' should expire during these frames
    bool sawReleased = false;
    double elapsedMs = 0.0;
    while (elapsedMs <= KeyHoldBridge::INITIAL_TIMEOUT_MS + 16.0) {  // Go slightly past timeout
        reader_.enqueue('b');  // 'b' still repeating
        drain(16.0);
        elapsedMs += 16.0;
        if (bridge_.isKeyReleased(']')) {
            sawReleased = true;
        }
    }

    EXPECT_TRUE(bridge_.isKeyDown('b'));
    EXPECT_FALSE(bridge_.isKeyDown(']'));
    EXPECT_TRUE(sawReleased) << "']' should have been released during timeout";
}

TEST_F(KeyHoldBridgeTest, BrakeScenario_HoldAndRelease) {
    // Simulate real brake usage: hold 'b' for 10 frames, then release

    // Hold phase
    for (int i = 0; i < 10; ++i) {
        reader_.enqueue('b');
        drain();
        EXPECT_TRUE(bridge_.isKeyDown('b'))
            << "brake should be down while held, frame " << (i + 1);
    }

    // Release phase — no more 'b' events
    // After 10 frames, we're in repeat mode, so use REPEAT_TIMEOUT_MS
    double elapsedMs = 0.0;
    while (elapsedMs < KeyHoldBridge::REPEAT_TIMEOUT_MS) {
        drain(16.0);
        elapsedMs += 16.0;
        if (elapsedMs < KeyHoldBridge::REPEAT_TIMEOUT_MS) {
            EXPECT_TRUE(bridge_.isKeyDown('b'))
                << "brake should be held during timeout at " << elapsedMs << "ms";
        }
    }

    // Expired on the frame that exceeds REPEAT_TIMEOUT_MS
    EXPECT_FALSE(bridge_.isKeyDown('b'));
    EXPECT_TRUE(bridge_.isKeyReleased('b'));
}

TEST_F(KeyHoldBridgeTest, PressedEdge_OneFrameOnly) {
    reader_.enqueue('b');
    drain();
    EXPECT_TRUE(bridge_.isKeyPressed('b'));  // frame 1: edge fires

    // Key still repeating
    reader_.enqueue('b');
    drain();
    EXPECT_FALSE(bridge_.isKeyPressed('b'));  // frame 2: edge does NOT fire again

    reader_.enqueue('b');
    drain();
    EXPECT_FALSE(bridge_.isKeyPressed('b'));  // frame 3: still no edge
}

TEST_F(KeyHoldBridgeTest, RePressed_AfterRelease) {
    // Press
    reader_.enqueue('b');
    drain();
    EXPECT_TRUE(bridge_.isKeyPressed('b'));

    // Release via timeout (initial mode)
    double elapsedMs = 0.0;
    while (elapsedMs < KeyHoldBridge::INITIAL_TIMEOUT_MS) {
        drain(16.0);
        elapsedMs += 16.0;
    }
    // On the frame that exceeds INITIAL_TIMEOUT_MS, key is released
    EXPECT_TRUE(bridge_.isKeyReleased('b'));

    // Press again
    reader_.enqueue('b');
    drain();
    EXPECT_TRUE(bridge_.isKeyDown('b'));
    EXPECT_TRUE(bridge_.isKeyPressed('b'));  // edge fires again after release
}

// NEW: Test dual-timeout behavior - initial press holds for 250ms
TEST_F(KeyHoldBridgeTest, DualTimeout_InitialPressHoldsFor250ms) {
    // Initial press (no repeat yet)
    reader_.enqueue('b');
    drain();
    EXPECT_TRUE(bridge_.isKeyDown('b'));

    // Wait 240ms - should still be down (initial mode)
    for (int i = 0; i < 15; ++i) {  // 15 * 16 = 240ms
        drain(16.0);
    }
    EXPECT_TRUE(bridge_.isKeyDown('b')) << "Key should be held at 240ms (initial mode)";

    // After 250ms, should expire
    drain(16.0);  // Total: 256ms
    EXPECT_FALSE(bridge_.isKeyDown('b'));
    EXPECT_TRUE(bridge_.isKeyReleased('b'));
}

// NEW: Test dual-timeout behavior - repeat mode expires in 50ms
TEST_F(KeyHoldBridgeTest, DualTimeout_RepeatModeExpiresIn50ms) {
    // Initial press + one repeat to enter repeat mode
    reader_.enqueue('b');
    drain();
    reader_.enqueue('b');  // First repeat - now in repeat mode
    drain();

    EXPECT_TRUE(bridge_.isKeyDown('b'));

    // Wait 48ms - should still be down (repeat mode)
    drain(16.0);  // 16ms
    drain(16.0);  // 32ms
    drain(16.0);  // 48ms
    EXPECT_TRUE(bridge_.isKeyDown('b')) << "Key should be held at 48ms (repeat mode)";

    // After 50ms, should expire
    drain(16.0);  // Total: 64ms
    EXPECT_FALSE(bridge_.isKeyDown('b'));
    EXPECT_TRUE(bridge_.isKeyReleased('b'));
}

// isKeyRepeating: fires once per frame when OS repeat event received (not initial press)

TEST_F(KeyHoldBridgeTest, Repeat_InitialPressNotARepeat) {
    reader_.enqueue('w');
    drain();
    EXPECT_TRUE(bridge_.isKeyPressed('w'));
    EXPECT_FALSE(bridge_.isKeyRepeating('w'))
        << "Initial press is NOT a repeat";
}

TEST_F(KeyHoldBridgeTest, Repeat_SecondEventIsARepeat) {
    reader_.enqueue('w');
    drain();
    EXPECT_FALSE(bridge_.isKeyRepeating('w'));

    // OS repeat event arrives
    reader_.enqueue('w');
    drain();
    EXPECT_TRUE(bridge_.isKeyRepeating('w'))
        << "Second event while key is already down is a repeat";
}

TEST_F(KeyHoldBridgeTest, Repeat_ContinuousHoldFiresEveryFrame) {
    reader_.enqueue('w');
    drain();
    EXPECT_FALSE(bridge_.isKeyRepeating('w'));  // initial press

    for (int i = 0; i < 5; ++i) {
        reader_.enqueue('w');
        drain();
        EXPECT_TRUE(bridge_.isKeyRepeating('w'))
            << "OS repeat on frame " << (i + 2) << " should be a repeat";
    }
}

TEST_F(KeyHoldBridgeTest, Repeat_NoEventNotARepeat) {
    reader_.enqueue('w');
    drain();

    // No event this frame — within timeout so still down, but NOT repeating
    drain();
    EXPECT_TRUE(bridge_.isKeyDown('w'));   // still held
    EXPECT_FALSE(bridge_.isKeyRepeating('w'))
        << "No event this frame means no repeat";
}

TEST_F(KeyHoldBridgeTest, Repeat_AfterReleaseAndRepress) {
    reader_.enqueue('w');
    drain();  // initial press

    reader_.enqueue('w');
    drain();  // repeat

    // Release via timeout
    double elapsedMs = 0.0;
    while (elapsedMs < KeyHoldBridge::REPEAT_TIMEOUT_MS) {
        drain(16.0);
        elapsedMs += 16.0;
    }
    EXPECT_FALSE(bridge_.isKeyDown('w'));

    // Press again — should be a new press, not a repeat
    reader_.enqueue('w');
    drain();
    EXPECT_TRUE(bridge_.isKeyPressed('w'));
    EXPECT_FALSE(bridge_.isKeyRepeating('w'))
        << "After release, new press is NOT a repeat";
}

// NEW: Test transition from initial to repeat mode
TEST_F(KeyHoldBridgeTest, DualTimeout_TransitionToRepeatMode) {
    // Initial press only
    reader_.enqueue('b');
    drain();

    // Wait 200ms - still in initial mode
    for (int i = 0; i < 12; ++i) {  // 12 * 16 = 192ms
        drain(16.0);
    }
    EXPECT_TRUE(bridge_.isKeyDown('b')) << "Key should be held at 192ms (initial mode)";

    // Now receive repeat event - should switch to repeat mode
    reader_.enqueue('b');
    drain();
    EXPECT_TRUE(bridge_.isKeyDown('b'));

    // Wait 48ms in repeat mode - should still be down
    for (int i = 0; i < 3; ++i) {  // 3 * 16 = 48ms
        drain(16.0);
    }
    EXPECT_TRUE(bridge_.isKeyDown('b')) << "Key should be held at 48ms in repeat mode";

    // After 50ms total in repeat mode, should expire
    drain(16.0);  // Total: 64ms
    EXPECT_FALSE(bridge_.isKeyDown('b'));
}
