#ifndef KEY_HOLD_BRIDGE_H
#define KEY_HOLD_BRIDGE_H

#include <unordered_map>
#include <unordered_set>
#include <initializer_list>

namespace input {

// Tracks keyboard state from OS key events. Translates OS key repeat into
// sustained "key down" state with automatic timeout.
//
// Provides:
// - isKeyDown(key): true while key is held (sustained by OS key repeat)
// - isKeyPressed(key): true for ONE frame when key first goes down
// - isKeyReleased(key): true for ONE frame when key goes up (timeout)
//
// Call drainInput() once per frame before querying state.
// Uses dual-timeout: initial grace for OS repeat delay, fast release for repeats.
class KeyHoldBridge {
public:
    static constexpr double INITIAL_TIMEOUT_MS = 250.0;  // Grace period for initial OS repeat delay
    static constexpr double REPEAT_TIMEOUT_MS = 50.0;    // Fast release once repeating starts

    // Drain all pending OS key events using the provided reader.
    // Reader returns key code, or -1 if no key available.
    // Called repeatedly until reader returns -1.
    // deltaTimeMs: time since last call, for timeout tracking
    template <typename Reader>
    void drainInput(Reader reader, double deltaTimeMs) {
        // Clear edge flags from last frame
        for (auto& [k, state] : keys_) {
            state.pressed = false;
            state.released = false;
            state.repeating = false;
        }

        // Drain all pending OS key events
        std::unordered_set<int> seen;
        int key;
        while ((key = reader()) >= 0) {
            auto& state = keys_[key];
            if (!state.down) {
                state.pressed = true;  // edge: up to down
                state.seenRepeat = false;  // Reset repeat flag on new press
            } else {
                state.repeating = true;  // OS repeat event while key is held
                state.seenRepeat = true;  // In repeat mode after first event
            }
            state.down = true;
            state.timeSinceEvent = 0.0;
            seen.insert(key);
        }

        // Age out held keys that received NO event this frame
        for (auto& [k, state] : keys_) {
            if (state.down && seen.find(k) == seen.end()) {
                state.timeSinceEvent += deltaTimeMs;
                // Use appropriate timeout based on whether we're in repeat mode
                double timeout = state.seenRepeat ? REPEAT_TIMEOUT_MS : INITIAL_TIMEOUT_MS;
                if (state.timeSinceEvent > timeout) {
                    state.down = false;
                    state.released = true;
                }
            }
        }
    }

    // True while key is held (sustained by OS key repeat)
    bool isKeyDown(int key) const;

    // True for one frame when key transitions from up → down
    bool isKeyPressed(int key) const;

    // True for one frame when an OS repeat event is received (not initial press)
    bool isKeyRepeating(int key) const;

    // True for one frame when key transitions from down → up (timeout)
    bool isKeyReleased(int key) const;

    // True on press OR OS repeat — the "hold to ramp" pattern
    bool isKeyActive(int key) const;

    // True if any of the given keys is active
    bool isKeyActiveAny(std::initializer_list<int> keys) const;

private:
    struct KeyState {
        double timeSinceEvent = 0.0;
        bool down = false;
        bool pressed = false;   // edge: up→down this frame
        bool released = false;  // edge: down→up this frame
        bool repeating = false; // edge: OS repeat event received this frame
        bool seenRepeat = false;  // true after receiving 2+ events (in repeat mode)
    };

    std::unordered_map<int, KeyState> keys_;
};

} // namespace input

#endif
