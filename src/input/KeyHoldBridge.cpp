#include "input/KeyHoldBridge.h"
#include <unordered_set>

namespace input {

void KeyHoldBridge::drainInput(std::function<int()> reader, double deltaTimeMs) {
    // Clear edge flags from last frame
    for (auto& [k, state] : keys_) {
        state.pressed = false;
        state.released = false;
    }

    // Drain all pending OS key events
    std::unordered_set<int> seen;
    int key;
    while ((key = reader()) >= 0) {
        auto& state = keys_[key];
        if (!state.down) {
            state.pressed = true;  // edge: up → down
            state.seenRepeat = false;  // Reset repeat flag on new press
        } else {
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

bool KeyHoldBridge::isKeyDown(int key) const {
    auto it = keys_.find(key);
    return it != keys_.end() && it->second.down;
}

bool KeyHoldBridge::isKeyPressed(int key) const {
    auto it = keys_.find(key);
    return it != keys_.end() && it->second.pressed;
}

bool KeyHoldBridge::isKeyReleased(int key) const {
    auto it = keys_.find(key);
    return it != keys_.end() && it->second.released;
}

} // namespace input
