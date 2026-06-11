#include "input/KeyHoldBridge.h"
#include <algorithm>

namespace input {

bool KeyHoldBridge::isKeyDown(int key) const {
    auto it = keys_.find(key);
    return it != keys_.end() && it->second.down;
}

bool KeyHoldBridge::isKeyPressed(int key) const {
    auto it = keys_.find(key);
    return it != keys_.end() && it->second.pressed;
}

bool KeyHoldBridge::isKeyRepeating(int key) const {
    auto it = keys_.find(key);
    return it != keys_.end() && it->second.repeating;
}

bool KeyHoldBridge::isKeyReleased(int key) const {
    auto it = keys_.find(key);
    return it != keys_.end() && it->second.released;
}

bool KeyHoldBridge::isKeyActive(int key) const {
    auto it = keys_.find(key);
    return it != keys_.end() && (it->second.pressed || it->second.repeating);
}

bool KeyHoldBridge::isKeyActiveAny(std::initializer_list<int> keys) const {
    return std::any_of(keys.begin(), keys.end(),
        [this](int key) { return isKeyActive(key); });
}

} // namespace input
