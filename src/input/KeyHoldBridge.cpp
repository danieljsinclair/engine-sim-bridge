#include "input/KeyHoldBridge.h"

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
    for (int key : keys) {
        if (isKeyActive(key)) return true;
    }
    return false;
}

} // namespace input
