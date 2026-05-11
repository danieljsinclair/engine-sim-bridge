#include "input/KeyboardDemoThrottleSource.h"

namespace input {

KeyboardDemoThrottleSource::KeyboardDemoThrottleSource(IKeyboardInput& keyboard)
    : keyboard_(keyboard)
    , lastThrottle_(0.0)
    , shouldContinue_(true) {
}

double KeyboardDemoThrottleSource::pollThrottle() {
    int key = keyboard_.getKey();

    // Q or ESC → exit
    if (key == 'q' || key == 'Q' || key == 27) {
        shouldContinue_ = false;
        return 0.0;
    }

    // Space → throttle 0.0
    if (key == ' ') {
        lastThrottle_ = 0.0;
        return 0.0;
    }

    // Keys 0-9 → throttle 0.1-1.0
    if (key >= '0' && key <= '9') {
        lastThrottle_ = (key - '0') / 10.0;
        if (key == '0') {
            lastThrottle_ = 1.0;
        }
        return lastThrottle_;
    }

    // No key pressed → throttle snaps to 0.0
    if (key == -1) {
        lastThrottle_ = 0.0;
        return 0.0;
    }

    // Unknown key → keep last throttle
    return lastThrottle_;
}

bool KeyboardDemoThrottleSource::shouldContinue() const {
    return shouldContinue_;
}

} // namespace input
