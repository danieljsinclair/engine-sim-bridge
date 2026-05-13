#include "input/KeyboardDemoThrottleSource.h"

namespace input {

KeyboardDemoThrottleSource::KeyboardDemoThrottleSource(IKeyboardInput& keyboard)
    : keyboard_(keyboard)
    , shouldContinue_(true)
    , gearSelector_(0)   // NEUTRAL
    , ignitionOn_(true) {
}

int KeyboardDemoThrottleSource::prndlIndex() const {
    for (int i = 0; i < PRNDL_COUNT; ++i) {
        if (PRNDL_ORDER[i] == gearSelector_) return i;
    }
    return 2; // Default to N if somehow out of range
}

void KeyboardDemoThrottleSource::shiftUp() {
    int idx = prndlIndex();
    if (idx < PRNDL_COUNT - 1) {
        gearSelector_ = PRNDL_ORDER[idx + 1];
    }
}

void KeyboardDemoThrottleSource::shiftDown() {
    int idx = prndlIndex();
    if (idx > 0) {
        gearSelector_ = PRNDL_ORDER[idx - 1];
    }
}

double KeyboardDemoThrottleSource::pollThrottle() {
    int key = keyboard_.getKey();

    // Q or ESC → exit
    if (key == 'q' || key == 'Q' || key == 27) {
        shouldContinue_ = false;
        return 0.0;
    }

    // 'i' → toggle ignition
    if (key == 'i' || key == 'I') {
        ignitionOn_ = !ignitionOn_;
    }

    // Gear selector: direct selection (N/D/P/R)
    if (key == 'p' || key == 'P') {
        gearSelector_ = static_cast<int>(bridge::GearSelector::PARK);
    } else if (key == 'r' || key == 'R') {
        gearSelector_ = static_cast<int>(bridge::GearSelector::REVERSE);
    } else if (key == 'n' || key == 'N') {
        gearSelector_ = static_cast<int>(bridge::GearSelector::NEUTRAL);
    } else if (key == 'd' || key == 'D') {
        gearSelector_ = static_cast<int>(bridge::GearSelector::DRIVE);
    }

    // Gear selector: cycle through PRNDL (] toward D, [ toward P)
    if (key == ']') {
        shiftUp();
    } else if (key == '[') {
        shiftDown();
    }

    // Throttle: hold-to-activate, bridges OS key repeat gaps, snaps to 0 on release
    if (key >= '1' && key <= '9') {
        lastThrottle_ = (key - '0') / 10.0;
        framesSinceThrottleKey_ = 0;
        return lastThrottle_;
    }
    if (key == '0') {
        lastThrottle_ = 1.0;
        framesSinceThrottleKey_ = 0;
        return lastThrottle_;
    }

    // No throttle key this frame — hold last value for a few frames
    // to bridge OS key repeat gaps, then snap to 0.0
    ++framesSinceThrottleKey_;
    if (framesSinceThrottleKey_ <= THROTTLE_HOLD_FRAMES) {
        return lastThrottle_;
    }
    lastThrottle_ = 0.0;
    return 0.0;
}

bool KeyboardDemoThrottleSource::shouldContinue() const {
    return shouldContinue_;
}

int KeyboardDemoThrottleSource::getGearSelector() const {
    return gearSelector_;
}

bool KeyboardDemoThrottleSource::getIgnition() const {
    return ignitionOn_;
}

} // namespace input
