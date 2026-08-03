#include "input/DemoThrottleSource.h"

namespace input {

DemoThrottleSource::DemoThrottleSource(int holdFrames)
    : shouldContinue_(true), holdFrames_(holdFrames) {
}

void DemoThrottleSource::setThrottleLevel(double level) {
    lastThrottle_ = level;
    framesSinceSet_ = 0;
}

void DemoThrottleSource::requestExit() {
    shouldContinue_ = false;
}

double DemoThrottleSource::pollThrottle() {
    if (framesSinceSet_ < holdFrames_) {
        ++framesSinceSet_;
        return lastThrottle_;
    }
    lastThrottle_ = 0.0;
    return 0.0;
}

bool DemoThrottleSource::shouldContinue() const {
    return shouldContinue_;
}

} // namespace input