#include "input/DemoThrottleSource.h"

namespace input {

DemoThrottleSource::DemoThrottleSource()
    : shouldContinue_(true) {
}

void DemoThrottleSource::setThrottleLevel(double level) {
    lastThrottle_ = level;
}

void DemoThrottleSource::requestExit() {
    shouldContinue_ = false;
}

double DemoThrottleSource::pollThrottle() {
    return lastThrottle_;
}

bool DemoThrottleSource::shouldContinue() const {
    return shouldContinue_;
}

} // namespace input