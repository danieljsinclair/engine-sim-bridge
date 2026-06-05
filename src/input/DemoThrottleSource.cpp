#include "input/DemoThrottleSource.h"

namespace input {

DemoThrottleSource::DemoThrottleSource() = default;

void DemoThrottleSource::setThrottleLevel(double level) {
    lastThrottle_ = level;
}

void DemoThrottleSource::requestExit() {
    // Exit is now handled by session->stop() in the simulation loop
    // This method remains for API compatibility but is a no-op
}

double DemoThrottleSource::pollThrottle() {
    return lastThrottle_;
}

} // namespace input