#include "input/DemoThrottleSource.h"

namespace input {

DemoThrottleSource::DemoThrottleSource(int holdFrames)
    : holdFrames_(holdFrames) {
}

void DemoThrottleSource::setThrottleLevel(double level) {
    lastThrottle_ = level;
    framesSinceSet_ = 0;
}

void DemoThrottleSource::requestExit() const {
    // Exit is now handled by session->stop() in the simulation loop
    // This method remains for API compatibility but is a no-op
}

double DemoThrottleSource::pollThrottle() {
    if (framesSinceSet_ < holdFrames_) {
        ++framesSinceSet_;
        return lastThrottle_;
    }
    lastThrottle_ = 0.0;
    return 0.0;
}

} // namespace input
