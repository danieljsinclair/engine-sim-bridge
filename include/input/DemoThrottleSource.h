#ifndef DEMO_THROTTLE_SOURCE_H
#define DEMO_THROTTLE_SOURCE_H

#include "input/IThrottleSource.h"

namespace input {

// Throttle state with latching behaviour. Consumer sets throttle level,
// this source holds the last value until explicitly changed.
// Suitable for keyboard, OBD, or any control source.
class DemoThrottleSource : public IThrottleSource {
public:
    DemoThrottleSource();
    ~DemoThrottleSource() override = default;

    // IThrottleSource
    double pollThrottle() override;
    bool shouldContinue() const override;

    // State control — called by the consumer (CLI, OBD driver, iOS app)
    void setThrottleLevel(double level);   // 0.0–1.0, latches until next call
    void requestExit();                    // signals shouldContinue = false

private:
    bool shouldContinue_;
    double lastThrottle_ = 0.0;
};

} // namespace input

#endif