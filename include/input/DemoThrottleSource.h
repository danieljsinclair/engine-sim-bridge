#ifndef DEMO_THROTTLE_SOURCE_H
#define DEMO_THROTTLE_SOURCE_H

#include "input/IThrottleSource.h"

namespace input {

// Throttle state with hold-and-decay behaviour. Consumer sets throttle level,
// this source holds the value for a configurable number of polls (bridging OS
// key repeat gaps), then decays to 0.0 when input stops.
class DemoThrottleSource : public IThrottleSource {
public:
    static constexpr int DEFAULT_HOLD_FRAMES = 8;  // ~130ms at 60Hz

    explicit DemoThrottleSource(int holdFrames = DEFAULT_HOLD_FRAMES);
    ~DemoThrottleSource() override = default;

    // IThrottleSource
    double pollThrottle() override;

    // State control — called by the consumer (CLI, OBD driver, iOS app)
    void setThrottleLevel(double level);   // 0.0–1.0, resets hold counter
    void requestExit() const;             // no-op in new lifecycle

private:
    double lastThrottle_ = 0.0;
    int holdFrames_;
    int framesSinceSet_ = 0;
};

} // namespace input

#endif