#ifndef KEYBOARD_DEMO_THROTTLE_SOURCE_H
#define KEYBOARD_DEMO_THROTTLE_SOURCE_H

#include "input/IThrottleSource.h"

namespace input {

// Abstract interface for keyboard input
// Implemented by parent repo's KeyboardInputAdapter when using real keyboard,
// or by test mocks when unit testing
class IKeyboardInput {
public:
    virtual ~IKeyboardInput() = default;
    virtual int getKey() = 0;
};

class KeyboardDemoThrottleSource : public IThrottleSource {
public:
    explicit KeyboardDemoThrottleSource(IKeyboardInput& keyboard);
    ~KeyboardDemoThrottleSource() override = default;

    double pollThrottle() override;
    bool shouldContinue() const override;

private:
    IKeyboardInput& keyboard_;
    double lastThrottle_;
    bool shouldContinue_;
};

} // namespace input

#endif
