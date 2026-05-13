#ifndef KEYBOARD_DEMO_THROTTLE_SOURCE_H
#define KEYBOARD_DEMO_THROTTLE_SOURCE_H

#include "input/IThrottleSource.h"
#include "simulator/GearConventions.h"

namespace input {

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

    // Returns current gear selector position (always a valid GearSelector value)
    int getGearSelector() const;

    // Returns current ignition state
    bool getIgnition() const;

    // PRNDL order for cycling: P(-2) -> R(-1) -> N(0) -> D(99)
    static constexpr int PRNDL_ORDER[] = {-2, -1, 0, 99};
    static constexpr int PRNDL_COUNT = 4;

    // Frames to hold last throttle after key release (bridges OS key repeat gaps)
    static constexpr int THROTTLE_HOLD_FRAMES = 9;

private:
    IKeyboardInput& keyboard_;
    bool shouldContinue_;
    int gearSelector_;  // Current selector position, always valid
    bool ignitionOn_;   // Current ignition state
    double lastThrottle_ = 0.0;
    int framesSinceThrottleKey_ = 0;

    int prndlIndex() const;
    void shiftUp();
    void shiftDown();
};

} // namespace input

#endif
