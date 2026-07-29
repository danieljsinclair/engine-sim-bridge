// DemoControlsTarget.h - Adapts IDemoControls to IKeyActionTarget
// Used in demo mode: consolidated KeyboardInputProvider dispatches to this target,
// which translates actions into IDemoControls calls.

#ifndef DEMO_CONTROLS_TARGET_H
#define DEMO_CONTROLS_TARGET_H

#include "input/IKeyActionTarget.h"

namespace input {

class IDemoControls;
class IInputProvider;

class DemoControlsTarget : public IKeyActionTarget {
public:
    explicit DemoControlsTarget(IDemoControls* controls);

    void setDemoProvider(IInputProvider* provider);

    void quit() override;
    void setThrottle(double level) override;
    void adjustThrottle(double delta) override;
    void shiftUp() override;
    void shiftDown() override;
    void toggleIgnition() override;
    void setStarter() override;
    void setBrake(double level) override;

    EngineInput buildEngineInput(double dt) override;

private:
    IDemoControls* controls_;
    IInputProvider* demoProvider_ = nullptr;
    double currentThrottle_ = 0.0;
    bool starterPressed_ = false;
};

} // namespace input

#endif // DEMO_CONTROLS_TARGET_H
