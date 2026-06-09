// IKeyActionTarget.h - Strategy interface for key action routing
// KeyboardInputProvider dispatches decoded key actions to this interface.
// Concrete targets: EngineInputTarget (standard), DemoControlsTarget (demo mode).

#ifndef I_KEY_ACTION_TARGET_H
#define I_KEY_ACTION_TARGET_H

#include "io/IInputProvider.h"

namespace input {

class IKeyActionTarget {
public:
    virtual ~IKeyActionTarget() = default;

    virtual void quit() {}
    virtual void setThrottle(double) {}
    virtual void adjustThrottle(double) {}
    virtual void shiftUp() {}
    virtual void shiftDown() {}
    virtual void toggleIgnition() {}
    virtual void setStarter() {}
    virtual void cyclePreset() {}
    virtual void adjustDynoTorque(double) {}
    virtual void releaseDynoTorque() {}
    virtual void setBrake(double) {}
    virtual void setThrottleMomentary(double) {}
    virtual void adjustSpeed(double) { (void)(double){0}; }  // km/h adjustment

    virtual EngineInput buildEngineInput(double dt) { (void)dt; return {}; }
};

} // namespace input

#endif // I_KEY_ACTION_TARGET_H
