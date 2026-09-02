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
    // Composable start/ignition primitives (owner spec 2026-09-02): the single
    // seam every frontend calls through. Concrete targets (EngineInputTarget)
    // override these to route the request into the shared VehicleStartController;
    // the default no-op lets targets that don't participate (DemoControlsTarget)
    // ignore them harmlessly.
    virtual void requestStarter() {}
    virtual void requestIgnition(bool /*on*/) {}
    virtual void requestCombinedStart() {}
    virtual void cyclePreset() {}
    virtual void adjustDynoTorque(double) {}
    virtual void releaseDynoTorque() {}
    virtual void setBrake(double) {}
    virtual void setThrottleMomentary(double) {}
    virtual void adjustSpeed(double) { (void)(double){0}; }  // km/h adjustment

    virtual EngineInput buildEngineInput(double dt) { (void)dt; return {}; }

    // Forward simulator feedback (RPM/speed/torque) to targets that need it
    // (EngineInputTarget -> speed enhancer -> twin/gearbox). Default no-op.
    virtual void provideFeedback(const EngineSimStats& /*stats*/) {}
};

} // namespace input

#endif // I_KEY_ACTION_TARGET_H
