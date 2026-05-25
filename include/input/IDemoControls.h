#ifndef IDEMO_CONTROLS_H
#define IDEMO_CONTROLS_H

namespace input {

// Control surface for the demo input provider. Allows external consumers
// (CLI, iOS app, OBD driver) to control the twin without knowing about
// keyboard internals. Obtained by casting IInputProvider to IDemoControls.
class IDemoControls {
public:
    virtual ~IDemoControls() = default;

    virtual void setThrottle(double level) = 0;
    virtual void shiftUp() = 0;
    virtual void shiftDown() = 0;
    virtual int getGearSelectorState() const = 0;
    virtual void setIgnition(bool on) = 0;
    virtual void toggleIgnition() = 0;
    virtual bool isIgnitionOn() const = 0;
    virtual void requestExit() = 0;
};

} // namespace input
#endif