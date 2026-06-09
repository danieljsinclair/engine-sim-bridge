// EngineInputTarget.h - Standard-mode key action target
// Owns throttle, gear, ignition, starter, brake, dyno state.
// Translates IKeyActionTarget calls into EngineInput state.

#ifndef ENGINE_INPUT_TARGET_H
#define ENGINE_INPUT_TARGET_H

#include "input/IKeyActionTarget.h"
#include "io/IInputProvider.h"

class ILogging;

namespace input {

class EngineInputTarget : public IKeyActionTarget {
public:
    explicit EngineInputTarget(ILogging* logger = nullptr);

    void quit() override;
    void setThrottle(double level) override;
    void adjustThrottle(double delta) override;
    void setThrottleMomentary(double level) override;
    void shiftUp() override;
    void shiftDown() override;
    void toggleIgnition() override;
    void setStarter() override;
    void cyclePreset() override;
    void adjustDynoTorque(double delta) override;
    void releaseDynoTorque() override;
    void setBrake(double level) override;

    // Build the EngineInput struct from current state.
    // Resets one-shot flags (gearDelta, starter, presetCycle).
    EngineInput buildInput();

    EngineInput buildEngineInput(double dt) override { (void)dt; return buildInput(); }

    bool quitRequested() const { return quitRequested_; }

private:
    double throttle_;
    bool ignition_;
    bool starterButton_;
    int gearDelta_;
    int gearSelector_;
    double dynoTorqueScale_;
    double brakeLevel_;
    bool presetCycle_;
    bool quitRequested_;
    bool throttleTouched_;  // true if set/adjusted this frame
    double latchedThrottle_;  // Baseline set by W/Z/R/Space
    bool momentaryActive_;   // True when a 0-9 key is being held
    ILogging* logger_;
};

} // namespace input

#endif // ENGINE_INPUT_TARGET_H
