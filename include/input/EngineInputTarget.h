// EngineInputTarget.h - Standard-mode key action target
// Owns throttle, gear, ignition, starter, brake, dyno state.
// Translates IKeyActionTarget calls into EngineInput state.

#ifndef ENGINE_INPUT_TARGET_H
#define ENGINE_INPUT_TARGET_H

#include "input/IKeyActionTarget.h"
#include "io/IInputProvider.h"

class ILogging;

namespace input {

// Forward declaration
class IDemoSpeedEnhancer;

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
    void adjustSpeed(double delta) override;

    // Set the speed enhancer (DemoInputProvider) for --connect-demo mode
    // The enhancer receives the base EngineInput and enhances it with speed data
    void setSpeedEnhancer(IDemoSpeedEnhancer* enhancer);

    // Build the EngineInput struct from current state.
    // Resets one-shot flags (gearDelta, starter, presetCycle).
    EngineInput buildInput();

    EngineInput buildEngineInput(double dt) override;

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
    // Negative sentinel = "no speed commanded". See EngineInput::roadSpeedKmh:
    // the loop's setSpeedTrackingTarget gate is >= 0.0, so a 0.0 default would
    // hold the engine at 0 RPM and stall it in gear. adjustSpeed() clamps to
    // [0, 300], so any user input immediately moves into the active range.
    double roadSpeedKmh_ = -1.0;  // Virtual ICE Twin: target road speed (km/h)
    ILogging* logger_;
    IDemoSpeedEnhancer* speedEnhancer_ = nullptr;  // Optional speed enhancer for demo mode
};

} // namespace input

#endif // ENGINE_INPUT_TARGET_H
