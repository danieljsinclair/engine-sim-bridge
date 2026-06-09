// EngineInputTarget.cpp - Standard-mode key action target
// TDD RED PHASE: Stub implementation. Tests will drive the real implementation.

#include "input/EngineInputTarget.h"
#include <algorithm>

namespace input {

EngineInputTarget::EngineInputTarget()
    : throttle_(0.1)
    , ignition_(true)
    , starterButton_(false)
    , gearDelta_(0)
    , gearSelector_(0)
    , dynoTorqueScale_(-1.0)
    , brakeLevel_(0.0)
    , presetCycle_(false)
    , quitRequested_(false)
    , throttleTouched_(false) {
}

void EngineInputTarget::quit() { quitRequested_ = true; }
void EngineInputTarget::setThrottle(double level) { throttle_ = level; throttleTouched_ = true; }
void EngineInputTarget::adjustThrottle(double delta) {
    throttle_ = std::clamp(throttle_ + delta, 0.0, 1.0);
    throttleTouched_ = true;
}
void EngineInputTarget::shiftUp() { gearDelta_ = 1; gearSelector_++; }
void EngineInputTarget::shiftDown() { gearDelta_ = -1; gearSelector_--; }
void EngineInputTarget::toggleIgnition() { ignition_ = !ignition_; }
void EngineInputTarget::setStarter() { starterButton_ = true; }
void EngineInputTarget::cyclePreset() { presetCycle_ = true; }
void EngineInputTarget::adjustDynoTorque(double delta) {
    if (dynoTorqueScale_ < 0.0) dynoTorqueScale_ = 0.0;
    dynoTorqueScale_ = std::clamp(dynoTorqueScale_ + delta, 0.0, 1.0);
}
void EngineInputTarget::releaseDynoTorque() { dynoTorqueScale_ = 0.0; }
void EngineInputTarget::setBrake(double level) { brakeLevel_ = level; }

EngineInput EngineInputTarget::buildInput() {
    // Auto-decay: throttle returns toward 0 when no key was held this frame
    if (!throttleTouched_) {
        throttle_ *= 0.95;
        throttle_ = std::max(throttle_, 0.0);
    }

    EngineInput input;
    input.throttle = std::clamp(throttle_, 0.0, 1.0);
    input.ignition = ignition_;
    input.starterButton = starterButton_;
    input.gearDelta = gearDelta_;
    input.gearSelector = gearSelector_;
    input.dynoTorqueScale = dynoTorqueScale_;
    input.brakeLevel = brakeLevel_;
    input.presetCycle = presetCycle_;

    gearDelta_ = 0;
    starterButton_ = false;
    presetCycle_ = false;
    throttleTouched_ = false;

    return input;
}

} // namespace input
