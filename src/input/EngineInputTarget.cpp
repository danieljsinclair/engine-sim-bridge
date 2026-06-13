// EngineInputTarget.cpp - Standard-mode key action target
// TDD RED PHASE: Stub implementation. Tests will drive the real implementation.

#include "input/EngineInputTarget.h"
#include "input/IDemoSpeedEnhancer.h"
#include "common/ILogging.h"
#include <algorithm>

namespace input {

EngineInputTarget::EngineInputTarget(ILogging* logger)
    : throttle_(0.0)
    , ignition_(true)
    , starterButton_(false)
    , gearDelta_(0)
    , gearSelector_(0)
    , dynoTorqueScale_(-1.0)
    , brakeLevel_(0.0)
    , presetCycle_(false)
    , quitRequested_(false)
    , throttleTouched_(false)
    , latchedThrottle_(0.00)
    , momentaryActive_(false)
    , logger_(logger) {
}

void EngineInputTarget::setSpeedEnhancer(IDemoSpeedEnhancer* enhancer) {
    speedEnhancer_ = enhancer;
}

void EngineInputTarget::quit() { quitRequested_ = true; }
void EngineInputTarget::setThrottle(double level) {
    throttle_ = level;
    latchedThrottle_ = level;
    throttleTouched_ = true;
    momentaryActive_ = false;
}
void EngineInputTarget::adjustThrottle(double delta) {
    throttle_ = std::clamp(throttle_ + delta, 0.0, 1.0);
    latchedThrottle_ = throttle_;
    throttleTouched_ = true;
    momentaryActive_ = false;
}
void EngineInputTarget::setThrottleMomentary(double level) {
    throttle_ = level;
    momentaryActive_ = true;
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
    if (logger_) logger_->info(LogMask::BRIDGE, __ilog_format("Dyno torque: %.2f", dynoTorqueScale_));
}
void EngineInputTarget::releaseDynoTorque() {
    dynoTorqueScale_ = 0.0;
    if (logger_) logger_->info(LogMask::BRIDGE, __ilog_format("Dyno torque released"));
}
void EngineInputTarget::setBrake(double level) { brakeLevel_ = level; }
void EngineInputTarget::adjustSpeed(double delta) {
    roadSpeedKmh_ = std::clamp(roadSpeedKmh_ + delta, 0.0, 300.0);
}

EngineInput EngineInputTarget::buildInput() {
    if (!throttleTouched_ && momentaryActive_) {
        // Smooth decay toward latched baseline (~15% of remaining distance per frame)
        throttle_ = latchedThrottle_ + (throttle_ - latchedThrottle_) * 0.85;
        if (std::abs(throttle_ - latchedThrottle_) < 0.005) {
            throttle_ = latchedThrottle_;
            momentaryActive_ = false;
        }
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
    input.gearAutoMode = false;
    input.roadSpeedKmh = roadSpeedKmh_;

    gearDelta_ = 0;
    starterButton_ = false;
    presetCycle_ = false;
    throttleTouched_ = false;

    return input;
}

EngineInput EngineInputTarget::buildEngineInput(double dt) {
    EngineInput input = buildInput();

    // If a speed enhancer is present, enhance the input with speed data
    // The enhancer receives the base keyboard state and adds computed speed/physics
    if (speedEnhancer_) {
        input = speedEnhancer_->enhanceInput(input, dt);
    }

    return input;
}

} // namespace input
