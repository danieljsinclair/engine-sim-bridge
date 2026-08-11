// EngineInputTarget.cpp - Standard-mode key action target
// TDD RED PHASE: Stub implementation. Tests will drive the real implementation.

#include "input/EngineInputTarget.h"
#include "input/IDemoSpeedEnhancer.h"
#include "input/IDemoControls.h"
#include "common/ILogging.h"
#include <algorithm>

namespace input {

// --start holds the starter for at most this many frames (~5s at 60Hz) so a
// dead engine can never pin the starter forever; the engine must catch by then.
constexpr int kAutoStartMaxFrames = 300;

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

void EngineInputTarget::setGearAutoMode(bool autoMode) {
    gearAutoMode_ = autoMode;
}

void EngineInputTarget::setDemoControls(IDemoControls* controls) {
    demoControls_ = controls;
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
void EngineInputTarget::shiftUp() {
    // Demo mode: advance the demo provider's PRNDL selector (P/R/N/D) so the
    // keyboard can drive it into DRIVE for the automatic gearbox. Still emit a
    // gearDelta so downstream consumers see the shift request.
    if (demoControls_) {
        demoControls_->shiftUp();
        gearDelta_ = 1;
        return;
    }
    // Pure auto mode (--auto, no demo): the box shifts itself; ]/[ disabled.
    if (gearAutoMode_) return;
    // Manual mode: ]/[ step the manual gear counter. Clamp to the valid
    // selector range (REVERSE=-1 .. EIGHTH=8) so the display can never render
    // a stray 'P' (PARK=-2) or '?' (>8).
    gearDelta_ = 1;
    gearSelector_ = std::min(gearSelector_ + 1, 8);
}
void EngineInputTarget::shiftDown() {
    if (demoControls_) {
        demoControls_->shiftDown();
        gearDelta_ = -1;
        return;
    }
    if (gearAutoMode_) return;
    gearDelta_ = -1;
    gearSelector_ = std::max(gearSelector_ - 1, -1);
}
void EngineInputTarget::toggleIgnition() { ignition_ = !ignition_; }
void EngineInputTarget::setStarter() { starterButton_ = true; }
void EngineInputTarget::setAutoStart() { autoStartHeld_ = true; }
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

    // --start: keep the starter switch depressed (S:1) until the engine reports
    // it is running (RPM caught) or a fail-safe frame cap is hit. The
    // CrankingController needs a sustained starterButton while the engine is in
    // the Stopped/Cranking phases; a single-frame pulse relies on the simulator
    // disengaging at exactly the right tick. Holding until Running is robust:
    // once running, the engine auto-disengages the starter itself.
    if (autoStartHeld_ && !engineRunning_ && autoStartFrames_ < kAutoStartMaxFrames) {
        starterButton_ = true;
        ++autoStartFrames_;
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
    input.gearAutoMode = gearAutoMode_;
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

void EngineInputTarget::provideFeedback(const EngineSimStats& stats) {
    // Once the engine has caught (RPM above the cranking-catch threshold), stop
    // holding the starter — the engine/simulator now owns the starter and
    // disengages it. Below the threshold the --start hold keeps cranking.
    if (autoStartHeld_ && stats.currentRPM >= 500.0) {
        engineRunning_ = true;
        autoStartHeld_ = false;
    }
    // Route simulator feedback to the speed enhancer (twin/gearbox) when present.
    if (speedEnhancer_) speedEnhancer_->provideFeedback(stats);
}

} // namespace input
