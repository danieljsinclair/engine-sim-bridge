// DemoControlsTarget.cpp - Adapts IDemoControls to IKeyActionTarget

#include "input/DemoControlsTarget.h"
#include "input/IDemoControls.h"
#include "io/IInputProvider.h"

#include <algorithm>

namespace input {

DemoControlsTarget::DemoControlsTarget(IDemoControls* controls)
    : controls_(controls) {
}

void DemoControlsTarget::setDemoProvider(IInputProvider* provider) {
    demoProvider_ = provider;
}

void DemoControlsTarget::quit() {
    if (controls_) controls_->requestExit();
}

void DemoControlsTarget::setThrottle(double level) {
    currentThrottle_ = level;
    if (controls_) controls_->setThrottle(level);
}

void DemoControlsTarget::adjustThrottle(double delta) {
    currentThrottle_ = std::clamp(currentThrottle_ + delta, 0.0, 1.0);
    if (controls_) controls_->setThrottle(currentThrottle_);
}

void DemoControlsTarget::shiftUp() {
    if (controls_) controls_->shiftUp();
}

void DemoControlsTarget::shiftDown() {
    if (controls_) controls_->shiftDown();
}

void DemoControlsTarget::toggleIgnition() {
    if (controls_) controls_->toggleIgnition();
}

void DemoControlsTarget::setStarter() {
    starterPressed_ = true;
}

void DemoControlsTarget::setBrake(double level) {
    if (controls_) controls_->setBrake(level);
}

EngineInput DemoControlsTarget::buildEngineInput(double dt) {
    if (demoProvider_) {
        auto input = demoProvider_->OnUpdateSimulation(dt);
        input.starterButton = starterPressed_;
        starterPressed_ = false;
        return input;
    }
    return {};
}

} // namespace input
