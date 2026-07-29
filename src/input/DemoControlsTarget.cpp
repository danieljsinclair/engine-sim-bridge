// DemoControlsTarget.cpp - Adapts IDemoControls to IKeyActionTarget

#include "input/DemoControlsTarget.h"
#include "input/IDemoControls.h"
#include "io/IInputProvider.h"

#include <algorithm>
#include <cassert>

namespace input {

DemoControlsTarget::DemoControlsTarget(IDemoControls* controls)
    : controls_(controls) {
    assert(controls_);  // controls_ is an immutable ctor param; must be non-null
}

void DemoControlsTarget::setDemoProvider(IInputProvider* provider) {
    demoProvider_ = provider;
}

void DemoControlsTarget::quit() {
    controls_->requestExit();
}

void DemoControlsTarget::setThrottle(double level) {
    currentThrottle_ = level;
    controls_->setThrottle(level);
}

void DemoControlsTarget::adjustThrottle(double delta) {
    currentThrottle_ = std::clamp(currentThrottle_ + delta, 0.0, 1.0);
    controls_->setThrottle(currentThrottle_);
}

void DemoControlsTarget::shiftUp() {
    controls_->shiftUp();
}

void DemoControlsTarget::shiftDown() {
    controls_->shiftDown();
}

void DemoControlsTarget::toggleIgnition() {
    controls_->toggleIgnition();
}

void DemoControlsTarget::setStarter() {
    starterPressed_ = true;
}

void DemoControlsTarget::setBrake(double level) {
    controls_->setBrake(level);
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
