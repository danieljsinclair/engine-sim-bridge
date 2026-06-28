// KeyboardInputProvider.cpp - Consolidated keyboard input provider
// TDD RED PHASE: Stub implementation. Tests will drive the real implementation.

#include "input/KeyboardInputProvider.h"
#include "input/IKeyboardInput.h"
#include "session/ISimulatorSession.h"

namespace input {

KeyboardInputProvider::KeyboardInputProvider(
    std::unique_ptr<IKeyboardInput> keyboard,
    IKeyActionTarget* target)
    : keyboard_(std::move(keyboard))
    , target_(target) {
}

KeyboardInputProvider::~KeyboardInputProvider() = default;

bool KeyboardInputProvider::Initialize() { return true; }
// Keyboard input is polled per-frame in processKeys(); nothing to tear down.
void KeyboardInputProvider::Shutdown() {
    // No-op: keyboard state is transient; each OnUpdateSimulation() call
    // reads fresh input from the device, so there is nothing to tear down.
}
bool KeyboardInputProvider::IsConnected() const { return true; }

EngineInput KeyboardInputProvider::OnUpdateSimulation(double dt) {
    processKeys(dt);
    return target_->buildEngineInput(dt);
}

void KeyboardInputProvider::provideFeedback(const EngineSimStats& stats) {
    // Close the feedback loop: forward real RPM/speed/torque to the target,
    // which routes it to the demo enhancer -> twin -> gearbox. Without this the
    // autobox's rpmFeedback_ stays 0 and the redline safety never upshifts.
    if (target_) target_->provideFeedback(stats);
}

void KeyboardInputProvider::processKeys(double dt) {
    keyHold_.drainInput([this]() { return keyboard_->getKey(); }, dt * 1000.0);

    if (processQuitAndThrottle()) {
        return;
    }

    processMomentaryThrottle();
    processGearAndIgnition();
    processDynoAndBrake();
    processSpeedControl();
    processPresetKeys();
}

bool KeyboardInputProvider::processQuitAndThrottle() {
    // Quit: edge-triggered (pressed only) — terminates the frame
    if (keyHold_.isKeyPressed('q') || keyHold_.isKeyPressed('Q') || keyHold_.isKeyPressed(27)) {
        target_->quit();
        if (session_) session_->stop();
        return true;
    }

    // Throttle ramp up: active (pressed or repeating)
    if (keyHold_.isKeyActiveAny({'w', 'a', 'W', 65})) {
        target_->adjustThrottle(0.05);
    }
    // Throttle ramp down: active (pressed or repeating)
    if (keyHold_.isKeyActiveAny({'z', 'Z', 66})) {
        target_->adjustThrottle(-0.05);
    }

    // Space: zero throttle (edge-triggered)
    if (keyHold_.isKeyPressed(' ')) {
        target_->setThrottle(0.0);
    }
    // R: 20% throttle (edge-triggered)
    if (keyHold_.isKeyPressed('r') || keyHold_.isKeyPressed('R')) {
        target_->setThrottle(0.2);
    }

    return false;
}

void KeyboardInputProvider::processMomentaryThrottle() {
    // Level-triggered throttle slots: fires every frame while held so the
    // value stays pinned without OS-repeat transitions causing snap-backs.
    for (int k = '1'; k <= '9'; ++k) {
        if (keyHold_.isKeyDown(k)) {
            target_->setThrottleMomentary(static_cast<double>(k - '0') / 10.0);
        }
    }
    if (keyHold_.isKeyDown('0')) {
        target_->setThrottleMomentary(1.0);
    }
}

void KeyboardInputProvider::processGearAndIgnition() {
    // Gear: edge-triggered
    if (keyHold_.isKeyPressed(']')) target_->shiftUp();
    if (keyHold_.isKeyPressed('[')) target_->shiftDown();

    // Ignition: edge-triggered
    if (keyHold_.isKeyPressed('i') || keyHold_.isKeyPressed('I')) {
        target_->toggleIgnition();
    }

    // Starter: edge-triggered
    if (keyHold_.isKeyPressed('s') || keyHold_.isKeyPressed('S')) {
        target_->setStarter();
    }
}

void KeyboardInputProvider::processDynoAndBrake() {
    // Dyno torque: active (held for ramp)
    if (keyHold_.isKeyActive('e')) {
        target_->adjustDynoTorque(-0.1);
    }
    if (keyHold_.isKeyActive('d')) {
        target_->adjustDynoTorque(0.1);
    }
    // Release dyno: edge-triggered
    if (keyHold_.isKeyPressed('c')) {
        target_->releaseDynoTorque();
    }

    // Brake: level-triggered (keyDown), fire 0.0 on release edge
    if (keyHold_.isKeyDown('b')) {
        target_->setBrake(1.0);
    } else if (keyHold_.isKeyReleased('b')) {
        target_->setBrake(0.0);
    }
}

void KeyboardInputProvider::processSpeedControl() {
    // Speed control: active (hold to ramp)
    if (keyHold_.isKeyActive(',')) {
        target_->adjustSpeed(-2.0);
    }
    if (keyHold_.isKeyActive('.')) {
        target_->adjustSpeed(2.0);
    }
}

void KeyboardInputProvider::processPresetKeys() {
    // Preset: edge-triggered
    if (keyHold_.isKeyPressed('p') || keyHold_.isKeyPressed('P')) {
        target_->cyclePreset();
    }
}

std::string KeyboardInputProvider::GetProviderName() const { return "Keyboard"; }
std::string KeyboardInputProvider::GetLastError() const { return lastError_; }

void KeyboardInputProvider::setSession(ISimulatorSession* session) {
    session_ = session;
}

} // namespace input
