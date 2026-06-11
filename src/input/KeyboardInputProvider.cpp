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
void KeyboardInputProvider::Shutdown() {
	    // No-op: KeyboardInputProvider holds no resources requiring explicit cleanup.
	    // IKeyboardInput lifetime is managed by the caller (CLI/GUI).
	}
bool KeyboardInputProvider::IsConnected() const { return true; }

EngineInput KeyboardInputProvider::OnUpdateSimulation(double dt) {
    processKeys(dt);
    return target_->buildEngineInput(dt);
}

bool KeyboardInputProvider::processQuitAndThrottle(double dt) {
    keyHold_.drainInput([this]() { return keyboard_->getKey(); }, dt * 1000.0);

    // Quit: edge-triggered (pressed only)
    if (keyHold_.isKeyPressed('q') || keyHold_.isKeyPressed('Q') || keyHold_.isKeyPressed(27)) {
        target_->quit();
        if (session_) session_->stop();
        return true;
    }

    // Throttle ramp up/down: active (pressed or repeating)
    if (keyHold_.isKeyActiveAny({'w', 'a', 'W', 65})) {
        target_->adjustThrottle(0.05);
    }
    if (keyHold_.isKeyActiveAny({'z', 'Z', 66})) {
        target_->adjustThrottle(-0.05);
    }

    // Space: zero throttle, R: 20% throttle (edge-triggered)
    if (keyHold_.isKeyPressed(' ')) {
        target_->setThrottle(0.0);
    }
    if (keyHold_.isKeyPressed('r') || keyHold_.isKeyPressed('R')) {
        target_->setThrottle(0.2);
    }
    return false;
}

void KeyboardInputProvider::processMomentaryThrottle() {
    // Momentary throttle: level-triggered (keyDown) (1-9 = 10%-90%, 0 = 100%)
    // Uses isKeyDown (not isKeyActive) so setThrottleMomentary fires EVERY FRAME
    // while the key is held, preventing snap-back between OS repeat events.
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
    if (keyHold_.isKeyPressed(']')) target_->shiftUp();
    if (keyHold_.isKeyPressed('[')) target_->shiftDown();
    if (keyHold_.isKeyPressed('i') || keyHold_.isKeyPressed('I')) target_->toggleIgnition();
    if (keyHold_.isKeyPressed('s') || keyHold_.isKeyPressed('S')) target_->setStarter();
    if (keyHold_.isKeyPressed('p') || keyHold_.isKeyPressed('P')) target_->cyclePreset();
}

void KeyboardInputProvider::processDynoAndBrake() {
    if (keyHold_.isKeyActive('e')) target_->adjustDynoTorque(-0.1);
    if (keyHold_.isKeyActive('d')) target_->adjustDynoTorque(0.1);
    if (keyHold_.isKeyPressed('c')) target_->releaseDynoTorque();

    // Brake: level-triggered (keyDown), also fire on release edge
    if (keyHold_.isKeyDown('b')) {
        target_->setBrake(1.0);
    } else if (keyHold_.isKeyReleased('b')) {
        target_->setBrake(0.0);
    }
}

void KeyboardInputProvider::processSpeedControl() {
    if (keyHold_.isKeyActive(',')) target_->adjustSpeed(-2.0);
    if (keyHold_.isKeyActive('.')) target_->adjustSpeed(2.0);
}

void KeyboardInputProvider::processKeys(double dt) {
    if (processQuitAndThrottle(dt)) {
        return;
    }
    processMomentaryThrottle();
    processGearAndIgnition();
    processDynoAndBrake();
    processSpeedControl();
}

std::string KeyboardInputProvider::GetProviderName() const { return "Keyboard"; }
std::string KeyboardInputProvider::GetLastError() const { return lastError_; }

void KeyboardInputProvider::setSession(ISimulatorSession* session) {
    session_ = session;
}

} // namespace input
