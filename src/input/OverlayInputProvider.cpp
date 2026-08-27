// OverlayInputProvider.cpp - Keyboard overlay over live/replay core provider.

#include "input/OverlayInputProvider.h"
#include "input/ReplayTelemetryProvider.h"
#include "input/LiveTelemetryProvider.h"
#include "session/ISimulatorSession.h"

namespace input {

OverlayInputProvider::OverlayInputProvider(
    std::unique_ptr<IInputProvider> core,
    std::unique_ptr<IKeyboardInput> keyboard,
    IKeyActionTarget* target)
    : core_(std::move(core))
    , keyboard_(std::move(keyboard))
    , target_(target) {
}

OverlayInputProvider::~OverlayInputProvider() = default;

bool OverlayInputProvider::Initialize() {
    // The core provider is initialized by CLIMain BEFORE being wrapped in the
    // overlay (so that coupling/start-from/gearbox-logger can be wired up
    // against a live provider). Re-initializing here would fail with "Already
    // initialized". Skip core init when the core is already initialized; only
    // initialize an uninitialized core (e.g. overlay constructed standalone in
    // a test).
    if (core_ && !core_->IsConnected() && !core_->Initialize()) {
        lastError_ = "Core provider initialization failed: " + core_->GetLastError();
        return false;
    }
    return true;
}

void OverlayInputProvider::Shutdown() {
    if (core_) core_->Shutdown();
}

bool OverlayInputProvider::IsConnected() const {
    if (!core_) return false;
    return core_->IsConnected();
}

EngineInput OverlayInputProvider::OnUpdateSimulation(double dt) {
    // 1. Core produces base input (CSV/stdin/replay through twin)
    EngineInput input{};
    if (core_) {
        input = core_->OnUpdateSimulation(dt);
    }

    // 2. Poll keyboard state and build keyboard-derived overrides
    if (keyboard_ && target_) {
        keyHold_.drainInput([this]() { return keyboard_->getKey(); }, dt * 1000.0);

        // Quit: edge-triggered
        if (keyHold_.isKeyPressed('q') || keyHold_.isKeyPressed('Q') || keyHold_.isKeyPressed(27)) {
            target_->quit();
            if (session_) session_->stop();
            return input;
        }

        // Ignition toggle (keyboard wins over CSV)
        if (keyHold_.isKeyPressed('i') || keyHold_.isKeyPressed('I')) {
            target_->toggleIgnition();
        }

        // Starter pulse (keyboard wins over CSV starter)
        if (keyHold_.isKeyPressed('s') || keyHold_.isKeyPressed('S')) {
            target_->setStarter();
        }

        // Throttle override: keyboard replaces CSV throttle when active
        if (keyHold_.isKeyActiveAny({'w', 'a', 'W', 65})) {
            target_->adjustThrottle(0.05);
        }
        if (keyHold_.isKeyActiveAny({'z', 'Z', 66})) {
            target_->adjustThrottle(-0.05);
        }
        if (keyHold_.isKeyPressed(' ')) {
            target_->setThrottle(0.0);
        }
        if (keyHold_.isKeyPressed('r') || keyHold_.isKeyPressed('R')) {
            target_->setThrottle(0.2);
        }
        // Momentary throttle keys override
        for (int k = '1'; k <= '9'; ++k) {
            if (keyHold_.isKeyDown(k)) {
                target_->setThrottleMomentary(static_cast<double>(k - '0') / 10.0);
            }
        }
        if (keyHold_.isKeyDown('0')) {
            target_->setThrottleMomentary(1.0);
        }

        // Gear shift (keyboard overrides CSV gear)
        if (keyHold_.isKeyPressed(']')) {
            target_->shiftUp();
            input.gearDelta = 1;
        }
        if (keyHold_.isKeyPressed('[')) {
            target_->shiftDown();
            input.gearDelta = -1;
        }

        // Dyno torque override
        if (keyHold_.isKeyActive('e')) {
            target_->adjustDynoTorque(-0.1);
        }
        if (keyHold_.isKeyActive('d')) {
            target_->adjustDynoTorque(0.1);
        }
        if (keyHold_.isKeyPressed('c')) {
            target_->releaseDynoTorque();
        }

        // Brake override
        if (keyHold_.isKeyDown('b')) {
            target_->setBrake(1.0);
            input.brakeLevel = 1.0;
        } else if (keyHold_.isKeyReleased('b')) {
            target_->setBrake(0.0);
            input.brakeLevel = 0.0;
        }

        // Speed control override
        if (keyHold_.isKeyActive(',')) {
            target_->adjustSpeed(-2.0);
        }
        if (keyHold_.isKeyActive('.')) {
            target_->adjustSpeed(2.0);
        }

        // Preset cycle
        if (keyHold_.isKeyPressed('p') || keyHold_.isKeyPressed('P')) {
            target_->cyclePreset();
            input.presetCycle = true;
        }

        // Build keyboard-derived EngineInput from the target and merge over core
        EngineInput keyboardInput = target_->buildEngineInput(dt);

        // Merge: keyboard wins on overlapping fields it touches.
        // Preserve CSV fields keyboard doesn't touch (replayTimestampS, clutchPressure,
        // gearAbsolute when keyboard doesn't press [ / ], vehicleSpeedTargetKmh when
        // keyboard doesn't press . / ,).
        if (keyboardInput.ignition != input.ignition) {
            // Only override if keyboard explicitly changed it; buildEngineInput
            // always produces a value. We use the keyboard value unconditionally
            // since the user wants keyboard to win.
            input.ignition = keyboardInput.ignition;
        }
        input.throttle = keyboardInput.throttle;
        input.starterButton = keyboardInput.starterButton ? keyboardInput.starterButton : input.starterButton;
        input.gearDelta = keyboardInput.gearDelta ? keyboardInput.gearDelta : input.gearDelta;
        input.dynoTorqueScale = keyboardInput.dynoTorqueScale >= 0.0 ? keyboardInput.dynoTorqueScale : input.dynoTorqueScale;
        input.brakeLevel = keyboardInput.brakeLevel >= 0.0 ? keyboardInput.brakeLevel : input.brakeLevel;
        input.presetCycle = keyboardInput.presetCycle ? keyboardInput.presetCycle : input.presetCycle;
        input.gearAutoMode = keyboardInput.gearAutoMode; // keyboard controls mode
        input.roadSpeedKmh = (keyboardInput.roadSpeedKmh >= 0.0) ? keyboardInput.roadSpeedKmh : input.roadSpeedKmh;
        // replayTimestampS preserved from core (CSV/live)
        // clutchPressure preserved from core unless keyboard changes it (not in design)
    }

    return input;
}

void OverlayInputProvider::provideFeedback(const EngineSimStats& stats) {
    if (core_) core_->provideFeedback(stats);
    if (target_) target_->provideFeedback(stats);
}

std::string OverlayInputProvider::GetProviderName() const {
    return "Overlay(Keyboard+Live)";
}

std::string OverlayInputProvider::GetLastError() const {
    return lastError_;
}

void OverlayInputProvider::setSession(ISimulatorSession* session) {
    session_ = session;
    if (core_) {
        // Replay/live providers may expose setSession; try dynamic cast
        if (auto* replay = dynamic_cast<ReplayTelemetryProvider*>(core_.get())) {
            replay->setSession(session);
        }
        if (auto* live = dynamic_cast<LiveTelemetryProvider*>(core_.get())) {
            // Live does not take session today, but keep for future
            (void)live;
        }
    }
}

// IReplayTimeline delegation

double OverlayInputProvider::durationS() const {
    if (!core_) return 0.0;
    if (const IReplayTimeline* tl = dynamic_cast<const IReplayTimeline*>(core_.get())) {
        return tl->durationS();
    }
    return 0.0;
}

void OverlayInputProvider::setEndAtS(double s) {
    if (core_) {
        if (IReplayTimeline* tl = dynamic_cast<IReplayTimeline*>(core_.get())) {
            tl->setEndAtS(s);
        }
    }
}

double OverlayInputProvider::getStartFromS() const {
    if (!core_) return -1.0;
    if (const IReplayTimeline* tl = dynamic_cast<const IReplayTimeline*>(core_.get())) {
        return tl->getStartFromS();
    }
    return -1.0;
}

} // namespace input
