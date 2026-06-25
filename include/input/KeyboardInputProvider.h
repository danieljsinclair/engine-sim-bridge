// KeyboardInputProvider.h - Consolidated keyboard input provider
// Uses IKeyActionTarget (Strategy) to route key actions to standard or demo mode.
// Uses KeyHoldBridge for key state tracking (pressed, repeating, down, released).

#ifndef KEYBOARD_INPUT_PROVIDER_CONSOLIDATED_H
#define KEYBOARD_INPUT_PROVIDER_CONSOLIDATED_H

#include "input/IKeyActionTarget.h"
#include "input/KeyHoldBridge.h"
#include "io/IInputProvider.h"

#include <memory>

class IKeyboardInput;
class ISimulatorSession;

namespace input {

class KeyboardInputProvider : public IInputProvider {
public:
    KeyboardInputProvider(
        std::unique_ptr<IKeyboardInput> keyboard,
        IKeyActionTarget* target);

    ~KeyboardInputProvider() override;

    bool Initialize() override;
    void Shutdown() override;
    bool IsConnected() const override;
    EngineInput OnUpdateSimulation(double dt) override;
    void provideFeedback(const EngineSimStats& stats) override;
    std::string GetProviderName() const override;
    std::string GetLastError() const override;

    void setSession(ISimulatorSession* session);

private:
    void processKeys(double dt);
    bool processQuitAndThrottle(double dt);
    void processGearAndIgnition();
    void processDynoAndBrake();
    void processSpeedControl();
    void processMomentaryThrottle();

    std::unique_ptr<IKeyboardInput> keyboard_;
    IKeyActionTarget* target_;
    KeyHoldBridge keyHold_;
    ISimulatorSession* session_ = nullptr;
    std::string lastError_;
};

} // namespace input

#endif // KEYBOARD_INPUT_PROVIDER_CONSOLIDATED_H
