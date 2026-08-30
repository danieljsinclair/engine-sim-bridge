// OverlayInputProvider.h - Composed keyboard overlay on live/replay core.
// Implements IInputProvider + IReplayTimeline by delegation (Option B).
// Keeps the twin unchanged: keyboard overrides apply at EngineInput layer,
// not twin upstream.

#ifndef OVERLAY_INPUT_PROVIDER_H
#define OVERLAY_INPUT_PROVIDER_H

#include "io/IInputProvider.h"
#include "input/IReplayTimeline.h"
#include "input/IKeyboardInput.h"
#include "input/KeyHoldBridge.h"
#include "input/EngineInputTarget.h"
#include "session/ISimulatorSession.h"

#include <memory>
#include <string>

namespace input {

class OverlayInputProvider : public IInputProvider, public IReplayTimeline {
public:
    OverlayInputProvider(std::unique_ptr<IInputProvider> core,
                         std::unique_ptr<IKeyboardInput> keyboard,
                         IKeyActionTarget* target);
    ~OverlayInputProvider() override;

    bool Initialize() override;
    void Shutdown() override;
    bool IsConnected() const override;
    EngineInput OnUpdateSimulation(double dt) override;
    void provideFeedback(const EngineSimStats& stats) override;
    std::string GetProviderName() const override;
    std::string GetLastError() const override;

    void setSession(ISimulatorSession* session);

    // IReplayTimeline delegation to core
    double durationS() const override;
    void setEndAtS(double s) override;
    double getStartFromS() const override;

private:
    std::unique_ptr<IInputProvider> core_;
    std::unique_ptr<IKeyboardInput> keyboard_;
    IKeyActionTarget* target_;
    KeyHoldBridge keyHold_;
    ISimulatorSession* session_ = nullptr;
    std::string lastError_;
};

} // namespace input

#endif // OVERLAY_INPUT_PROVIDER_H
