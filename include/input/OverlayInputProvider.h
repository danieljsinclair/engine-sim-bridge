// OverlayInputProvider.h - Composed keyboard overlay on live/replay core.
// Implements IInputProvider + IReplayTimeline + IArrivalStatePrimer by
// delegation (Option B). Keeps the twin unchanged: keyboard overrides apply
// at EngineInput layer, not twin upstream. The primer delegation keeps the
// wrapper TRANSPARENT for --start-from: SimulationLoop asks the provider it
// was handed, and the overlay forwards the arrival prime to the wrapped core
// (#66 — previously only IReplayTimeline was forwarded and the loop's
// capability cast on the wrapper returned null).

#ifndef OVERLAY_INPUT_PROVIDER_H
#define OVERLAY_INPUT_PROVIDER_H

#include "io/IInputProvider.h"
#include "input/IArrivalStatePrimer.h"
#include "input/IReplayTimeline.h"
#include "input/IKeyboardInput.h"
#include "input/KeyHoldBridge.h"
#include "input/EngineInputTarget.h"
#include "session/ISimulatorSession.h"

#include <memory>
#include <string>

namespace input {

class OverlayInputProvider : public IInputProvider,
                               public IReplayTimeline,
                               public IArrivalStatePrimer {
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

    // IArrivalStatePrimer delegation to core (null-safe: a core without the
    // primer — live/keyboard-ish — is untouched, same contract as the
    // timeline surface above).
    void primeArrivalState() override;
    void releaseArrivalHold() override;

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
