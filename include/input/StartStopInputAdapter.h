// StartStopInputAdapter.h - decorator IInputProvider that applies the
// vehicle-driven start/stop decision to the EngineInput the simulation reads,
// WITHOUT reaching the actuator behind SimulationLoop's back.
//
// Why a decorator (not a hook callback, not a SimulationLoop change):
//   - SRP: the start/stop decision (VehicleStartController) stays a pure decision
//     layer; the adapter's only job is to bridge that decision onto the upstream
//     provider's output. SimulationLoop and LiveTelemetryProvider are untouched.
//   - The controller is bound to an internal ObserverActuator, never to the real
//     BridgeSimulator. The live-actuator path is exclusively
//     SimulationLoop::applyCrankingDecision -> CrankingController (single authority).
//     Our controller only ever observes via a mock actuator so its ignition/starter
//     *level* decisions can be flattened into EngineInput — it never double-acts.
//   - OCP: adding start/stop to a new provider path is one decorator wrap, not an
//     edit to the loop or the provider.
//
// The starter-contract subtlety: CrankingController::engageStarter TOGGLES on a
// held-high starterButton (Stopped -> Cranking -> Stopped). A level `true` that
// persists across frames would cancel the crank on the next frame. So the
// controller's starter *level* is converted to a single-frame PULSE: the pulse is
// emitted on the frame the controller's starter rises true, then suppressed until
// the controller drives it low again. This mirrors EngineInputTarget::setStarter().

#ifndef START_STOP_INPUT_ADAPTER_H
#define START_STOP_INPUT_ADAPTER_H

#include "io/IInputProvider.h"
#include "io/UpstreamSignal.h"
#include "input/VehicleStartController.h"
#include "common/Verification.h"

#include <memory>

namespace input {

// Forward declaration: the decorator is only ever built for the live-telemetry
// path, so it reaches the upstream signal via LiveTelemetryProvider::getCurrentSignal().
class LiveTelemetryProvider;


// Minimal actuator the controller drives so we can READ its level decisions.
// It is observer-only: nothing downstream consumes it; the adapter translates the
// levels into EngineInput fields. This keeps the real actuator (BridgeSimulator)
// exclusively under CrankingController's authority.
class ObserverActuator : public IEngineActuator {
public:
    void setIgnition(bool on) override { ignition_ = on; }
    void setStarterMotor(bool on) override { starter_ = on; }

    bool ignition_ = false;
    bool starter_ = false;
};

// Decorates an inner IInputProvider: polls it each frame, runs the start/stop
// decision from the upstream signal (brake + gear), and overwrites the
// ignition/starterButton fields before returning.
class StartStopInputAdapter : public IInputProvider {
public:
    static constexpr double kDefaultCrankDelayS =
        VehicleStartController::kDefaultCrankDelayS;

    explicit StartStopInputAdapter(std::unique_ptr<IInputProvider> inner,
                                    double crankDelayS = kDefaultCrankDelayS);

    ~StartStopInputAdapter() override;

    // IInputProvider
    bool Initialize() override;
    void Shutdown() override;
    bool IsConnected() const override;
    EngineInput OnUpdateSimulation(double dt) override;
    std::string GetProviderName() const override;
    std::string GetLastError() const override;

    // The decorated inner provider (non-owning access for callers needing it).
    IInputProvider* inner() const { return inner_.get(); }

private:
    // Map the upstream signal to (brakePressed, gear) for the decision layer.
    // brake: ON(1) => pressed; OFF(0) and INVALID(2) => not pressed.
    static void decodeSignal(const UpstreamSignal& signal,
                             bool& outBrakePressed,
                             bridge::GearSelector& outGear);

    // Convert the controller's starter LEVEL to a single-frame pulse so the
    // CrankingController toggle does not cancel a held crank. Returns true on the
    // one frame the pulse should fire.
    bool emitStarterPulse(bool controllerStarterLevel);

    std::unique_ptr<IInputProvider> inner_;
    ObserverActuator observer_;
    VehicleStartController controller_;

    bool prevStarterLevel_ = false;  // edge detection for the pulse
    bool starterPulsePending_ = false;
};

} // namespace input

#endif // START_STOP_INPUT_ADAPTER_H
