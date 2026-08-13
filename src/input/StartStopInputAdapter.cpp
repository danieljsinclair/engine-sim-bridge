// StartStopInputAdapter.cpp - decorator that applies the vehicle start/stop
// decision onto the EngineInput flowing into SimulationLoop.
//
// See StartStopInputAdapter.h for the design rationale and the starter-pulse
// contract. This file mutates NO SimulationLoop code and NO provider code.

#include "input/StartStopInputAdapter.h"
#include "input/LiveTelemetryProvider.h"

#include <string>

namespace input {

StartStopInputAdapter::StartStopInputAdapter(std::unique_ptr<IInputProvider> inner,
                                              double crankDelayS)
    : inner_(std::move(inner))
    , controller_(observer_, crankDelayS) {
    ASSERT(inner_, "StartStopInputAdapter requires a non-null inner provider");
}

StartStopInputAdapter::~StartStopInputAdapter() = default;

bool StartStopInputAdapter::Initialize() {
    return inner_ && inner_->Initialize();
}

void StartStopInputAdapter::Shutdown() {
    if (inner_) inner_->Shutdown();
}

bool StartStopInputAdapter::IsConnected() const {
    return inner_ && inner_->IsConnected();
}

EngineInput StartStopInputAdapter::OnUpdateSimulation(double dt) {
    ASSERT(inner_, "StartStopInputAdapter has no inner provider");

    // 1. Poll the upstream provider (live or otherwise) for the base EngineInput.
    EngineInput input = inner_->OnUpdateSimulation(dt);

    // 2. Derive brake + gear from the latest upstream signal the provider holds.
    //    The decorator is only built for the live path, where the inner provider
    //    is a LiveTelemetryProvider exposing getCurrentSignal().
    auto* live = dynamic_cast<LiveTelemetryProvider*>(inner_.get());
    ASSERT(live, "StartStopInputAdapter inner must be a LiveTelemetryProvider");
    UpstreamSignal signal = live->getCurrentSignal();
    bool brakePressed = false;
    bridge::GearSelector gear = bridge::GearSelector::NEUTRAL;
    decodeSignal(signal, brakePressed, gear);
    if (std::getenv("BENCH_SS") != nullptr)
        std::fprintf(stderr, "[SS] dt=%.3f brake=%d gear=%d engOn=%d obsIgn=%d obsStr=%d\n",
                     dt, signal.brakePedalState, static_cast<int>(signal.gearSelector),
                     controller_.isEngineOn(), observer_.ignition_, observer_.starter_);

    // 3. Advance the decision layer. The controller writes to observer_ only;
    //    it does NOT touch the real actuator.
    controller_.update(dt, brakePressed, gear);

    // 4. Apply the decision as the single source of ignition/starter for the sim.
    //    Overwrite the inner provider's ignition/starterButton so the only
    //    authority SimulationLoop sees is this decorator's flattened output.
    input.ignition = observer_.ignition_;
    input.starterButton = emitStarterPulse(observer_.starter_);

    return input;
}

std::string StartStopInputAdapter::GetProviderName() const {
    return "StartStopInputAdapter(" +
           (inner_ ? inner_->GetProviderName() : "null") + ")";
}

std::string StartStopInputAdapter::GetLastError() const {
    return inner_ ? inner_->GetLastError() : "no inner provider";
}

void StartStopInputAdapter::decodeSignal(const UpstreamSignal& signal,
                                          bool& outBrakePressed,
                                          bridge::GearSelector& outGear) {
    // DI_brakePedalState: ON=1 pressed, OFF=0 / INVALID=2 not pressed.
    outBrakePressed = (signal.brakePedalState == 1);
    outGear = signal.gearSelector;
}

bool StartStopInputAdapter::emitStarterPulse(bool controllerStarterLevel) {
    // The controller can hold starter=true across many frames (e.g. a held brake
    // crank). CrankingController::engageStarter TOGGLES on a held-high button
    // (Stopped -> Cranking -> Stopped), which would abort the crank. So we emit a
    // single-frame pulse on the rising edge and hold it low until the controller
    // drives starter low again.
    bool pulse = false;
    if (controllerStarterLevel && !prevStarterLevel_) {
        pulse = true;               // rising edge: fire one frame
        starterPulsePending_ = true;
    } else if (controllerStarterLevel && prevStarterLevel_) {
        pulse = false;              // held high: suppress (pending, awaiting low)
    } else if (!controllerStarterLevel) {
        pulse = false;              // controller released: clear pending
        starterPulsePending_ = false;
    }
    prevStarterLevel_ = controllerStarterLevel;
    return pulse;
}

} // namespace input
