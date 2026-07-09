#include "input/ManualTwinProvider.h"
#include "simulator/ISimulator.h"

namespace input {

ManualTwinProvider::ManualTwinProvider(std::unique_ptr<IThrottleSource> throttleSource, ISimulator& simulator)
    : throttleSource_(std::move(throttleSource))
    , simulator_(simulator) {
}

ManualTwinProvider::~ManualTwinProvider() {
    doShutdown();
}

bool ManualTwinProvider::Initialize() {
    if (initialized_) {
        lastError_ = "Already initialized";
        return false;
    }
    if (!throttleSource_) {
        lastError_ = "No throttle source";
        return false;
    }
    initialized_ = true;
    return true;
}

void ManualTwinProvider::doShutdown() {
    initialized_ = false;
}

void ManualTwinProvider::Shutdown() {
    doShutdown();
}

bool ManualTwinProvider::IsConnected() const {
    return initialized_;
}

EngineInput ManualTwinProvider::OnUpdateSimulation(double dt) {
    if (!initialized_ || !throttleSource_) {
        EngineInput input;
        return input;
    }

    double throttle = throttleSource_->pollThrottle();
    twin_.setThrottle(throttle);

    if (gearUpRequested_) {
        twin_.requestGearUp();
        gearUpRequested_ = false;
    }
    if (gearDownRequested_) {
        twin_.requestGearDown();
        gearDownRequested_ = false;
    }

    twin_.setIgnition(ignitionRequested_);
    twin_.setStarterMotor(starterRequested_);

    twin::TwinFeedback feedback;
    feedback.engineRpm = simulator_.getEngineRpm();
    feedback.isValid = true;

    twin::TwinOutput output = twin_.update(dt, feedback);

    EngineInput input;
    input.throttle = output.throttle;
    input.ignition = output.ignition;
    input.starterButton = output.starterMotor;
    input.gearAbsolute = output.gear;
    input.clutchPressure = output.clutchPressure;
    input.gearSelector = static_cast<int>(output.gearSelector);
    input.gearAutoMode = false; // ManualTwin uses manual gearbox

    return input;
}

std::string ManualTwinProvider::GetProviderName() const {
    return "ManualTwinProvider";
}

std::string ManualTwinProvider::GetLastError() const {
    return lastError_;
}

void ManualTwinProvider::setGearUpRequested(bool requested) {
    gearUpRequested_ = requested;
}

void ManualTwinProvider::setGearDownRequested(bool requested) {
    gearDownRequested_ = requested;
}

void ManualTwinProvider::setIgnitionRequested(bool requested) {
    ignitionRequested_ = requested;
}

void ManualTwinProvider::setStarterRequested(bool requested) {
    starterRequested_ = requested;
}

} // namespace input
