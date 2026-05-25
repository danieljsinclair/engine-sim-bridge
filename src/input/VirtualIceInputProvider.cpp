#include "input/VirtualIceInputProvider.h"

namespace input {

VirtualIceInputProvider::VirtualIceInputProvider(const twin::IceVehicleProfile& profile)
    : profile_(profile), isInitialized_(false) {
}

VirtualIceInputProvider::~VirtualIceInputProvider() {
    Shutdown();
}

bool VirtualIceInputProvider::Initialize() {
    if (isInitialized_) {
        lastError_ = "Already initialized";
        return false;
    }

    try {
        twin_ = std::make_unique<twin::VirtualIceTwin>(profile_);
        if (pendingLogger_) {
            twin_->setGearboxLogger(pendingLogger_);
        }
        isInitialized_ = true;
        return true;
    } catch (const std::exception& e) {
        lastError_ = std::string("Failed to create twin: ") + e.what();
        return false;
    }
}

void VirtualIceInputProvider::Shutdown() {
    twin_.reset();
    isInitialized_ = false;
}

bool VirtualIceInputProvider::IsConnected() const {
    return isInitialized_ && twin_ != nullptr;
}

EngineInput VirtualIceInputProvider::OnUpdateSimulation(double dt) {
    EngineInput input{};
    input.shouldContinue = true;

    if (!isInitialized_ || !twin_) {
        lastError_ = "Provider not initialized";
        return input;
    }

    // Feed the current upstream signal to the twin
    twin::TwinOutput output = twin_->update(dt, currentSignal_);

    // Translate twin output to engine input
    input.throttle = output.throttle;
    input.gearAbsolute = output.gear;
    input.clutchPressure = output.clutchPressure;
    input.ignition = output.ignition;
    input.starterMotor = output.starterMotor;
    input.gearSelector = static_cast<int>(output.gearSelector);
    input.gearAutoMode = true; // VirtualIceTwin uses automatic gearbox

    return input;
}

std::string VirtualIceInputProvider::GetProviderName() const {
    return "VirtualIceInputProvider";
}

std::string VirtualIceInputProvider::GetLastError() const {
    return lastError_;
}

void VirtualIceInputProvider::setUpstreamSignal(const UpstreamSignal& signal) {
    currentSignal_ = signal;
}

void VirtualIceInputProvider::setGearSelector(int selector) {
    if (twin_) {
        twin_->setGearSelector(static_cast<bridge::GearSelector>(selector));
    }
}

void VirtualIceInputProvider::setIgnition(bool on) {
    if (twin_) {
        twin_->setIgnition(on);
    }
}

void VirtualIceInputProvider::setGearboxLogger(twin::IGearboxLogger* logger) {
    pendingLogger_ = logger;
    if (twin_) {
        twin_->setGearboxLogger(logger);
    }
}

void VirtualIceInputProvider::provideFeedback(const EngineSimStats& stats) {
    if (twin_) {
        twin_->setEngineRpmFeedback(stats.currentRPM);
        twin_->setVehicleSpeedFeedback(stats.vehicleSpeedKmh);
    }
}

}
