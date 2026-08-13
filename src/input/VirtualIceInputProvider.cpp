#include "input/VirtualIceInputProvider.h"
#include "common/PresetExceptions.h"

namespace input {

VirtualIceInputProvider::VirtualIceInputProvider(twin::IceVehicleProfile profile)
    : profile_(std::move(profile)), isInitialized_(false) {
}

VirtualIceInputProvider::~VirtualIceInputProvider() {
    twin_.reset();
    isInitialized_ = false;
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
    } catch (const PresetException& e) {
        lastError_ = std::string("Failed to create twin (preset error): ") + e.what();
        return false;
    } catch (const SimulatorException& e) {
        lastError_ = std::string("Failed to create twin (simulator error): ") + e.what();
        return false;
    } catch (const std::bad_alloc& e) {
        lastError_ = std::string("Failed to create twin (out of memory): ") + e.what();
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
    input.starterButton = output.starterMotor;
    input.gearSelector = static_cast<int>(output.gearSelector);
    input.gearAutoMode = true; // VirtualIceTwin uses automatic gearbox
    input.dynoTorqueScale = output.dynoTorqueScale;

    // Surface the upstream (CSV/feedback) road speed so the presentation layer
    // can display it (EngineInput.roadSpeedKmh -> EngineState.controls.
    // commandedSpeedKmh). Without this the live path dropped the road speed, so
    // the speed readout showed only the engine-sim vehicle physics — which
    // creeps near standstill because the wheels are not driven here — instead of
    // the commanded road speed. Mirrors ReplayTelemetryProvider, which sets
    // input.roadSpeedKmh = s.roadSpeedKmh for every sample.
    input.roadSpeedKmh = currentSignal_.speedKmh;

    // Surface the twin's vehicle-speed pin (set in RUNNING by the wheel-coupling
    // strategy). FREE leaves -1.0 (no pin); PIN copies the CSV speed. The
    // downstream SimulationLoop gates -1 as "don't change", so this is an
    // unconditional copy — no scattered conditional here.
    input.vehicleSpeedTargetKmh = output.pinVehicleSpeedTargetKmh;

    // Surface the twin's recorded-input-torque injection (MATCH/Torque mode).
    // FREE/PIN leave 0.0 (no-op on the rotating mass); Torque copies the recorded
    // motor_torque_nm so the solver integrates road speed from it.
    input.drivetrainInputTorqueNm = output.drivetrainInputTorqueNm;

    // Thread live clutch diagnostics through to presentation (inline clutch
    // readout + CSV-out spelunking).
    input.roadImpliedRpm = output.roadImpliedRpm;
    input.creepReliefFired = output.creepReliefFired;

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

void VirtualIceInputProvider::setWheelCouplingMode(twin::WheelCouplingMode mode) {
    if (twin_) {
        twin_->setWheelCouplingMode(mode);
    }
}

void VirtualIceInputProvider::setCouplingModel(twin::CouplingModelKind kind) {
    if (twin_) {
        twin_->setCouplingModel(kind);
    }
}

void VirtualIceInputProvider::reconfigureProfile(const std::vector<double>& gearRatios,
                                                  double diffRatio, double tireRadiusM) {
    if (twin_) twin_->reconfigureProfile(gearRatios, diffRatio, tireRadiusM);
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
        twin_->setDrivetrainTorqueFeedback(stats.drivetrainTorqueNm);
    }
}

}
