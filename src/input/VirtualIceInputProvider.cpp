#include "input/VirtualIceInputProvider.h"
#include "simulator/BridgeSimulator.h"
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
        // Apply any torque-feature configs that arrived before Initialize()
        // (store + re-apply; the defaults are inert no-ops on the twin).
        twin_->setEffectiveThrottleConfig(pendingEffectiveThrottle_);
        twin_->setTorqueInformedGearboxConfig(pendingTorqueInformedGearbox_);
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
    // Propagate the trace contract (UpstreamSignal::traceDriven) so the
    // simulation loop's crank decision can clamp its effective throttle to
    // the trace — the twin already honors it internally for its own floors.
    input.traceDrivenThrottle = currentSignal_.traceDriven;
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
    input.couplingIsTorqueConverter = output.couplingIsTorqueConverter;

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

void VirtualIceInputProvider::setPinTauMs(double tauMs) {
    if (twin_) {
        twin_->setPinTauMs(tauMs);
    }
}

void VirtualIceInputProvider::setEffectiveThrottleConfig(
    const twin::EffectiveThrottleConfig& config) {
    pendingEffectiveThrottle_ = config;
    if (twin_) {
        twin_->setEffectiveThrottleConfig(config);
    }
}

void VirtualIceInputProvider::setTorqueInformedGearboxConfig(
    const twin::TorqueInformedGearboxConfig& config) {
    pendingTorqueInformedGearbox_ = config;
    if (twin_) {
        twin_->setTorqueInformedGearboxConfig(config);
    }
}

void VirtualIceInputProvider::setCouplingModel(twin::CouplingModelKind kind) {
    if (twin_) {
        twin_->setCouplingModel(kind);
    }
    // The proper torque converter is an SCS direct-torque constraint installed
    // on the live transmission. Remember the request and apply it to the bridge
    // sim (which may not exist yet when this is called from CLI setup).
    pendingTorqueConverter_ = (kind == twin::CouplingModelKind::TorqueConverter);
    if (bridgeSim_ != nullptr) {
        bridgeSim_->setUseTorqueConverter(pendingTorqueConverter_);
    }
}

void VirtualIceInputProvider::setBridgeSimulator(BridgeSimulator* sim) {
    bridgeSim_ = sim;
    if (sim != nullptr && pendingTorqueConverter_) {
        sim->setUseTorqueConverter(true);
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

void VirtualIceInputProvider::primeWarmUp() {
    if (!isInitialized_ || !twin_) return;

    // Hold a light throttle / slow speed so the twin settles into the WARM
    // cruise basin (gear4/gear5, clutch ~0.75) rather than the cold attractor
    // (gear3, clutch ~0.15). One-shot: guard on a flag so re-Initialize() can
    // re-run it but a single Initialize() never double-primes.
    if (warmedUp_) {
        // Already primed (re-Initialize path): no synthetic frames ran here,
        // but the earlier prime's tail debt must still not bleed into the
        // real run.
        twin_->armFreshCrankBudget();
        return;
    }
    warmedUp_ = true;

    constexpr int kWarmupFrames = 300;
    constexpr double kWarmupThrottle = 0.20;
    constexpr double kWarmupSpeedKmh = 10.0;
    constexpr double kWarmupDt = 0.05;

    UpstreamSignal signal;
    signal.throttleFraction = kWarmupThrottle;
    signal.speedKmh = kWarmupSpeedKmh;
    signal.isValid = true;
    signal.timestampUtcMs = 1;
    setUpstreamSignal(signal);

    for (int i = 0; i < kWarmupFrames; ++i) {
        OnUpdateSimulation(kWarmupDt);
    }

    // LAST act of the prime: the synthetic frames above can pass through IDLE
    // with a Stopped core, firing re-crank edges whose output is discarded but
    // which consume reCrankCooldownS_. The REAL run starts right after this
    // with a Stopped core needing its first genuine edge — arm a fresh crank
    // budget so it fires immediately instead of waiting out synthetic debt.
    twin_->armFreshCrankBudget();
}

void VirtualIceInputProvider::provideFeedback(const EngineSimStats& stats) {
    if (twin_) {
        twin_->setEngineRpmFeedback(stats.currentRPM);
        twin_->setVehicleSpeedFeedback(stats.vehicleSpeedKmh);
        twin_->setDrivetrainTorqueFeedback(stats.drivetrainTorqueNm);
    }
}

}
