#ifndef VIRTUAL_ICE_INPUT_PROVIDER_H
#define VIRTUAL_ICE_INPUT_PROVIDER_H

#include "io/IInputProvider.h"
#include "simulator/EngineSimTypes.h"
#include "io/UpstreamSignal.h"
#include "twin/IceVehicleProfile.h"
#include "twin/VirtualIceTwin.h"
#include "twin/IGearboxLogger.h"

#include <memory>
#include <vector>
#include <string>

class BridgeSimulator;  // forward decl — full definition via simulator/BridgeSimulator.h in the .cpp

namespace input {

class VirtualIceInputProvider : public IInputProvider {
public:
    explicit VirtualIceInputProvider(twin::IceVehicleProfile profile);

    // Reconfigure the gearbox to match the actual engine preset's ratios.
    void reconfigureProfile(const std::vector<double>& gearRatios,
                             double diffRatio, double tireRadiusM);
    ~VirtualIceInputProvider() override;

    // IInputProvider lifecycle
    bool Initialize() override;
    void Shutdown() override;
    bool IsConnected() const override;

    // IInputProvider input queries
    EngineInput OnUpdateSimulation(double dt) override;
    std::string GetProviderName() const override;
    std::string GetLastError() const override;

    // Set the upstream signal source (for testing or manual injection)
    void setUpstreamSignal(const UpstreamSignal& signal);

    // Forward gear selector changes to the twin
    void setGearSelector(int selector);

    // Forward ignition state to the twin
    void setIgnition(bool on);

    // Select the live clutch wheel-coupling strategy (FREE/PIN).
    void setWheelCouplingMode(twin::WheelCouplingMode mode);

    // PIN-coupling compliance tau in ms (--pin-tau-ms): 0 = rigid pin.
    void setPinTauMs(double tauMs);

    // --effective-throttle / --torque-informed-gearbox configs (both DEFAULT
    // disabled = inert). Stored + re-applied like the pending logger: the CLI
    // may set them BEFORE Initialize() creates the twin (ReplayTelemetryProvider
    // seeds its twin provider at Initialize time).
    void setEffectiveThrottleConfig(const twin::EffectiveThrottleConfig& config);
    void setTorqueInformedGearboxConfig(const twin::TorqueInformedGearboxConfig& config);

    // Select the coupling MODEL (clutch-map default, torque-converter, legacy).
    void setCouplingModel(twin::CouplingModelKind kind);

    // Forward simulator RPM feedback to the twin for cranking transition
    void provideFeedback(const EngineSimStats& stats) override;

    // Enable gearbox diagnostic logging
    void setGearboxLogger(twin::IGearboxLogger* logger);

    // Warm-up prime: after the RUNNING-prime, drive the twin for kWarmupFrames
    // at a light throttle (0.20) / 10 km/h so the gearbox shift phase, clutch
    // pressure ramp and idle-hold integrators settle into the warm cruise basin
    // before the first real frame. One-shot; idempotent (no-op once warmed).
    void primeWarmUp();

    // Arm a fresh re-crank budget on the twin: any synthetic prime frames
    // (warm-up or an instant --start-from arrival settle) can pass through a
    // Stopped core and fire discarded starter edges that consume the re-crank
    // cooldown. Arming clears that debt so the FIRST real crank attempt fires
    // immediately (same tail act as primeWarmUp performs).
    void armFreshCrankBudget() {
        if (twin_) twin_->armFreshCrankBudget();
    }

    // Bind the live BridgeSimulator so the provider can install the fluid-
    // coupling torque converter on the transmission when --coupling-model
    // torque-converter is selected. The session sets this AFTER creating the
    // simulator (the provider is constructed before the simulator exists), so
    // setCouplingModel() may arrive earlier — the requested kind is remembered
    // and applied here (and on any later setCouplingModel).
    void setBridgeSimulator(BridgeSimulator* sim);

private:
    twin::IceVehicleProfile profile_;  // owned (was const ref)
    std::unique_ptr<twin::VirtualIceTwin> twin_;
    std::string lastError_;
    bool isInitialized_;
    twin::IGearboxLogger* pendingLogger_ = nullptr;

    // Pending torque-feature configs (see the setter contract above); applied
    // to the twin when Initialize() creates it. Defaults are disabled.
    twin::EffectiveThrottleConfig pendingEffectiveThrottle_;
    twin::TorqueInformedGearboxConfig pendingTorqueInformedGearbox_;

    UpstreamSignal currentSignal_;

    // Live BridgeSimulator handle (set by the session after simulator creation).
    // Null until then; the requested torque-converter install is deferred until
    // it is set (or re-applied on a later setCouplingModel).
    BridgeSimulator* bridgeSim_ = nullptr;
    bool pendingTorqueConverter_ = false;
    bool warmedUp_ = false;
};

}

#endif
