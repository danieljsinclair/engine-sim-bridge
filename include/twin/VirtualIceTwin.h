#ifndef VIRTUAL_ICE_TWIN_H
#define VIRTUAL_ICE_TWIN_H

#include <twin/IVehicleTwin.h>
#include <twin/IceVehicleProfile.h>
#include <twin/AutomaticGearbox.h>
#include <twin/ThrottleSmoother.h>
#include <twin/PinTargetChase.h>
#include <twin/IGearboxLogger.h>
#include <twin/WheelCoupling.h>
#include <twin/CouplingModelSelector.h>
#include <twin/EffectiveThrottle.h>
#include <twin/UpstreamTorqueHint.h>
#include <io/UpstreamSignal.h>
#include <simulator/GearConventions.h>
#include <memory>
#include <vector>

namespace twin {

class VirtualIceTwin {
public:
    explicit VirtualIceTwin(IceVehicleProfile profile);

    // Reconfigure the gearbox to match the actual engine preset's ratios.
    // Auto-generates a shift table from the ratios + redline (same logic as
    // ReplayTelemetryProvider::reconfigureProfile). Call after the simulator
    // loads so the gearbox matches whatever engine preset is active.
    void reconfigureProfile(const std::vector<double>& gearRatios,
                             double diffRatio, double tireRadiusM);

    TwinOutput update(double dt, const input::UpstreamSignal& signal);

    TwinState getState() const { return state_; }

    void setEngineRpmFeedback(double rpm) { engineRpmFeedback_ = rpm; }
    void setVehicleSpeedFeedback(double kmh) { vehicleSpeedFeedbackKmh_ = kmh; }

    // Prime seam: warm-boot/warm-up synthetic frames can pass through IDLE
    // while the engine core is Stopped, firing re-crank edges whose output is
    // discarded but which still consume reCrankCooldownS_. Arm a fresh crank
    // budget so the first REAL frame can engage the starter immediately.
    void armFreshCrankBudget() { reCrankCooldownS_ = 0.0; }
    // Drivetrain torque feedback (Nm) feeds the torque-driven shift logic.
    void setDrivetrainTorqueFeedback(double nm) { drivetrainTorqueNm_ = nm; }
    void setGearboxLogger(IGearboxLogger* logger);

    void setGearSelector(bridge::GearSelector s) { selector_ = s; }
    bridge::GearSelector getGearSelector() const { return selector_; }

    void setIgnition(bool on) { ignitionOn_ = on; }
    bool getIgnition() const { return ignitionOn_; }

    int getCurrentGear() const { return gearbox_->getCurrentGear(); }

    // Test/diagnostic accessor: the live (reconfigured) profile the twin uses.
    const IceVehicleProfile& getProfile() const { return profile_; }

    double getSmoothedThrottle() const { return throttleSmoother_.getCurrentValue(); }

    // Select the wheel-coupling strategy (FREE default, or PIN to mirror replay).
    void setWheelCouplingMode(WheelCouplingMode mode) {
        wheelCouplingMode_ = mode;
        coupling_ = makeWheelCoupling(mode);
    }

    // Compliance for the PIN coupling target (--pin-tau-ms): chase the surfaced
    // vehicle-speed pin with a critically-damped response instead of
    // teleporting between the CSV's held road-speed levels (the audible rpm
    // staircase at the ~5.5 Hz CAN cadence). 0 = the rigid pin, bit-identical
    // (the default and the regression contract). Scoped to the pin TARGET:
    // the gearbox shift map and slip-lock math keep the raw speed.
    void setPinTauMs(double tauMs) { pinTargetChase_.setTauMs(tauMs); }

    // --effective-throttle (CLI feature toggle, DEFAULT OFF). When enabled,
    // the twin's ENGINE DRIVE throttle is derived ONCE per frame from the
    // commanded motor torque whenever the pedal sits at the foot-off deadband
    // and AP torque is pulling (see EffectiveThrottle.h for the pinned blend
    // + hysteresis contract). Scoped to the engine drive only: the gearbox
    // decision, coupling and pin target keep reading the raw signal. With
    // enabled=false (the default) the twin must behave byte-identically to a
    // twin that was never configured.
    void setEffectiveThrottleConfig(const EffectiveThrottleConfig& config);

    // --torque-informed-gearbox (CLI feature toggle, DEFAULT OFF). When
    // enabled, the UPSTREAM commanded motor torque (sign + magnitude, from
    // UpstreamSignal::motorTorqueNm) enters the gearbox shift decision as a
    // demand hint through the ITorqueHint seam (see UpstreamTorqueHint.h for
    // the pinned demand-magnitude contract). Decision input ONLY — never
    // physics. With enabled=false the hint must be indistinguishable from
    // NullTorqueHint (byte-identical decisions).
    void setTorqueInformedGearboxConfig(const TorqueInformedGearboxConfig& config);

    // Select the coupling MODEL (how the clutch pressure is derived). Default is
    // ClutchMap (declarative smooth governor — no binary relief, no oscillation).
    // Legacy runs the historical slip-lock + binary-relief path for A/B;
    // TorqueConverter runs the fluid-coupling model. See CouplingModelSelector.h.
    void setCouplingModel(CouplingModelKind kind) {
        couplingModelKind_ = kind;
        couplingModel_ = makeCouplingModel(kind);
    }
    CouplingModelKind getCouplingModelKind() const { return couplingModelKind_; }

private:
    IceVehicleProfile profile_;  // owned (was const ref — caused dangling + no reconfigure)
    std::unique_ptr<AutomaticGearbox> gearbox_;
    ThrottleSmoother throttleSmoother_;
    PinTargetChase pinTargetChase_;

    TwinState state_ = TwinState::OFF;
    double timeWithoutValidTelemetryS_ = 0.0;
    double shiftTimerS_ = 0.0;
    double crankingTimerS_ = 0.0;
    // Re-crank cooldown for the RUNNING-state restart-on-stall. Counts down from
    // RECRANK_PERIOD_S after a one-tick starter edge; the twin pulses the starter
    // ONLY when this reaches 0 while stalled. The bridge's
    // CrankingController::engageStarter is a momentary TOGGLE -- a starterButton
    // edge while already Cranking forces the phase BACK to Stopped
    // (CrankingController.cpp:27-31) -- so re-firing faster than a crank attempt
    // toggles Stopped<->Cranking every tick (the 1/0/1/0 oscillation documented
    // in the CRANKING case). One edge ENGAGES; the bridge's step() then cranks
    // autonomously until catch. The cooldown bounds RETRY: if the engine is still
    // stalled RECRANK_PERIOD_S after the last edge (crank failed to raise rpm
    // above kStallRpm), fire one fresh edge for a new clean attempt. Resets to 0
    // the instant the engine recovers, so the next genuine stall fires promptly.
    // See VirtualIceTwin.cpp RUNNING case.
    double reCrankCooldownS_ = 0.0;
    // Idle-hold controller (ECU idle-air equivalent) for the RUNNING case: a
    // small PI on (idleRpm - feedbackRpm) that replaces the old static 5%
    // IDLE_SUSTAIN_THROTTLE floor. Its output is a FLOOR — effective throttle
    // = max(driverThrottle, controllerOutput) — never a replacement; at driver
    // 60% the engine sees 60%. Engages below idleRpm, releases at
    // idleRpm + kIdleHoldReleaseMarginRpm (hysteresis). The proportional term
    // has IMMEDIATE authority (same-tick, only output-clamped): under the
    // converter's low-speed capacity drag the engine dies in under a second,
    // so a rate-limited ramp-up would defeat the controller's purpose. Only
    // the integral term and the release (downward) direction are rate-limited,
    // for smooth gearbox inputs. The integral DECAYS (tau ~2s) while
    // disengaged — a flush-on-release reset the steady-state hold offset on
    // every idle sawtooth cycle; the hard flush lives only at the
    // starter/crank handoff, where the idle-flare risk actually is.
    // Regulates on engineRpmFeedback_, which is one step in arrears of the
    // engine's own phase latch — see the kStallRpm note below for why that
    // ordering is safe, and the sweep traces for hunting at the engage band.
    bool idleHoldActive_ = false;
    double idleHoldIntegralPct_ = 0.0;
    double idleHoldOutputPct_ = 0.0;
    double engineRpmFeedback_ = 0.0;
    double vehicleSpeedFeedbackKmh_ = 0.0;
    double drivetrainTorqueNm_ = 0.0;
    double clutchPressure_ = 1.0;
    bridge::GearSelector selector_ = bridge::GearSelector::NEUTRAL;
    // Ignition is OFF until a start decision commands it on (setIgnition). The
    // twin must never assume a running engine: on live paths
    // VehicleStartController (via SimulationLoop) owns every start, so a
    // default-on here self-started the engine at the first valid telemetry
    // frame — before any driver input (UpLeckHill: running at t=0.62s, first
    // brake frame only at t=10.04s). Owners that legitimately start on their
    // own (keyboard demo, manual twin) command ignition explicitly.
    bool ignitionOn_ = false;

    WheelCouplingMode wheelCouplingMode_ = WheelCouplingMode::Free;
    std::unique_ptr<IWheelCoupling> coupling_ = makeWheelCoupling(WheelCouplingMode::Free);

    // Coupling MODEL (how the clutch pressure is derived). Default ClutchMap: a
    // declarative smooth governor curve that replaces the legacy binary relief
    // (the bang-bang oscillation source). See CouplingModelSelector.h.
    CouplingModelKind couplingModelKind_ = CouplingModelKind::ClutchMap;
    std::unique_ptr<ICouplingModel> couplingModel_ = makeCouplingModel(CouplingModelKind::ClutchMap);

    // RPM the engine would be at if locked to the given wheel speed in the
    // current gear (clones the replay formula). Fallback idleRpm when gear is
    // out of range.
    double roadSpeedImpliedRpmFor(double wheelSpeedKmh) const;

    // Idle-hold controller step: advances the engage/release state machine
    // and PI, returns the throttle FLOOR to apply this tick (a fraction of
    // full throttle), or 0.0 when the controller has nothing to add. See the
    // member block above for the design contract and the implementation
    // (VirtualIceTwin.cpp) for the constants and their evidence.
    double idleHoldFloor(double dt, double feedbackRpm);

    // Restart-on-stall guard, shared by the IDLE and RUNNING states: with
    // ignition on and feedback RPM at/below CrankingController::STOPPED_RPM,
    // pulse a ONE-TICK starter edge (retry-bounded by reCrankCooldownS_), flush
    // the idle-hold controller and floor the throttle at cranking level.
    // Returns true when stalled (the caller may skip its own idle handling).
    // A real car cranks and idles in PARK too, so IDLE needs the same guard
    // RUNNING has — without it a PARK-start trace (engine core Stopped, twin
    // primed to IDLE) sat dead until the driver selected D.
    bool restartIfStalled(TwinOutput& output, double dt);

    void updateShiftExecution(double dt);
};

}

#endif
