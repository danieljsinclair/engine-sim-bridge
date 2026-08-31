// TorqueConverter.h - Fluid-coupling torque-converter coupling model.
//
// Ports the two-body torque-converter physics from the engine-sim rescue branch
// (origin/torque-converter-rescue, commit 7fd5abb: include/torque_converter.h +
// src/torque_converter.cpp) into the bridge's clutch-pressure coupling axis.
//
// PHYSICS (a real TC, no thresholds, no bang-bang)
// -----------------------------------------------
// A torque converter is a fluid coupling: a pump (impeller, engine side) and a
// turbine (gearbox side) with fluid between them. It transmits torque by the
// pump flinging fluid at the turbine. Two declarative curves fully describe it:
//
//   Capacity (the pump law):  T_capacity = K * N_impeller^2
//       The torque the converter can pump grows with the SQUARE of impeller
//       (engine) speed. At idle the fluid barely pumps -> little torque -> the
//       engine is barely loaded (gentle creep). As the engine revs the capacity
//       rises quadratically -> it transmits more torque -> launch.
//
//   Torque ratio (TR):  TR(SR) = T_turbine / T_impeller
//       where SR = N_turbine / N_impeller is the speed ratio (0 at stall, 1 at
//       lock). At stall (SR=0) the fluid multiplies torque (TR ~ 2.0, the stall
//       torque ratio); as the turbine catches the pump (SR -> 1) the
//       multiplication falls to 1.0 (the coupling point). This is the standard
//       MathWorks characteristic, resampled to a uniform grid.
//
//   Lockup clutch:  above a speed ratio and an impeller RPM the converter locks
//       the pump to the turbine directly (a solid 1:1 shaft, no slip, no
//       multiplication). Hysteretic so it cannot chatter.
//
// WHY IT CANNOT OSCILLATE / STALL
// --------------------------------
// The pressure is a SINGLE smooth monotonic ramp of the road-implied (turbine)
// rpm. It is a function of ROAD-IMPLIED rpm ONLY - never of engine rpm and never
// of the speed ratio. Two consequences:
//   * STANDSTILL/CREASE: road-implied < idle*0.9 => pressure flat at the
//     MODERATE creep capacity (creepPressure). The fluid LOADS a flaring engine
//     (quadratic pump resistance -> the engine settles at its stall speed, it
//     cannot free-rev unloaded) while still slipping against the pinned
//     stationary wheel (no rigid couple, no standstill oscillation).
//   * STABILITY: road-implied is EXOGENOUS (CSV-pinned wheels), so the pressure
//     cannot feed back into engine rpm and cannot form a limit cycle. Any engine-
//     rpm term (a pump gate, a speed-ratio lockup blend, the TR factor) closes a
//     feedback loop - the engine lugs, the pressure shifts, the lug shifts, -> a
//     cycle. The measured bench chatter (Cl 63%->100%->17%->83%->92%->3%->66%
//     frame-to-frame at 16 mph) came from exactly such a term: a lockup blend
//     keyed on the speed ratio (turbine/ENGINE), which cycled the pressure
//     0.03<->1.0 as the engine wandered through the narrow SR band. Dropping
//     EVERY engine-rpm term makes the schedule open-loop stable; the converter
//     then converges by pulling the engine TOWARD road speed instead of cycling.
// The TC's torque-ratio curve (stall multiplication) is kept as TELEMETRY only -
// TR is a function of the speed ratio, so folding it into the pressure would
// reintroduce the engine-rpm feedback above.
//
// MAPPING TO CLUTCH PRESSURE (the bridge runs a friction clutch, not an SCS
// constraint)
// ---------------------------------------------------------------------------
// In TC mode the pressure drives the converter's CAPACITY SCALE (transmission.cpp
// sets capacityScale = clutchPressure and holds the friction clutch open), so the
// normalized pressure IS the coupling strength. The governor is a mechanical-
// schedule ramp of the road-implied rpm:
//
//     p = creep + (1 - creep)
//         * smoothstep(idle*engageStartIdleFactor, idle*lockIdleFactor, road)
//     where creep = idleCreepPressure -> creepPressure blended on driver
//     throttle (the standstill regime split; both inputs exogenous).
//
// Flat at the (throttle-blended) creep capacity through the creep band,
// progressive through the slip band (the engine tracks road-implied
// x ~1.0-1.3), flat at 1.0 (locked) at and above the cruise lock point.
// Monotonic, C1-continuous, and engine-independent by construction. All
// declarative, all smooth.

#ifndef TWIN_TORQUE_CONVERTER_H
#define TWIN_TORQUE_CONVERTER_H

#include <twin/ICouplingModel.h>

namespace twin {

// Tunable converter characteristic. All declarative data (no thresholds baked
// into logic); the governor anchors are expressed as multiples of the engine's
// idle RPM so the curve scales with the profile.
struct TorqueConverterParameters {
    // Stall torque ratio TR(0) (T_turbine/T_impeller at stall). Scales the
    // reference curve's multiplication band. Rescue default 2.0. TELEMETRY
    // ONLY — never folded into the pressure (TR is a function of the speed
    // ratio, i.e. of engine rpm, i.e. feedback).
    double stallTorqueRatio = 2.0;

    // Standstill/creep capacity scale UNDER THROTTLE (the launch regime).
    // The fluid's creep capacity at zero turbine speed while launching: enough
    // to LOAD the engine (a flaring engine meets quadratic pump resistance and
    // settles at its stall speed, it cannot free-rev unloaded), small enough
    // to slip against the pinned stationary wheel (no rigid couple, no
    // standstill oscillation). The engine-side K*N^2 pump law shapes the load;
    // this only scales it. 0.6 holds a ~25% throttle launch flare under the
    // 3000 rpm free-rev bar (equilibrium N = sqrt(T_engine / (0.6 * K * TR))
    // ~ 2500 for ~180 Nm).
    //
    // NOTE (2026-08 sweep evidence, single-number trap — resolved 2026-08-31
    // by the idle/launch split): this ONE flat scale was serving two regimes
    // at once — standstill idle-hold support and part/WOT launch flare
    // loading. The 2026-08-31 creep grid re-measured the trap on live-master
    // code (C63_TeslaY, PIN+tau150, PinFixDrive3): a flat 0.4/0.35 cut the
    // standstill pump load 33-42% (stall margin at the D-engage bite up
    // +7..+131 rpm) but scaled the part-throttle launch-flare equilibrium as
    // 1/sqrt(capacity) past the 0.6 design point (3041 -> 3486 -> 3603 rpm at
    // the t=131 launch probe) — the documented free-rev-bar regression. The
    // split keeps creepPressure as the under-throttle standstill capacity and
    // adds idleCreepPressure for the zero-throttle band; the governor blends
    // between them on DRIVER THROTTLE (exogenous input, like road-implied
    // rpm — never engine rpm or the speed ratio, so the no-feedback-loop
    // guarantee is preserved).
    double creepPressure = 0.6;

    // Zero-throttle standstill capacity (the stoplight / creep-band scale).
    // Driveability evidence (owner 2026-08-30/31: "it still hunts idle in gear
    // and it stalls almost right away unless I have the throttle up"; bench:
    // D-idle droop ~-350 rpm vs a real C63's ~50-150, exhaust-flow
    // sign-flutter ~3x Park, deeper rpm dips at the D-engage bite with 0.6).
    // 0.35 cuts the zero-throttle standstill pump load ~40% (grid-measured)
    // while a launch (throttle at/above creepThrottleRampEnd) still sees the
    // full creepPressure above.
    double idleCreepPressure = 0.35;

    // Driver-throttle band (fraction) over which the standstill capacity
    // blends idleCreepPressure -> creepPressure. Below the start the
    // converter idles at idleCreepPressure; at/above the end it launches at
    // creepPressure. Smooth (C1) in throttle: no bang-bang at tip-in.
    double creepThrottleRampStart = 0.05;
    double creepThrottleRampEnd = 0.25;

    // Road-implied rpm (as a fraction of idle) where the progressive band
    // STARTS. Below this the pressure sits flat at creepPressure.
    double engageStartIdleFactor = 0.9;

    // Road-implied rpm (as a fraction of idle) where the converter is fully
    // LOCKED (pressure 1.0). Above this the pressure sits flat at 1.0.
    double lockIdleFactor = 1.6;

    // Release band below the lock point (RPM), preventing lock-state chatter
    // around the engagement boundary.
    double lockupHysteresisRpm = 150.0;

    // Master enable for the lockup clutch. With it off the converter stays in
    // the fluid (slip) regime at all speeds - useful for A/B tuning.
    bool lockupEnabled = true;

    // Cruise slip ceiling: the maximum slip (1 - speedRatio) the converter is
    // allowed to carry once above the cruise band. Caps the turbine lag at
    // steady cruise so road-implied rpm stays above the ~2900 attractor floor
    // (gear4/gear5 basin). 0.75 keeps the box in g4/g5 at cruise.
    double cruiseSlipCeiling = 0.75;
};

// Stateful torque-converter coupling model. The lockup clutch has hysteresis
// (state), so this is a class, not a pure function. Thread-unsafe: one instance
// per twin (the twin updates single-threaded).
class TorqueConverter : public ICouplingModel {
public:
    explicit TorqueConverter(const TorqueConverterParameters& params = TorqueConverterParameters{});
    ~TorqueConverter() override = default;

    // ICouplingModel: compute the desired clutch pressure for this frame.
    // Updates the internal lockup state (hysteresis).
    CouplingOutput compute(const CouplingInput& input) override;

    // Reconfigure the characteristic at runtime (e.g. when the twin learns the
    // transmission's real maxClutchTorque).
    void configure(const TorqueConverterParameters& params);

    // Telemetry accessors (for display / tests).
    double getSpeedRatio() const { return speedRatio_; }
    double getTorqueRatio() const { return torqueRatio_; }
    double getFluidPressure() const { return fluidPressure_; }
    double getSlip() const { return 1.0 - speedRatio_; }
    bool isLockupEngaged() const { return lockupEngaged_; }

    // Lookup the torque ratio at a speed ratio without constructing state
    // (exposed for unit tests of the curve shape).
    double lookupTorqueRatio(double speedRatio) const;

private:
    void buildTorqueRatioTable();

    TorqueConverterParameters params_;
    bool lockupEngaged_ = false;

    double speedRatio_ = 0.0;
    double torqueRatio_ = 1.0;
    double fluidPressure_ = 0.0;

    // TR curve resampled onto a uniform speed-ratio grid so the hot path is an
    // index-and-lerp (no per-frame search). Same scheme as the rescue branch.
    static constexpr int kTableResolution = 256;
    double torqueRatioTable_[kTableResolution];
};

}  // namespace twin

#endif  // TWIN_TORQUE_CONVERTER_H
