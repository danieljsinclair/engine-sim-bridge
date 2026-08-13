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
// The slip-phase pressure is a SINGLE smoothstep of the road-implied (turbine)
// rpm, scaled by the capacity. It is a function of ROAD-IMPLIED rpm ONLY - never
// of engine rpm or the speed ratio. Two consequences:
//   * STANDSTILL/CREASE: road-implied < idle => pressure at the floor (0) => the
//     engine idles DECOUPLED. A post-crank flare cannot drag it to stall (the
//     standstill-stall fix), and a creeping engine is not lugged below idle (the
//     creep-lug fix).
//   * STABILITY: road-implied is EXOGENOUS (CSV-pinned wheels), so the pressure
//     cannot feed back into engine rpm and cannot form a limit cycle. Any engine-
//     rpm term (a pump law K*N^2, or the speed-ratio-based torque-ratio factor)
//     closes a feedback loop - the engine lugs, the pressure shifts, the lug
//     shifts, -> a cycle. The legacy binary relief cycled 0<->1149 rpm because it
//     was bang-bang; an engine-rpm "pump gate" cycled 1442<->70 the same way. The
//     friction clutch converges on its own by pulling the engine TOWARD road
//     speed (it cannot drag it below road), so an open-loop, road-driven schedule
//     settles instead of cycling.
// The TC's torque-ratio curve (stall multiplication) is kept as TELEMETRY only -
// TR is a function of the speed ratio, so folding it into the pressure would
// reintroduce the engine-rpm feedback above.
//
// MAPPING TO CLUTCH PRESSURE (the bridge runs a friction clutch, not an SCS
// constraint)
// ---------------------------------------------------------------------------
// The bridge's engine-sim clutch transmits torque ~= pressure * maxClutchTorque.
// The converter's fluid coupling is therefore expressed as a NORMALIZED pressure
//
//     p_fluid = floor + (capacity - floor) * roadGate(road-implied)
//
// (clamped to [floor, capacity]); the lockup clutch blends p up to 1.0 at cruise.
// All declarative, all smooth.

#ifndef TWIN_TORQUE_CONVERTER_H
#define TWIN_TORQUE_CONVERTER_H

#include <twin/ICouplingModel.h>

namespace twin {

// Tunable converter characteristic. All declarative data (no thresholds baked
// into logic); defaults follow the rescue branch + the bridge's clutch torque.
struct TorqueConverterParameters {
    // Stall torque ratio TR(0) (T_turbine/T_impeller at stall). Scales the
    // reference curve's multiplication band. Rescue default 2.0.
    double stallTorqueRatio = 2.0;

    // Fluid pressure ceiling for the slip phase (launch + coupling). SMALL: the
    // bridge's friction clutch transmits ~= pressure * maxClutchTorque (12000 Nm
    // here), so the capacity sets the max drag the fluid coupling can apply. Too
    // large and the clutch yanks the engine through road speed (overshoot -> the
    // 1442->70 yank at 0.12); too small and the engine free-revs instead of
    // tracking road. 0.025 ~= 300 Nm - enough to pull the engine to road speed
    // through the coupling band. The road gate isolates the standstill/creep band
    // (pressure at the floor there), and the lockup clutch blends p up to 1.0 at
    // cruise, so the capacity only governs the slip/coupling band.
    double capacityPressure = 0.025;

    // Pressure floor. Default 0: a real TC transmits a gentle creep at idle (the
    // pump law supplies it — the smoothstep low edge sits below idle so idle sees
    // a small positive pressure), and ~zero drag when the engine lugs BELOW idle
    // (so a lugging engine recovers instead of being dragged to stall). A hard
    // non-zero floor here would be a constant clutch drag that lugs the engine to
    // stall at low speed — the very creep-drag bug the model must avoid.
    double pressureFloor = 0.0;

    // Impeller speed at/above which lockup may engage, RPM. Rescue default 1500.
    double lockupRpm = 1500.0;

    // Speed ratio at/above which lockup may engage (near the coupling point).
    double lockupSpeedRatio = 0.85;

    // Release band below lockupRpm (RPM) and below lockupSpeedRatio (absolute),
    // preventing lockup chatter around the engagement boundary.
    double lockupHysteresisRpm = 150.0;
    double lockupHysteresisSpeedRatio = 0.05;

    // Master enable for the lockup clutch. With it off the converter stays in
    // the fluid (slip) regime at all speeds - useful for A/B tuning.
    bool lockupEnabled = true;
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
