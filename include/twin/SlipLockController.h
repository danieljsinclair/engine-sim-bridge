// SlipLockController.h - Pure torque-converter slip-lock pressure function.
//
// A friction clutch in engine-sim transmits torque linearly with pressure. At
// standstill any non-zero pressure loads the engine against the speed-pinned
// wheels -> RPM drops -> stall. With zero pressure the engine free-revs ->
// redline at any throttle. This function computes a clutch pressure that
// resolves that circle:
//
//   - At standstill (road-implied < idle):    pressure == 0   (engine free to idle)
//   - At launch under throttle (high slip):   partial pressure (TC slip in power band)
//   - As road catches up (slip -> 0):         pressure -> 1.0 (locked, direct coupling)
//   - On decel (engine slower than road):     pressure == 1.0 (locked, engine braking)
//
// Hard rule from the stall circle: the clutch MUST be open whenever
// roadSpeedImpliedRpm < idleRpm. Coupling below that floor drags the engine
// under idle and stalls it. The TC characteristic only applies above the floor.
//
// This is a pure function with no dependency on engine-sim physics.

#ifndef TWIN_SLIP_LOCK_CONTROLLER_H
#define TWIN_SLIP_LOCK_CONTROLLER_H

namespace twin {

// Inputs to the slip-lock computation.
struct SlipLockInput {
    double engineRpm;            // Current engine RPM (from feedback).
    double roadSpeedImpliedRpm;  // RPM the engine would be at if locked to road speed in current gear.
    double throttleFraction;     // 0..1.
    double idleRpm;              // Engine idle RPM (e.g. 700).
    double redlineRpm;           // Engine redline RPM (e.g. 6500).
};

// Outputs of the slip-lock computation.
struct SlipLockOutput {
    double clutchPressure;  // 0..1 (0 = open, 1 = fully locked).
    bool locked;            // true when effectively locked (for display / state).
};

// Compute clutch pressure from slip. Algorithm:
//   1. If roadSpeedImpliedRpm < idleRpm:
//        - creepPressure = throttleFraction * maxCreepPressure  (TC fluid coupling at stall)
//        - If creepPressure > 0: return {creepPressure, false}  (partial coupling, engine loaded)
//        - Otherwise:            return {0.0, false}            (true neutral, no creep)
//   2. slip      = max(0, engineRpm - roadSpeedImpliedRpm)
//      stallBand = redlineRpm * (0.10 + 0.40 * throttleFraction)  (wider at WOT)
//      slipRatio = clamp(slip / stallBand, 0, 1)
//      pressure  = 1 - sqrt(slipRatio)                            (non-linear K-factor)
//      locked    = (slipRatio < 0.1)
//
// The creep mode (step 1) mimics a real torque converter's fluid coupling:
// even at zero road speed, some torque is transmitted proportional to throttle.
// This prevents the engine from free-revving at standstill while keeping the
// vehicle moving. maxCreepPressure is the clutch pressure at full throttle
// with zero road speed (typical: 0.05-0.15).
SlipLockOutput computeSlipLockPressure(const SlipLockInput& input, double maxCreepPressure = 0.10);

}  // namespace twin

#endif  // TWIN_SLIP_LOCK_CONTROLLER_H
