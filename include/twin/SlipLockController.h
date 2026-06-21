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
//
//   1. Creep mode (roadSpeedImpliedRpm < idleRpm):
//        Models a real torque converter's fluid coupling at stall/low speed.
//        Unlike a friction clutch (binary lock/slip), a TC transmits torque
//        proportional to slip — maximum at zero road speed, tapering to zero
//        as engine and turbine speeds converge.
//
//        slip = max(0, engineRpm - roadSpeedImpliedRpm)
//        slipRatio = clamp(slip / redlineRpm, 0, 1)
//
//        // TC coupling coefficient: 1.0 at max slip, 0.0 at zero slip
//        tcCoupling = 1.0 - slipRatio
//
//        // Throttle scaling: baseline coupling even at light throttle,
//        // stronger at high throttle. At 0% throttle, ~30% of max coupling.
//        throttleScale = 0.3 + 0.7 * throttleFraction
//
//        // Final creep pressure
//        creep = maxCreepPressure * tcCoupling * throttleScale
//
//        if creep > 0.001: return {clamp(creep, 0, 1), false}
//        else:             return {0.0, false}
//
//   2. Normal slip mode (roadSpeedImpliedRpm >= idleRpm):
//      slip      = max(0, engineRpm - roadSpeedImpliedRpm)
//      stallBand = redlineRpm * (0.10 + 0.40 * throttleFraction)
//      slipRatio = clamp(slip / stallBand, 0, 1)
//      pressure  = 1.0 - sqrt(slipRatio)
//      locked    = (slipRatio < 0.1)
//
// maxCreepPressure: peak clutch pressure at full throttle, zero road speed,
// max slip. Typical range 0.10-0.25. Higher = more engine loading at stall
// but risk of stall at high throttle.
SlipLockOutput computeSlipLockPressure(const SlipLockInput& input, double maxCreepPressure = 0.35);

}  // namespace twin

#endif  // TWIN_SLIP_LOCK_CONTROLLER_H
