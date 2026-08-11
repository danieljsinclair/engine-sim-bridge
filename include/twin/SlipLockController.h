// SlipLockController.h - Pure torque-converter slip-lock pressure function.
//
// A friction clutch in engine-sim transmits torque linearly with pressure. At
// standstill any non-zero pressure loads the engine against the speed-pinned
// wheels -> RPM drops -> stall. With zero pressure the engine free-revs ->
// redline at any throttle. This function computes a clutch pressure that
// resolves that circle:
//
//   - At standstill (road-implied < idle):    floor pressure (engine loaded, no stall)
//   - At launch under throttle (high slip):   partial pressure (slip, engine can rev)
//   - As road catches up (slip -> 0):         pressure -> 1.0 (locked, direct coupling)
//   - On decel (engine slower than road):     pressure == 1.0 (locked, engine braking)
//
// HARD RULE: the clutch pressure is NEVER fully open (0). A pressure floor
// (kSlipLockPressureFloor, ~0.05-0.15) is applied in every branch because a
// fully-open clutch decouples the engine from the road and lets it free-rev to
// the redline. The floor keeps the clutch transmitting torque at all times while
// still allowing slip (the engine revs on launch via partial pressure instead of
// being rigidly locked, and never fully opens). Below idle the clutch is kept at
// the floor (or creep pressure, itself floored) so the engine is loaded, not
// free. The TC characteristic only applies above the idle floor.
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

// Minimum clutch pressure floor (see kSlipLockPressureFloor in the .cpp). The
// clutch MUST NEVER be fully open (0): a zero pressure decouples the engine from
// the road and lets it free-rev to redline. Exposed so callers/tests can assert
// the floor without re-deriving the magic constant.
//
// iter2: lowered from 0.10 to 0.05. 0.10 over-loaded the engine at standstill
// (zero throttle) and dragged RPM below idle (the ~328-488 "over-quiet" lug in
// the PIN replay). 0.05 still keeps the clutch transmitting torque (never fully
// open, so no free-rev) but stops over-loading the engine at idle, letting RPM
// settle at/above idle. The ratio fix (C63 diff 2.82) also cut the reflected
// wheel load, so 0.05 is ample to prevent free-rev.
constexpr double kSlipLockPressureFloor = 0.05;

// Cruise-lock threshold, as a fraction of redline. Once the road-implied RPM
// (the RPM the engine WOULD be at if locked to the wheels in the current gear)
// exceeds this, the wheels are genuinely rolling at cruise speed and the clutch
// LOCKS (pressure -> 1.0): the engine is loaded and forced to track
// road×gear×diff. Below it (standstill + low-speed launch) the clutch slips with
// partial pressure so the engine can rev.
//
// DRIVING THE LOCK OFF ROAD SPEED (not slip) IS THE FIX for the free-rev bug:
// the old model set pressure from (engine - road) slip, which deadlocks — a
// high engine RPM produces a large slip, which produces a LOW pressure, which
// lets the engine rev HIGHER, producing even more slip. At cruise the engine
// therefore free-revved uncoupled (the redline/free-rev symptom) with no engine
// braking. Keying the lock off road-implied RPM breaks the circle: once the
// wheels are turning at cruise, the clutch locks regardless of how far the
// engine has over-revved, and the lock drags the engine RPM back down to the
// road. ~0.18 of a 7250 redline ≈ 1305 RPM, comfortably above idle (750) so
// standstill/launch still slips, and at/above a 40 mph cruise (implied ≈ 1350)
// so the engine is locked, not free-revving.
constexpr double kCruiseLockRpmFraction = 0.18;

// Compute clutch pressure from slip. Algorithm:
//   1. If roadSpeedImpliedRpm < idleRpm:
//        - creepPressure = throttleFraction * maxCreepPressure  (TC fluid coupling at stall)
//        - If creepPressure > 0: return {max(creepPressure, floor), false}  (floored creep)
//        - Otherwise:            return {floor, false}            (floored, engine loaded)
//   2. Rolling band (roadSpeedImpliedRpm >= idleRpm): the wheels are turning.
//      Drive the lock off ROAD SPEED, not slip, so a high engine RPM cannot
//      deadlock the pressure to the floor (the old free-rev bug). Pressure ramps
//      from the floor (at idle) up to 1.0 as road-implied rises to the cruise
//      threshold (kCruiseLockRpmFraction * redline), then locks hard:
//        cruiseRpm = redlineRpm * kCruiseLockRpmFraction
//        t         = clamp((roadSpeedImpliedRpm - idleRpm) / (cruiseRpm - idleRpm), 0, 1)
//        pressure  = floor + (1 - floor) * t
//        locked    = (roadSpeedImpliedRpm >= cruiseRpm)
//      Once locked, the engine is loaded and tracks road×gear×diff (no free-rev,
//      engine braking restored).
//
// The creep mode (step 1) mimics a real torque converter's fluid coupling:
// even at zero road speed, some torque is transmitted proportional to throttle.
// This prevents the engine from free-revving at standstill while keeping the
// vehicle moving. maxCreepPressure is the clutch pressure at full throttle
// with zero road speed (typical: 0.05-0.15).
SlipLockOutput computeSlipLockPressure(const SlipLockInput& input, double maxCreepPressure = 0.10);

}  // namespace twin

#endif  // TWIN_SLIP_LOCK_CONTROLLER_H
