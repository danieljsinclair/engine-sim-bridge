// SlipLockController.cpp - Implementation of the pure slip-lock pressure function.

#include <twin/SlipLockController.h>

#include <algorithm>
#include <cmath>

namespace twin {

namespace {

// Clamp a value into [lo, hi].
inline double clampDouble(double v, double lo, double hi) {
    return std::max(lo, std::min(v, hi));
}

// Clamp throttle into its physical 0..1 range so out-of-range callers cannot
// widen the stall band beyond its intended envelope.
inline double clampThrottle(double throttle) {
    return clampDouble(throttle, 0.0, 1.0);
}

}  // namespace

// The clutch MUST NEVER be fully open (pressure 0): at zero pressure the engine
// decouples from the road and free-revs to the redline. The floor
// (kSlipLockPressureFloor, ~0.05-0.15) keeps enough clutch clamp engaged at all
// times to transmit torque while still allowing slip (the engine can rev on
// launch/transients via partial pressure rather than being rigidly locked to the
// wheels). 0.10 chosen so the clutch always carries meaningful torque.

SlipLockOutput computeSlipLockPressure(const SlipLockInput& input, double maxCreepPressure) {
    // Creep mode: when road speed would imply below-idle RPM (standstill or
    // very low speed), apply a small clutch pressure proportional to throttle.
    // This mimics a real torque converter's fluid coupling — even at stall,
    // some torque is transmitted. The engine feels load and doesn't free-rev.
    // maxCreepPressure is typically 0.05-0.15 (5-15% clutch at full throttle).
    if (input.roadSpeedImpliedRpm < input.idleRpm) {
        const double throttle = clampThrottle(input.throttleFraction);
        if (const double creep = throttle * clampDouble(maxCreepPressure, 0.0, 1.0);
            creep > 0.0) {
            // Floor the creep too: even stalled creep must keep the clutch
            // engaged enough to load the engine and never fully open.
            return SlipLockOutput{std::max(creep, kSlipLockPressureFloor), false};
        }
        return SlipLockOutput{kSlipLockPressureFloor, false};
    }

    // Rolling band: the wheels are turning (road-implied >= idle). Drive the
    // clutch LOCK off ROAD SPEED, not slip — this is the fix for the cruise
    // free-rev bug. The old model set pressure from (engine - road) slip, which
    // deadlocks: a high engine RPM -> large slip -> low pressure -> engine revs
    // higher -> larger slip, so at cruise the engine free-revved uncoupled with
    // no engine braking. Keying the lock off road-implied RPM breaks the circle:
    // once the wheels are rolling the clutch locks regardless of how far the
    // engine has over-revved, and the lock drags engine RPM back down to
    // road×gear×diff (loaded, with engine braking restored).
    //
    // The lock-engage point is IDLE-RELATIVE (kLockEngageIdleFactor × idle), not
    // redline-relative: the engine must couple as soon as the wheels can sustain
    // idle (road-implied > idle), so it tracks the road instead of drooping back
    // to idle (the low-speed silence/droop bug). The prior redline-fraction
    // threshold left a wide low-pressure slip band [idle..1170] in which the
    // engine idled decoupled at 4-5 mph; idle-relative engagement couples it
    // firmly by ~5 mph while preserving a narrow launch slip band.
    const double lockRpm = input.idleRpm * kLockEngageIdleFactor;
    const double band = lockRpm - input.idleRpm;

    // Guard against a degenerate band (e.g. redlineRpm <= idle): treat as fully
    // locked rather than divide by zero.
    if (band <= 0.0) {
        return SlipLockOutput{1.0, true};
    }

    // Pressure ramps from the floor (at idle) up to 1.0 (at the lock-engage
    // point) as the wheels roll faster. Independent of engine RPM, so a
    // spinning-up / over-revving engine cannot deadlock the pressure to the
    // floor (the old bug). The floor keeps the clutch NEVER fully open.
    const double t = clampDouble(
        (input.roadSpeedImpliedRpm - input.idleRpm) / band, 0.0, 1.0);
    const double pressure = kSlipLockPressureFloor +
                            (1.0 - kSlipLockPressureFloor) * t;

    // Over-redline bound (iter2): the clutch must NEVER lock while the engine is
    // at or above the redline. If the engine has been driven to the redline (the
    // slip-lock alone must not be the thing that pins it there), force the
    // clutch to slip (pressure capped at the floor, never locked) so the engine
    // can shed speed under the rev limiter instead of being rigidly held at
    // redline. This is the hard ceiling that complements the floor. The road-
    // speed lock still applies below the redline: below redline the lock is
    // driven purely by road speed (the line above), which is what drags a
    // free-revving engine back down to the road.
    const bool overRedline = (input.redlineRpm > 0.0) &&
                             (input.engineRpm >= input.redlineRpm);
    const bool locked = overRedline ? false : (input.roadSpeedImpliedRpm >= lockRpm);

    return SlipLockOutput{overRedline ? kSlipLockPressureFloor : pressure, locked};
}

}  // namespace twin
