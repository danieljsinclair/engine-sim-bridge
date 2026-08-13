// ClutchMapModel.cpp - The declarative governor pressure curve.
//
// Evaluation is a single C1-continuous expression (no branches that select
// between disjoint pressure regimes): a smoothstep blend of a lock-up curve and
// a creep curve. See ClutchMapModel.h for the full rationale.

#include <twin/ClutchMapModel.h>

#include <algorithm>
#include <cmath>

namespace twin {

namespace {

// Clamp to [0,1] (the physical clutch-pressure / normalized-throttle range).
inline double clampUnit(double v) { return std::max(0.0, std::min(v, 1.0)); }

// Hermite smoothstep: 3t^2 - 2t^3. C1-continuous on [0,1], flat (0/1) at the
// ends so the blend and the lock ramp ease in/out with no kink. The standard
// smooth ramp used throughout the bridge's pressure schedules.
inline double smoothstep(double t) {
    t = clampUnit(t);
    return t * t * (3.0 - 2.0 * t);
}

}  // namespace

ClutchMapModel::ClutchMapModel(ClutchMapParameters params) : params_(params) {}

CouplingOutput ClutchMapModel::compute(const CouplingInput& input) {
    const ClutchMapParameters& p = params_;

    // Degenerate idle (e.g. unconfigured profile) -> fully locked, avoid div-by-0.
    if (input.idleRpm <= 0.0) {
        return CouplingOutput{1.0, true};
    }

    const double throttle = clampUnit(input.throttleFraction);

    // --- Creep pressure (sub-idle / standstill regime): a floored, throttle-
    //     modulated pressure (a torque-converter-like fluid coupling at stall).
    //     The FLOOR is essential: it keeps the clutch transmitting torque at all
    //     times so an unloaded engine cannot free-rev to redline (pressure 0
    //     decouples the engine, and the idle-sustain throttle then free-revs it —
    //     measured). The floor DOES mean that under PIN-pinned wheels decelerating
    //     to a stop the engine is dragged down with them (a friction-clutch
    //     cannot slip-hold at idle the way a torque converter's fluid does); the
    //     twin's stall-recovery re-cranks it and it tracks cleanly once the road
    //     moves again (see report). The throttle term adds launch torque.
    const double creepPressure =
        std::min(p.pressureFloor + p.creepThrottleGain * throttle, p.creepCeiling);

    // --- Lock-up pressure (rolling regime): smooth ramp floor -> 1 over
    //     [idle, idle*lockEngageIdleFactor]. Driven off ROAD-implied rpm (not
    //     slip) so a free-revving engine cannot deadlock the pressure low (the
    //     old cruise free-rev bug).
    const double lockRpm = input.idleRpm * p.lockEngageIdleFactor;
    const double lockSpan = std::max(lockRpm - input.idleRpm, 1.0);  // guard div-by-0
    const double lockT = clampUnit((input.roadSpeedImpliedRpm - input.idleRpm) / lockSpan);
    const double lockPressure =
        p.pressureFloor + (1.0 - p.pressureFloor) * smoothstep(lockT);

    // --- Blend: creep owns the sub-idle regime (lockWeight 0 below idle); the
    //     lock ramp takes over across [idle, lock]. smoothstep on lockT makes the
    //     whole expression C1-continuous AND branchless (no if/else, no threshold)
    //     — there is nothing to bang between, so no limit cycle can form. The
    //     legacy binary relief cycled 0<->redline precisely because it was a 0/1
    //     threshold; this curve replaces the discontinuity with a continuous map.
    const double lockWeight = smoothstep(lockT);  // 0 below idle .. 1 at lock
    const double pressure = creepPressure + (lockPressure - creepPressure) * lockWeight;

    const bool locked = input.roadSpeedImpliedRpm >= lockRpm;
    return CouplingOutput{clampUnit(pressure), locked};
}

}  // namespace twin
