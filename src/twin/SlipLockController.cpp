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

SlipLockOutput computeSlipLockPressure(const SlipLockInput& input) {
    // Stall-prevention floor: when the road would drag the engine below idle,
    // the clutch must be fully open. This is the lesson from the stall circle.
    if (input.roadSpeedImpliedRpm < input.idleRpm) {
        return SlipLockOutput{0.0, false};
    }

    // Slip is non-negative: if the engine is slower than the road (decel /
    // engine braking), slip is treated as 0 -> the clutch locks.
    const double slip = std::max(0.0, input.engineRpm - input.roadSpeedImpliedRpm);

    // Stall band widens with throttle (10% of redline closed, 50% at WOT).
    const double throttle = clampThrottle(input.throttleFraction);
    const double stallBand = input.redlineRpm * (0.10 + 0.40 * throttle);

    // Guard against a degenerate zero band (e.g. redlineRpm == 0): treat as
    // fully locked rather than divide by zero.
    if (stallBand <= 0.0) {
        return SlipLockOutput{1.0, true};
    }

    const double slipRatio = clampDouble(slip / stallBand, 0.0, 1.0);

    // Non-linear K-factor: pressure rises steeply as slip approaches 0.
    const double pressure = 1.0 - std::sqrt(slipRatio);
    const bool locked = slipRatio < 0.1;

    return SlipLockOutput{pressure, locked};
}

}  // namespace twin
