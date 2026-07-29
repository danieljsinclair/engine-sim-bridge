// SpeedRpmConversion.h - Pure functions for speed↔RPM conversion
//
// These functions are pure (no side effects, no dependencies on mutable state)
// and can be safely used in both production code and tests.

#ifndef TWIN_SPEED_RPM_CONVERSION_H
#define TWIN_SPEED_RPM_CONVERSION_H

#include <cmath>

namespace twin {

// Pure function: compute target RPM from road speed
// Formula: engineRPM = (speedKmh / 3.6) / (2 * PI * tireRadius) * 60 * gearRatio * diffRatio
//
// Parameters:
//   speedKmh: Road speed in km/h
//   gearRatio: Gear ratio for current gear (from transmission)
//   tireRadiusM: Tire radius in meters
//   diffRatio: Differential ratio
//   redlineRpm: Engine redline for clamping (optional, default 0 = no clamp)
//
// Returns: Target engine RPM (0 for invalid inputs, clamped to redline if specified)
inline double computeTargetRpm(
    double speedKmh,
    double gearRatio,
    double tireRadiusM,
    double diffRatio,
    double redlineRpm = 0.0
) {
    // Validate inputs (fail-fast for programmer errors)
    if (speedKmh <= 0.0 || gearRatio <= 0.0 || tireRadiusM <= 0.0 || diffRatio <= 0.0) {
        return 0.0;
    }

    // Convert speed from km/h to m/s
    const double speedMs = speedKmh / 3.6;

    // Wheel angular velocity (rad/s) = v / r
    const double wheelRadPerSec = speedMs / tireRadiusM;

    // Engine angular velocity (rad/s) = wheel speed * gear ratio * diff ratio
    const double engineRadPerSec = wheelRadPerSec * gearRatio * diffRatio;

    // Convert to RPM: rad/s * 60 / (2 * PI)
    const double rpm = engineRadPerSec * 60.0 / (2.0 * M_PI);

    // Clamp to redline if specified (for speed-tracking mode)
    if (redlineRpm > 0.0 && rpm > redlineRpm) {
        return redlineRpm;
    }

    return rpm;
}

} // namespace twin

#endif // TWIN_SPEED_RPM_CONVERSION_H
