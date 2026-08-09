// TorqueConverterLaunch.cpp - Implementation of the pure launch-pressure function.

#include <twin/TorqueConverterLaunch.h>

#include <algorithm>

namespace twin {

double computeLaunchPressure(const LaunchPressureInput& input,
                             const LaunchPressureOptions& options) {
    // Wheels coupled/rolling -> defer to the slip-lock lock-up schedule.
    if (input.roadSpeedImpliedRpm >= input.idleRpm) {
        return LAUNCH_PRESSURE_DEFER;
    }

    // Clamp throttle to its physical 0..1 range so an out-of-range caller cannot
    // push the launch torque beyond its intended envelope.
    const double throttle = std::clamp(input.throttleFraction, 0.0, 1.0);

    // Negative feedback: pressure ramps from 0 at idle to the stall cap as the
    // engine reaches its stall speed. If the clutch loads the engine down toward
    // idle the pressure drops, the clutch unloads, and the engine recovers -- so
    // the engine hovers near its stall RPM while transmitting torque, instead of
    // being yanked to zero (the velocity-match catch-22).
    const double launchBand = std::max(1.0, input.idleRpm * options.launchBandFactor);
    const double loadFactor = std::clamp((input.engineRpm - input.idleRpm) / launchBand, 0.0, 1.0);
    return loadFactor * throttle * options.stallPressureMax;
}

}  // namespace twin
