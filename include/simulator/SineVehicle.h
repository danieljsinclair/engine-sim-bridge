// SineVehicle.h - Minimal Vehicle stub for sine wave mode
#ifndef ENGINE_SIM_BRIDGE_SINE_VEHICLE_H
#define ENGINE_SIM_BRIDGE_SINE_VEHICLE_H

#include "vehicle.h"
#include "units.h"

/// @brief A minimal Engine implementation for sine wave mode, where throttle directly controls RPM with no lag.
/// Essentially a dummy engine that does nothing but allows the plumbing to function
class SineVehicle : public Vehicle {
public:
    SineVehicle() {
        Parameters p = {};
        p.mass       = units::mass(1000, units::kg);
        p.diffRatio  = 1.0;
        p.tireRadius = units::distance(0.3, units::m);
        initialize(p);
    }
};

#endif // ENGINE_SIM_BRIDGE_SINE_VEHICLE_H
