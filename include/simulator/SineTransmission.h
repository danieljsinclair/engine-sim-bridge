// SineTransmission.h - Minimal Transmission stub for sine wave mode
#ifndef ENGINE_SIM_BRIDGE_SINE_TRANSMISSION_H
#define ENGINE_SIM_BRIDGE_SINE_TRANSMISSION_H

#include "transmission.h"
#include "units.h"

/// @brief A minimal Transmission implementation for sine wave mode, where throttle directly controls RPM with no lag.
/// Essentially a dummy transmission that does nothing but allows the plumbing to function
class SineTransmission : public Transmission {
public:
    SineTransmission() {
        static const double gearRatios[] = {1.0};
        Parameters p = {};
        p.GearCount         = 1;
        p.GearRatios        = gearRatios;
        p.MaxClutchTorque   = units::torque(1000, units::Nm);
        initialize(p);
    }
};

#endif // ENGINE_SIM_BRIDGE_SINE_TRANSMISSION_H
