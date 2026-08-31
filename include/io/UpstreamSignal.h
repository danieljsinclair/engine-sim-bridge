#ifndef UPSTREAM_SIGNAL_H
#define UPSTREAM_SIGNAL_H

#include <cstdint>
#include <optional>

#include "simulator/GearConventions.h"

namespace input {

struct UpstreamSignal {
    double throttleFraction = 0.0;
    double speedKmh = 0.0;
    double accelerationG = 0.0;
    double brakeFraction = 0.0;
    double motorTorqueNm = 0.0;       // Recorded motor/engine torque (Nm) for MATCH mode
    uint64_t timestampUtcMs = 0;
    bool isValid = false;

    // Live start/stop feed (VEHICLE_START_STOP): the selected gear position, and
    // the brake-light state (VCLEFT_brakeLightStatus; true = pedal pressed).
    // nullopt = signal absent (treated as not pressed). The provider translates
    // these to (brakePressed, gear) before calling the VehicleStartController.
    // Defaults are neutral / not-pressed.
    bridge::GearSelector gearSelector = bridge::GearSelector::NEUTRAL;
    std::optional<bool> brakeLight;

    // Steering wheel angle in degrees (signed; CAN 0x129 SCCM_steeringAngle).
    // nullopt = signal absent. Display-only: flows to EngineState.Controls for
    // the CLI console steering readout; never touches physics.
    std::optional<double> steeringAngleDeg;
};

}

#endif
