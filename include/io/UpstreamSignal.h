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

    // The throttle comes from a recorded/real vehicle trace (replay CSV, live
    // TCP/CSV stream) rather than a scripted or human driver. Contract: on a
    // trace-driven run the trace is GROUND TRUTH — the twin and the crank path
    // must not synthesize a start character the recording never contained (the
    // startup-flare bug: CRANKING_THROTTLE 0.6 overrode a trace 0.00 at a
    // standstill start and the unloaded catch flared to full scale). When
    // traceDriven, crank throttles are floored at the trace value; scripted/
    // keyboard runs (default false) keep the absolute catch-guarantee floors,
    // byte-identical to the pre-flag behavior.
    bool traceDriven = false;

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
