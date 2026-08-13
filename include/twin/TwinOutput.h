#ifndef TWIN_OUTPUT_H
#define TWIN_OUTPUT_H

#include <simulator/GearConventions.h>

namespace twin {

struct TwinOutput {
    double throttle = 0.0;
    int gear = 0;
    double clutchPressure = 1.0;
    bool starterMotor = false;
    bool ignition = false;
    bridge::GearSelector gearSelector = bridge::GearSelector::NEUTRAL;
    // Vehicle-speed pin to surface to the simulator (-1.0 = don't pin). Set by
    // the wheel-coupling strategy in RUNNING: FREE leaves -1 (sim speed
    // independent, diagnostic visible); PIN pins the sim speed to the CSV speed.
    double pinVehicleSpeedTargetKmh = -1.0;
    // Dyno torque scale (0 = off, >0 = braking load). FREE mode cranking sets
    // this to give the starter a resistive load so RPM can build; PIN mode
    // leaves it 0 because its vehicle-speed constraint already provides load.
    double dynoTorqueScale = 0.0;
    // MATCH (Torque) mode: recorded drivetrain torque (Nm) to inject at the
    // transmission input so the solver integrates road speed from it. 0.0 in
    // FREE/PIN (no injection — a true no-op on the rotating mass).
    double drivetrainInputTorqueNm = 0.0;
    // Diagnostics (surfaced to presentation for the inline clutch readout and
    // the CSV-out spelunking path). roadImpliedRpm is the RPM the engine would
    // be at if locked to the wheels in the current gear; creepReliefFired is
    // true on the frame the creep-drag relief opened the clutch (pressure 0).
    double roadImpliedRpm = 0.0;
    bool creepReliefFired = false;
};

}

#endif
