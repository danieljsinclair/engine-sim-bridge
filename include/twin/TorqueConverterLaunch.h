// TorqueConverterLaunch.h - Pure torque-converter LAUNCH pressure function.
//
// Companion to SlipLockController.h. SlipLockController computes the lock-up
// pressure once the wheels are coupled to the engine (road-speed-implied RPM
// >= idle). This function computes the clutch pressure for the LAUNCH phase —
// when the wheels are at/near standstill (road-speed-implied RPM < idle) —
// where the plain velocity-match clutch cannot transmit torque without stalling
// the engine.
//
// THE LAUNCH PROBLEM (the velocity-match catch-22):
//   The engine-sim clutch is a velocity-MATCH constraint (J=[0,0,-1,0,0,1],
//   spring m_ks=10) whose impulse is clamped to +/- maxClutchTorque * pressure.
//   At standstill the input shaft is at 0, so any non-trivial pressure yanks the
//   engine toward 0 -> RPM crashes -> stall. Observed: the engine catches at
//   ~1090 RPM then, the moment the clutch engages under a fixed creep pressure,
//   is dragged down to ~105 RPM (near-stall) before the wheels finally move.
//   A FIXED creep/floor pressure loads the engine with a constant torque
//   regardless of engine RPM, so the engine cannot sustain anywhere and crashes.
//
// THE TORQUE-CONVERTER MODEL:
//   A real fluid coupling transmits torque while letting the impeller (engine)
//   spin faster than the turbine (input shaft). The engine settles at its STALL
//   speed — the RPM where its combustion torque equals the converter's reaction
//   torque — instead of being dragged to zero. We emulate that by making the
//   launch pressure RISE WITH engine RPM above idle:
//
//     - Engine near idle  -> pressure ~ 0   (clutch open, engine free to rev up)
//     - Engine at/above stall speed -> pressure -> stallPressureMax (transmits
//       sustainable launch torque, scaled by throttle)
//
//   This is negative feedback: if the clutch loads the engine down toward idle,
//   the pressure drops, the clutch unloads, and the engine recovers — so the
//   engine hovers around a healthy stall RPM while transmitting torque, instead
//   of crashing to zero. More throttle -> more launch torque (like a real TC).
//
//   Once the wheels are coupled (road-speed-implied RPM >= idleRpm), this
//   function returns LAUNCH_PRESSURE_DEFER so the caller falls back to the
//   slip-lock lock-up schedule (SlipLockController) for direct coupling.
//
// This is a pure function with no dependency on engine-sim physics. It is
// applied ONLY in the FREE wheel-coupling branch (PIN pins the wheels directly
// and never launches through the clutch).

#ifndef TWIN_TORQUE_CONVERTER_LAUNCH_H
#define TWIN_TORQUE_CONVERTER_LAUNCH_H

namespace twin {

// Inputs to the launch-pressure computation.
struct LaunchPressureInput {
    double engineRpm;            // Current engine RPM (from feedback).
    double roadSpeedImpliedRpm;  // RPM the engine would be at if locked to road speed in current gear.
    double throttleFraction;     // 0..1.
    double idleRpm;              // Engine idle RPM (e.g. 750).
    double redlineRpm;           // Engine redline RPM (e.g. 6500).
};

// Tunable TC launch characteristic. Defaults are conservative (sustainable).
struct LaunchPressureOptions {
    // Cap on launch clutch pressure. With maxClutchTorque=12000 Nm, 0.06 caps
    // the transmitted launch torque at ~720 Nm — enough to creep the vehicle,
    // not enough to yank the engine down (the stall gating does the rest).
    double stallPressureMax = 0.06;

    // Engine-RPM band ABOVE idle over which the pressure ramps from 0 to
    // stallPressureMax, expressed as a multiple of idleRpm. 1.0 means the full
    // stall pressure is reached at 2*idleRpm (e.g. idle=750 -> stall @ 1500).
    double launchBandFactor = 1.0;
};

// Sentinel returned when the wheels are coupled/rolling and the caller should
// defer to the slip-lock lock-up schedule (direct coupling). Negative so it can
// never be confused with a valid 0..1 pressure.
constexpr double LAUNCH_PRESSURE_DEFER = -1.0;

// Compute the FREE-mode launch clutch pressure.
//
// Returns a pressure in [0, stallPressureMax] while the wheels are at/near
// standstill (roadSpeedImpliedRpm < idleRpm), rising with engine RPM above idle
// and scaled by throttle. Returns LAUNCH_PRESSURE_DEFER once the wheels are
// coupled (roadSpeedImpliedRpm >= idleRpm) so the caller keeps the slip-lock
// lock-up pressure instead.
double computeLaunchPressure(const LaunchPressureInput& input,
                             const LaunchPressureOptions& options = LaunchPressureOptions{});

}  // namespace twin

#endif  // TWIN_TORQUE_CONVERTER_LAUNCH_H
