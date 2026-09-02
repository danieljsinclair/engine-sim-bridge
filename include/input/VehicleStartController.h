#ifndef VEHICLE_START_CONTROLLER_H
#define VEHICLE_START_CONTROLLER_H

// VehicleStartController.h - the SINGLE shared start/ignition state machine.
//
// SRP: owns exactly two decision bits (engineOn, stopLatch) plus the crank
// delay timer, and drives an injected IEngineActuator. It does NOT reimplement
// the RPM-catch phase logic of CrankingController (a lower layer), nor does it
// reach into gearbox / twin internals.
//
// This is the seam every frontend calls through (owner spec 2026-09-02):
//   - iOS app start button            -> requestCombinedStart / requestIgnition
//   - CLI keyboard S (starter)        -> requestStarter
//   - CLI keyboard I (ignition)       -> requestIgnition
//   - CLI --start flag                -> requestCombinedStart
//   - CSV auto path (live/replay)     -> update(brake, gear)  [gear-driven]
// Each frontend is a thin caller; the decision logic lives ONLY here.
//
// Composable primitives (the three atomic operations every caller uses):
//   requestIgnition(bool on)  - set ignition ON/OFF directly (CLI/iOS separate
//                               ignition control: the user can elongate cranking
//                               for as long as they want, crank without ignition).
//   requestStarter(bool on)   - engage/release the starter motor (the McLaren
//                               mod: engage starter a moment BEFORE ignition —
//                               starter-then-ignition sequencing, not assumed
//                               combined-only).
//   requestCombinedStart()    - the combined operation the CSV path uses by
//                               design: fire starter+ignition together (a drive
//                               gear or --start is a one-shot "start the engine"
//                               demand). Brake-initiated cranks still honor the
//                               crank delay (starter-then-ignition with delay).
//
// Decision rules (derived from VEHICLE_START_STOP_PLAN.md + the test cases):
//   START  (only if engine off AND not latched):
//            brake pressed OR gear in {DRIVE, REVERSE} (PARK does not start).
//            -> a start demand that INCLUDES a drive gear fires the starter AND
//               ignition together on the same update (t=0, no crank delay; no
//               timer is armed). A brake-only start engages the starter
//               immediately and ignites after crankDelayS of ACCUMULATED dt; a
//               drive gear selected while that crank is still pending ignites
//               immediately and cancels the timer (safety: the engine must not
//               finish cranking after the car is already moving).
//   STOP   (only while RUNNING, i.e. after the crank has completed):
//            brake RELEASED (light ON->OFF edge) while gear == PARK AND a drive
//            gear (D/R) has been selected at least once since the current
//            engine run started (drive-since-start gate — the plan's deferred
//            PARK-after-motion item resolved via gear history). A brake PRESS
//            in PARK never cuts ignition (the driver may be about to select a
//            gear); shifting out of PARK before the release is a drive-off and
//            no stop occurs. Without the drive gate, the brake that performed
//            a brake-initiated start in PARK would stop the engine it just
//            cranked the moment the pedal is released.
//   LATCH  blocks START while set; releases on brake release ALONE (plan:
//            `stopLatch && !brake`). While a drive gear is selected, the tick
//            that releases the latch also satisfies START (gear trigger) and
//            restarts the engine — releasing the brake in DRIVE is a drive-off,
//            not a stay-off.

#include "input/IEngineActuator.h"
#include "input/IStartController.h"
#include "simulator/GearConventions.h"

namespace input {

// Minimal actuator the controller drives so its caller can READ the level
// decisions. It is observer-only: nothing downstream consumes it; the caller
// (SimulationLoop) translates the levels into EngineInput fields. This keeps
// the real actuator (BridgeSimulator) exclusively under CrankingController's
// authority — the start/stop decision only ever flattens into EngineInput.
class ObserverActuator : public IEngineActuator {
public:
    void setIgnition(bool on) override { ignition_ = on; }
    void setStarterMotor(bool on) override { starter_ = on; }

    bool ignition_ = false;
    bool starter_ = false;
};

class VehicleStartController : public IStartController {
public:
    // Default crank delay (seconds) between starter engagement and ignition.
    static constexpr double kDefaultCrankDelayS = 0.5;

    explicit VehicleStartController(IEngineActuator& actuator,
                                    double crankDelayS = kDefaultCrankDelayS);

    // Advance the decision by one frame. dt is the elapsed time (seconds);
    // brakePressed is the honest boolean derived from the brake-light signal
    // (true => pressed; false/absent => not pressed); gear is the selector
    // position.
    void update(double dt, bool brakePressed, bridge::GearSelector gear);

    // ---- Composable primitives (the single seam every frontend calls) ----
    // Direct ignition control (CLI/iOS separate ignition: the user can elongate
    // cranking, crank with ignition OFF). Sets the ignition level and clears
    // any pending crank timer (an explicit ignition command resolves the crank).
    void requestIgnition(bool on);

    // Direct starter control (McLaren mod: engage starter a moment BEFORE
    // ignition — starter-then-ignition sequencing). Engages the starter motor;
    // does NOT fire ignition (the combined path is requestCombinedStart).
    void requestStarter();

    // Combined start (the CSV path's by-design operation, and --start): fire
    // starter+ignition together as a one-shot "start the engine" demand.
    void requestCombinedStart();

    bool isEngineOn() const { return engineOn_; }
    bool isStopLatched() const { return stopLatch_; }

private:
    void beginStart(bool driveSelected);
    void advanceCrank(double dt, bool driveSelected);
    void fireIgnition();
    void stopEngine();

    IEngineActuator& actuator_;
    const double crankDelayS_;

    bool engineOn_ = false;          // sim has been started (starter engaged onward)
    bool stopLatch_ = false;         // park release-edge stop fired; blocks restart until released
    bool crankPending_ = false;      // starter engaged, ignition not yet fired
    bool crankFromBrakeOnly_ = false; // crank was initiated without a drive gear
    double crankAccumS_ = 0.0;       // accumulated dt while crank delay is pending
    bool driveSelectedSinceStart_ = false; // a D/R gear was seen in the current run
    bool brakeWasPressed_ = false;   // previous frame's brake light (release-edge detection)
};

} // namespace input

#endif // VEHICLE_START_CONTROLLER_H
