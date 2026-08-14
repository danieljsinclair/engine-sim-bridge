#ifndef VEHICLE_START_CONTROLLER_H
#define VEHICLE_START_CONTROLLER_H

// VehicleStartController.h - vehicle-driven start/stop DECISION layer.
//
// SRP: owns exactly two decision bits (engineOn, stopLatch) plus the crank
// delay timer, and drives an injected IEngineActuator. It does NOT reimplement
// the RPM-catch phase logic of CrankingController (a lower layer), nor does it
// reach into gearbox / twin internals, nor the manual input path.
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
//            brake pressed AND gear == PARK -> ignition off, stopLatch set.
//   LATCH  blocks START while set; clears on the first tick with no start
//            demand (brake released AND not in a drive gear).

#include "input/IEngineActuator.h"
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

class VehicleStartController {
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
    bool stopLatch_ = false;         // P+B stop fired; blocks restart until released
    bool crankPending_ = false;      // starter engaged, ignition not yet fired
    bool crankFromBrakeOnly_ = false; // crank was initiated without a drive gear
    double crankAccumS_ = 0.0;       // accumulated dt while crank delay is pending
};

} // namespace input

#endif // VEHICLE_START_CONTROLLER_H
