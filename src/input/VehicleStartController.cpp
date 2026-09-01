#include "input/VehicleStartController.h"

namespace input {

VehicleStartController::VehicleStartController(IEngineActuator& actuator,
                                               double crankDelayS)
    : actuator_(actuator)
    , crankDelayS_(crankDelayS) {
}

void VehicleStartController::update(double dt,
                                    bool brakePressed,
                                    bridge::GearSelector gear) {
    if (dt < 0.0) {
        dt = 0.0; // clamp a negative runtime dt; never silently swallow bugs elsewhere
    }

    const bool driveSelected =
        (gear == bridge::GearSelector::DRIVE ||
         gear == bridge::GearSelector::REVERSE);

    // Edge detection (owner 2026-08-30): the stop trigger is the brake-light
    // ON->OFF transition, not the level. The history advances every tick, first
    // thing, so every exit path below observes the same per-tick edge.
    const bool brakeReleased = brakeWasPressed_ && !brakePressed;
    brakeWasPressed_ = brakePressed;

    // Drive-since-start history for the stop gate below. A gear-initiated start
    // seeds the flag in beginStart; any D/R seen later in the run sets it too.
    if (engineOn_ && driveSelected) {
        driveSelectedSinceStart_ = true;
    }

    // STOP (only while RUNNING, i.e. after the crank has completed): the brake
    // RELEASED in PARK turns the engine off and latches the stop. A brake press
    // in PARK alone never cuts ignition — the driver may be pressing it to
    // select a gear and drive off; if they shift out of PARK before releasing,
    // the release edge lands outside PARK and no stop occurs (drive-off).
    // Three gates: brakeReleased so only the ON->OFF transition fires (a held
    // brake, pressed or unpressed, is not an event); !crankPending_ so a brake
    // release during the crank delay stays a normal start rather than a stop;
    // driveSelectedSinceStart_ so the brake that PERFORMED a brake-initiated
    // start in PARK cannot stop the engine it just cranked the moment the
    // driver's foot leaves the pedal — the release-edge cut applies only after
    // a drive gear has been selected in the current run (the plan's
    // PARK-after-motion item).
    if (engineOn_ && !crankPending_ && brakeReleased &&
        gear == bridge::GearSelector::PARK && driveSelectedSinceStart_) {
        stopEngine();
        return; // stop takes precedence over any simultaneous start trigger
    }

    // Latch release: clears on the first tick with the brake NOT pressed — brake
    // release alone (plan: `stopLatch && !brake`). If a drive gear is selected,
    // the same tick also satisfies START below (gear trigger) and restarts the
    // engine: releasing the brake in DRIVE is a drive-off, not a stay-off.
    if (stopLatch_ && !brakePressed) {
        stopLatch_ = false;
    }

    // START: off + not latched + (brake OR drive gear).
    if (!engineOn_ && !stopLatch_ && (brakePressed || driveSelected)) {
        beginStart(driveSelected);
        if (crankPending_) {
            // Normal delayed crank: advance the timer on THIS and every trigger
            // tick until ignition fires.
            advanceCrank(dt, driveSelected);
        }
        return;
    }

    // Already cranking: advance the timer even when there is no fresh trigger on
    // this tick (e.g. brake held, gear unchanged). A brake-only crank stays
    // pending while the brake is held; selecting a drive gear while pending
    // fast-forwards ignition (safety).
    if (crankPending_) {
        advanceCrank(dt, driveSelected);
    }
}

void VehicleStartController::beginStart(bool driveSelected) {
    if (!engineOn_) {
        actuator_.setStarterMotor(true);
        // New engine run: drive history restarts with it. A gear-initiated start
        // counts as drive-since-start immediately (the selector IS in D/R), so a
        // brake release in PARK after shifting back out of gear cuts the engine.
        driveSelectedSinceStart_ = driveSelected;
    }
    engineOn_ = true;
    if (driveSelected) {
        // Gear-initiated start: ignition fires on the SAME update as the
        // starter (t=0, no crank delay) and no timer is armed. Any start
        // demand that includes a drive gear is instant — even when the brake
        // is also pressed (the gear dominates the delay rule).
        fireIgnition();
        return;
    }
    if (!crankPending_) {
        // Brake-initiated crank: accumulate crankDelayS of dt, then ignite.
        // Only this path arms the timer, so a pending crank is always
        // brake-only; a drive gear arriving mid-crank fast-forwards ignition.
        crankPending_ = true;
        crankAccumS_ = 0.0;
        crankFromBrakeOnly_ = true;
    }
}

void VehicleStartController::advanceCrank(double dt, bool driveSelected) {
    if (crankFromBrakeOnly_ && driveSelected) {
        // Safety: ignition fires immediately once the car is put in gear during
        // a brake-initiated crank; cancel the pending timer.
        fireIgnition();
        return;
    }
    crankAccumS_ += dt;
    if (crankAccumS_ >= crankDelayS_) {
        fireIgnition();
    }
}

void VehicleStartController::fireIgnition() {
    actuator_.setIgnition(true);
    crankPending_ = false;
    crankAccumS_ = 0.0;
    crankFromBrakeOnly_ = false;
}

void VehicleStartController::stopEngine() {
    actuator_.setIgnition(false);
    actuator_.setStarterMotor(false);
    engineOn_ = false;
    stopLatch_ = true;
    crankPending_ = false;
    crankFromBrakeOnly_ = false;
    crankAccumS_ = 0.0;
    driveSelectedSinceStart_ = false; // run over: the next start re-arms the gate
}

} // namespace input
