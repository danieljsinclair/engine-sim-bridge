#ifndef TWIN_AUTOMATIC_GEARBOX_H
#define TWIN_AUTOMATIC_GEARBOX_H

#include <twin/IceVehicleProfile.h>
#include <twin/IGearboxLogger.h>
#include <simulator/GearConventions.h>

namespace twin {

// A declarative automatic gearbox modelled on a 1960s hydraulic unit: a speed
// governor, a throttle valve and a spring-detented shift valve. The shift
// tables are the sole authority — given speed and throttle they interpolate the
// next-gear threshold — and the separate downshift table provides hysteresis by
// construction (its thresholds sit below the upshift thresholds for every gear
// and throttle, so a single speed can never satisfy both and hunting is
// impossible). The only override is kickdown: a sudden large throttle increase
// forces one RPM-safe downshift. A short dwell after any shift suppresses
// sub-frame oscillation; kickdown is exempt.
class AutomaticGearbox {
public:
    explicit AutomaticGearbox(const IceVehicleProfile& profile);

    // Legacy 3-arg update (throttle/speed only). Equivalent to the torque-aware
    // update with drivetrainTorqueNm = 0 and selector = DRIVE.
    void update(double dt, double speedKmh, double throttleFraction);

    // Torque- and selector-aware update. Torque is accepted for API symmetry
    // with the closed-loop twin but does not influence the declarative shift
    // decision — the tables and kickdown fully specify behaviour. In
    // NEUTRAL/PARK/REVERSE no shift is requested and the current gear holds.
    void update(double dt, double speedKmh, double throttleFraction,
                double drivetrainTorqueNm);

    int getCurrentGear() const;
    bool requestsShift() const;
    int getTargetGear() const;
    bool isInKickdown() const;

    void setLogger(IGearboxLogger* logger) { logger_ = logger; }
    IGearboxLogger* getLogger() const { return logger_; }
    void setTwinContext(int twinState, double clutchPressure, double speedFeedbackKmh, double rpmFeedback) {
        twinState_ = twinState;
        clutchPressureFeedback_ = clutchPressure;
        speedFeedbackKmh_ = speedFeedbackKmh;
        rpmFeedback_ = rpmFeedback;
    }
    // Selector drives the neutral/park hold behaviour (AC6). Forward shifting
    // only happens in DRIVE (or a manual forward position).
    void setGearSelector(bridge::GearSelector s) { selector_ = s; }
    bridge::GearSelector getGearSelector() const { return selector_; }

    int getLastShiftDirection() const { return lastShiftDirection_; }

private:
    // A single frame's declarative shift verdict.
    struct ShiftDecision {
        int gear;        // resolved target gear
        int direction;   // +1 upshift, -1 downshift, 0 hold
        bool kickdown;   // true when the shift was a kickdown override
    };

    const IceVehicleProfile& profile_;
    int currentGear_ = 1;
    int targetGear_ = 1;
    bool requestsShift_ = false;
    double timeSinceLastShiftS_ = 0.0;
    bool hasShiftedBefore_ = false;
    int lastShiftDirection_ = 0;

    // Throttle input conditioning: a one-pole low-pass rejects sub-percent CAN
    // jitter so the table input is stable, and a rolling delta within the
    // kickdown window detects sudden pedal stabs.
    double smoothedThrottle_ = -1.0;
    double previousThrottle_ = 0.0;
    double throttleDeltaHistory_ = 0.0;
    double throttleDeltaTimeS_ = 0.0;
    bool kickdownActive_ = false;

    // Stored for parity with the closed-loop twin / logger; not used to shift.
    double drivetrainTorqueNm_ = 0.0;

    IGearboxLogger* logger_ = nullptr;
    int twinState_ = 0;
    double clutchPressureFeedback_ = 0.0;
    double speedFeedbackKmh_ = 0.0;
    double rpmFeedback_ = 0.0;
    uint64_t frame_ = 0;

    bridge::GearSelector selector_ = bridge::GearSelector::DRIVE;

    // Table lookups (pure interpolation over the profile shift tables).
    double getShiftSpeed(int fromGear, int toGear, double throttle) const;
    double getDownshiftSpeed(int fromGear, int toGear, double throttle) const;
    double getEngineRpm(double speedKmh, int gear) const;

    // Torque nudge: signed shift-table hint derived from drivetrainTorqueNm_.
    // Positive torque → higher effective throttle (earlier downshift / later upshift).
    // Negative torque → lower effective throttle (inhibits upshift / holds gear).
    // Bounded to ±15% throttle equivalent; never touches road speed or torque injection.
    double effectiveThrottleForShift() const;

    // Declarative shift decision helpers.
    bool shouldKickdown(double throttleFraction) const;
    int findSafeGear(double speedKmh, int maxDownshifts) const;
    bool speedExceedsUpshift(double speedKmh, int gear) const;
    bool speedBelowDownshift(double speedKmh, int gear) const;
    ShiftDecision decideGear(double speedKmh) const;
    void applyShift(const ShiftDecision& decision);

    // True when the selector is in a forward position that allows shifting.
    bool isShifterInDrive() const;

    // Throttle pre-processing (smoothing + kickdown-delta tracking).
    void smoothThrottleInput(double throttleFraction, double dt);
    void trackThrottleDelta(double throttleFraction, double dt);

    // Logger snapshot: populate and emit a GearboxLogEntry when logger_ is set.
    void logShiftState(double throttleFraction, double dt, double speedKmh);

    // Core throttle/speed/torque shift decision (shared by both public updates).
    void runShiftLogic(double dt, double speedKmh, double throttleFraction);
};

}

#endif
