#ifndef TWIN_AUTOMATIC_GEARBOX_H
#define TWIN_AUTOMATIC_GEARBOX_H

#include <twin/IceVehicleProfile.h>
#include <twin/IGearboxLogger.h>
#include <simulator/GearConventions.h>

namespace twin {

class AutomaticGearbox {
public:
    explicit AutomaticGearbox(const IceVehicleProfile& profile);

    // Legacy 3-arg update (throttle/speed only). Equivalent to torque-aware
    // update with drivetrainTorqueNm = 0 and selector = DRIVE.
    void update(double dt, double speedKmh, double throttleFraction);

    // Torque- and selector-aware update per the Virtual ICE Twin shift spec:
    // upshift when approaching redline or under low drivetrain load; downshift
    // on kickdown (high throttle) or high drivetrain torque. Throttle biases
    // the decision; hysteresis prevents hunting. In NEUTRAL/PARK/REVERSE no
    // shift is requested and the current gear holds.
    void update(double dt, double speedKmh, double throttleFraction,
                double drivetrainTorqueNm);

    int getCurrentGear() const;
    bool requestsShift() const;
    int getTargetGear() const;
    bool isInKickdown() const;

    void setLogger(IGearboxLogger* logger) { logger_ = logger; }
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
    const IceVehicleProfile& profile_;
    int currentGear_ = 1;
    int targetGear_ = 1;
    bool requestsShift_ = false;
    double timeSinceLastShiftS_ = 0.0;
    bool hasShiftedBefore_ = false;
    int lastShiftDirection_ = 0;
    double smoothedThrottle_ = -1.0;
    double previousThrottle_ = 0.0;
    double throttleDeltaHistory_ = 0.0;
    double throttleDeltaTimeS_ = 0.0;
    bool kickdownActive_ = false;
    double throttleGradient_ = 0.0;       // %/s, for tip-in/tip-out detection
    bool tipBlocksUpshift_ = false;       // true when gradient exceeds threshold
    bool hadPreviousThrottle_ = false;    // skip gradient on first frame

    IGearboxLogger* logger_ = nullptr;
    int twinState_ = 0;
    double clutchPressureFeedback_ = 0.0;
    double speedFeedbackKmh_ = 0.0;
    double rpmFeedback_ = 0.0;
    uint64_t frame_ = 0;

    // Torque-driven shift logic state (Virtual ICE Twin spec).
    bridge::GearSelector selector_ = bridge::GearSelector::DRIVE;
    double drivetrainTorqueNm_ = 0.0;        // last commanded drivetrain torque
    double highLoadTorqueThresholdNm_ = 300.0;  // above this = high load (downshift bias)
    double lowLoadTorqueThresholdNm_ = 120.0;   // below this = low load (upshift bias)
    int torqueLoadBandStableFrames_ = 0;     // sustained-load frames before a torque-driven shift

    double getShiftSpeed(int fromGear, int toGear, double throttle) const;
    double getDownshiftSpeed(int fromGear, int toGear, double throttle) const;
    double getEngineRpm(double speedKmh, int gear) const;
    bool shouldKickdown(double throttleFraction, double dt) const;
    int findSafeGear(double speedKmh, int maxDownshifts) const;
    // True when the selector is in a forward position that allows shifting.
    bool isShifterInDrive() const;

    // Throttle pre-processing (smoothing + delta tracking + tip correction).
    void smoothThrottleInput(double throttleFraction, double dt);
    void trackThrottleDelta(double throttleFraction, double dt);
    void applyTipCorrection(double throttleDelta, double dt);

    // Engine-brake gate: true when coasting conditions block upshifts only.
    bool isEngineBrakingActive(double throttleFraction, double speedKmh) const;

    // Shift decision passes — each returns true if a shift was committed.
    bool tryKickdown(double speedKmh);
    bool tryTorqueDownshift(double speedKmh);
    bool trySpeedUpshift(double speedKmh);
    bool trySpeedDownshift(double speedKmh);

    // Logger snapshot: populate and emit a GearboxLogEntry when logger_ is set.
    void logShiftState(double throttleFraction, double dt, double speedKmh);

    // Core throttle/speed/torque shift decision (shared by both public updates).
    void runShiftLogic(double dt, double speedKmh, double throttleFraction);
};

}

#endif
