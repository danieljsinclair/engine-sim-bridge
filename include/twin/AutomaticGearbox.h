#ifndef TWIN_AUTOMATIC_GEARBOX_H
#define TWIN_AUTOMATIC_GEARBOX_H

#include <twin/IceVehicleProfile.h>
#include <twin/IGearboxLogger.h>

namespace twin {

class AutomaticGearbox {
public:
    explicit AutomaticGearbox(const IceVehicleProfile& profile);

    void update(double dt, double speedKmh, double throttleFraction);

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

    double getShiftSpeed(int fromGear, int toGear, double throttle) const;
    double getDownshiftSpeed(int fromGear, int toGear, double throttle) const;
    double getEngineRpm(double speedKmh, int gear) const;
    bool shouldKickdown(double throttleFraction, double dt);
    int findSafeGear(double speedKmh, int maxDownshifts) const;
};

}

#endif
