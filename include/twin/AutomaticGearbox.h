#ifndef TWIN_AUTOMATIC_GEARBOX_H
#define TWIN_AUTOMATIC_GEARBOX_H

#include <twin/IceVehicleProfile.h>

namespace twin {

class AutomaticGearbox {
public:
    explicit AutomaticGearbox(const IceVehicleProfile& profile);

    void update(double dt, double speedKmh, double throttleFraction);

    int getCurrentGear() const;
    bool requestsShift() const;
    int getTargetGear() const;
    bool isInKickdown() const;

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

    double getShiftSpeed(int fromGear, int toGear, double throttle) const;
    double getEngineRpm(double speedKmh, int gear) const;
    bool shouldKickdown(double throttleFraction, double dt);
    int findSafeGear(double speedKmh, int maxDownshifts) const;
};

}

#endif
