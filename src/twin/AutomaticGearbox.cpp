#include <twin/AutomaticGearbox.h>
#include <algorithm>
#include <cmath>

namespace twin {

AutomaticGearbox::AutomaticGearbox(const IceVehicleProfile& profile)
    : profile_(profile) {
}

void AutomaticGearbox::update(double dt, double speedKmh, double throttleFraction) {
    throttleFraction = std::clamp(throttleFraction, 0.0, 1.0);
    timeSinceLastShiftS_ += dt;

    // Smooth throttle input (initialize on first call)
    double tau = profile_.throttleSmoothingTauMs / 1000.0;
    double alpha = dt / (tau + dt);
    if (smoothedThrottle_ < 0.0) {
        smoothedThrottle_ = throttleFraction;
    } else {
        smoothedThrottle_ += alpha * (throttleFraction - smoothedThrottle_);
    }

    // Track throttle delta for kickdown detection
    double throttleDelta = throttleFraction - previousThrottle_;
    throttleDeltaTimeS_ += dt;
    throttleDeltaHistory_ = std::max(throttleDeltaHistory_, throttleDelta);

    // Reset delta history if window has passed
    if (throttleDeltaTimeS_ > profile_.kickdownWindowMs / 1000.0) {
        throttleDeltaHistory_ = 0.0;
        throttleDeltaTimeS_ = 0.0;
    }

    previousThrottle_ = throttleFraction;

    requestsShift_ = false;
    targetGear_ = currentGear_;
    kickdownActive_ = shouldKickdown(throttleFraction, dt);

    // Don't shift at standstill
    if (speedKmh < profile_.standstillThresholdKmh) {
        return;
    }

    // Check for kickdown (downshift on throttle spike)
    if (kickdownActive_) {
        int safeGear = findSafeGear(speedKmh, currentGear_ - 1);
        if (safeGear < currentGear_) {
            // Check if we can downshift (3s minimum between downshifts)
            if (lastShiftDirection_ != -1 || timeSinceLastShiftS_ >= profile_.minShiftIntervalS) {
                currentGear_ = safeGear;
                targetGear_ = safeGear;
                requestsShift_ = true;
                hasShiftedBefore_ = true;
                lastShiftDirection_ = -1;
                timeSinceLastShiftS_ = 0.0;
                throttleDeltaHistory_ = 0.0;
                throttleDeltaTimeS_ = 0.0;
                return;
            }
        }
    }

    // Normal shift logic
    bool canUpshift = (lastShiftDirection_ != 1 || timeSinceLastShiftS_ >= profile_.minShiftIntervalS);
    bool canDownshift = (lastShiftDirection_ != -1 || timeSinceLastShiftS_ >= profile_.minShiftIntervalS);

    // Check for upshift(s)
    if (canUpshift) {
        while (currentGear_ < static_cast<int>(profile_.gearRatios.size())) {
            double upshiftSpeed = getShiftSpeed(currentGear_, currentGear_ + 1, smoothedThrottle_);
            if (upshiftSpeed > 0 && speedKmh > upshiftSpeed) {
                currentGear_ = currentGear_ + 1;
                requestsShift_ = true;
                hasShiftedBefore_ = true;
                lastShiftDirection_ = 1;
            } else {
                break;
            }
        }
        if (requestsShift_) {
            targetGear_ = currentGear_;
            timeSinceLastShiftS_ = 0.0;
        }
    }

    // Check for downshift(s)
    if (!requestsShift_ && canDownshift) {
        while (currentGear_ > 1) {
            double downshiftSpeed = getShiftSpeed(currentGear_ - 1, currentGear_, smoothedThrottle_) * profile_.hysteresisFactor;
            if (speedKmh < downshiftSpeed) {
                currentGear_ = currentGear_ - 1;
                requestsShift_ = true;
                hasShiftedBefore_ = true;
                lastShiftDirection_ = -1;
            } else {
                break;
            }
        }
        if (requestsShift_) {
            targetGear_ = currentGear_;
            timeSinceLastShiftS_ = 0.0;
        }
    }
}

int AutomaticGearbox::getCurrentGear() const {
    return currentGear_;
}

bool AutomaticGearbox::requestsShift() const {
    return requestsShift_;
}

int AutomaticGearbox::getTargetGear() const {
    return targetGear_;
}

bool AutomaticGearbox::isInKickdown() const {
    return kickdownActive_;
}

double AutomaticGearbox::getShiftSpeed(int fromGear, int toGear, double throttle) const {
    if (fromGear < 1 || toGear < 1 || fromGear >= toGear) {
        return 0.0;
    }

    size_t tableIndex = static_cast<size_t>(fromGear) - 1;
    if (tableIndex >= profile_.shiftTable[0].size()) {
        return 0.0;
    }

    // Shift table rows are at throttle levels: 10%, 25%, 50%, 75%, 100%
    const double throttleLevels[] = {0.1, 0.25, 0.5, 0.75, 1.0};
    const size_t numLevels = 5;

    // Clamp throttle to table bounds
    throttle = std::clamp(throttle, throttleLevels[0], throttleLevels[numLevels - 1]);

    // Find the two throttle levels to interpolate between
    size_t lowerIndex = 0;
    size_t upperIndex = numLevels - 1;

    for (size_t i = 0; i < numLevels - 1; ++i) {
        if (throttle >= throttleLevels[i] && throttle <= throttleLevels[i + 1]) {
            lowerIndex = i;
            upperIndex = i + 1;
            break;
        }
    }

    // Get the shift speeds from the table
    double lowerSpeed = profile_.shiftTable[lowerIndex][tableIndex];
    double upperSpeed = profile_.shiftTable[upperIndex][tableIndex];

    // Linear interpolation
    if (upperIndex == lowerIndex) {
        return lowerSpeed;
    }

    double t = (throttle - throttleLevels[lowerIndex]) /
               (throttleLevels[upperIndex] - throttleLevels[lowerIndex]);

    return lowerSpeed + t * (upperSpeed - lowerSpeed);
}

double AutomaticGearbox::getEngineRpm(double speedKmh, int gear) const {
    if (gear < 1 || gear > static_cast<int>(profile_.gearRatios.size())) {
        return 0.0;
    }

    double speedMs = speedKmh / 3.6;
    double wheelRpm = speedMs / (2.0 * M_PI * profile_.tireRadiusM) * 60.0;
    double engineRpm = wheelRpm * profile_.gearRatios[gear - 1] * profile_.diffRatio;

    return engineRpm;
}

bool AutomaticGearbox::shouldKickdown(double throttleFraction, double dt) {
    // Kickdown if throttle exceeds threshold
    if (throttleFraction >= profile_.kickdownThrottleThreshold) {
        return true;
    }

    // Kickdown if throttle delta exceeds threshold within window
    if (throttleDeltaHistory_ >= profile_.kickdownDelta) {
        return true;
    }

    return false;
}

int AutomaticGearbox::findSafeGear(double speedKmh, int maxDownshifts) const {
    if (maxDownshifts < 1) {
        return currentGear_;
    }

    // Try gears from current-1 down to 1
    for (int gear = currentGear_ - 1; gear >= std::max(1, currentGear_ - maxDownshifts); --gear) {
        double rpm = getEngineRpm(speedKmh, gear);
        if (rpm <= profile_.redlineRpm * 0.9) {
            return gear;
        }
    }

    return currentGear_;
}

}
