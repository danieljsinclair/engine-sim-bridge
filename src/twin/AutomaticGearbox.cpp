#include <twin/AutomaticGearbox.h>
#include <twin/IGearboxLogger.h>
#include <twin/GearboxLogEntry.h>
#include <simulator/EngineSimTypes.h>
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

    // Tip-in/tip-out correction (per x-engineer ch6 s4.4)
    // Level-sensitive: blocks upshifts while throttle gradient exceeds threshold
    tipBlocksUpshift_ = false;
    if (profile_.tipCorrectionEnabled && hadPreviousThrottle_ && dt > 0.0) {
        throttleGradient_ = throttleDelta / dt * 100.0;  // %/s
        if (throttleGradient_ > profile_.tipInGradientThreshold ||
            throttleGradient_ < profile_.tipOutGradientThreshold) {
            tipBlocksUpshift_ = true;
        }
    }
    hadPreviousThrottle_ = true;

    requestsShift_ = false;
    targetGear_ = currentGear_;
    kickdownActive_ = shouldKickdown(throttleFraction, dt);

    // Don't shift at standstill
    if (speedKmh < profile_.standstillThresholdKmh) {
        return;
    }

    // Engine braking inhibitor: blocks upshifts when coasting (per x-engineer ch6 s4.3)
    // Condition: throttle < 1% AND speed >= threshold → block upshifts only
    bool engineBrakingActive = false;
    if (profile_.engineBrakingInhibitorEnabled &&
        throttleFraction < profile_.engineBrakingMaxThrottle &&
        speedKmh >= profile_.engineBrakingMinSpeedKmh) {
        engineBrakingActive = true;
    }

    // Check for kickdown (downshift on throttle spike) — overrides inhibitor
    if (kickdownActive_) {
        int safeGear = findSafeGear(speedKmh, currentGear_ - 1);
        if (safeGear < currentGear_) {
            // Use downshift interval for kickdown gate
            double downInterval = profile_.downshiftMinIntervalS > 0.0
                ? profile_.downshiftMinIntervalS
                : profile_.minShiftIntervalS;
            if (lastShiftDirection_ != -1 || timeSinceLastShiftS_ >= downInterval) {
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

    // Asymmetric shift intervals: use direction-specific interval if set
    double upInterval = profile_.upshiftMinIntervalS > 0.0
        ? profile_.upshiftMinIntervalS
        : profile_.minShiftIntervalS;
    double downInterval2 = profile_.downshiftMinIntervalS > 0.0
        ? profile_.downshiftMinIntervalS
        : profile_.minShiftIntervalS;

    bool canUpshift = !engineBrakingActive && !tipBlocksUpshift_ &&
        (lastShiftDirection_ != 1 || timeSinceLastShiftS_ >= upInterval);
    bool canDownshift =
        (lastShiftDirection_ != -1 || timeSinceLastShiftS_ >= downInterval2);

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
            timeSinceLastShiftS_ = 0.0;  // Cross-reset: resets both up/down timers
        }
    }

    // Check for downshift(s) — use getDownshiftSpeed() for separate table lookup
    if (!requestsShift_ && canDownshift) {
        while (currentGear_ > 1) {
            double downshiftSpeed = getDownshiftSpeed(currentGear_ - 1, currentGear_, smoothedThrottle_);
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
            timeSinceLastShiftS_ = 0.0;  // Cross-reset: resets both up/down timers
        }
    }

    if (logger_) {
        GearboxLogEntry e;
        e.frame = frame_++;
        e.dt = dt;
        e.speedKmh = speedKmh;
        e.throttleRaw = throttleFraction;
        e.throttleSmoothed = smoothedThrottle_;
        e.vehicleSpeedFeedbackKmh = speedFeedbackKmh_;
        e.engineRpmFeedback = rpmFeedback_;
        e.currentGear = currentGear_;
        e.targetGear = targetGear_;
        e.requestsShift = requestsShift_;
        e.lastShiftDirection = lastShiftDirection_;
        e.timeSinceLastShiftS = timeSinceLastShiftS_;
        e.kickdownActive = kickdownActive_;
        e.throttleDeltaHistory = throttleDeltaHistory_;
        e.engineRpm = getEngineRpm(speedKmh, currentGear_);
        e.twinState = twinState_;
        e.clutchPressure = clutchPressureFeedback_;
        if (currentGear_ >= 1 && speedKmh >= 0.1) {
            e.upshiftSpeed = getShiftSpeed(currentGear_, currentGear_ + 1, smoothedThrottle_);
            if (currentGear_ > 1) {
                e.downshiftSpeed = getDownshiftSpeed(currentGear_ - 1, currentGear_, smoothedThrottle_);
            }
        }
        logger_->log(e);
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

    if (profile_.shiftTable.empty()) {
        return 0.0;
    }

    size_t tableIndex = static_cast<size_t>(fromGear) - 1;
    if (tableIndex >= profile_.shiftTable[0].size()) {
        return 0.0;
    }

    // Use profile throttle levels if available, otherwise fall back to legacy 5-level
    const double legacyLevels[] = {0.1, 0.25, 0.5, 0.75, 1.0};
    const size_t legacyNumLevels = 5;

    size_t numLevels;
    bool useProfileLevels = !profile_.shiftTableThrottleLevels.empty();

    double levelLow, levelHigh;
    size_t lowerIndex = 0;
    size_t upperIndex = 0;

    if (useProfileLevels) {
        numLevels = profile_.shiftTableThrottleLevels.size();
        throttle = std::clamp(throttle, profile_.shiftTableThrottleLevels[0],
                              profile_.shiftTableThrottleLevels[numLevels - 1]);
        upperIndex = numLevels - 1;

        for (size_t i = 0; i < numLevels - 1; ++i) {
            if (throttle >= profile_.shiftTableThrottleLevels[i] &&
                throttle <= profile_.shiftTableThrottleLevels[i + 1]) {
                lowerIndex = i;
                upperIndex = i + 1;
                break;
            }
        }

        levelLow = profile_.shiftTableThrottleLevels[lowerIndex];
        levelHigh = profile_.shiftTableThrottleLevels[upperIndex];
    } else {
        numLevels = legacyNumLevels;
        throttle = std::clamp(throttle, legacyLevels[0], legacyLevels[numLevels - 1]);
        upperIndex = numLevels - 1;

        for (size_t i = 0; i < numLevels - 1; ++i) {
            if (throttle >= legacyLevels[i] && throttle <= legacyLevels[i + 1]) {
                lowerIndex = i;
                upperIndex = i + 1;
                break;
            }
        }

        levelLow = legacyLevels[lowerIndex];
        levelHigh = legacyLevels[upperIndex];
    }

    // Get the shift speeds from the table
    double lowerSpeed = profile_.shiftTable[lowerIndex][tableIndex];
    double upperSpeed = profile_.shiftTable[upperIndex][tableIndex];

    // Linear interpolation
    if (upperIndex == lowerIndex) {
        return lowerSpeed;
    }

    double t = (throttle - levelLow) / (levelHigh - levelLow);

    return lowerSpeed + t * (upperSpeed - lowerSpeed);
}

double AutomaticGearbox::getDownshiftSpeed(int fromGear, int toGear, double throttle) const {
    if (!profile_.separateDownshiftTableEnabled) {
        return getShiftSpeed(fromGear, toGear, throttle) * profile_.hysteresisFactor;
    }

    if (fromGear < 1 || toGear < 1 || fromGear >= toGear) {
        return 0.0;
    }

    size_t tableIndex = static_cast<size_t>(fromGear) - 1;
    if (profile_.downshiftTable.empty() || tableIndex >= profile_.downshiftTable[0].size()) {
        return 0.0;
    }

    const auto& levels = profile_.downshiftTableThrottleLevels;
    if (levels.empty()) {
        return 0.0;
    }

    size_t numLevels = levels.size();
    throttle = std::clamp(throttle, levels[0], levels[numLevels - 1]);

    size_t lowerIndex = 0;
    size_t upperIndex = numLevels - 1;

    for (size_t i = 0; i < numLevels - 1; ++i) {
        if (throttle >= levels[i] && throttle <= levels[i + 1]) {
            lowerIndex = i;
            upperIndex = i + 1;
            break;
        }
    }

    double lowerSpeed = profile_.downshiftTable[lowerIndex][tableIndex];
    double upperSpeed = profile_.downshiftTable[upperIndex][tableIndex];

    if (upperIndex == lowerIndex) {
        return lowerSpeed;
    }

    double t = (throttle - levels[lowerIndex]) /
               (levels[upperIndex] - levels[lowerIndex]);

    return lowerSpeed + t * (upperSpeed - lowerSpeed);
}

double AutomaticGearbox::getEngineRpm(double speedKmh, int gear) const {
    if (gear < 1 || gear > static_cast<int>(profile_.gearRatios.size())) {
        return 0.0;
    }

    double speedMs = speedKmh / EngineSimDefaults::MS_TO_KMH;
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
