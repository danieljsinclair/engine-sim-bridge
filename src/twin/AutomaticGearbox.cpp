#include <twin/AutomaticGearbox.h>
#include <twin/IGearboxLogger.h>
#include <twin/GearboxLogEntry.h>
#include <simulator/EngineSimTypes.h>
#include <common/Verification.h>
#include <algorithm>
#include <cmath>
#include <array>

namespace twin {

// Shared interpolation core for the upshift and downshift speed lookups.
// Selects the bracketing throttle-level rows around `throttle` and linearly
// interpolates between the corresponding table speeds. Pure computation: no
// validation — callers guarantee a populated table/levels and a valid index.
template <typename LevelContainer>
static double interpolateShiftSpeed(const std::vector<std::vector<double>>& table,
                                    double throttle,
                                    const LevelContainer& levels,
                                    size_t tableIndex) {
    const size_t numLevels = levels.size();
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

    const double lowerSpeed = table[lowerIndex][tableIndex];
    const double upperSpeed = table[upperIndex][tableIndex];

    if (upperIndex == lowerIndex) {
        return lowerSpeed;
    }

    const double t = (throttle - levels[lowerIndex]) /
                     (levels[upperIndex] - levels[lowerIndex]);
    return lowerSpeed + t * (upperSpeed - lowerSpeed);
}

AutomaticGearbox::AutomaticGearbox(const IceVehicleProfile& profile)
    : profile_(profile) {
}

bool AutomaticGearbox::isShifterInDrive() const {
    return selector_ == bridge::GearSelector::DRIVE;
}

void AutomaticGearbox::update(double dt, double speedKmh, double throttleFraction) {
    // Legacy entry point: no torque signal, default selector already set.
    drivetrainTorqueNm_ = 0.0;
    runShiftLogic(dt, speedKmh, throttleFraction);
}

void AutomaticGearbox::update(double dt, double speedKmh, double throttleFraction,
                              double drivetrainTorqueNm) {
    drivetrainTorqueNm_ = drivetrainTorqueNm;

    // AC6: clutch-out positions hold the gear and never request a shift.
    if (!isShifterInDrive()) {
        // Keep throttle-smoothing/timers consistent so re-engaging DRIVE is
        // well-behaved, but emit no shift decision.
        throttleFraction = std::clamp(throttleFraction, 0.0, 1.0);
        timeSinceLastShiftS_ += dt;
        double tau = profile_.throttleSmoothingTauMs / 1000.0;
        double alpha = dt / (tau + dt);
        if (smoothedThrottle_ < 0.0) smoothedThrottle_ = throttleFraction;
        else smoothedThrottle_ += alpha * (throttleFraction - smoothedThrottle_);
        previousThrottle_ = throttleFraction;
        requestsShift_ = false;
        targetGear_ = currentGear_;
        kickdownActive_ = false;
        tipBlocksUpshift_ = false;
        hadPreviousThrottle_ = true;
        return;
    }

    runShiftLogic(dt, speedKmh, throttleFraction);
}

void AutomaticGearbox::runShiftLogic(double dt, double speedKmh, double throttleFraction) {
    throttleFraction = std::clamp(throttleFraction, 0.0, 1.0);
    timeSinceLastShiftS_ += dt;

    double throttleDelta = throttleFraction - previousThrottle_;
    smoothThrottleInput(throttleFraction, dt);
    trackThrottleDelta(throttleFraction, dt);
    applyTipCorrection(throttleDelta, dt);

    requestsShift_ = false;
    targetGear_ = currentGear_;
    kickdownActive_ = shouldKickdown(throttleFraction, dt);

    // Don't shift at standstill
    if (speedKmh < profile_.standstillThresholdKmh) {
        logShiftState(throttleFraction, dt, speedKmh);
        return;
    }

    bool engineBrakingActive = isEngineBrakingActive(throttleFraction, speedKmh);

    if (kickdownActive_ && tryKickdown(speedKmh)) {
        logShiftState(throttleFraction, dt, speedKmh);
        return;
    }

    if (currentGear_ > 1 && tryTorqueDownshift(speedKmh)) {
        logShiftState(throttleFraction, dt, speedKmh);
        return;
    }

    // Asymmetric shift intervals
    double upInterval = profile_.upshiftMinIntervalS > 0.0
        ? profile_.upshiftMinIntervalS
        : profile_.minShiftIntervalS;
    double downInterval = profile_.downshiftMinIntervalS > 0.0
        ? profile_.downshiftMinIntervalS
        : profile_.minShiftIntervalS;

    bool canUpshift = !engineBrakingActive && !tipBlocksUpshift_ &&
        (lastShiftDirection_ != 1 || timeSinceLastShiftS_ >= upInterval);
    bool canDownshift =
        (lastShiftDirection_ != -1 || timeSinceLastShiftS_ >= downInterval);

    if (canUpshift && !requestsShift_) {
        trySpeedUpshift(speedKmh);
    }

    if (!requestsShift_ && canDownshift) {
        trySpeedDownshift(speedKmh);
    }

    logShiftState(throttleFraction, dt, speedKmh);
}

void AutomaticGearbox::smoothThrottleInput(double throttleFraction, double dt) {
    double tau = profile_.throttleSmoothingTauMs / 1000.0;
    double alpha = dt / (tau + dt);
    if (smoothedThrottle_ < 0.0) {
        smoothedThrottle_ = throttleFraction;
    } else {
        smoothedThrottle_ += alpha * (throttleFraction - smoothedThrottle_);
    }
}

void AutomaticGearbox::trackThrottleDelta(double throttleFraction, double dt) {
    double throttleDelta = throttleFraction - previousThrottle_;
    throttleDeltaTimeS_ += dt;
    throttleDeltaHistory_ = std::max(throttleDeltaHistory_, throttleDelta);

    if (throttleDeltaTimeS_ > profile_.kickdownWindowMs / 1000.0) {
        throttleDeltaHistory_ = 0.0;
        throttleDeltaTimeS_ = 0.0;
    }

    previousThrottle_ = throttleFraction;
}

void AutomaticGearbox::applyTipCorrection(double throttleDelta, double dt) {
    tipBlocksUpshift_ = false;
    if (profile_.tipCorrectionEnabled && hadPreviousThrottle_ && dt > 0.0) {
        throttleGradient_ = throttleDelta / dt * 100.0;  // %/s
        if (throttleGradient_ > profile_.tipInGradientThreshold ||
            throttleGradient_ < profile_.tipOutGradientThreshold) {
            tipBlocksUpshift_ = true;
        }
    }
    hadPreviousThrottle_ = true;
}

bool AutomaticGearbox::isEngineBrakingActive(double throttleFraction, double speedKmh) const {
    if (!profile_.engineBrakingInhibitorEnabled) {
        return false;
    }
    return throttleFraction < profile_.engineBrakingMaxThrottle &&
           speedKmh >= profile_.engineBrakingMinSpeedKmh;
}

bool AutomaticGearbox::tryKickdown(double speedKmh) {
    int safeGear = findSafeGear(speedKmh, currentGear_ - 1);
    if (safeGear >= currentGear_) {
        return false;
    }

    if (lastShiftDirection_ == -1 && timeSinceLastShiftS_ < (profile_.downshiftMinIntervalS > 0.0
        ? profile_.downshiftMinIntervalS
        : profile_.minShiftIntervalS)) {
        return false;
    }

    currentGear_ = safeGear;
    targetGear_ = safeGear;
    requestsShift_ = true;
    hasShiftedBefore_ = true;
    lastShiftDirection_ = -1;
    timeSinceLastShiftS_ = 0.0;
    throttleDeltaHistory_ = 0.0;
    throttleDeltaTimeS_ = 0.0;
    return true;
}

bool AutomaticGearbox::tryTorqueDownshift(double speedKmh) {
    if (drivetrainTorqueNm_ >= highLoadTorqueThresholdNm_) {
        ++torqueLoadBandStableFrames_;
    } else if (drivetrainTorqueNm_ <= lowLoadTorqueThresholdNm_) {
        torqueLoadBandStableFrames_ = 0;
    }

    if (torqueLoadBandStableFrames_ < 3 || requestsShift_) {
        return false;
    }

    int safeGear = findSafeGear(speedKmh, currentGear_ - 1);
    if (safeGear >= currentGear_) {
        return false;
    }

    if (lastShiftDirection_ == -1 && timeSinceLastShiftS_ < (profile_.downshiftMinIntervalS > 0.0
        ? profile_.downshiftMinIntervalS
        : profile_.minShiftIntervalS)) {
        return false;
    }

    currentGear_ = safeGear;
    targetGear_ = safeGear;
    requestsShift_ = true;
    hasShiftedBefore_ = true;
    lastShiftDirection_ = -1;
    timeSinceLastShiftS_ = 0.0;
    torqueLoadBandStableFrames_ = 0;
    return true;
}

bool AutomaticGearbox::trySpeedUpshift(double speedKmh) {
    while (currentGear_ < static_cast<int>(profile_.gearRatios.size())) {
        double upshiftSpeed = getShiftSpeed(currentGear_, currentGear_ + 1, smoothedThrottle_);
        bool speedUpshift = (upshiftSpeed > 0 && speedKmh > upshiftSpeed);
        bool redlineUpshift = (getEngineRpm(speedKmh, currentGear_) > profile_.redlineRpm * 0.95);
        if (speedUpshift || redlineUpshift) {
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
    return requestsShift_;
}

bool AutomaticGearbox::trySpeedDownshift(double speedKmh) {
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
        timeSinceLastShiftS_ = 0.0;
    }
    return requestsShift_;
}

void AutomaticGearbox::logShiftState(double throttleFraction, double dt, double speedKmh) {
    if (!logger_) {
        return;
    }
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
    ASSERT(fromGear >= 1 && toGear >= 1 && fromGear < toGear, "getShiftSpeed: gear indexes out of range");
    ASSERT(!profile_.shiftTable.empty(), "getShiftSpeed: shift table must be populated");

    // Use profile throttle levels if available, otherwise fall back to legacy 5-level.
    static constexpr std::array<double, 5> legacyLevels = {0.1, 0.25, 0.5, 0.75, 1.0};

    const size_t tableIndex = static_cast<size_t>(fromGear) - 1;
    ASSERT(tableIndex < profile_.shiftTable[0].size(), "getShiftSpeed: table index out of range for shift table");

    if (!profile_.shiftTableThrottleLevels.empty()) {
        return interpolateShiftSpeed(profile_.shiftTable, throttle, profile_.shiftTableThrottleLevels, tableIndex);
    }

    return interpolateShiftSpeed(profile_.shiftTable, throttle, legacyLevels, tableIndex);
}

double AutomaticGearbox::getDownshiftSpeed(int fromGear, int toGear, double throttle) const {
    // Hysteresis fallback: without a separate downshift table, downshift at a
    // fixed fraction of the upshift speed (profile_.hysteresisFactor).
    if (!profile_.separateDownshiftTableEnabled) {
        return getShiftSpeed(fromGear, toGear, throttle) * profile_.hysteresisFactor;
    }

    ASSERT(fromGear >= 1 && toGear >= 1 && fromGear < toGear,
           "getDownshiftSpeed: gear indexes out of range");

    const size_t tableIndex = static_cast<size_t>(fromGear) - 1;
    ASSERT(!profile_.downshiftTable.empty() &&
           tableIndex < profile_.downshiftTable[0].size(),
           "getDownshiftSpeed: downshift table/index out of range");
    ASSERT(!profile_.downshiftTableThrottleLevels.empty(),
           "getDownshiftSpeed: downshift throttle levels must be populated");

    return interpolateShiftSpeed(profile_.downshiftTable, throttle,
                                 profile_.downshiftTableThrottleLevels, tableIndex);
}

double AutomaticGearbox::getEngineRpm(double speedKmh, int gear) const {
    ASSERT(gear >= 1 && gear <= static_cast<int>(profile_.gearRatios.size()),
           "getEngineRpm: gear index out of range");

    double speedMs = speedKmh / EngineSimDefaults::MS_TO_KMH;
    double wheelRpm = speedMs / (2.0 * M_PI * profile_.tireRadiusM) * 60.0;
    double engineRpm = wheelRpm * profile_.gearRatios[gear - 1] * profile_.diffRatio;

    return engineRpm;
}

bool AutomaticGearbox::shouldKickdown(double throttleFraction, [[maybe_unused]] double dt) const {
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
