#include <twin/AutomaticGearbox.h>
#include <twin/IGearboxLogger.h>
#include <twin/GearboxLogEntry.h>
#include <simulator/EngineSimTypes.h>
#include <common/Verification.h>
#include <algorithm>
#include <cmath>

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
        if (dt > 0.0) {
            smoothThrottleInput(throttleFraction, dt);
            trackThrottleDelta(throttleFraction, dt);
        }
        previousThrottle_ = throttleFraction;
        requestsShift_ = false;
        targetGear_ = currentGear_;
        kickdownActive_ = false;
        return;
    }

    runShiftLogic(dt, speedKmh, throttleFraction);
}

double AutomaticGearbox::effectiveThrottleForShift() const {
    // Torque nudge: signed shift-table hint derived from drivetrainTorqueNm_.
    // Positive torque → higher effective throttle (earlier downshift / later upshift).
    // Negative torque → lower effective throttle (inhibits upshift / holds gear).
    // Bounded to ±15% throttle equivalent; never touches road speed or torque injection.
    constexpr double kMaxTorqueNm = 3000.0;
    constexpr double kMaxThrottleNudge = 0.15;
    const double normTorque = std::clamp(std::abs(drivetrainTorqueNm_) / kMaxTorqueNm, 0.0, 1.0);
    const double sign = drivetrainTorqueNm_ >= 0.0 ? 1.0 : -1.0;
    const double nudge = sign * normTorque * kMaxThrottleNudge;
    return std::clamp(smoothedThrottle_ + nudge, 0.0, 1.0);
}

void AutomaticGearbox::runShiftLogic(double dt, double speedKmh, double throttleFraction) {
    // Zero-time calls (the SHIFTING placeholder update(0,0,0) from VirtualIceTwin)
    // must be a no-op: with dt == 0 they would corrupt throttle-tracking state
    // (previousThrottle_, smoothed/delta history) and fabricate phantom kickdown
    // events on the next real frame, making shift timing depend on the
    // shift-execution cadence instead of the throttle/speed input.
    if (dt <= 0.0) {
        return;
    }
    throttleFraction = std::clamp(throttleFraction, 0.0, 1.0);
    timeSinceLastShiftS_ += dt;

    // Input conditioning only: smoothing stabilises the table input, the delta
    // window detects kickdown stabs. Neither is a shift gate.
    smoothThrottleInput(throttleFraction, dt);
    trackThrottleDelta(throttleFraction, dt);
    kickdownActive_ = shouldKickdown(throttleFraction);

    requestsShift_ = false;
    targetGear_ = currentGear_;

    // Guards: only DRIVE shifts (AC6) and only once moving (AC10).
    if (!isShifterInDrive() || speedKmh < profile_.standstillThresholdKmh) {
        logShiftState(throttleFraction, dt, speedKmh);
        return;
    }

    applyShift(decideGear(speedKmh));
    logShiftState(throttleFraction, dt, speedKmh);
}

void AutomaticGearbox::smoothThrottleInput(double throttleFraction, double dt) {
    const double tau = profile_.throttleSmoothingTauMs / 1000.0;
    const double alpha = dt / (tau + dt);
    if (smoothedThrottle_ < 0.0) {
        smoothedThrottle_ = throttleFraction;
    } else {
        smoothedThrottle_ += alpha * (throttleFraction - smoothedThrottle_);
    }
}

void AutomaticGearbox::trackThrottleDelta(double throttleFraction, double dt) {
    const double throttleDelta = throttleFraction - previousThrottle_;
    throttleDeltaTimeS_ += dt;
    throttleDeltaHistory_ = std::max(throttleDeltaHistory_, throttleDelta);

    if (throttleDeltaTimeS_ > profile_.kickdownWindowMs / 1000.0) {
        throttleDeltaHistory_ = 0.0;
        throttleDeltaTimeS_ = 0.0;
    }

    previousThrottle_ = throttleFraction;
}

AutomaticGearbox::ShiftDecision
AutomaticGearbox::decideGear(double speedKmh) const {
    // 1. Kickdown — the sole override of the tables. A sudden large throttle
    //    increase (or WOT) forces one RPM-safe downshift, even inside the dwell
    //    window: a legitimate power demand cannot wait.
    if (currentGear_ > 1 && kickdownActive_) {
        const int depth = static_cast<int>(profile_.kickdownDownshiftGears);
        const int safeGear = findSafeGear(speedKmh, depth);
        if (safeGear < currentGear_) {
            return {safeGear, -1, true};
        }
    }

    // 2. Minimal dwell blocks table-driven shifts right after any shift, so a
    //    single speed sample cannot thrash the shift valve sub-frame.
    if (const bool withinDwell = hasShiftedBefore_ &&
                                 timeSinceLastShiftS_ < profile_.shiftDwellS;
        withinDwell) {
        return {currentGear_, 0, false};
    }

    // 3. Tables are the authority. Cascade up then, only if no upshift fired,
    //    down. The downshift table sits below the upshift table for every
    //    (gear, throttle), so after an upshift the speed is above the lower
    //    gear's downshift threshold and the down-pass cannot also fire — the
    //    two are mutually exclusive and no oscillation is possible.
    int gear = currentGear_;
    while (speedExceedsUpshift(speedKmh, gear)) {
        ++gear;
    }
    if (gear == currentGear_) {
        while (gear > 1 && speedBelowDownshift(speedKmh, gear)) {
            --gear;
        }
    }
    int direction = 0;
    if (gear > currentGear_) {
        direction = +1;
    } else if (gear < currentGear_) {
        direction = -1;
    }
    return {gear, direction, false};
}

bool AutomaticGearbox::speedExceedsUpshift(double speedKmh, int gear) const {
    if (gear >= static_cast<int>(profile_.gearRatios.size())) {
        return false;
    }
    if (const double upshiftSpeed = getShiftSpeed(gear, gear + 1, effectiveThrottleForShift());
        upshiftSpeed > 0.0 && speedKmh > upshiftSpeed) {
        return true;
    }
    // Redline safety net, coherent with the speed model: a hard ceiling that
    // only ever upshifts, so it cannot cause hunting.
    return getEngineRpm(speedKmh, gear) > profile_.redlineRpm * 0.95;
}

bool AutomaticGearbox::speedBelowDownshift(double speedKmh, int gear) const {
    if (gear <= 1) {
        return false;
    }
    const double downshiftSpeed = getDownshiftSpeed(gear - 1, gear, effectiveThrottleForShift());
    return downshiftSpeed > 0.0 && speedKmh < downshiftSpeed;
}

void AutomaticGearbox::applyShift(const ShiftDecision& decision) {
    if (decision.direction == 0) {
        return;
    }
    currentGear_ = decision.gear;
    targetGear_ = decision.gear;
    requestsShift_ = true;
    hasShiftedBefore_ = true;
    lastShiftDirection_ = decision.direction;
    timeSinceLastShiftS_ = 0.0;
    if (decision.kickdown) {
        // Consume the kickdown stab so a single pedal movement fires once.
        throttleDeltaHistory_ = 0.0;
        throttleDeltaTimeS_ = 0.0;
    }
}

bool AutomaticGearbox::shouldKickdown(double throttleFraction) const {
    // Kickdown on absolute WOT demand, or on a sudden large throttle increase
    // captured within the kickdown window by trackThrottleDelta().
    if (throttleFraction >= profile_.kickdownThrottleThreshold) {
        return true;
    }
    if (throttleDeltaHistory_ >= profile_.kickdownDelta) {
        return true;
    }
    return false;
}

int AutomaticGearbox::findSafeGear(double speedKmh, int maxDownshifts) const {
    // Walk down from the current gear, returning the highest gear whose engine
    // speed stays under 90% redline. Bounded by maxDownshifts.
    for (int gear = currentGear_ - 1; gear >= std::max(1, currentGear_ - maxDownshifts); --gear) {
        const double rpm = getEngineRpm(speedKmh, gear);
        if (rpm <= profile_.redlineRpm * 0.9) {
            return gear;
        }
    }
    return currentGear_;
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

    size_t tableIndex = static_cast<size_t>(fromGear) - 1;
    if (!profile_.shiftTable[0].empty() && tableIndex >= profile_.shiftTable[0].size()) {
        tableIndex = profile_.shiftTable[0].size() - 1;  // clamp to last valid column
    }

    ASSERT(!profile_.shiftTableThrottleLevels.empty(), "getShiftSpeed: throttle levels must be populated");

    return interpolateShiftSpeed(profile_.shiftTable, throttle, profile_.shiftTableThrottleLevels, tableIndex);
}

double AutomaticGearbox::getDownshiftSpeed(int fromGear, int toGear, double throttle) const {
    // Hysteresis fallback: without a separate downshift table, downshift at a
    // fixed fraction of the upshift speed (profile_.hysteresisFactor).
    if (!profile_.separateDownshiftTableEnabled) {
        return getShiftSpeed(fromGear, toGear, throttle) * profile_.hysteresisFactor;
    }

    ASSERT(fromGear >= 1 && toGear >= 1 && fromGear < toGear,
           "getDownshiftSpeed: gear indexes out of range");

    size_t tableIndex = static_cast<size_t>(fromGear) - 1;
    ASSERT(!profile_.downshiftTable.empty(), "getDownshiftSpeed: downshift table must be populated");
    if (!profile_.downshiftTable[0].empty() && tableIndex >= profile_.downshiftTable[0].size()) {
        tableIndex = profile_.downshiftTable[0].size() - 1;  // clamp to last valid column
    }
    ASSERT(!profile_.downshiftTableThrottleLevels.empty(),
           "getDownshiftSpeed: downshift throttle levels must be populated");

    return interpolateShiftSpeed(profile_.downshiftTable, throttle,
                                 profile_.downshiftTableThrottleLevels, tableIndex);
}

double AutomaticGearbox::getEngineRpm(double speedKmh, int gear) const {
    ASSERT(gear >= 1 && gear <= static_cast<int>(profile_.gearRatios.size()),
           "getEngineRpm: gear index out of range");

    const double speedMs = speedKmh / EngineSimDefaults::MS_TO_KMH;
    const double wheelRpm = speedMs / (2.0 * M_PI * profile_.tireRadiusM) * 60.0;
    const double engineRpm = wheelRpm * profile_.gearRatios[gear - 1] * profile_.diffRatio;

    return engineRpm;
}

}
