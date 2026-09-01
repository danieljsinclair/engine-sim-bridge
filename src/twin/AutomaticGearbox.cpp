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
    : profile_(profile),
      torqueHint_(std::make_unique<NullTorqueHint>()) {
    // Init-time invariant (the no-hunting proof): every downshift cell must sit
    // below its upshift cell for every (gear, throttle), so a single speed can
    // never satisfy both an up- and a down-shift at once. Enforced declaratively
    // at construction — fail-fast if a profile violates it.
    validateShiftTables(profile_);
}

void AutomaticGearbox::validateShiftTables(const IceVehicleProfile& profile) {
    if (!profile.separateDownshiftTableEnabled) return;
    if (profile.shiftTable.empty() || profile.downshiftTable.empty()) return;
    const size_t numLevels = std::min(profile.shiftTable.size(),
                                      profile.downshiftTable.size());
    for (size_t lvl = 0; lvl < numLevels; ++lvl) {
        const size_t numCols = std::min(profile.shiftTable[lvl].size(),
                                        profile.downshiftTable[lvl].size());
        for (size_t col = 0; col < numCols; ++col) {
            ASSERT(profile.downshiftTable[lvl][col] <= profile.shiftTable[lvl][col],
                   "validateShiftTables: downshift cell must be <= upshift cell "
                   "(no-hunting invariant violated)");
        }
    }
}

bool AutomaticGearbox::isShifterInDrive() const {
    return selector_ == bridge::GearSelector::DRIVE;
}

void AutomaticGearbox::update(double dt, double speedKmh, double throttleFraction) {
    // Legacy entry point: no torque signal → NullTorqueHint (zero bias).
    torqueHint_ = std::make_unique<NullTorqueHint>();
    runShiftLogic(dt, speedKmh, throttleFraction);
}

void AutomaticGearbox::update(double dt, double speedKmh, double throttleFraction,
                              double drivetrainTorqueNm) {
    update(dt, speedKmh, throttleFraction, drivetrainTorqueNm, nullptr);
}

void AutomaticGearbox::update(double dt, double speedKmh, double throttleFraction,
                              double drivetrainTorqueNm,
                              std::unique_ptr<ITorqueHint> upstreamHint) {
    // The frame's torque-hint strategy. Default: wrap the SIM-FEEDBACK torque
    // sample in the SimTorqueHint strategy (bounded bias). With an upstream
    // override (--torque-informed-gearbox) the caller's strategy replaces it.
    // Either way the decision consumes only ITorqueHint::shiftBias() —
    // availability is encapsulated here, never an `if (hasTorque)` in the
    // decision.
    torqueHint_ = std::move(upstreamHint);
    if (!torqueHint_) {
        torqueHint_ = std::make_unique<SimTorqueHint>(drivetrainTorqueNm,
                                                      profile_.torqueHintMaxNm,
                                                      profile_.torqueHintMaxBias);
    }

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
    // Kickdown is a DEMAND EVENT, one per pedal movement: a sustained WOT
    // level fires ONE kickdown (consumed when the shift executes, re-armed
    // when the demand drops back below the threshold). A level-held kickdown
    // re-firing every dwell expiry fights the upshift tables at sustained
    // WOT — the box downshifts to the redline-safe gear, the speed tables
    // upshift as speed climbs, the kickdown downshifts again: gear churn.
    // The stab-delta branch keeps its own one-shot (throttleDeltaHistory_
    // cleared on execution in applyShift).
    if (throttleFraction < profile_.kickdownThrottleThreshold) {
        wotKickdownConsumed_ = false;
    }
    kickdownActive_ =
        shouldKickdown(throttleFraction) && !wotKickdownConsumed_;

    requestsShift_ = false;
    targetGear_ = currentGear_;

    // Guards: only DRIVE shifts (AC6) and only once moving (AC10).
    if (!isShifterInDrive() || speedKmh < profile_.standstillThresholdKmh) {
        logShiftState(throttleFraction, dt, speedKmh);
        return;
    }

    applyShift(decideGear(CommandedRoadSpeed{speedKmh}));
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

// Thin wrapper: gather member state into a ShiftFrame and delegate to the pure
// decide(). The decision reads NO mutable member state — only the frame, the
// torque-hint strategy and the (const) profile.
AutomaticGearbox::ShiftDecision
AutomaticGearbox::decideGear(CommandedRoadSpeed speed) const {
    const ShiftFrame frame{
        currentGear_,
        speed,
        actualEngineRpm_,
        ThrottleState{smoothedThrottle_},
        kickdownActive_,
        rpmFeedbackValid_,
        timeSinceLastShiftS_,
        hasShiftedBefore_,
    };
    return decide(frame, *torqueHint_, profile_);
}

AutomaticGearbox::ShiftDecision
AutomaticGearbox::decide(const ShiftFrame& frame,
                         const ITorqueHint& torqueHint,
                         const IceVehicleProfile& profile) {
    // The TABLE authority is the COMMANDED road speed (frame.speed). The guard
    // authority is the ACTUAL engine feedback rpm (frame.actualRpm). These are
    // distinct strong types (CommandedRoadSpeed vs ActualEngineRpm) — they do
    // not convert, so a commanded-speed value can never silently reach the guard.
    const double speedKmh = frame.speed.kmh;
    const double effThrottle = effectiveThrottle(frame.throttle, torqueHint);

    // 1. Kickdown — the sole override of the tables. A sudden large throttle
    //    increase (or WOT) forces one RPM-safe downshift, even inside the dwell
    //    window: a legitimate power demand cannot wait.
    if (frame.currentGear > 1 && frame.kickdownActive) {
        const int depth = static_cast<int>(profile.kickdownDownshiftGears);
        const int safeGear = findSafeGear(frame.currentGear, speedKmh, depth, profile);
        if (safeGear < frame.currentGear) {
            return {safeGear, -1, true};
        }
    }

    // 2. Minimal dwell blocks table-driven shifts right after any shift, so a
    //    single speed sample cannot thrash the shift valve sub-frame.
    if (frame.hasShiftedBefore &&
        frame.timeSinceLastShiftS < profile.shiftDwellS) {
        return {frame.currentGear, 0, false};
    }

    // 3. Merged anti-lug guard (the death-spiral fix). One declarative floor:
    //    max(idle + margin, downshiftRpmFloor). When the ACTUAL engine RPM
    //    (engine-sim feedback, typed ActualEngineRpm — never a road-speed-derived
    //    value) drops below the floor the engine is lugging (a tall gear held at
    //    low speed). Force ONE downshift so the engine stays above idle under
    //    load, exactly as a real torque-converter box would. One gear per frame
    //    keeps the descent monotonic and prevents oscillation. A STOPPED engine
    //    (actualRpm <= engineStoppedRpm) is not lugging — the stall/restart path
    //    (VirtualIceTwin::kStallRpm) owns that. Guard is inert until engine
    //    feedback is wired (rpmFeedbackValid). Replaces the former two-guard
    //    cascade (idle+margin pre-table, downshiftRpmFloor post-table) with a
    //    single declarative floor.
    const double lugFloor = std::max(profile.idleRpm + profile.lugFloorMarginRpm,
                                     profile.downshiftRpmFloor);
    if (frame.rpmFeedbackValid && frame.currentGear > 1 &&
        isEngineLugging(frame.actualRpm, lugFloor, profile.engineStoppedRpm)) {
        return {frame.currentGear - 1, -1, false};
    }

    // 4. Tables are the authority. Resolve up, then — only if no upshift fired —
    //    down. The downshift table sits at/below the upshift table for every
    //    (gear, throttle) by the init-time invariant, so after an upshift the
    //    speed is above the lower gear's downshift threshold and the down-pass
    //    cannot also fire: the two are mutually exclusive and hunting is
    //    impossible by construction.
    int gear = frame.currentGear;
    while (speedExceedsUpshift(speedKmh, gear, effThrottle, profile)) {
        ++gear;
    }
    if (gear == frame.currentGear) {
        while (gear > 1 && speedBelowDownshift(speedKmh, gear, effThrottle, profile)) {
            --gear;
        }
    }

    int direction = 0;
    if (gear > frame.currentGear) {
        direction = +1;
    } else if (gear < frame.currentGear) {
        direction = -1;
    }
    return {gear, direction, false};
}

bool AutomaticGearbox::speedExceedsUpshift(double speedKmh, int gear, double effThrottle,
                                           const IceVehicleProfile& profile) {
    if (gear >= static_cast<int>(profile.gearRatios.size())) {
        return false;
    }
    if (const double upshiftSpeed = getShiftSpeed(gear, gear + 1, effThrottle, profile);
        upshiftSpeed > 0.0 && speedKmh > upshiftSpeed) {
        return true;
    }
    // Redline safety net, coherent with the speed model: a hard ceiling that
    // only ever upshifts, so it cannot cause hunting.
    return getEngineRpm(speedKmh, gear, profile) > profile.redlineRpm * profile.redlineUpshiftFraction;
}

bool AutomaticGearbox::speedBelowDownshift(double speedKmh, int gear, double effThrottle,
                                           const IceVehicleProfile& profile) {
    if (gear <= 1) {
        return false;
    }
    const double downshiftSpeed = getDownshiftSpeed(gear - 1, gear, effThrottle, profile);
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
        wotKickdownConsumed_ = true;
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

int AutomaticGearbox::findSafeGear(int currentGear, double speedKmh, int maxDownshifts,
                                   const IceVehicleProfile& profile) {
    // Walk down from the current gear, returning the highest gear whose engine
    // speed stays under the kickdown redline fraction. Bounded by maxDownshifts.
    const int floor = std::max(1, currentGear - maxDownshifts);
    for (int gear = currentGear - 1; gear >= floor; --gear) {
        const double rpm = getEngineRpm(speedKmh, gear, profile);
        if (rpm <= profile.redlineRpm * profile.redlineKickdownFraction) {
            return gear;
        }
    }
    return currentGear;
}

bool AutomaticGearbox::isEngineLugging(ActualEngineRpm actualRpm, double floorRpm,
                                       double engineStoppedRpm) {
    // The anti-lug guard. `actualRpm` is the ACTUAL engine-sim feedback rpm
    // (typed ActualEngineRpm, never a road-speed-derived value — passing a
    // `double` such as getEngineRpm(commanded, gear) here is a compile error
    // because ActualEngineRpm has no converting constructor). When the live
    // engine is below the floor it is lugging and a downshift is required.
    // A reading at/below engineStoppedRpm is a STOPPED engine (or feedback not
    // wired — e.g. paths that never call setTwinContext leave it at 0.0).
    // Treating 0 rpm as lugging made the guard fire every frame and pin the box
    // in 1st, preempting the upshift tables. The stopped case is owned by the
    // stall/restart path (VirtualIceTwin::kStallRpm), not this guard.
    return actualRpm.rpm > engineStoppedRpm && actualRpm.rpm < floorRpm;
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
    e.engineRpm = getEngineRpm(speedKmh, currentGear_, profile_);
    e.twinState = twinState_;
    e.clutchPressure = clutchPressureFeedback_;
    if (currentGear_ >= 1 && speedKmh >= 0.1) {
        e.upshiftSpeed = getShiftSpeed(currentGear_, currentGear_ + 1, smoothedThrottle_, profile_);
        if (currentGear_ > 1) {
            e.downshiftSpeed = getDownshiftSpeed(currentGear_ - 1, currentGear_, smoothedThrottle_, profile_);
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

double AutomaticGearbox::getShiftSpeed(int fromGear, int toGear, double throttle,
                                       const IceVehicleProfile& profile) {
    ASSERT(fromGear >= 1 && toGear >= 1 && fromGear < toGear, "getShiftSpeed: gear indexes out of range");
    ASSERT(!profile.shiftTable.empty(), "getShiftSpeed: shift table must be populated");

    size_t tableIndex = static_cast<size_t>(fromGear) - 1;
    if (!profile.shiftTable[0].empty() && tableIndex >= profile.shiftTable[0].size()) {
        tableIndex = profile.shiftTable[0].size() - 1;  // clamp to last valid column
    }

    ASSERT(!profile.shiftTableThrottleLevels.empty(), "getShiftSpeed: throttle levels must be populated");

    return interpolateShiftSpeed(profile.shiftTable, throttle, profile.shiftTableThrottleLevels, tableIndex);
}

double AutomaticGearbox::getDownshiftSpeed(int fromGear, int toGear, double throttle,
                                           const IceVehicleProfile& profile) {
    // Hysteresis fallback: without a separate downshift table, downshift at a
    // fixed fraction of the upshift speed (profile.hysteresisFactor).
    if (!profile.separateDownshiftTableEnabled) {
        return getShiftSpeed(fromGear, toGear, throttle, profile) * profile.hysteresisFactor;
    }

    ASSERT(fromGear >= 1 && toGear >= 1 && fromGear < toGear,
           "getDownshiftSpeed: gear indexes out of range");

    size_t tableIndex = static_cast<size_t>(fromGear) - 1;
    ASSERT(!profile.downshiftTable.empty(), "getDownshiftSpeed: downshift table must be populated");
    if (!profile.downshiftTable[0].empty() && tableIndex >= profile.downshiftTable[0].size()) {
        tableIndex = profile.downshiftTable[0].size() - 1;  // clamp to last valid column
    }
    ASSERT(!profile.downshiftTableThrottleLevels.empty(),
           "getDownshiftSpeed: downshift throttle levels must be populated");

    return interpolateShiftSpeed(profile.downshiftTable, throttle,
                                 profile.downshiftTableThrottleLevels, tableIndex);
}

double AutomaticGearbox::getEngineRpm(double speedKmh, int gear,
                                      const IceVehicleProfile& profile) {
    ASSERT(gear >= 1 && gear <= static_cast<int>(profile.gearRatios.size()),
           "getEngineRpm: gear index out of range");

    const double speedMs = speedKmh / EngineSimDefaults::MS_TO_KMH;
    const double wheelRpm = speedMs / (2.0 * M_PI * profile.tireRadiusM) * 60.0;
    const double engineRpm = wheelRpm * profile.gearRatios[gear - 1] * profile.diffRatio;

    return engineRpm;
}

}
