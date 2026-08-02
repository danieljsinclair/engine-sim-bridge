#include <twin/VirtualIceTwin.h>
#include <simulator/GearConventions.h>
#include <simulator/EngineSimTypes.h>

namespace twin {

VirtualIceTwin::VirtualIceTwin(IceVehicleProfile profile)
    : profile_(std::move(profile)),
      gearbox_(std::make_unique<AutomaticGearbox>(profile_)),
      throttleSmoother_(profile_.throttleSmoothingTauMs),
      state_(TwinState::OFF) {}

void VirtualIceTwin::reconfigureProfile(const std::vector<double>& gearRatios,
                                          double diffRatio, double tireRadiusM) {
    if (gearRatios.empty()) return;
    profile_.gearRatios = gearRatios;
    profile_.diffRatio = diffRatio;
    profile_.tireRadiusM = tireRadiusM;
    // Auto-generate shift table (same logic as ReplayTelemetryProvider)
    profile_.shiftTableThrottleLevels = {0.05,0.15,0.25,0.40,0.55,0.70,0.80,0.90,0.95,1.00};
    profile_.shiftTable.clear();
    for (double thr : profile_.shiftTableThrottleLevels) {
        std::vector<double> row;
        double shiftRpm = profile_.redlineRpm * (0.40 + 0.45 * thr);
        for (size_t i = 0; i + 1 < gearRatios.size(); ++i) {
            double speedMs = shiftRpm / 60.0 * 2.0 * 3.14159265358979 * tireRadiusM
                           / (gearRatios[i] * diffRatio);
            row.push_back(speedMs * 3.6);
        }
        profile_.shiftTable.push_back(row);
    }
    profile_.separateDownshiftTableEnabled = true;
    profile_.downshiftTableThrottleLevels = profile_.shiftTableThrottleLevels;
    profile_.downshiftTable.clear();
    for (const auto& srcRow : profile_.shiftTable) {
        std::vector<double> row;
        for (double upSpeed : srcRow) row.push_back(upSpeed * 0.70);
        profile_.downshiftTable.push_back(row);
    }
    profile_.hysteresisFactor = 0.85;
    // Reconstruct gearbox with matched profile
    profile_.downshiftTable.clear();
    for (const auto& srcRow : profile_.shiftTable) {
        std::vector<double> row;
        for (double upSpeed : srcRow) row.push_back(upSpeed * 0.70);
        profile_.downshiftTable.push_back(row);
    }
    gearbox_ = std::make_unique<AutomaticGearbox>(profile_);
    gearbox_->setGearSelector(selector_);
    gearbox_->setLogger(nullptr);  // logger not preserved across reconstruct
}

void VirtualIceTwin::setGearboxLogger(IGearboxLogger* logger) {
    gearbox_->setLogger(logger);
}

TwinOutput VirtualIceTwin::update(double dt, const input::UpstreamSignal& signal) {
    TwinOutput output;

    if (!signal.isValid || signal.timestampUtcMs == 0) {
        timeWithoutValidTelemetryS_ += dt;
        if (timeWithoutValidTelemetryS_ >= EngineSimDefaults::TELEMETRY_TIMEOUT_S) {
            state_ = TwinState::OFF;
        }
        output.gear = gearbox_->getCurrentGear();
        return output;
    }

    timeWithoutValidTelemetryS_ = 0.0;

    // Ignition off → force to OFF state, engine dies
    if (!ignitionOn_ && state_ != TwinState::OFF) {
        state_ = TwinState::OFF;
        crankingTimerS_ = 0.0;
    }

    // Stay OFF while ignition is off
    if (!ignitionOn_) {
        output.gear = gearbox_->getCurrentGear();
        output.ignition = false;
        output.clutchPressure = clutchPressure_;
        output.gearSelector = selector_;
        return output;
    }

    throttleSmoother_.update(dt, signal.throttleFraction);
    output.throttle = throttleSmoother_.getCurrentValue();

    switch (state_) {
        case TwinState::OFF:
            state_ = TwinState::CRANKING;
            output.starterMotor = true;
            output.ignition = true;
            output.gear = static_cast<int>(bridge::BridgeGear::NEUTRAL);
            clutchPressure_ = 0.0;
            break;

        case TwinState::CRANKING: {
            crankingTimerS_ += dt;
            output.throttle = EngineSimDefaults::CRANKING_THROTTLE;
            output.starterMotor = true;
            output.ignition = true;

            // CRANKING -> IDLE: the engine catches via EITHER
            //   (a) the physics fast-path — fed-back RPM exceeds the catch
            //       threshold (the closed loop confirms combustion is sustained), OR
            //   (b) the deterministic time fallback — the starter has cranked for
            //       CRANK_FALLBACK_DURATION_S of sim-time. (b) is decoupled from the
            //       RPM value on purpose: with live CSV pacing (one row per frame,
            //       real-time) the fed-back cranking RPM is one tick in arrears and
            //       frequently plateaus below the threshold, so gating the fallback
            //       on the RPM reading (the old `engineRpmFeedback_ == 0.0` guard)
            //       left the twin stuck in CRANKING whenever the plateau was
            //       non-zero-but-low. The time fallback guarantees a deterministic
            //       start: same input frames -> same outcome every run.
            const double CRANK_IDLE_RPM_THRESHOLD = 500.0;
            const double CRANK_FALLBACK_DURATION_S = 3.0;
            const bool rpmCaught = engineRpmFeedback_ > CRANK_IDLE_RPM_THRESHOLD;

            if (const bool fallbackExpired = crankingTimerS_ >= CRANK_FALLBACK_DURATION_S;
                rpmCaught || fallbackExpired) {
                state_ = TwinState::IDLE;
                output.starterMotor = false;
            }
            output.gear = static_cast<int>(bridge::BridgeGear::NEUTRAL);
            clutchPressure_ = 0.0;
            break;
        }

        case TwinState::IDLE:
            output.throttle = throttleSmoother_.getCurrentValue();
            output.ignition = true;
            output.gear = static_cast<int>(bridge::BridgeGear::NEUTRAL);
            clutchPressure_ = 0.0;

            // IDLE->RUNNING: requires selector in DRIVE AND throttle above idle threshold
            if (selector_ == bridge::GearSelector::DRIVE &&
                signal.throttleFraction > profile_.throttleIdleThreshold) {
                state_ = TwinState::RUNNING;
            }
            break;

        case TwinState::RUNNING: {
            double gearboxSpeedKmh = signal.speedKmh;
            gearbox_->setTwinContext(static_cast<int>(state_), clutchPressure_, vehicleSpeedFeedbackKmh_, engineRpmFeedback_);
            gearbox_->setGearSelector(selector_);
            gearbox_->update(dt, gearboxSpeedKmh, signal.throttleFraction, drivetrainTorqueNm_);
            output.ignition = true;

            // RUNNING->IDLE: selector moved to NEUTRAL or PARK
            if (selector_ == bridge::GearSelector::NEUTRAL ||
                selector_ == bridge::GearSelector::PARK) {
                state_ = TwinState::IDLE;
            } else if (signal.speedKmh < profile_.standstillThresholdKmh &&
                       signal.throttleFraction < profile_.throttleIdleThreshold) {
                state_ = TwinState::IDLE;
            } else if (gearbox_->requestsShift()) {
                state_ = TwinState::SHIFTING;
                shiftTimerS_ = 0.0;
            }

            output.gear = gearbox_->getCurrentGear();
            clutchPressure_ = 1.0;
            break;
        }

        case TwinState::SHIFTING:
            gearbox_->setTwinContext(static_cast<int>(state_), clutchPressure_, vehicleSpeedFeedbackKmh_, engineRpmFeedback_);
            updateShiftExecution(dt);
            output.ignition = true;
            output.gear = gearbox_->getCurrentGear();
            break;
    }

    output.clutchPressure = clutchPressure_;
    output.gearSelector = selector_;
    return output;
}

void VirtualIceTwin::updateShiftExecution(double dt) {
    shiftTimerS_ += dt;

    // Determine shift timing based on throttle and shift direction
    double disengageDuration;
    double pauseDuration;
    double reengageDuration;

    double currentThrottle = throttleSmoother_.getCurrentValue();
    int shiftDirection = gearbox_->getLastShiftDirection();
    (void)currentThrottle;
    (void)shiftDirection;

    // Shift timing read from profile vectors
    disengageDuration = profile_.shiftDisengageMs * EngineSimDefaults::MS_TO_SECONDS;
    pauseDuration = profile_.shiftPauseMs * EngineSimDefaults::MS_TO_SECONDS;
    reengageDuration = profile_.shiftReengageMs * EngineSimDefaults::MS_TO_SECONDS;

    if (shiftTimerS_ <= disengageDuration) {
        clutchPressure_ = 1.0 - (shiftTimerS_ / disengageDuration);
    } else if (shiftTimerS_ <= disengageDuration + pauseDuration) {
        clutchPressure_ = 0.0;
        if (shiftTimerS_ > disengageDuration + pauseDuration / 2.0) {
            gearbox_->update(0, 0, 0);
        }
    } else if (shiftTimerS_ <= disengageDuration + pauseDuration + reengageDuration) {
        double reengageProgress = (shiftTimerS_ - disengageDuration - pauseDuration) / reengageDuration;
        clutchPressure_ = reengageProgress;
    } else {
        clutchPressure_ = 1.0;
        state_ = TwinState::RUNNING;
    }
}

}
