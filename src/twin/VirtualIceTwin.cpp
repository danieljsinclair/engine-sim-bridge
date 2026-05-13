#include <twin/VirtualIceTwin.h>
#include <simulator/GearConventions.h>
#include <simulator/EngineSimTypes.h>

namespace twin {

VirtualIceTwin::VirtualIceTwin(const IceVehicleProfile& profile)
    : profile_(profile),
      gearbox_(profile),
      throttleSmoother_(profile.throttleSmoothingTauMs),
      state_(TwinState::OFF) {}

TwinOutput VirtualIceTwin::update(double dt, const input::UpstreamSignal& signal) {
    TwinOutput output;

    if (!signal.isValid || signal.timestampUtcMs == 0) {
        timeWithoutValidTelemetryS_ += dt;
        if (timeWithoutValidTelemetryS_ >= EngineSimDefaults::TELEMETRY_TIMEOUT_S) {
            state_ = TwinState::OFF;
        }
        output.gear = gearbox_.getCurrentGear();
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
        output.gear = gearbox_.getCurrentGear();
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

            // CRANKING->IDLE: engine catches at relatively low RPM
            const double CRANK_IDLE_RPM_THRESHOLD = 500.0;
            const double CRANK_FALLBACK_DURATION_S = 3.0;
            bool rpmCaught = engineRpmFeedback_ > CRANK_IDLE_RPM_THRESHOLD;
            bool timerExpired = (engineRpmFeedback_ == 0.0) && (crankingTimerS_ >= CRANK_FALLBACK_DURATION_S);

            if (rpmCaught || timerExpired) {
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
            double gearboxSpeedKmh = vehicleSpeedFeedbackKmh_ > 0.0 ? vehicleSpeedFeedbackKmh_ : signal.speedKmh;
            gearbox_.update(dt, gearboxSpeedKmh, signal.throttleFraction);
            output.ignition = true;

            // RUNNING->IDLE: selector moved to NEUTRAL or PARK
            if (selector_ == bridge::GearSelector::NEUTRAL ||
                selector_ == bridge::GearSelector::PARK) {
                state_ = TwinState::IDLE;
            } else if (signal.speedKmh == 0.0 && signal.throttleFraction == 0.0) {
                state_ = TwinState::IDLE;
            } else if (gearbox_.requestsShift()) {
                state_ = TwinState::SHIFTING;
                shiftTimerS_ = 0.0;
            }

            output.gear = gearbox_.getCurrentGear();
            clutchPressure_ = 1.0;
            break;
        }

        case TwinState::SHIFTING:
            updateShiftExecution(dt);
            output.ignition = true;
            output.gear = gearbox_.getCurrentGear();
            break;
    }

    output.clutchPressure = clutchPressure_;
    output.gearSelector = selector_;
    return output;
}

void VirtualIceTwin::updateShiftExecution(double dt) {
    shiftTimerS_ += dt;

    double disengageDuration = profile_.shiftDisengageMs * EngineSimDefaults::MS_TO_SECONDS;
    double pauseDuration = profile_.shiftPauseMs * EngineSimDefaults::MS_TO_SECONDS;
    double reengageDuration = profile_.shiftReengageMs * EngineSimDefaults::MS_TO_SECONDS;

    if (shiftTimerS_ <= disengageDuration) {
        clutchPressure_ = 1.0 - (shiftTimerS_ / disengageDuration);
    } else if (shiftTimerS_ <= disengageDuration + pauseDuration) {
        clutchPressure_ = 0.0;
        if (shiftTimerS_ > disengageDuration + pauseDuration / 2.0) {
            gearbox_.update(0, 0, 0);
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
