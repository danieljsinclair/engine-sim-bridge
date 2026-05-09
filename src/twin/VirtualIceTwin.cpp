#include <twin/VirtualIceTwin.h>

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
        if (timeWithoutValidTelemetryS_ >= 5.0) {
            state_ = TwinState::OFF;
        }
        output.gear = gearbox_.getCurrentGear();
        return output;
    }

    timeWithoutValidTelemetryS_ = 0.0;

    throttleSmoother_.update(dt, signal.throttleFraction);
    output.throttle = throttleSmoother_.getCurrentValue();

    switch (state_) {
        case TwinState::OFF:
            state_ = TwinState::CRANKING;
            output.starterMotor = true;
            output.ignition = true;
            output.gear = 1;
            clutchPressure_ = 1.0;
            break;

        case TwinState::CRANKING: {
            double rpm = profile_.idleRpm;
            if (rpm > 550.0) {
                state_ = TwinState::IDLE;
                output.starterMotor = false;
            }
            output.gear = 1;
            clutchPressure_ = 1.0;
            break;
        }

        case TwinState::IDLE:
            if (signal.throttleFraction > profile_.throttleIdleThreshold) {
                state_ = TwinState::RUNNING;
            }
            output.gear = 1;
            clutchPressure_ = 1.0;
            break;

        case TwinState::RUNNING: {
            gearbox_.update(dt, signal.speedKmh, signal.throttleFraction);

            if (signal.speedKmh == 0.0 && signal.throttleFraction == 0.0) {
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
            output.gear = gearbox_.getCurrentGear();
            break;
    }

    output.clutchPressure = clutchPressure_;
    return output;
}

void VirtualIceTwin::updateShiftExecution(double dt) {
    shiftTimerS_ += dt;

    double disengageDuration = profile_.shiftDisengageMs / 1000.0;
    double pauseDuration = profile_.shiftPauseMs / 1000.0;
    double reengageDuration = profile_.shiftReengageMs / 1000.0;

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
