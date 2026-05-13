#include <twin/ManualTwin.h>
#include <algorithm>

namespace twin {

TwinOutput ManualTwin::update(double dt, const TwinFeedback& feedback) {
    TwinOutput output;

    if (!feedback.isValid) {
        timeWithoutValidFeedbackS_ += dt;
        if (timeWithoutValidFeedbackS_ >= INVALID_FEEDBACK_TIMEOUT_S) {
            state_ = TwinState::OFF;
        }
        output.gear = currentGear_;
        return output;
    }

    timeWithoutValidFeedbackS_ = 0.0;

    // Process gear requests
    if (gearUpRequested_) {
        gearUpRequested_ = false;
        if (currentGear_ < static_cast<int>(bridge::BridgeGear::EIGHTH)) {
            currentGear_++;
        }
    }
    if (gearDownRequested_) {
        gearDownRequested_ = false;
        if (currentGear_ > static_cast<int>(bridge::BridgeGear::NEUTRAL)) {
            currentGear_--;
        }
    }

    switch (state_) {
        case TwinState::OFF:
            // OFF -> CRANKING: auto-start if engine RPM indicates warmup already ran,
            // or user explicitly requests ignition+starter
            if (feedback.engineRpm > IDLE_RPM_THRESHOLD) {
                // Engine already running from warmup — go straight to RUNNING
                state_ = TwinState::RUNNING;
                ignitionRequested_ = true;
                starterRequested_ = false;
                [[fallthrough]];
            } else if (ignitionRequested_ && starterRequested_) {
                state_ = TwinState::CRANKING;
                [[fallthrough]];
            } else {
                output.gear = static_cast<int>(bridge::BridgeGear::NEUTRAL);
                currentGear_ = static_cast<int>(bridge::BridgeGear::NEUTRAL);
                output.gearSelector = bridge::GearSelector::NEUTRAL;
                break;
            }

        case TwinState::CRANKING:
            output.throttle = std::clamp(inputThrottle_, 0.0, 1.0);
            output.starterMotor = starterRequested_;
            output.ignition = ignitionRequested_;
            output.gear = static_cast<int>(bridge::BridgeGear::NEUTRAL);
            output.gearSelector = bridge::GearSelector::NEUTRAL;
            output.clutchPressure = 0.0;

            // Auto-release starter when engine catches
            if (feedback.engineRpm > IDLE_RPM_THRESHOLD) {
                state_ = TwinState::RUNNING;
                output.starterMotor = false;
                output.gear = currentGear_;
                output.gearSelector = static_cast<bridge::GearSelector>(currentGear_);
                output.clutchPressure = 1.0;
            }

            // User turned off ignition -> back to OFF
            if (!ignitionRequested_) {
                state_ = TwinState::OFF;
            }
            break;

        case TwinState::IDLE:
            state_ = TwinState::RUNNING;
            [[fallthrough]];

        case TwinState::RUNNING:
            output.throttle = std::clamp(inputThrottle_, 0.0, 1.0);
            output.starterMotor = false;
            output.ignition = true;
            output.gear = currentGear_;
            output.gearSelector = static_cast<bridge::GearSelector>(currentGear_);
            output.clutchPressure = 1.0;

            if (!ignitionRequested_) {
                state_ = TwinState::OFF;
            }
            break;

        case TwinState::SHIFTING:
            state_ = TwinState::RUNNING;
            output.throttle = std::clamp(inputThrottle_, 0.0, 1.0);
            output.gear = currentGear_;
            output.gearSelector = static_cast<bridge::GearSelector>(currentGear_);
            output.clutchPressure = 1.0;
            break;
    }

    return output;
}

TwinState ManualTwin::getState() const {
    return state_;
}

void ManualTwin::setThrottle(double throttle) {
    inputThrottle_ = throttle;
}

void ManualTwin::setGear(int gear) {
    inputGear_ = gear;
    currentGear_ = gear;
}

void ManualTwin::requestGearUp() {
    gearUpRequested_ = true;
}

void ManualTwin::requestGearDown() {
    gearDownRequested_ = true;
}

void ManualTwin::setIgnition(bool on) {
    ignitionRequested_ = on;
}

void ManualTwin::setStarterMotor(bool on) {
    starterRequested_ = on;
}

} // namespace twin
