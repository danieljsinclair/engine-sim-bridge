#include <twin/EffectiveThrottle.h>

#include <algorithm>

namespace twin {

EffectiveThrottleDerivation::EffectiveThrottleDerivation(const EffectiveThrottleConfig& config)
    : config_(config) {}

void EffectiveThrottleDerivation::reset() {
    torqueDriving_ = false;
}

double EffectiveThrottleDerivation::update(double pedalFraction, double motorTorqueNm) {
    // OFF contract: the pedal passes through bit-identical for ANY torque and
    // the latch never arms — the disabled feature is provably inert.
    if (!config_.enabled) {
        torqueDriving_ = false;
        return pedalFraction;
    }

    // Pedal ownership: real foot pressure (above the deadband) releases the
    // latch and the drive is exactly the pedal. The torque term only ever
    // fills the silence under the deadband; it can never override the driver.
    if (pedalFraction > config_.pedalDeadbandFraction) {
        torqueDriving_ = false;
        return pedalFraction;
    }

    // Takeover latch with hysteresis: engage at/above engageTorqueNm, hold
    // through the release..engage band, release only below releaseTorqueNm —
    // a torque hovering inside the band cannot chatter the drive.
    if (!torqueDriving_) {
        if (motorTorqueNm < config_.engageTorqueNm) {
            return pedalFraction;  // true coast: drive falls back to the pedal
        }
        torqueDriving_ = true;
    } else if (motorTorqueNm < config_.releaseTorqueNm) {
        torqueDriving_ = false;
        return pedalFraction;
    }

    // Blend while latched: the larger of pedal and the normalized POSITIVE
    // torque term, saturated at full throttle. Negative (regen) torque
    // contributes zero — braking is the gearbox feature's demand, never the
    // engine drive.
    const double normalized =
        (config_.maxAxleTorqueNm > 0.0)
            ? std::clamp(config_.torqueToThrottleGain
                             * std::max(motorTorqueNm, 0.0) / config_.maxAxleTorqueNm,
                         0.0, 1.0)
            : 0.0;
    return std::max(pedalFraction, normalized);
}

bool EffectiveThrottleDerivation::isTorqueDriving() const {
    return torqueDriving_;
}

const EffectiveThrottleConfig& EffectiveThrottleDerivation::config() const {
    return config_;
}

}  // namespace twin
