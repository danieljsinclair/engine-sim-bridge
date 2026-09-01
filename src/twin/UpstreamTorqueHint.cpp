#include <twin/UpstreamTorqueHint.h>

#include <algorithm>
#include <cmath>

namespace twin {

UpstreamTorqueHint::UpstreamTorqueHint(const TorqueInformedGearboxConfig& config,
                                       double motorTorqueNm)
    : config_(config), motorTorqueNm_(motorTorqueNm) {}

double UpstreamTorqueHint::shiftBias() const {
    // OFF contract: indistinguishable from NullTorqueHint — zero bias for
    // every torque value, so the disabled feature is bit-identical.
    if (!config_.enabled) {
        return 0.0;
    }

    // Demand-magnitude convention (deliberately NOT SimTorqueHint's sign
    // convention): BOTH directions of commanded demand are POSITIVE bias.
    // Pulling holds a lower gear; braking reads as braking (earlier downshift),
    // never as lift-off coast — the misread this feature exists to fix.
    const double demandNm = std::abs(motorTorqueNm_);
    if (demandNm < config_.engageTorqueNm) {
        return 0.0;  // true coast: today's exact read
    }

    // Each direction normalizes against its own denominator (braking torque
    // exceeds drive torque in the evidence, so it gets its own scale), then
    // saturates at the throttle-equivalent cap.
    const double denominator =
        (motorTorqueNm_ >= 0.0) ? config_.maxDriveTorqueNm : config_.maxRegenTorqueNm;
    if (denominator <= 0.0) {
        return 0.0;
    }
    return std::clamp(demandNm / denominator, 0.0, 1.0) * config_.maxShiftBias;
}

}  // namespace twin
