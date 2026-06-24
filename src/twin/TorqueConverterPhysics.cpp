#include "twin/TorqueConverterPhysics.h"
#include <algorithm>
#include <cmath>

namespace twin {

// Reference torque ratio curve (speedRatio -> torqueRatio)
// Based on MathWorks Simscape default, scaled to stallTorqueRatio
static constexpr double s_speedPoints[] = {
    0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.85, 0.90, 0.92, 0.94, 0.96, 0.97, 1.0
};
static constexpr double s_torquePoints[] = {
    2.0, 1.85, 1.70, 1.55, 1.40, 1.25, 1.12, 1.02, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00
};
static constexpr int s_nPoints = 16;

static double interpolateTorqueRatio(double speedRatio, double stallTorqueRatio) {
    speedRatio = std::clamp(speedRatio, 0.0, 1.0);

    // Find bracketing points
    int idx = 0;
    for (int i = 0; i < s_nPoints - 1; ++i) {
        if (speedRatio >= s_speedPoints[i] && speedRatio <= s_speedPoints[i + 1]) {
            idx = i;
            break;
        }
        if (i == s_nPoints - 2) idx = i;
    }

    double t = (s_speedPoints[idx + 1] > s_speedPoints[idx])
        ? (speedRatio - s_speedPoints[idx]) / (s_speedPoints[idx + 1] - s_speedPoints[idx])
        : 0.0;

    double baseTR = s_torquePoints[idx] * (1.0 - t) + s_torquePoints[idx + 1] * t;

    // Scale to actual stall torque ratio (reference assumes 2.0)
    return 1.0 + (baseTR - 1.0) * (stallTorqueRatio - 1.0);
}

TorqueConverterOutput computeTorqueConverter(const TorqueConverterInput& input) {
    TorqueConverterOutput output;

    // Guard: non-negative RPM
    double inputRpm = std::max(input.engineRpm, 0.0);
    double outputRpm = std::max(input.turbineRpm, 0.0);

    // Speed ratio: 0 at stall, ~1 at coupling
    output.speedRatio = (inputRpm > 0.0) ? outputRpm / inputRpm : 0.0;
    output.speedRatio = std::clamp(output.speedRatio, 0.0, 1.0);

    // Slip
    output.slip = 1.0 - output.speedRatio;

    // Torque ratio from lookup curve
    output.torqueRatio = interpolateTorqueRatio(output.speedRatio, input.stallTorqueRatio);

    // Capacity: K * N^2 (fluid coupling law)
    double capacity = input.capacityFactor * inputRpm * inputRpm;
    output.maxInputTorque = std::min(capacity, input.maxInputTorque);

    // Turbine torque = input torque * torque ratio
    // At equilibrium: engine_torque = inputTorque, turbine_torque = inputTorque * torqueRatio
    output.turbineTorque = output.maxInputTorque * output.torqueRatio;

    return output;
}

} // namespace twin
