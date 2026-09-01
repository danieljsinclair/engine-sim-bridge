// TorqueConverter.cpp - Fluid-coupling torque-converter coupling model.
//
// See TorqueConverter.h for the full physics. This implementation ports the
// rescue-branch torque_converter (origin/torque-converter-rescue, 7fd5abb)
// TR curve + pump law onto the bridge's clutch-pressure coupling axis. The
// bridge runs a velocity-match friction clutch (pressure 0..1), not the
// rescue's SCS torque constraint (lambda in Nm), so the coupling strength is
// expressed as a NORMALIZED capacity scale driven by a smooth, monotonic,
// ROAD-DRIVEN governor ramp (see compute()).

#include <twin/TorqueConverter.h>

#include <algorithm>
#include <cmath>

namespace twin {

namespace {

// MathWorks-standard torque-converter characteristic, salvaged verbatim from
// the rescue branch (src/torque_converter.cpp). Speed ratio
// SR = w_turbine / w_impeller; torque ratio TR = T_out / T_in. Authored for a
// 2.0 stall ratio; buildTorqueRatioTable() rescales it for other stall ratios.
constexpr int kReferenceTableSize = 16;
constexpr double kReferenceSpeedRatio[kReferenceTableSize] = {
    0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7,
    0.8, 0.85, 0.90, 0.92, 0.94, 0.96, 0.97, 1.0
};
constexpr double kReferenceTorqueRatio[kReferenceTableSize] = {
    2.0, 1.85, 1.70, 1.55, 1.40, 1.25, 1.12, 1.02,
    1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.0
};
constexpr double kReferenceStallTorqueRatio = 2.0;

// Hermite smoothstep: 0 at edge0, 1 at edge1, C1-continuous in between. Used for
// every blend so the pressure is a smooth function of its inputs (no bang-bang,
// the property that keeps the converter from oscillating).
inline double smoothstep(double edge0, double edge1, double x) {
    if (edge0 == edge1) return (x >= edge1) ? 1.0 : 0.0;
    const double t = std::clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

}  // namespace

TorqueConverter::TorqueConverter(const TorqueConverterParameters& params)
    : params_(params), torqueRatioTable_{} {
    // Clamp the stall ratio >= 1 so the rescale divisor below is never zero.
    params_.stallTorqueRatio = std::max(params_.stallTorqueRatio, 1.0);
    buildTorqueRatioTable();
}

void TorqueConverter::configure(const TorqueConverterParameters& params) {
    params_ = params;
    params_.stallTorqueRatio = std::max(params_.stallTorqueRatio, 1.0);
    buildTorqueRatioTable();
    lockupEngaged_ = false;
}

void TorqueConverter::buildTorqueRatioTable() {
    // Resample the 16-point reference curve onto a uniform grid so the hot path
    // is an index-and-lerp (rescue-branch scheme, verbatim).
    const double referenceBand = kReferenceStallTorqueRatio - 1.0;
    for (int i = 0; i < kTableResolution; ++i) {
        const double speedRatio =
            static_cast<double>(i) / static_cast<double>(kTableResolution - 1);

        double referenceRatio = 1.0;
        for (int j = 0; j < kReferenceTableSize - 1; ++j) {
            const double lo = kReferenceSpeedRatio[j];
            const double hi = kReferenceSpeedRatio[j + 1];
            if (speedRatio >= lo && speedRatio <= hi) {
                const double t = (speedRatio - lo) / (hi - lo);
                referenceRatio =
                    kReferenceTorqueRatio[j] * (1.0 - t)
                    + kReferenceTorqueRatio[j + 1] * t;
                break;
            }
        }

        // Rescale the curve's multiplication band (1.0 .. referenceStall) onto
        // 1.0 .. this converter's stall ratio.
        const double scaled =
            1.0
            + (referenceRatio - 1.0) * (params_.stallTorqueRatio - 1.0) / referenceBand;
        torqueRatioTable_[i] = std::max(scaled, 1.0);
    }
}

double TorqueConverter::lookupTorqueRatio(double speedRatio) const {
    const double clamped = std::clamp(speedRatio, 0.0, 1.0);
    const double index = clamped * static_cast<double>(kTableResolution - 1);
    const int i0 = static_cast<int>(index);
    const int i1 = std::min(i0 + 1, kTableResolution - 1);
    const double frac = index - static_cast<double>(i0);
    return torqueRatioTable_[i0] * (1.0 - frac) + torqueRatioTable_[i1] * frac;
}

CouplingOutput TorqueConverter::compute(const CouplingInput& input) {
    const double engineRpm = std::max(input.engineRpm, 0.0);
    const double turbineRpm = std::max(input.roadSpeedImpliedRpm, 0.0);

    // Speed ratio (turbine / impeller). At stall SR=0, at lock SR=1.
    // TELEMETRY ONLY — SR is a function of ENGINE rpm, so nothing downstream
    // may key the pressure on it (see the governor below).
    speedRatio_ = (engineRpm > 0.0)
        ? std::clamp(turbineRpm / engineRpm, 0.0, 1.0)
        : 0.0;

    torqueRatio_ = lookupTorqueRatio(speedRatio_);

    // --- Governor: a smooth monotonic ramp of the ROAD-IMPLIED (turbine) rpm --
    // The pressure (the converter's capacity scale) is a function of road-
    // implied rpm ONLY — never engine rpm, never the speed ratio:
    //
    //   * STANDSTILL / CREEP (road < idle*0.9): flat at the MODERATE creep
    //     capacity. The fluid LOADS a flaring engine (the engine-side K*N^2
    //     pump law gives quadratic resistance, so a flaring engine settles at
    //     its stall speed instead of free-revving unloaded) while slipping
    //     against the pinned stationary wheel (no rigid couple, no stall).
    //   * SLIP BAND (idle*0.9 .. idle*1.6): progressive — the converter pulls
    //     the engine toward road speed, transmitting torque at part throttle.
    //   * CRUISE (road >= idle*1.6): flat at 1.0 — locked, stable, no chatter.
    //
    //   * STABILITY: road-implied is EXOGENOUS (CSV-pinned wheels), so the
    //     pressure CANNOT feed back into engine rpm — it cannot form a limit
    //     cycle. The measured bench chatter (Cl 63->100->17->83->92->3->66%
    //     frame-to-frame at 16 mph) came from the previous lockup blend keyed
    //     on the speed ratio (turbine/ENGINE): the engine wandered through the
    //     narrow SR band [0.85, 0.97], the blend swung the pressure 0.03<->1.0,
    //     the capacity step yanked the engine back out of the band, and the
    //     cycle repeated. The same feedback blocked engagement from ever
    //     completing on the way UP: lockup required SR >= 0.85, but SR was low
    //     precisely BECAUSE the unloaded engine was flaring — a self-blocking
    //     deadlock that left the engine at redline at 16 mph (the free-rev
    //     bench failure). A pure road ramp has neither failure mode.
    const double idle = std::max(input.idleRpm, 1.0);
    const double engageStart = idle * params_.engageStartIdleFactor;
    const double lockRpm = idle * params_.lockIdleFactor;

    // Standstill/creep capacity, split by regime on DRIVER THROTTLE (the
    // 2026-08-31 creep-grid resolution of the single-number trap): zero
    // throttle idles on the reduced idleCreepPressure (the stoplight regime —
    // D-idle droop/flow-flutter and the stall-prone standstill load), while
    // a launch (throttle at/above creepThrottleRampEnd) sees the full
    // creepPressure so the part-throttle flare equilibrium stays at the 0.6
    // design point below the free-rev bar. Throttle is exogenous driver
    // input — like road-implied rpm, unlike engine rpm — so the blend cannot
    // close an engine-rpm feedback loop (the no-oscillation guarantee above).
    const double launchCreep = std::clamp(params_.creepPressure, 0.0, 1.0);
    const double idleCreep =
        std::clamp(params_.idleCreepPressure, 0.0, launchCreep);
    const double throttleGate = smoothstep(
        params_.creepThrottleRampStart, params_.creepThrottleRampEnd,
        std::clamp(input.throttleFraction, 0.0, 1.0));
    const double creep = idleCreep + (launchCreep - idleCreep) * throttleGate;
    const double roadGate = smoothstep(engageStart, lockRpm, turbineRpm);
    fluidPressure_ = std::clamp(creep + (1.0 - creep) * roadGate, 0.0, 1.0);

    // --- Lockup STATE (telemetry / display) ----------------------------------
    // Hysteretic on the SAME exogenous road-implied rpm, so the engaged flag
    // cannot chatter with engine feedback either. The pressure itself is the
    // continuous ramp above — the hysteresis never shifts a blend edge.
    if (!params_.lockupEnabled) {
        lockupEngaged_ = false;
    } else if (lockupEngaged_) {
        lockupEngaged_ = turbineRpm >= (lockRpm - params_.lockupHysteresisRpm);
    } else {
        lockupEngaged_ = turbineRpm >= lockRpm;
    }

    return CouplingOutput{fluidPressure_, lockupEngaged_};
}

}  // namespace twin
