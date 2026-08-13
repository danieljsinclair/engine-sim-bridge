// TorqueConverter.cpp - Fluid-coupling torque-converter coupling model.
//
// See TorqueConverter.h for the full physics. This implementation ports the
// rescue-branch torque_converter (origin/torque-converter-rescue, 7fd5abb)
// TR curve + pump law + hysteretic lockup onto the bridge's clutch-pressure
// coupling axis. The bridge runs a velocity-match friction clutch (pressure
// 0..1), not the rescue's SCS torque constraint (lambda in Nm), so the K*N^2
// pump torque is expressed as a NORMALIZED pressure that preserves the
// quadratic-in-engine-speed shape and the TR-modulated multiplication; the
// absolute capacity is the declarative `capacityPressure` knob.

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

inline double lerp(double a, double b, double t) { return a + (b - a) * t; }

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
    speedRatio_ = (engineRpm > 0.0)
        ? std::clamp(turbineRpm / engineRpm, 0.0, 1.0)
        : 0.0;

    torqueRatio_ = lookupTorqueRatio(speedRatio_);

    // --- Fluid coupling pressure (slip regime: launch + coupling) -------------
    // The fluid pressure is a SMOOTH, ROAD-DRIVEN ramp - a single smoothstep of
    // the road-implied (turbine) rpm, scaled by the capacity. It is deliberately
    // a function of ROAD-IMPLIED rpm ONLY (not engine rpm, not the speed ratio):
    //
    //   * STANDSTILL / CREEP: road-implied < idle => roadGate 0 => pressure at the
    //     floor. The fluid coupling has no capacity at zero turbine speed, so the
    //     engine idles DECOUPLED - a post-crank flare cannot drag it to stall (the
    //     standstill-stall fix), and a creeping engine is not lugged below idle
    //     (the creep-lug fix).
    //   * STABILITY: road-implied is EXOGENOUS (CSV-pinned wheels), so the pressure
    //     CANNOT feed back into engine rpm - it cannot form a limit cycle. Any
    //     engine-rpm term (a pump law, or the speed-ratio-based torque-ratio
    //     factor) closes a feedback loop: the engine lugs -> the pressure shifts ->
    //     the lug shifts -> a cycle. The 1442->70 yank measured on the prior
    //     attempt came from exactly such an engine-rpm term (a "pump gate" that
    //     collapsed pressure on lug, then re-engaged on recovery - bang-bang).
    //     Dropping every engine-rpm term makes the schedule open-loop stable; the
    //     friction clutch then converges by pulling the engine TOWARD road speed
    //     (it cannot drag it below road), instead of cycling.
    //
    // The TC's torque-ratio (stall multiplication) curve is kept as a TELEMETRY
    // quantity (getTorqueRatio / the TR-curve unit test) but is NOT folded into
    // the pressure: TR is a function of the speed ratio (turbine/impeller), so
    // multiplying by it would reintroduce engine-rpm feedback and the oscillation
    // above. On the friction-clutch axis the stable mapping is the road-driven
    // ramp here; the lockup clutch (below) supplies the cruise lock.
    //
    // The capacity is SMALL: the bridge's friction clutch transmits ~= pressure *
    // maxClutchTorque (12000 Nm here), so 0.025 pressure ~= 300 Nm - enough to
    // pull the engine to road speed through the coupling band, without the violent
    // overshoot a larger capacity produced (0.12 -> 1440 Nm -> the 1442->70 yank).
    const double idle = std::max(input.idleRpm, 1.0);
    const double roadGate = smoothstep(idle, idle * 1.6, turbineRpm);

    const double capacityPressure = std::clamp(params_.capacityPressure,
                                               params_.pressureFloor, 1.0);
    fluidPressure_ = std::clamp(
        params_.pressureFloor
            + (capacityPressure - params_.pressureFloor) * roadGate,
        params_.pressureFloor, capacityPressure);

    // --- Lockup clutch (cruise) ----------------------------------------------
    // Above a speed ratio and an impeller RPM the converter locks the pump to the
    // turbine directly (1:1, no slip). Two SEPARATE concerns:
    //   (1) the lockup STATE (binary, hysteretic) - drives the `locked` flag and
    //       prevents the clutch plate's engaged indicator from chattering;
    //   (2) the lockup PRESSURE BLEND (continuous, fixed smoothstep edges) -
    //       ramps the clutch pressure from the fluid value up to full lock.
    // Decoupling them is what keeps the pressure smooth: the hysteresis lives in
    // the state machine only and NEVER shifts a blend edge (shifting an edge
    // would re-introduce a discontinuity, the very bang-bang we are avoiding).
    if (!params_.lockupEnabled) {
        lockupEngaged_ = false;
    } else if (lockupEngaged_) {
        // Release when EITHER RPM or SR falls below its (lowered) release floor.
        const bool rpmReleased =
            engineRpm < (params_.lockupRpm - params_.lockupHysteresisRpm);
        const bool slipReleased =
            speedRatio_ < (params_.lockupSpeedRatio - params_.lockupHysteresisSpeedRatio);
        lockupEngaged_ = !(rpmReleased || slipReleased);
    } else {
        lockupEngaged_ = engineRpm >= params_.lockupRpm
                         && speedRatio_ >= params_.lockupSpeedRatio;
    }

    // Continuous blend over FIXED edges (no hysteresis shift). Rises through the
    // SR band just above the lock point, gated by impeller speed. A real lockup
    // clutch applies its plate pressure over ~100 ms; this smooth band models
    // that, so the engine never sees a step.
    // Blend over the SR band [lockupSR, lockupSR + 0.12] so lockup is essentially
    // complete by SR ~ 0.95 (a real TC is locked by then). Width 0.12 keeps the
    // per-frame pressure delta well under a bang-bang step (the no-oscillation
    // bar). Gated by impeller speed via a smooth RPM factor.
    const double lockupSrHigh = std::min(params_.lockupSpeedRatio + 0.12, 1.0);
    const double lockupBlend = params_.lockupEnabled
        ? smoothstep(params_.lockupSpeedRatio, lockupSrHigh, speedRatio_)
          * smoothstep(params_.lockupRpm * 0.9, params_.lockupRpm, engineRpm)
        : 0.0;

    const double pressure = std::clamp(
        lerp(fluidPressure_, 1.0, lockupBlend), 0.0, 1.0);

    return CouplingOutput{pressure, lockupEngaged_};
}

}  // namespace twin
