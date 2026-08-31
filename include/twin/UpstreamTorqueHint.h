#ifndef TWIN_UPSTREAM_TORQUE_HINT_H
#define TWIN_UPSTREAM_TORQUE_HINT_H

#include <twin/ITorqueHint.h>

namespace twin {

// ============================================================================
// --torque-informed-gearbox (CLI feature toggle, DEFAULT OFF)
//
// Problem: the gearbox shift decision reads the PEDAL as its demand input. On
// Autopilot the pedal sits at 0.00 while the car pulls hard (598 Nm observed,
// pedal=0) or brakes hard (regen -986 Nm with the brake light on, pedal=0).
// Both are misread as COASTING — the exact failure the owner described.
//
// Contract: feed the UPSTREAM COMMANDED motor torque (UpstreamSignal::
// motorTorqueNm — sign AND magnitude) into the existing ITorqueHint seam as a
// DEMAND hint. This is a shift-DECISION input only: it never touches road
// speed, torque injection, the kinematic RPM calc, or the coupling — the
// proven 61-mph failure mode stays impossible (same boundary as SimTorqueHint).
//
// Semantics pinned (demand-magnitude convention — deliberately NOT the
// SimTorqueHint sign convention):
//   torque >= +engageTorqueNm  (AP pulling)  -> bias = +torque/maxDrive * cap
//   torque <= -engageTorqueNm (AP braking)   -> bias = +|torque|/maxRegen * cap
//   |torque| <  engageTorqueNm (true coast)  -> bias = 0 (reads like today)
//   enabled == false                         -> bias = 0 always (byte-identical)
//
// Both demand directions are POSITIVE bias: in the declarative table lookup a
// higher effective throttle raises downshift thresholds (downshifts fire at
// higher speed — "inform downshift like braking does") and raises upshift
// thresholds (no eager upshifts into tall gears while the car pulls). A
// sign-preserving negative bias for regen (the SimTorqueHint coast read:
// earlier upshift) was REJECTED — it would make AP braking look like lifting
// off, the exact coasting misread this feature exists to fix.
//
// Pinned constants and why (road-test evidence, 2026-08-30/31 captures):
//   maxDriveTorqueNm = 600 — same denominator as --effective-throttle
//                           (observed max positive 598 Nm); one story for both
//                           features.
//   maxRegenTorqueNm = 1000 — observed max |negative| torque is 986 Nm (AP
//                           brake event 1, 2026-08-31); braking torque exceeds
//                           drive torque so it gets its own denominator.
//   maxShiftBias = 0.30 — throttle-equivalent cap. Twice SimTorqueHint's 0.15:
//                           the upstream command is the DRIVER'S INTENT
//                           (Autopilot is the driver), not sim-internal
//                           feedback. 0.30 is a firm part-throttle pull —
//                           enough to move highway shift decisions, below
//                           kickdown territory (kickdown stays pedal-owned).
//   engageTorqueNm = 20 — |torque| below this is true coasting (lightest real
//                           commanded torque in evidence is 46 Nm); matches
//                           EffectiveThrottleConfig's engage threshold.
// ============================================================================
struct TorqueInformedGearboxConfig {
    // Master switch. DEFAULT OFF: with enabled=false the hint MUST report
    // zero bias for every torque value (bit-identical decisions).
    bool enabled = false;

    // Normalization denominators (Nm), per torque direction.
    double maxDriveTorqueNm = 600.0;
    double maxRegenTorqueNm = 1000.0;

    // Throttle-equivalent bias cap fed into the declarative shift tables.
    double maxShiftBias = 0.30;

    // |torque| at-or-above this counts as demand; below it reads as coast.
    double engageTorqueNm = 20.0;
};

// ITorqueHint strategy over the UPSTREAM commanded motor torque. Constructed
// per frame from the signal; the gearbox decision consumes only shiftBias().
class UpstreamTorqueHint : public ITorqueHint {
public:
    UpstreamTorqueHint(const TorqueInformedGearboxConfig& config,
                       double motorTorqueNm);

    double shiftBias() const override;

private:
    TorqueInformedGearboxConfig config_;
    double motorTorqueNm_;
};

}  // namespace twin

#endif  // TWIN_UPSTREAM_TORQUE_HINT_H
