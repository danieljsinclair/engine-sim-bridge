#ifndef TWIN_EFFECTIVE_THROTTLE_H
#define TWIN_EFFECTIVE_THROTTLE_H

namespace twin {

// ============================================================================
// --effective-throttle (CLI feature toggle, DEFAULT OFF)
//
// Problem: on Autopilot the recorded pedal (throttle_percent) sits at 0.00
// while the car holds speed via COMMANDED motor torque (evidence: 1,998 rows
// at 95.6 km/h with pedal=0.00 and torque=+134 Nm steady in
// ReadingPickup_2026-08-30/AUTOPILOT-SIG-30s-95kmh.csv). Driving the twin's
// engine from the pedal alone renders the engine SILENT at cruise.
//
// Contract: when the driver's pedal is at the floor-rest deadband AND positive
// motor torque is commanded, derive an EFFECTIVE throttle
//
//     effective = max(pedal, gain * max(torqueNm, 0) / maxAxleTorqueNm)
//
// Negative (regen) torque contributes ZERO effective throttle (regen sounds
// like lift-off; the gearbox-side reading of braking is the separate
// --torque-informed-gearbox feature). The pedal always owns the drive when it
// exceeds the deadband: the torque term is only ever blended in while the
// latch is ENGAGED, and engaging requires pedal <= pedalDeadbandFraction.
//
// The derivation is scoped to the ENGINE DRIVE (TwinOutput.throttle upstream
// of the throttle smoother). It deliberately does NOT touch the gearbox shift
// decision, the coupling model, or the pin target — that separation keeps the
// two torque features independently toggleable and independently provable
// byte-identical when off.
//
// Pinned constants and why (road-test evidence, 2026-08-30/31 captures):
//   maxAxleTorqueNm = 600  — observed max POSITIVE motor torque is 598 Nm
//                            (641 rows, pedal=0). 600 rounds it. The DBC max
//                            (7500 Nm) would render cruise 134 Nm as ~1.8%
//                            throttle — inaudible and wrong; 600 Nm renders
//                            the 134 Nm cruise as 22.3%, a believable
//                            light-cruise throttle, and the 598 Nm AP pull
//                            as 99.7% (~WOT).
//   torqueToThrottleGain = 1.0 — one-for-one normalized torque -> throttle;
//                            no evidence for a different slope.
//   pedalDeadbandFraction = 0.02 — 2% pedal = foot-off rest (CAN jitter sits
//                            below this; real foot pressure reads well above).
//   engageTorqueNm = 20  — AP-takeover ENGAGE threshold. The lightest REAL
//                          commanded torque in the evidence is 46 Nm; 20 sits
//                          below it with margin and above CAN noise.
//   releaseTorqueNm = 10 — AP-release threshold. 10 < 20 gives a 10 Nm
//                          hysteresis band so a torque hovering 10..20 Nm
//                          cannot chatter the latch.
// ============================================================================
struct EffectiveThrottleConfig {
    // Master switch. DEFAULT OFF: with enabled=false the derivation must be a
    // byte-identical no-op (pedal passes through unchanged, latch never arms).
    bool enabled = false;

    // Normalization denominator for POSITIVE motor torque (Nm).
    double maxAxleTorqueNm = 600.0;

    // Blend gain applied to the normalized torque term.
    double torqueToThrottleGain = 1.0;

    // Pedal at-or-below this fraction counts as "foot off" (AP may own the drive).
    double pedalDeadbandFraction = 0.02;

    // Hysteresis: ENGAGE when pedal <= deadband AND torque >= engageTorqueNm;
    // RELEASE when pedal > deadband OR torque < releaseTorqueNm.
    double engageTorqueNm = 20.0;
    double releaseTorqueNm = 10.0;
};

// Stateful per-twin derivation: owns the AP-takeover/release latch so the
// effective throttle cannot chatter around the engage/release thresholds.
// Instantiated with a config; update() is called ONCE per frame at the twin's
// single derivation point (mirrors the brake-light single-derivation pattern
// in SimulationLoop::step).
class EffectiveThrottleDerivation {
public:
    explicit EffectiveThrottleDerivation(const EffectiveThrottleConfig& config);

    // Clear the latch (fresh drive / reconfigure).
    void reset();

    // One frame: pedal fraction [0,1] as recorded, motor torque in Nm as
    // commanded (signed). Returns the throttle fraction the twin should drive
    // the engine with this frame. With config.enabled == false this MUST
    // return pedalFraction unchanged for any torque input.
    double update(double pedalFraction, double motorTorqueNm);

    // True while the latch is engaged (torque is contributing to the drive).
    bool isTorqueDriving() const;

    const EffectiveThrottleConfig& config() const;

private:
    EffectiveThrottleConfig config_;
    bool torqueDriving_ = false;
};

}  // namespace twin

#endif  // TWIN_EFFECTIVE_THROTTLE_H
