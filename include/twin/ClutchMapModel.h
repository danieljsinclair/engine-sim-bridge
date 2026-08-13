// ClutchMapModel.h - Declarative clutch-pressure governor map (the default
// coupling model), implementing the ICouplingModel contract.
//
// WHY: the legacy live clutch path ends in a bang-bang "creep-drag relief" that
// slams the clutch fully open (pressure 0) when the engine lugs, then re-engages
// when it recovers, then lugs again -> violent RPM oscillation (0<->redline).
// That is structurally incapable of stabilising: any threshold controller bangs
// between the two sides. A real automatic gearbox does NOT threshold — a
// hydraulic governor modulates the clutch pressure PROPORTIONALLY: a smooth,
// continuous curve of road speed (and throttle). This class models THAT.
//
// THE CURVE (a governor map, no if/else, no thresholds in the evaluation):
//   The pressure [0,1] is a C1-continuous blend of two smooth sub-curves:
//
//   1. Lock-up curve  — the wheels are rolling. Pressure ramps from the floor
//      (at idle-implied) up to 1.0 (fully locked) as road-implied RPM rises from
//      idle to idle*lockEngageIdleFactor, via a smoothstep. Above the lock point
//      the clutch is locked (direct coupling, engine braking). Driven off ROAD
//      IMPPLIED rpm (not slip) so a free-revving engine cannot deadlock the
//      pressure low (the old cruise free-rev bug).
//
//   2. Creep curve    — standstill / sub-idle. Pressure = floor + throttle*gain
//      (capped): a torque-converter-like fluid coupling that transmits creep
//      torque at 0 road speed proportional to throttle, so the engine is LOADED
//      (holds idle, slips) rather than free-revving.
//
//   A smoothstep blend weight (0 below idle, where creep owns the sub-idle
//   regime -> 1 at the lock point) cross-fades creep -> lock. The whole
//   expression is C1-continuous: there is no branch, no step, no threshold, so
//   there is nothing to bang between -> no limit cycle. The pressure is also
//   FLOORED (never 0): a fully-open clutch decouples the engine and lets it
//   free-rev (measured), which this map forbids.
//
// All thresholds/ratios are FIELDS of ClutchMapParameters (declarative), not
// magic constants in the logic — tune the governor by editing the struct.

#ifndef TWIN_CLUTCH_MAP_MODEL_H
#define TWIN_CLUTCH_MAP_MODEL_H

#include <twin/ICouplingModel.h>

namespace twin {

// Declarative config for the governor curve. Defaults model a hydraulic
// governor: minimum line pressure at stall (creep), progressive lock-up as road
// speed builds, fully locked at cruise.
struct ClutchMapParameters {
    // Minimum clutch pressure (used by BOTH the creep and lock curves). The
    // clutch is NEVER fully open (0 would decouple the engine and let it free-rev
    // to redline on the idle-sustain throttle). At standstill this floor
    // transmits creep torque so the engine is loaded. Tuned to load — not lug —
    // the engine at idle (0.05 mirrors the proven SlipLockController floor).
    double pressureFloor = 0.05;

    // Creep (sub-idle / standstill) pressure is THROTTLE-PROPORTIONAL — a real
    // torque converter's stall law (transmitted torque grows with throttle). At
    // throttle=0 (decelerating to a stop) the creep pressure is 0, so the engine
    // idles DECOUPLED from the slow/stopped wheels: no lug. This is what a fixed
    // floor CANNOT do under PIN-pinned decelerating wheels (any nonzero floor
    // couples the engine to the stopping wheels and drags it to stall). At
    // throttle>0 (launch) the pressure rises, loading the engine so it cannot
    // free-rev. creepThrottleGain is the pressure per unit throttle; creepCeiling
    // caps it.
    double creepThrottleGain = 0.10;
    double creepCeiling = 0.18;

    // Lock-engage point as a multiple of idle. Once road-implied RPM reaches
    // idle*lockEngageIdleFactor the clutch is fully locked (pressure -> 1.0).
    // Between idle and this point the pressure ramps smoothly (launch slip band).
    double lockEngageIdleFactor = 1.6;
};

// Stateless declarative clutch-pressure map. compute() is a pure function of its
// inputs (declared non-const only to match the ICouplingModel signature, which
// the stateful TorqueConverter model requires).
class ClutchMapModel : public ICouplingModel {
public:
    explicit ClutchMapModel(ClutchMapParameters params = ClutchMapParameters{});

    // ICouplingModel: the smooth governor pressure for this frame. Always in
    // [pressureFloor, 1.0]; `locked` is true once road-implied >= lock point.
    CouplingOutput compute(const CouplingInput& input) override;

    const ClutchMapParameters& parameters() const { return params_; }

private:
    ClutchMapParameters params_;
};

}  // namespace twin

#endif  // TWIN_CLUTCH_MAP_MODEL_H
