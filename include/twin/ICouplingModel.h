// ICouplingModel.h - Strategy interface for how the live twin computes the
// engine<->drivetrain clutch pressure each frame.
//
// This is the COUPLING axis (how the clutch pressure is derived), orthogonal to
// the WHEEL-COUPLING axis (IWheelCoupling: which wheel speed drives the math).
// Three coupling models sit behind this interface, selected by the
// --coupling-model CLI toggle:
//
//   legacy            - the SlipLockController + TorqueConverterLaunch pair
//                       (velocity-match pressure schedule, the original).
//   clutch-map        - a declarative clutch-pressure map (see ClutchMapModel).
//   torque-converter  - a fluid-coupling model (pump/turbine + TR/K curves,
//                       see TorqueConverter) ported from the rescue branch.
//
// The contract mirrors what VirtualIceTwin already feeds the slip-lock / launch
// functions today, so a coupling model is a drop-in replacement for that pair.
// The twin injects the model (DI) and carries no coupling conditional (OCP): a
// new model is added by implementing this interface + a factory branch.

#ifndef TWIN_I_COUPLING_MODEL_H
#define TWIN_I_COUPLING_MODEL_H

namespace twin {

// Per-frame inputs to the coupling computation. All values are in the units the
// twin already uses (RPM, 0..1 throttle, seconds).
struct CouplingInput {
    double engineRpm = 0.0;             // Current engine RPM (feedback).
    double roadSpeedImpliedRpm = 0.0;   // RPM the engine would be at if locked to road speed in current gear (turbine side).
    double throttleFraction = 0.0;      // Driver torque demand, 0..1.
    double idleRpm = 0.0;               // Engine idle RPM.
    double redlineRpm = 0.0;            // Engine redline RPM.
    double maxClutchTorqueNm = 0.0;     // Clutch torque at full pressure (1.0), Nm. 0 = use model default.
    double dt = 0.0;                    // Frame dt, seconds.
};

// Per-frame coupling output. The twin consumes clutchPressure as the desired
// pressure (it still rate-limits the actual value toward it); `locked` is
// telemetry (display / state).
struct CouplingOutput {
    double clutchPressure = 0.0;        // 0..1 (0 = open, 1 = fully locked).
    bool locked = false;                // true when the model considers the coupling locked (for display).
};

class ICouplingModel {
public:
    virtual ~ICouplingModel() = default;

    // Compute the desired clutch coupling for this frame. MUST be smooth in its
    // inputs (no bang-bang, no thresholds) so the engine RPM does not cycle.
    virtual CouplingOutput compute(const CouplingInput& input) = 0;
};

}  // namespace twin

#endif  // TWIN_I_COUPLING_MODEL_H
