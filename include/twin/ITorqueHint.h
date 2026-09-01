#ifndef TWIN_ITORQUE_HINT_H
#define TWIN_ITORQUE_HINT_H

#include <algorithm>
#include <cmath>

namespace twin {

// OCP strategy for the OPTIONAL torque hint that biases the declarative shift
// decision. Torque data may be absent (other vehicles/captures have none), so
// the twin MUST run correctly with NO torque data and use it when present.
//
// The decision consumes ONLY shiftBias() — a throttle-equivalent nudge in the
// range [-maxBias, +maxBias]. It is agnostic to whether torque exists:
// availability is encapsulated in the strategy, never expressed as an
// `if (hasTorque)` branch in the decision logic. A new torque source (a
// different CSV column, a model-based estimate) is a new ITorqueHint
// implementation; the decision is closed for modification.
//
//   shiftBias() > 0  (drive torque)   → effective throttle rises → later
//                                       upshift / earlier downshift
//                                       (holds a lower gear under load).
//   shiftBias() < 0  (regen/braking)  → effective throttle falls  → earlier
//                                       upshift / holds gear.
//   shiftBias() == 0 (no torque data) → no effect (NullTorqueHint).
//
// The hint touches ONLY gear selection (the shift-table lookup). It never
// alters road speed, torque injection, or the kinematic RPM calc — the proven
// 61-mph failure mode stays impossible.
class ITorqueHint {
public:
    virtual ~ITorqueHint() = default;
    virtual double shiftBias() const = 0;
};

// No torque data. Correct behaviour with zero torque information — the default
// so the twin runs standalone (e.g. the replay validation path, which passes
// torque == 0.0 every frame).
class NullTorqueHint : public ITorqueHint {
public:
    double shiftBias() const override { return 0.0; }
};

// Torque hint derived from a measured/estimated drivetrain torque (signed).
// Produces the bounded throttle-equivalent nudge the decision adds to the
// smoothed throttle before the table lookup. Bounded by (maxNm, maxBias) which
// are declarative config (IceVehicleProfile::torqueHintMaxNm / torqueHintMaxBias).
class SimTorqueHint : public ITorqueHint {
public:
    SimTorqueHint(double drivetrainTorqueNm, double maxNm, double maxBias)
        : torqueNm_(drivetrainTorqueNm), maxNm_(maxNm), maxBias_(maxBias) {}

    double shiftBias() const override {
        const double norm = (maxNm_ > 0.0)
            ? std::clamp(std::abs(torqueNm_) / maxNm_, 0.0, 1.0)
            : 0.0;
        const double sign = (torqueNm_ >= 0.0) ? 1.0 : -1.0;
        return sign * norm * maxBias_;
    }

private:
    double torqueNm_;
    double maxNm_;
    double maxBias_;
};

}  // namespace twin

#endif  // TWIN_ITORQUE_HINT_H
