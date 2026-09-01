#ifndef TWIN_DRIVETRAIN_QUANTITIES_H
#define TWIN_DRIVETRAIN_QUANTITIES_H

namespace twin {

// Strong types for the two road-speed / engine-speed quantities the gearbox
// must NEVER confuse. They are deliberately non-interchangeable: there is no
// converting constructor, no conversion operator, and no arithmetic that
// crosses the boundary. A value of one type CANNOT be passed where the other
// is expected — the compiler rejects it.
//
//   CommandedRoadSpeed — the driver/CSV TARGET road speed (signal.speedKmh).
//                         Feeds the shift TABLES ("shift where you're going").
//   ActualEngineRpm    — the engine-sim FEEDBACK engine speed (rpmFeedback).
//                         Feeds the anti-lug GUARD ("don't let the real engine
//                         lug below idle under load").
//   ThrottleState      — the CONDITIONED (smoothed) throttle fraction fed to
//                         the shift tables. Raw CAN throttle is pre-conditioned
//                         (low-pass + kickdown-delta) before being wrapped here.
//
// The death-spiral bug was the guard reading a speed-derived rpm computed from
// CommandedRoadSpeed; it must read ActualEngineRpm. Because the two types do
// not convert, reintroducing that bug (passing a CommandedRoadSpeed where an
// ActualEngineRpm is required) fails to compile.

struct CommandedRoadSpeed {
    double kmh;
};

struct ActualEngineRpm {
    double rpm;
};

struct ThrottleState {
    double fraction;
};

}  // namespace twin

#endif  // TWIN_DRIVETRAIN_QUANTITIES_H
