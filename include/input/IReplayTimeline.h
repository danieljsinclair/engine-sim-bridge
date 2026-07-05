// IReplayTimeline.h - Injectable seam for replay time-slice validation.
//
// WHY THIS INTERFACE EXISTS (ISP + DIP):
//   The CLI's validateReplayTimeSlicing() needs only two things from a replay
//   source: the trace's total duration, and the ability to clamp the end time.
//   ReplayTelemetryProvider (which lives in the bridge) carries far more surface
//   than that. This interface exposes only the two methods the validator needs
//   (Interface Segregation) and lives with its implementer in the bridge layer
//   so the dependency points the right way: both the CLI validator and the
//   concrete provider depend on this abstraction (Dependency Inversion), and the
//   bridge never reaches up into the CLI layer.
//
// CONTRACT:
//   - durationS(): total span of the parsed trace, in seconds. 0 if empty.
//   - setEndAtS(s): set the stop time. -1 disables the clamp (play to end).

#ifndef INPUT_I_REPLAY_TIMELINE_H
#define INPUT_I_REPLAY_TIMELINE_H

namespace input {

class IReplayTimeline {
public:
    virtual ~IReplayTimeline() = default;

    /// Total span of the parsed trace, in seconds. 0 if empty.
    virtual double durationS() const = 0;

    /// Set the stop time in seconds. -1 disables the clamp (play to end).
    virtual void setEndAtS(double s) = 0;
};

} // namespace input

#endif  // INPUT_I_REPLAY_TIMELINE_H
