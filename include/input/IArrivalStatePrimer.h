// IArrivalStatePrimer.h - Instant --start-from seam for file-trace providers.
//
// WHY THIS INTERFACE EXISTS (ISP + DIP):
//   The owner's --start-from contract: rows before the offset must NEVER be
//   simulated — the sim must drop in at ANY offset (start, middle, end) in
//   ~instant wall time, because a capture may only contain the middle of a
//   drive. The old loop-side warm-start prefix (stepping every frame from 0
//   to the offset at CPU speed, ~0.35x real time) violated that contract, and
//   the owner has rejected the trade ("Shove your negative flow basin - find
//   another way").
//
//   The replacement splits the arrival-state synthesis at the provider seam:
//   the PROVIDER owns everything derived from the trace (which row is the
//   arrival row, the twin warm-boot seeded from that row, the display clock
//   cold-jump to the arrival timecode), while the LOOP owns the engine-core
//   settle at the held arrival operating point (see SimulationLoop). This
//   interface is the narrow contract between them — the loop depends on the
//   abstraction, not on ReplayTelemetryProvider (Dependency Inversion), and
//   live streams keep their own instant-skip path (a live stream cannot be
//   seeked, only consumed) without implementing this.
//
// CONTRACT:
//   - primeArrivalState(): seed provider-side state from the FIRST row at or
//     after --start-from (twin warm-boot at that row's throttle/road speed/
//     selector; replay clock anchored on the arrival row's timecode) and HOLD
//     the arrival row as the synthetic input: every OnUpdateSimulation call
//     returns that row unchanged while the hold is active, so the loop can
//     settle the engine core at a CONSTANT operating point. No pre-offset row
//     is ever read through the sampling path.
//   - releaseArrivalHold(): end the hold. The clock resumes advancing from
//     the arrival row; the next OnUpdateSimulation poll emits the trace from
//     the arrival row onward.
//   - No-op when no offset is set (startFromS <= 0) — from-0 behavior is
//     byte-identical to a provider that never heard of this interface.

#ifndef INPUT_I_ARRIVAL_STATE_PRIMER_H
#define INPUT_I_ARRIVAL_STATE_PRIMER_H

namespace input {

class IArrivalStatePrimer {
public:
    virtual ~IArrivalStatePrimer() = default;

    /// Prime provider-side state from the arrival row and freeze the replay
    /// clock on it (constant synthetic input) until releaseArrivalHold().
    virtual void primeArrivalState() = 0;

    /// Release the arrival hold: the replay clock resumes advancing from the
    /// arrival row and rows are emitted from there onward.
    virtual void releaseArrivalHold() = 0;
};

}  // namespace input

#endif  // INPUT_I_ARRIVAL_STATE_PRIMER_H
