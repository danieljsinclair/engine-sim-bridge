// PresentationStateBuilders.h - Domain-specific EngineState construction helpers
// SRP: Each function builds one domain sub-struct from its authoritative source
// Pure functions: no side effects, no mutations to simulator or stats

#ifndef PRESENTATION_STATE_BUILDERS_H
#define PRESENTATION_STATE_BUILDERS_H

#include "io/IPresentation.h"
#include "simulator/EngineSimTypes.h"
#include "simulation/CrankingController.h"
#include "simulation/EnginePhase.h"
#include "simulation/SimulationLoop.h"
#include "io/IInputProvider.h"
#include "telemetry/ITelemetryProvider.h"
#include "strategy/IAudioBuffer.h"

#include <cstdint>

class ISimulator;

namespace presentation {
namespace builders {

// ---- CSV column semantics (F8 consolidation) -------------------------------
// Machine-parseable presentation companions: plain-text phase naming and the
// pipe-latency derivation. These coexist with — and deliberately do NOT
// converge to — the console presentation: the console renders the colored
// EnginePhaseName (EnginePhase.h), while the CSV needs ANSI-free text where
// Rollover has historically rendered as "Unknown". Both forms are pinned by
// characterization nets and must stay byte-identical.

// Plain-text phase name for the CSV engine_state column (no ANSI). Unlike the
// console's colored EnginePhaseName, Rollover (and any invalid value) has no
// case here and renders "Unknown" — the historical, net-pinned CSV behaviour.
constexpr const char* csvPhaseName(EnginePhase phase) noexcept {
    switch (phase) {
        case EnginePhase::Stopped:  return "Stopped";
        case EnginePhase::Cranking: return "Cranking";
        case EnginePhase::Stopping: return "Stopping";
        case EnginePhase::Running:  return "Running";
        default: return "Unknown";
    }
}

// latency_ms identity: wall_clock_ms - input_timestamp_ms, both derived from
// ONE wall-clock capture (the identity is net-pinned with zero tolerance).
// Returns -1 when either timestamp is negative (source reports none).
constexpr int64_t inputLatencyMs(int64_t wallClockMs, int64_t inputTimestampMs) noexcept {
    return (inputTimestampMs >= 0 && wallClockMs >= 0)
               ? (wallClockMs - inputTimestampMs)
               : -1;
}

// Build engine physics + operational state from simulator stats + cranking controller
EngineState::Engine buildEngineState(
    const EngineSimStats& stats,
    const CrankingController::State& cranking);

// Build drivetrain mechanical state from simulator stats + input (for replay timestamp)
EngineState::Drivetrain buildDrivetrainState(const EngineSimStats& stats,
                                              const input::EngineInput& input);

// Build user control inputs from input provider + cranking controller
EngineState::Controls buildControlState(
    const input::EngineInput& input,
    const CrankingController::State& cranking);

// Build audio/timing diagnostics from telemetry + audio buffer + config
EngineState::Audio buildAudioState(
    const telemetry::AudioTimingTelemetry& timing,
    const telemetry::ITelemetryReader* telemetryReader,
    const IAudioBuffer& audioBuffer,
    const struct SimulationConfig& config,
    double currentTime,
    const ISimulator& simulator);

} // namespace builders
} // namespace presentation

#endif // PRESENTATION_STATE_BUILDERS_H
