// PresentationStateBuilders.h - Domain-specific EngineState construction helpers
// SRP: Each function builds one domain sub-struct from its authoritative source
// Pure functions: no side effects, no mutations to simulator or stats

#ifndef PRESENTATION_STATE_BUILDERS_H
#define PRESENTATION_STATE_BUILDERS_H

#include "io/IPresentation.h"
#include "simulator/EngineSimTypes.h"
#include "simulation/CrankingController.h"
#include "simulation/SimulationLoop.h"
#include "io/IInputProvider.h"
#include "telemetry/ITelemetryProvider.h"
#include "strategy/IAudioBuffer.h"

class ISimulator;

namespace presentation {
namespace builders {

// Build engine physics + operational state from simulator stats + cranking controller
EngineState::Engine buildEngineState(
    const EngineSimStats& stats,
    const CrankingController::State& cranking);

// Build drivetrain mechanical state from simulator stats
EngineState::Drivetrain buildDrivetrainState(const EngineSimStats& stats);

// Build user control inputs from input provider + cranking controller
EngineState::Controls buildControlState(
    const input::EngineInput& input,
    const CrankingController::State& cranking);

// Build audio/timing diagnostics from telemetry + audio buffer + config
EngineState::Audio buildAudioState(
    const telemetry::AudioTimingTelemetry& timing,
    telemetry::ITelemetryReader* telemetryReader,
    IAudioBuffer& audioBuffer,
    const struct SimulationConfig& config,
    double currentTime,
    ISimulator& simulator);

} // namespace builders
} // namespace presentation

#endif // PRESENTATION_STATE_BUILDERS_H
