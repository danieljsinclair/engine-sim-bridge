// SimulationLoop.h - Simulation loop functions
// Extracted from engine_sim_cli.cpp for SOLID SRP compliance
// Phase E: Takes ISimulator* instead of EngineSimHandle/EngineSimAPI&
// Phase F: Moved to engine-sim-bridge for reusability (GUI, iOS, headless)

#ifndef SIMULATION_LOOP_H
#define SIMULATION_LOOP_H

#include "simulator/EngineSimTypes.h"
#include <string>
#include <vector>
#include <memory>

class IAudioBuffer;
class ISimulator;

// Forward declarations for injectable interfaces
namespace input { class IInputProvider; }
namespace presentation { class IPresentation; }
class ILogging;
namespace telemetry { class ITelemetryWriter; class ITelemetryReader; }

// Exit code returned by runUnifiedAudioLoop when preset cycling is requested
constexpr int EXIT_CODE_PRESET_CYCLE = 2;

// ============================================================================
// SimulationConfig - Simulation parameters only (no infrastructure deps)
// ============================================================================

struct SimulationConfig {
    // Engine config — value type, inline defaults from EngineSimDefaults
    ISimulatorConfig engineConfig;

    // Simulation parameters
    std::string configPath;
    std::string assetBasePath;
    double duration = EngineSimDefaults::DEFAULT_DURATION_SECONDS;
    bool interactive = false;
    bool playAudio = false;
    float volume = EngineSimDefaults::DEFAULT_HARDWARE_VOLUME;
    bool syncPull = true;
    double targetLoad = -1.0;   // -1 = no dyno, 0.0-1.0 = load torque fraction
    bool useDefaultEngine = false;
    const char* outputWav = nullptr;
    int preFillMs = EngineSimDefaults::DEFAULT_PREFILL_MS;

    // Optional display label for logging (e.g. ANSI-colored by CLI). Empty = auto-derive.
    std::string simulatorLabel;

    // Computed helpers
    double updateInterval() const { return 1.0 / 60.0; }
    int framesPerUpdate() const { return engineConfig.sampleRate / 60; }
    int sampleRate() const { return engineConfig.sampleRate; }
};

// ============================================================================
// Main simulation entry point
// Dependencies injected: simulator, strategy, inputProvider, presentation, logger
// Throws std::runtime_error on initialization failure (fail-fast).
// ============================================================================

int runUnifiedAudioLoop(
    ISimulator& simulator,
    const SimulationConfig& config,
    IAudioBuffer& audioBuffer,
    input::IInputProvider* inputProvider,
    presentation::IPresentation* presentation,
    telemetry::ITelemetryWriter* telemetryWriter,
    telemetry::ITelemetryReader* telemetryReader,
    ILogging* logger);

int runSimulation(
    const SimulationConfig& config,
    ISimulator& simulator,
    IAudioBuffer* audioBuffer,
    input::IInputProvider* inputProvider,
    presentation::IPresentation* presentation,
    telemetry::ITelemetryWriter* telemetryWriter,
    telemetry::ITelemetryReader* telemetryReader,
    ILogging* logger
);

// ============================================================================
// Hot-swap preset at runtime (bridge-level function)
// Stops playback, saves state, destroys simulator, creates new one, restores state
// Returns 0 on success, non-zero on failure
// ============================================================================

int hotSwapPreset(
    std::unique_ptr<ISimulator>& simulator,
    IAudioBuffer* audioBuffer,
    const std::string& newPresetPath,
    SimulationConfig& config,
    ILogging* logger,
    telemetry::ITelemetryWriter* telemetry);

// ============================================================================
// Hot-swap to next preset in the presetPaths list
// Computes next index, calls hotSwapPreset, updates config.configPath.
// Returns new index for convenience.
// ============================================================================

size_t hotSwapSimulation(
    std::unique_ptr<ISimulator>& simulator,
    IAudioBuffer* audioBuffer,
    const std::vector<std::string>& presetPaths,
    size_t currentIndex,
    SimulationConfig& config,
    ILogging* logger,
    telemetry::ITelemetryWriter* telemetry);

// ============================================================================
// Iterator-pattern simulation entry point
// Internally loops runSimulation + hotSwapSimulation on preset cycle.
// Clients call once and get a final exit code. EXIT_CODE_PRESET_CYCLE never
// escapes this function.
// ============================================================================

int runNextSimulation(
    SimulationConfig& config,
    std::unique_ptr<ISimulator>& simulator,
    IAudioBuffer* audioBuffer,
    input::IInputProvider* inputProvider,
    presentation::IPresentation* presentation,
    telemetry::ITelemetryWriter* telemetryWriter,
    telemetry::ITelemetryReader* telemetryReader,
    ILogging* logger);

#endif // SIMULATION_LOOP_H
