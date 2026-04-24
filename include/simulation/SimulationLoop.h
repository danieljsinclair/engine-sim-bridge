// SimulationLoop.h - Simulation loop functions
// Extracted from engine_sim_cli.cpp for SOLID SRP compliance
// Phase E: Takes ISimulator* instead of EngineSimHandle/EngineSimAPI&
// Phase F: Moved to engine-sim-bridge for reusability (GUI, iOS, headless)

#ifndef SIMULATION_LOOP_H
#define SIMULATION_LOOP_H

#include "simulator/EngineSimTypes.h"
#include <string>

class IAudioBuffer;
class ISimulator;

// Forward declarations for injectable interfaces
namespace input { class IInputProvider; }
namespace presentation { class IPresentation; }
class ILogging;
namespace telemetry { class ITelemetryWriter; class ITelemetryReader; }

// ============================================================================
// SimulationConfig - Simulation parameters only (no infrastructure deps)
// ============================================================================

class SimulationConfig {
public:
    SimulationConfig() = default;  // Inline initializers provide all defaults

    SimulationConfig(const SimulationConfig&) = delete;
    SimulationConfig& operator=(const SimulationConfig&) = delete;

    SimulationConfig(SimulationConfig&&) = default;
    SimulationConfig& operator=(SimulationConfig&&) = default;

    ~SimulationConfig();  // Clean up owned ISimulatorConfig

    // Public members
    std::string configPath;
    std::string assetBasePath;
    double duration = 3.0;
    bool interactive = false;
    bool playAudio = false;
    float volume = 1.0f;
    bool syncPull = true;
    double targetRPM = 0.0;
    double targetLoad = -1.0;
    bool useDefaultEngine = false;
    const char* outputWav = nullptr;
    int preFillMs = 50;

    // Engine configuration - single source of truth for audio/simulation constants
    const ISimulatorConfig* engineConfig = nullptr;  // Owned by this struct

    // Optional display label for logging (e.g. ANSI-colored by CLI). Empty = auto-derive.
    std::string simulatorLabel;

    // Computed helpers - inline to avoid storing duplicates
    inline double updateInterval() const { return 1.0 / 60.0; }  // 60Hz loop
    inline int framesPerUpdate() const { return engineConfig ? engineConfig->sampleRate / 60 : 0; }
    inline int sampleRate() const { return engineConfig ? engineConfig->sampleRate : 0; }
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

#endif // SIMULATION_LOOP_H
