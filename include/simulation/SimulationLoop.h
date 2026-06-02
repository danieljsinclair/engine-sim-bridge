// SimulationLoop.h - Simulation loop functions
// Extracted from engine_sim_cli.cpp for SOLID SRP compliance
// Phase E: Takes ISimulator* instead of EngineSimHandle/EngineSimAPI&
// Phase F: Moved to engine-sim-bridge for reusability (GUI, iOS, headless)

#ifndef SIMULATION_LOOP_H
#define SIMULATION_LOOP_H

#include "simulator/EngineSimTypes.h"
#include "simulation/CrankingController.h"
#include <string>
#include <vector>
#include <memory>

class IAudioBuffer;
class ISimulator;
class ISimulatorSession;

// Forward declarations for injectable interfaces
namespace input { class IInputProvider; }
namespace presentation { class IPresentation; }
class ILogging;
namespace telemetry { class ITelemetryWriter; class ITelemetryReader; }

// Forward declaration of simulator type enum (defined in simulator/SimulatorFactory.h)
enum class SimulatorType;

// Exit code returned by runUnifiedAudioLoop when preset cycling is requested
constexpr int EXIT_BUT_CONTINUE_NEXT = 2;

// ============================================================================
// SimulationConfig - Simulation parameters only (no infrastructure deps)
// ============================================================================

struct SimulationConfig {
    // Engine config — value type, inline defaults from EngineSimDefaults
    ISimulatorConfig engineConfig;
    SimulatorType simulatorType{};

    // Simulation parameters
    std::string configPath;
    std::string assetBasePath;
    double duration = EngineSimDefaults::DEFAULT_DURATION_SECONDS;
    bool interactive = false;
    bool playAudio = false;
    float volume = EngineSimDefaults::DEFAULT_HARDWARE_VOLUME;
    bool syncPull = true;
    double targetLoad = -1.0;   // -1 = no dyno, 0.0-1.0 = load torque fraction
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
    CrankingController& crankingController,
    input::IInputProvider* inputProvider,
    presentation::IPresentation* presentation,
    telemetry::ITelemetryWriter* telemetryWriter,
    telemetry::ITelemetryReader* telemetryReader,
    ILogging* logger);

std::unique_ptr<ISimulatorSession> initSimulation(
    const SimulationConfig& config,
    const std::string& scriptPath,
    std::unique_ptr<ISimulator> simulator,
    IAudioBuffer* audioBuffer,
    std::unique_ptr<ISimulatorSession> existingSession = nullptr,
    input::IInputProvider* inputProvider = nullptr,
    presentation::IPresentation* presentation = nullptr,
    telemetry::ITelemetryWriter* telemetryWriter = nullptr,
    telemetry::ITelemetryReader* telemetryReader = nullptr,
    ILogging* logger = nullptr);

#endif // SIMULATION_LOOP_H
