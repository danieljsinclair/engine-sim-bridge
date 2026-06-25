// SimulationLoop.h - Simulation loop class and session factory
// Extracted from engine_sim_cli.cpp for SOLID SRP compliance
// Phase E: Takes ISimulator* instead of EngineSimHandle/EngineSimAPI&
// Phase F: Moved to engine-sim-bridge for reusability (GUI, iOS, headless)
// Phase G: Refactored from free function to class with injected dependencies

#ifndef SIMULATION_LOOP_H
#define SIMULATION_LOOP_H

#include "simulator/EngineSimTypes.h"
#include "simulation/CrankingController.h"
#include "io/IInputProvider.h"
#include "io/IPresentation.h"  // DiagnosticOutputFilter (carried via SimulationConfig)
#include <atomic>
#include <string>
#include <memory>

class IAudioBuffer;
class ISimulator;
class ISimulatorSession;

// Forward declarations for injectable interfaces
namespace presentation { class IPresentation; }
class ILogging;
namespace telemetry { class ITelemetryWriter; class ITelemetryReader; }

// Forward declaration of simulator type enum (defined in simulator/SimulatorFactory.h)
enum class SimulatorType;

// Exit code returned by SimulationLoop::run() when preset cycling is requested
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
    bool autoGearbox = false;    // Automatic gearbox mode (--auto), default is manual

    // Optional display label for logging (e.g. ANSI-colored by CLI). Empty = auto-derive.
    std::string simulatorLabel;

    // Selective debug categories, forwarded to PresentationConfig.
    presentation::DiagnosticOutputFilter diagnostics;

    // Computed helpers
    double updateInterval() const { return 1.0 / 60.0; }
    int framesPerUpdate() const { return engineConfig.sampleRate / 60; }
    int sampleRate() const { return engineConfig.sampleRate; }
};

// ============================================================================
// SimulationLoop - Main tick loop with injected dependencies
// Dependencies are set once at construction; run() needs no parameters.
// ============================================================================

class SimulationLoop {
public: 
  SimulationLoop(
    ISimulator& simulator,
    const SimulationConfig& config,
    IAudioBuffer& audioBuffer,
    CrankingController& crankingController,
    const std::atomic<bool>& stopRequested,
    input::IInputProvider* inputProvider,
    presentation::IPresentation* presentation,
    telemetry::ITelemetryWriter* telemetryWriter,
    telemetry::ITelemetryReader* telemetryReader,
    ILogging* logger);

    // Run the loop until stopRequested or duration expires.
    // Returns EXIT_BUT_CONTINUE_NEXT on preset cycle, 0 on normal exit.
    int run();

private:
    // Cranking decision — single entry point for combustion/sine-mode fork
    CrankingController::State applyCrankingDecision(
        ICombustionEngine* combustionEngine,
        const input::EngineInput& engineInput);

    // Apply throttle, ignition, gear, dyno, clutch, brake to simulator
    void applyVehicleControls(
        ICombustionEngine* combustionEngine,
        const input::EngineInput& input,
        const CrankingController::State& crankingState,
        double& lastDynoTorqueScale);

    // Push telemetry for this tick
    void writeTelemetry(double currentTime, double throttle,
                        bool ignition, bool starterEngaged);

    // Build and present engine state for this tick
    void updatePresentation(const EngineSimStats& stats,
                            const CrankingController::State& crankingState,
                            const input::EngineInput& input,
                            double tickTime);

    // Poll input provider or generate timed input for non-interactive mode
    input::EngineInput pollInput(double currentTime, double updateInterval, bool isFirstTick);

    // Injected dependencies — set once, read-only during run()
    ISimulator& simulator_;
    const SimulationConfig& config_;
    IAudioBuffer& audioBuffer_;
    CrankingController& crankingController_;
    const std::atomic<bool>& stopRequested_;
    input::IInputProvider* inputProvider_;
    presentation::IPresentation* presentation_;
    telemetry::ITelemetryWriter* telemetryWriter_;
    telemetry::ITelemetryReader* telemetryReader_;
    ILogging* logger_;
};

// ============================================================================
// Session factory — creates or hot-swaps a SimulatorSession
// Throws std::runtime_error on initialization failure (fail-fast).
// ============================================================================

std::unique_ptr<ISimulatorSession> createSession(
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
