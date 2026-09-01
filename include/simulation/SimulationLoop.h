// SimulationLoop.h - Simulation loop class and session factory
// Extracted from engine_sim_cli.cpp for SOLID SRP compliance
// Phase E: Takes ISimulator* instead of EngineSimHandle/EngineSimAPI&
// Phase F: Moved to engine-sim-bridge for reusability (GUI, iOS, headless)
// Phase G: Refactored from free function to class with injected dependencies

#ifndef SIMULATION_LOOP_H
#define SIMULATION_LOOP_H

#include "simulator/EngineSimTypes.h"
#include "simulation/CrankingController.h"
#include "simulation/ILoopClock.h"
#include "input/VehicleStartController.h"
#include "io/IInputProvider.h"
#include "io/IPresentation.h"  // DiagnosticOutputFilter (carried via SimulationConfig)
#include "hardware/IAudioHardwareProvider.h"  // IAudioHardwareProvider (complete type: named in createSession signature)
#include <atomic>
#include <string>
#include <memory>

class IAudioBuffer;
class IAudioHardwareProvider;
class ISimulator;
class ISimulatorSession;

// Forward declarations for injectable interfaces
namespace presentation { class IPresentation; }
namespace input { class IArrivalStatePrimer; }
class ILogging;
namespace telemetry { class ITelemetryWriter; class ITelemetryReader; }

// Forward declaration of simulator type enum (defined in simulator/SimulatorFactory.h)
enum class SimulatorType;

// ============================================================================
// SessionDependencies - Bundled infrastructure deps for session construction.
// Cuts parameter count on SimulationLoop, SimulatorSession, and createSession
// from 10 → 6 (S107). Each field is an infra dependency; the owning object
// retains the non-null pointer/reference for the session lifetime.
// ============================================================================

struct SessionDependencies {
    IAudioBuffer* audioBuffer = nullptr;
    CrankingController* crankingController = nullptr;
    std::atomic<bool>* stopRequested = nullptr;
    input::IInputProvider* inputProvider = nullptr;
    presentation::IPresentation* presentation = nullptr;
    telemetry::ITelemetryWriter* telemetryWriter = nullptr;
    telemetry::ITelemetryReader* telemetryReader = nullptr;
    ILogging* logger = nullptr;
    ILoopClock* clock = nullptr;  // Optional injected clock; null → SteadyClockLoopClock
};

// Exit code returned by SimulationLoop::run() when preset cycling is requested
constexpr int EXIT_BUT_CONTINUE_NEXT = 2;

// ============================================================================
// StepResult - Result type for SimulationLoop::step()
// Encapsulates the three possible outcomes of a single simulation tick
// ============================================================================

enum class StepResult {
    Continue,      // Normal tick, continue the loop
    PresetCycle,  // Preset cycling requested, exit with EXIT_BUT_CONTINUE_NEXT
    Stop          // Normal stop condition (duration or stopRequested)
};

// ============================================================================
// LoopState - Per-tick state for SimulationLoop::step()
// Contains all the mutating locals from run()'s loop body
// ============================================================================

struct LoopState {
    double currentTime = 0.0;
    input::EngineInput engineInput;
    double lastDynoTorqueScale = -1.0;
    EngineSimStats previousStats = {};
    bool isFirstTick = true;
    ICombustionEngine* combustionEngine = nullptr;
};

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
    // Deterministic replay/gate mode: the simulation advances on the loop
    // thread at the fixed update interval (DeterministicStrategy) with a
    // no-op loop clock — no audio callback thread, no wall-clock pacing.
    // Live playback (the default) keeps the sync-pull behavior untouched.
    bool deterministic = false;
    double targetLoad = -1.0;   // -1 = no dyno, 0.0-1.0 = load torque fraction
    const char* outputWav = nullptr;
    int preFillMs = EngineSimDefaults::DEFAULT_PREFILL_MS;
    bool autoGearbox = false;    // Automatic gearbox mode (--auto), default is manual
    bool deterministicTickLock = false;  // tick-locked integer termination + fakeClock selection; default false = legacy FP/wall-clock behavior

    // Vehicle start/stop: crank delay (seconds) between starter engagement and
    // ignition on a brake-initiated start. Owned here so the loop constructs
    // its VehicleStartController from config — no per-provider wiring.
    double startStopCrankDelayS = input::VehicleStartController::kDefaultCrankDelayS;

    // Optional display label for logging (e.g. ANSI-colored by CLI). Empty = auto-derive.
    std::string simulatorLabel;

    // Selective debug categories, forwarded to PresentationConfig.
    presentation::DiagnosticOutputFilter diagnostics;

    // Machine-parseable CSV output path (--csv-out). Empty = no CSV. Forwarded
    // to createPresentation which composes a CsvPresentation alongside console.
    std::string csvOutPath;

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
    const SessionDependencies& deps);

    // Run the loop until stopRequested or duration expires.
    // Returns EXIT_BUT_CONTINUE_NEXT on preset cycle, 0 on normal exit.
    int run();

    // Execute a single simulation tick.
    // Returns the step result (Continue, PresetCycle, or Stop).
    StepResult step(LoopState& state);

private:
    // Vehicle start/stop decision — the ONE invocation site every mode
    // (keyboard, demo, replay, live) traverses. Runs from the canonical
    // brakeLight + gearSelector; flattens the controller's ignition/starter
    // levels into EngineInput. No-ops while no vehicle-control signal has
    // been seen, so provider-owned starts (e.g. replay autoStart) survive.
    void applyStartStopDecision(LoopState& state, bool lightReportedByTelemetry);

    // Convert the controller's held starter LEVEL into a single-frame pulse
    // (CrankingController::engageStarter toggles on a held-high button).
    bool starterPulseFromLevel(bool controllerStarterLevel);

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

    // Instant --start-from (file traces): prime the provider's arrival state
    // (twin warm-boot from the arrival row + clock anchor + arrival hold),
    // settle the engine core for a bounded window at that HELD operating
    // point with all emission suppressed, then hand off at the offset —
    // rows before --start-from are never simulated. See run()'s block
    // comment for the full contract.
    void settleAtArrivalPoint(LoopState& state,
                              input::IArrivalStatePrimer& primer,
                              double offsetS);

    // One suppressed settle tick (used by settleAtArrivalPoint). Returns
    // false when settling must stop: stop request, disconnected source, or a
    // terminal step() result (Stop/PresetCycle).
    bool settleTick(LoopState& state);

    // Injected dependencies — set once, read-only during run()
    ISimulator& simulator_;
    const SimulationConfig& config_;
    IAudioBuffer& audioBuffer_;
    CrankingController& crankingController_;
    const std::atomic<bool>* stopRequested_;
    input::IInputProvider* inputProvider_;
    presentation::IPresentation* presentation_;
    telemetry::ITelemetryWriter* telemetryWriter_;
    telemetry::ITelemetryReader* telemetryReader_;
    ILogging* logger_;

    // Clock for loop pacing (injected for testability)
    // When clock is injected via deps.clock, we use that; otherwise we own steadyClock_
    ILoopClock* clock_ = nullptr;  // Non-owning pointer, points to either injectedClock_ or steadyClock_
    std::unique_ptr<ILoopClock> steadyClock_;  // Owning pointer for default steady clock

    // Last-frame starter state — the cranking notice logs on the engage
    // TRANSITION, not every frame the starter is held.
    bool lastStarterEngaged_ = false;

    // Suppress OUTPUT emission during the suppressed settle (instant
    // --start-from arrival settle): CSV telemetry write AND presentation are
    // both tied to this gate. The per-tick PHYSICS (start/stop decision,
    // cranking, vehicle controls, audioBuffer_.updateSimulation core tick)
    // always runs in step() regardless — only outputs are suppressed, so the
    // settle advances the gas path exactly like the main loop while emitting
    // nothing.
    bool emitCsv_ = true;

    // Suppress audio queueing during the suppressed settle. The physics tick
    // (audioBuffer_.updateSimulation -> simulator->update) ALWAYS runs; only
    // fillBufferFromEngine (rendered-sample queueing into the playback ring)
    // is gated, so the silent settle queues no audio. The ring is drained and
    // reset at the settle handoff (resetBufferAfterWarmup), so playback starts
    // clean at the --start-from offset instead of replaying settle audio.
    bool emitAudio_ = true;

    // Vehicle start/stop decision layer + the observer it drives. The loop
    // owns both: the controller's ignition/starter LEVEL decisions are read
    // off the observer and flattened into EngineInput.ignition/starterButton;
    // CrankingController stays the single actuator authority downstream.
    input::ObserverActuator startStopObserver_;
    input::VehicleStartController startStopController_{startStopObserver_,
                                                        config_.startStopCrankDelayS};
    bool startStopEngaged_ = false;   // authority latch: once a vehicle-control
                                      // signal is seen, the controller keeps it
    bool prevStarterLevel_ = false;   // edge detection for the starter pulse
};

// ============================================================================
// Session factory — creates or hot-swaps a SimulatorSession
// Throws std::runtime_error on initialization failure (fail-fast).
// ============================================================================

std::unique_ptr<ISimulatorSession> createSession(
    const SimulationConfig& config,
    const std::string& scriptPath,
    std::unique_ptr<ISimulator> simulator,
    SessionDependencies& deps,
    std::unique_ptr<ISimulatorSession> existingSession = nullptr,
    std::unique_ptr<IAudioHardwareProvider> audioProvider = nullptr);

#endif // SIMULATION_LOOP_H
