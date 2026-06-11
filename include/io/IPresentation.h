// IPresentation.h - Presentation interface
// Abstracts output: console text, TUI/TMUX, headless logging, etc.
// OCP: New presentation modes can be added without modifying existing code
// DI: Presentation is injected into simulation loop
// Phase F: Moved to engine-sim-bridge for reusability (GUI, iOS, headless)

#ifndef I_PRESENTATION_H
#define I_PRESENTATION_H

#include <string>
#include <memory>
#include <functional>
#include "simulation/EnginePhase.h"

namespace presentation {

// ============================================================================
// EngineState - Componentised display data, each sub-struct from one source
// ============================================================================

struct EngineState {
    // Engine physics + operational state
    struct Engine {
        double rpm = 0.0;
        double load = 0.0;
        double exhaustFlow = 0.0;     // m^3/s
        double engineTorqueNm = 0.0;
        double drivetrainTorqueNm = 0.0;
        bool starterEngaged = false;   // Starter motor physically engaged with engine
        EnginePhase phase = EnginePhase::Stopped;
    } engine;

    // Mechanical transmission state
    struct Drivetrain {
        double speedMph = 0.0;
        double vehicleSpeedKmh = 0.0;
        int gear = 0;
        double dynoTorque = 0.0;
        double dynoTargetRPM = 0.0;
    } drivetrain;

    // User control inputs (what the driver is commanding)
    struct Controls {
        double throttle = 0.0;         // Effective (cranking-aware)
        bool ignition = false;
        double brakeLevel = 0.0;
        int gearSelector = 0;
        bool gearAutoMode = false;
        // Commanded road-speed target (km/h) from the ','/'.' keys. Negative
        // sentinel = "no speed commanded". Surfaced so it is visible in neutral
        // where the vehicle speed readout otherwise shows physics only.
        double commandedSpeedKmh = -1.0;
    } controls;

    // Audio + timing diagnostics (observability only)
    struct Audio {
        double timestamp = 0.0;
        int simulationFrequency = 0;
        int underrunCount = 0;
        std::string audioMode;
        double renderMs = 0.0;
        double headroomMs = 0.0;
        double budgetPct = 0.0;
        int framesRequested = 0;
        int framesRendered = 0;
        double callbackRateHz = 0.0;
        double generatingRateFps = 0.0;
        double trendPct = 0.0;
        int sampleRate = 0;
    } audio;

    // Cross-cutting
    std::string presetShortName;
};

// ============================================================================
// Diagnostic Output Filter
// Single bucket selecting which optional per-frame debug lines are printed.
// Open/Closed: adding a category = adding a field here + a CLI flag that sets
// it. No call-site signatures change. Injected by const ref via PresentationConfig.
// ============================================================================

struct DiagnosticOutputFilter {
    bool frames = false;   // audio frame/buffer timing line (req=/got=/took=/room=)
    bool freq   = false;   // update-call frequency line (calls=/need/kfps)
};

// ============================================================================
// Presentation Configuration
// ============================================================================

struct PresentationConfig {
    bool interactive = false;
    double duration = 0.0;  // 0 = infinite
    bool showProgress = true;
    bool showDiagnostics = true;

    // Selective debug categories. Source of truth for which optional diagnostic
    // lines get printed; defaults to all-off so the console stays quiet.
    DiagnosticOutputFilter diagnostics;
};

// ============================================================================
// IPresentation Interface
// Implemented by: ConsolePresentation, TUI Presentation, HeadlessLogger
// ============================================================================

class IPresentation {
public:
    virtual ~IPresentation() = default;

    // ========================================================================
    // Lifecycle
    // ========================================================================

    virtual bool Initialize(const PresentationConfig& config) = 0;
    virtual void Shutdown() = 0;

    // ========================================================================
    // Output Methods
    // ========================================================================

    virtual void ShowSimulatorStates(const EngineState& state) = 0;
    virtual void ShowMessage(const std::string& message) = 0;
    virtual void ShowError(const std::string& error) = 0;
    virtual void ShowProgress(double currentTime, double duration) = 0;

    virtual void Update(double dt) = 0;
};

} // namespace presentation

#endif // I_PRESENTATION_H
