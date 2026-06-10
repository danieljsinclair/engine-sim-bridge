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
    // From simulator stats (ISimulator::getStats)
    struct Engine {
        double rpm = 0.0;
        double load = 0.0;
        double exhaustFlow = 0.0;     // m^3/s
        double engineTorqueNm = 0.0;
        double drivetrainTorqueNm = 0.0;
    } engine;

    // Mechanical + operational state (sim stats + cranking controller + input)
    struct Drivetrain {
        double speedMph = 0.0;
        double vehicleSpeedKmh = 0.0;
        int gear = 0;
        int gearSelector = 0;          // From input provider (source of truth)
        bool gearAutoMode = false;     // From input provider
        double dynoTorque = 0.0;       // ft*lbs (0 when dyno disabled)
        double dynoTargetRPM = 0.0;
        double throttle = 0.0;         // Effective throttle (cranking-aware, from CrankingController)
        bool starterEngaged = false;   // From CrankingController
    } drivetrain;

    // User controls (from input provider only)
    struct Controls {
        bool ignition = false;
        double brakeLevel = 0.0;
    } controls;

    // From audio buffer / telemetry (observability, not functional)
    struct Audio {
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
    double timestamp = 0.0;
    EnginePhase phase = EnginePhase::Stopped;
    int simulationFrequency = 0;
    std::string presetShortName;
};

// ============================================================================
// Presentation Configuration
// ============================================================================

struct PresentationConfig {
    bool interactive = false;
    double duration = 0.0;  // 0 = infinite
    bool showProgress = true;
    bool showDiagnostics = true;
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

    virtual void ShowEngineState(const EngineState& state) = 0;
    virtual void ShowMessage(const std::string& message) = 0;
    virtual void ShowError(const std::string& error) = 0;
    virtual void ShowProgress(double currentTime, double duration) = 0;

    virtual void Update(double dt) = 0;
};

} // namespace presentation

#endif // I_PRESENTATION_H
