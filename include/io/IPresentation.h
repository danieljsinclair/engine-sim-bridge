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
// EngineState - Data to display
// ============================================================================

struct EngineState {
    double timestamp;
    double rpm;
    double throttle;
    double load;
    double speed;
    int underrunCount;
    std::string audioMode;
    bool ignition;
    bool starterMotorEngaged;
    EnginePhase enginePhase = EnginePhase::Stopped;
    std::string presetShortName;  // Short name of current preset (for display)
    double exhaustFlow;  // m^3/s
    int gear = 0;
    double dynoTorque = 0.0;      // ft*lbs (0 when dyno disabled)
    double dynoTargetRPM = 0.0;   // 0 when dyno disabled

    // Audio timing diagnostics (from strategy)
    double renderMs = 0.0;
    double headroomMs = 0.0;
    double budgetPct = 0.0;
    int framesRequested = 0;
    int framesRendered = 0;
    double callbackRateHz = 0.0;
    double generatingRateFps = 0.0;
    double trendPct = 0.0;
    int sampleRate = 0;  // Set by SimulationLoop from upstream provider
    int simulationFrequency = 0;  // Actual physics Hz from the loaded engine
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
