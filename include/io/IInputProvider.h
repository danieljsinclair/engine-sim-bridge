// IInputProvider.h - Input provider interface
// Abstracts input source: keyboard, upstream (ODB2/VirtualICE), etc.
// OCP: New input providers can be added without modifying existing code
// DI: Provider is injected into simulation loop
// Phase F: Moved to engine-sim-bridge for reusability (GUI, iOS, headless)

#ifndef I_INPUT_PROVIDER_H
#define I_INPUT_PROVIDER_H

#include <optional>
#include <string>

struct EngineSimStats;

namespace input {

// ============================================================================
// EngineInput - What the simulation receives from input sources
// Only external inputs - simulator handles its own state (starter, etc.)
// ============================================================================

struct EngineInput {
    double throttle = 0.0;       // 0.0 - 1.0 (from keyboard/upstream)
    bool ignition = true;        // true = on (from keyboard/upstream)
    bool starterButton = false;  // momentary: true for one frame when pressed

    // Mirrors UpstreamSignal::traceDriven (set by the twin providers): the
    // throttle field is the recorded/real vehicle trace, not a scripted driver.
    // Consumed by SimulationLoop::applyCrankingDecision to clamp the crank
    // controller's effective throttle to the trace — a telemetry-defined run
    // must not synthesize a start character (crank-throttle flare) the
    // recording never contained. Default false = scripted/keyboard behavior
    // unchanged.
    bool traceDrivenThrottle = false;

    // Gear control
    int gearDelta = 0;           // +1 = shift up, -1 = shift down, 0 = no change

    // Dyno control
    double dynoTorqueScale = -1.0;  // -1 = unchanged, 0.0-1.0 = fraction of max torque

    // Brake control — the PHYSICS input (dyno/BrakeConstraint pressure).
    // The keyboard 'B' key is its only writer.
    double brakeLevel = 0.0;        // 0.0 = no brake, 1.0 = full brake

    // Vehicle brake LIGHT — the canonical display/start-stop signal. Supplied
    // directly by telemetry (CSV brake_light column / network signal), or
    // derived from brakeLevel at the single assembly point in
    // SimulationLoop::step() when no telemetry reports it. Never drives
    // physics. nullopt = not reported (pre-assembly).
    std::optional<bool> brakeLight;

    // Steering wheel angle (degrees, signed; CAN 0x129 SCCM_steeringAngle via
    // the vehicle-sim steering_angle_deg CSV column). nullopt = not reported.
    // Display-only: surfaced to EngineState.Controls for the console readout;
    // never drives physics or the start/stop controller.
    std::optional<double> steeringAngleDeg;

    // Twin control
    int gearAbsolute = -1;          // -1 = use gearDelta logic, 0+ = set this gear directly
    double clutchPressure = -1.0;   // -1 = unchanged, 0.0-1.0 = set clutch pressure
    double vehicleSpeedTargetKmh = -1.0; // -1 = unchanged, for future SpeedTrackingForce
    int gearSelector = 0;           // GearSelector value for display
    bool gearAutoMode = false;      // true=auto(ZF), false=manual
    // Negative sentinel = "no speed commanded this frame" (unchanged).
    // SimulationLoop gates setSpeedTrackingTarget on >= 0.0, so a default of 0.0
    // would force the dyno to hold the engine at 0 RPM (stall) every frame.
    // Once the user presses '.' the value is clamped to [0, 300] and stays >= 0.
    double roadSpeedKmh = -2.0;     // Virtual ICE Twin: manual road speed control (km/h)
    double engineRpmFloor = 0.0;    // >=0: launch floor on the dyno target RPM (torque-converter style)

    // Replay timestamp: absolute CSV time in seconds, -1 if not replaying.
    // Passed through to EngineState.drivetrain.replayTimestampS for display.
    double replayTimestampS = -1.0;

    // MATCH (Torque) mode: recorded drivetrain torque (Nm) injected at the
    // transmission input each frame. 0.0 = no injection (FREE/PIN leave this at
    // 0; applying 0.0 Nm is a true no-op on the rotating mass).
    double drivetrainInputTorqueNm = 0.0;

    // Live twin diagnostics threaded through to presentation (inline clutch
    // readout + CSV-out). roadImpliedRpm = RPM if locked to wheels in current
    // gear; creepReliefFired = the creep-drag relief opened the clutch (the
    // #24 slow-speed stall protection) this frame; couplingIsTorqueConverter
    // selects the display label for clutchPressure ('TC' vs 'Cl').
    double roadImpliedRpm = 0.0;
    bool creepReliefFired = false;
    bool couplingIsTorqueConverter = false;

    // Simulator auto-disengages starter when RPM > threshold
    // Preset cycling
    bool presetCycle = false;       // true = cycle to next preset engine configuration

    // ---- Start/ignition intent events (composable primitive requests) ----
    // Frontends (CLI keyboard S/I, --start flag, iOS app button) request
    // start/ignition actions through these flags; SimulationLoop's
    // applyStartStopDecision forwards them to its VehicleStartController —
    // the single authority for start/ignition decisions (owner spec
    // 2026-09-02). One-shot: copied into EngineInput by the producer, then
    // reset. This keeps every frontend a thin caller of the shared state
    // machine while the gear/brake-driven CSV path continues to call
    // update(dt, brake, gear) directly.
    bool starterRequested = false;        // requestStarter(): engage starter, no ignition
    std::optional<bool> ignitionRequest;  // requestIgnition(on): explicit ON/OFF (nullopt = no change)
    bool combinedStartRequested = false;  // requestCombinedStart(): starter+ignition together
};

// ============================================================================
// IInputProvider Interface
// Implemented by: KeyboardInputProvider, UpstreamProvider (ODB2/VirtualICE)
// ============================================================================

class IInputProvider {
public:
    virtual ~IInputProvider() = default;

    // ========================================================================
    // Lifecycle
    // ========================================================================

    virtual bool Initialize() = 0;
    virtual void Shutdown() = 0;
    virtual bool IsConnected() const = 0;

    /**
     * True when THIS provider ended the run by hitting its --end-at bound
     * (the provider reported EOF because its elapsed clock crossed the
     * requested timecode). Lets the CLI report the honest stop reason
     * ("--end-at Ns reached") instead of the generic duration/trace-end
     * message, which would otherwise quote the full trace length. Default
     * false: most providers never bound themselves.
     */
    virtual bool endAtReached() const { return false; }

    // ========================================================================
    // Input Queries (called from simulation thread)
    // ========================================================================

    /**
     * Poll for input and return current engine inputs.
     */
    virtual EngineInput OnUpdateSimulation(double dt) = 0;

    /**
     * Provide simulator feedback from the previous frame.
     * Providers that need RPM feedback (e.g. twin-based providers) override this.
     */
    virtual void provideFeedback(const EngineSimStats& /* stats */) {}

    virtual std::string GetProviderName() const = 0;
    virtual std::string GetLastError() const = 0;
};

} // namespace input

#endif // I_INPUT_PROVIDER_H
