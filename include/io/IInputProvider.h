// IInputProvider.h - Input provider interface
// Abstracts input source: keyboard, upstream (ODB2/VirtualICE), etc.
// OCP: New input providers can be added without modifying existing code
// DI: Provider is injected into simulation loop
// Phase F: Moved to engine-sim-bridge for reusability (GUI, iOS, headless)

#ifndef I_INPUT_PROVIDER_H
#define I_INPUT_PROVIDER_H

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

    // Gear control
    int gearDelta = 0;           // +1 = shift up, -1 = shift down, 0 = no change

    // Dyno control
    double dynoTorqueScale = -1.0;  // -1 = unchanged, 0.0-1.0 = fraction of max torque

    // Brake control (HACK: engine braking via dyno, not wheel braking)
    double brakeLevel = 0.0;        // 0.0 = no brake, 1.0 = full brake

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

    // Simulator auto-disengages starter when RPM > threshold
    // Preset cycling
    bool presetCycle = false;       // true = cycle to next preset engine configuration
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
