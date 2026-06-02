// ISimulatorSession.h - Session interface wrapping audio hardware + simulator
// Encapsulates the full runtime lifecycle: audio init, sim loop, cleanup.
// One object to manage for CLI, iOS, and any future clients.
//
// DESIGN NOTES
// ============
//
// Hot-swap:
//   Preset changes are triggered by passing existingSession to initSimulation().
//   The session swaps its internal simulator pointer, preserving the CrankingController
//   state and audio hardware. The old simulator is kept alive (previousSimulator_)
//   until the next swap to prevent use-after-free in the audio callback.
//   swapPreset() is the internal mechanism — it is NOT called directly by clients.
//
// Lifecycle:
//   1. Create simulator via SimulatorFactory::createAndConfigure()
//   2. Call initSimulation(config, path, simulator, audioBuffer, ...) — every time
//      First run (no existing session): initializes audio buffer and creates hardware provider.
//      Subsequent runs (pass existingSession): hot-swaps simulator within existing session.
//   3. Call run() -- blocks until stop(), duration expires, or preset cycle requested
//   4. Call close() to tear down audio hardware and simulator
//   5. Delete session

#ifndef ISIMULATOR_SESSION_H
#define ISIMULATOR_SESSION_H

#include <memory>
#include <string>

class ISimulator;

// Forward declarations for injectable interfaces
class IAudioBuffer;
class IAudioHardwareProvider;

// ============================================================================
// ISimulatorSession - Manages audio hardware + simulator lifecycle
// ============================================================================

class ISimulatorSession {
public:
    virtual ~ISimulatorSession() = default;

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /// Run the simulation loop. Blocks the calling thread.
    /// @return Exit code: 0 on normal exit, EXIT_BUT_CONTINUE_NEXT on preset cycle.
    virtual int run() = 0;

    /// Stop the running simulation loop. Thread-safe.
    virtual void stop() = 0;

    /// Tear down audio hardware and simulator. Not restartable after close().
    virtual void close() = 0;

    // ========================================================================
    // Accessors
    // ========================================================================

    /// Access the simulator for telemetry queries.
    virtual ISimulator* getSimulator() const = 0;
};

#endif // ISIMULATOR_SESSION_H
