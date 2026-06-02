// ISimulatorSession.h - Session interface wrapping audio hardware + simulator
// Encapsulates the full runtime lifecycle: audio init, sim loop, cleanup.
// One object to manage for CLI, iOS, and any future clients.
//
// DESIGN NOTES
// ============
//
// Problem:
//   Creating a new session (including new IAudioHardwareProvider) on every preset swap
//   destroys and recreates audio hardware each time -- causing pops and gaps.
//
// Solution:
//   initSimulation() handles hot-swap internally. When called with an existingSession,
//   it transfers audio hardware and engine state to the new session. The caller just
//   calls initSimulation() every time: first run (no session) creates hardware,
//   subsequent runs (pass old session) reuse it. Single API, no special swap method.
//
// Lifecycle:
//   1. Create simulator via SimulatorFactory::createAndConfigure()
//   2. Create session via initSimulation(config, path, simulator, audioBuffer, existingSession, ...)
//      - No existingSession → fresh session with new audio hardware
//      - existingSession → hot-swap: transfers hardware + state, returns new session
//   3. Call run() -- blocks until stop(), duration expires, or preset cycle requested
//   4. For preset cycling: create new simulator, call initSimulation(old session), call run()
//   5. Call close() to tear down audio hardware and simulator
//   6. Delete session

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

    /// Release ownership of audio hardware provider for reuse by a new session.
    /// Called internally by initSimulation during hot-swap.
    virtual std::unique_ptr<IAudioHardwareProvider> releaseHardwareProvider() = 0;

    /// Tear down audio hardware and simulator. Not restartable after close().
    virtual void close() = 0;

    // ========================================================================
    // Accessors
    // ========================================================================

    /// Access the simulator for telemetry queries.
    virtual ISimulator* getSimulator() const = 0;
};

#endif // ISIMULATOR_SESSION_H
