// ISimulatorSession.h - Session interface wrapping audio hardware + simulator
// Encapsulates the full runtime lifecycle: audio init, sim loop, preset swap, cleanup.
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
//   ISimulatorSession owns the audio hardware provider and simulator for the lifetime of a
//   "session" (from user opening the app to closing it). Audio hardware is created once
//   and survives preset swaps. The audioBuffer is borrowed (raw pointer) from the client
//   — the client owns it and ensures it outlives the session. On swap, the caller extracts
//   the hardware provider via releaseHardwareProvider() before closing the old session,
//   then passes it to initSimulation() for the new session.
//
// Thread model:
//   - run() blocks the calling thread (main thread) in the simulation loop
//   - stop() and swapPreset() are callable from any thread (iOS UI, signal handler)
//   - The main loop checks for pending swap/stop requests each tick (no locks on hot path)
//
// Lifecycle:
//   1. Create session via initSimulation() factory function (called ONCE)
//   2. Call run() -- blocks until stop() is called, duration expires, or preset cycle requested
//   3. run() returns exit code -- session is still valid, audio hardware is alive
//      Returns 0 on normal exit, EXIT_BUT_CONTINUE_NEXT when user requests preset cycle
//   4. For preset cycling: caller extracts hardware provider from old session via
//      releaseHardwareProvider(), closes old session, creates new session with reused
//      hardware provider. Audio buffer stays alive (client-owned) -- no gap, no re-init.
//   5. Call run() again after swap to resume simulation
//   6. Call close() to tear down audio hardware and simulator
//   7. Delete session

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
//
// ISP: Focused interface for session-level operations.
// OCP: New session types (headless, recording) extend this interface.
// DIP: Clients depend on this abstraction, not on audio/simulator internals.

class ISimulatorSession {
public:
    virtual ~ISimulatorSession() = default;

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /// Run the simulation loop. Blocks the calling thread.
    ///
    /// The loop ticks at ~60Hz, polling input, advancing the simulator,
    /// feeding audio, and updating presentation.
    ///
    /// Returns when:
    ///   - stop() is called from another thread
    ///   - input.shouldContinue becomes false (duration expired, user quit)
    ///
    /// Thread safety: must be called from one thread at a time (the "main" thread).
    /// Not re-entrant -- do not call run() from multiple threads simultaneously.
    ///
    /// @return Exit code: 0 on normal exit, EXIT_BUT_CONTINUE_NEXT when user
    ///         requests preset cycling (e.g. pressed P in CLI).
    virtual int run() = 0;

    /// Stop the running simulation loop.
    ///
    /// Thread-safe: callable from any thread (iOS main thread, signal handler).
    /// Causes run() to return on its next tick (within ~16ms at 60Hz).
    ///
    /// Does NOT tear down audio hardware -- call close() for that.
    /// After stop(), run() can be called again to resume.
    virtual void stop() = 0;

    /// Swap to a new preset mid-run or while stopped.
    ///
    /// For external callers (e.g. iOS UI): triggers an immediate preset swap
    /// without involving the input provider's presetCycle mechanism.
    ///
    /// Thread-safe: callable from any thread. The actual swap is deferred to
    /// the main loop tick (run() thread) to avoid racing with audio callbacks.
    ///
    /// Preserves:
    ///   - Audio hardware (no gap, no re-init)
    ///   - Engine state via ISimulator::saveState/restoreState
    ///   - Drivetrain state (vehicle speed, gear)
    ///
    /// If called while run() is active, the swap happens on the next tick.
    /// If called while stopped, the swap happens immediately (synchronous).
    ///
    /// @param presetPath  Path to new preset (.mr script or .json config)
    virtual void swapPreset(const std::string& presetPath) = 0;

    /// Release ownership of audio hardware provider for reuse by a new session.
    /// Call before close(). After release, close() skips audio cleanup.
    virtual std::unique_ptr<IAudioHardwareProvider> releaseHardwareProvider() = 0;

    /// Tear down audio hardware and simulator.
    ///
    /// Call after run() returns and you are done with the session.
    /// After close(), the session cannot be restarted -- create a new one.
    ///
    /// Not thread-safe: call from the same thread that called run().
    virtual void close() = 0;

    // ========================================================================
    // Accessors
    // ========================================================================

    /// Access the simulator for telemetry queries.
    ///
    /// Returned pointer is valid between session creation and close().
    /// The simulator changes on swapPreset() -- do not cache across swaps.
    ///
    /// Thread safety: safe to call from any thread, but the returned pointer
    /// must not be used during a swapPreset() transition (brief window).
    virtual ISimulator* getSimulator() const = 0;
};

#endif // ISIMULATOR_SESSION_H
