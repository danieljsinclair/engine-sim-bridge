#ifndef MOCK_ENGINE_SIM_H
#define MOCK_ENGINE_SIM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <math.h>

// ============================================================================
// MOCK ENGINE-SIM BRIDGE API
// C-style API identical to real engine-sim for seamless replacement
//
// Design Goals:
// 1. Perfect API compatibility with real engine-sim
// 2. Generate sine waves at frequency corresponding to RPM
// 3. Same threading model (audio thread + main thread)
// 4. Replicate all complex behaviors (throttle smoothing, start/stop, etc.)
// 5. Return identical data structures and error codes
// ============================================================================

// Include the real definitions
#include "engine_sim_bridge.h"

// ============================================================================
// LIFECYCLE FUNCTIONS
// ============================================================================

/**
 * Creates a new mock simulator instance with specified configuration.
 *
 * @param config Pointer to configuration structure
 * @param outHandle Pointer to receive the simulator handle
 * @return ESIM_SUCCESS on success, error code otherwise
 *
 * Thread Safety: Safe to call from any thread
 * Allocations: Yes (one-time)
 */
EngineSimResult EngineSimCreate(
    const EngineSimConfig* config,
    EngineSimHandle* outHandle
);

/**
 * Loads a mock engine configuration.
 * This must be called before the simulator can generate audio.
 *
 * @param handle Simulator handle
 * @param scriptPath Absolute path to .mr file (UTF-8) - ignored in mock
 * @param assetBasePath Optional base path for assets (WAV files) - ignored in mock
 * @return ESIM_SUCCESS on success, error code otherwise
 *
 * Thread Safety: Must be called before audio thread starts
 * Allocations: Yes (mock setup)
 */
EngineSimResult EngineSimLoadScript(
    EngineSimHandle handle,
    const char* scriptPath,
    const char* assetBasePath  // Optional, null = use default
);

/**
 * Starts the internal audio processing thread.
 * Call this after loading a script but before rendering audio.
 *
 * @param handle Simulator handle
 * @return ESIM_SUCCESS on success, error code otherwise
 *
 * Thread Safety: Must be called from main thread before audio callbacks
 * Allocations: Creates audio thread
 */
EngineSimResult EngineSimStartAudioThread(
    EngineSimHandle handle
);

/**
 * Destroys the simulator and releases all resources.
 * Stops audio thread if running.
 *
 * @param handle Simulator handle
 * @return ESIM_SUCCESS on success, error code otherwise
 *
 * Thread Safety: Must be called after audio thread has stopped
 * Allocations: Frees all allocated memory
 */
EngineSimResult EngineSimDestroy(
    EngineSimHandle handle
);

// ============================================================================
// CONTROL FUNCTIONS (Hot Path - Called from Main Loop)
// ============================================================================

/**
 * Sets the throttle position (0.0 = closed, 1.0 = WOT).
 * Maps to accelerator pedal input. Thread-safe.
 *
 * @param handle Simulator handle
 * @param position Throttle position (0.0 - 1.0)
 * @return ESIM_SUCCESS on success, error code otherwise
 *
 * Thread Safety: Safe to call from any thread
 * Allocations: NONE (allocation-free)
 */
EngineSimResult EngineSimSetThrottle(
    EngineSimHandle handle,
    double position
);

/**
 * Sets the speed control input (accelerator pedal).
 * This invokes the Governor's feedback loop for dynamic throttle adjustment.
 *
 * @param handle Simulator handle
 * @param position Speed control position (0.0 - 1.0)
 * @return ESIM_SUCCESS on success
 */
EngineSimResult EngineSimSetSpeedControl(
    EngineSimHandle handle,
    double position
);

/**
 * Enables or disables the starter motor.
 *
 * @param handle Simulator handle
 * @param enabled 1 to enable starter, 0 to disable
 * @return ESIM_SUCCESS on success
 */
EngineSimResult EngineSimSetStarterMotor(
    EngineSimHandle handle,
    int enabled
);

/**
 * Enables or disables the ignition system.
 */
EngineSimResult EngineSimSetIgnition(
    EngineSimHandle handle,
    int enabled
);

/**
 * Changes transmission gear.
 * -1 = reverse, 0 = neutral, 1+ = forward gears
 */
EngineSimResult EngineSimShiftGear(
    EngineSimHandle handle,
    int gear
);

/**
 * Sets clutch pressure (0.0 = disengaged, 1.0 = engaged).
 */
EngineSimResult EngineSimSetClutch(
    EngineSimHandle handle,
    double pressure
);

/**
 * Enables or disables dynamometer.
 */
EngineSimResult EngineSimSetDyno(
    EngineSimHandle handle,
    int enabled
);

/**
 * Sets dyno hold mode and target speed.
 */
EngineSimResult EngineSimSetDynoHold(
    EngineSimHandle handle,
    int enabled,
    double speed
);

/**
 * Advances the simulation by the specified time delta.
 * This drives the physics and combustion simulation.
 * Typically called at 60Hz or 120Hz from main loop.
 *
 * @param handle Simulator handle
 * @param deltaTime Time step in seconds (e.g., 1/60 = 0.01666)
 * @return ESIM_SUCCESS on success, error code otherwise
 *
 * Thread Safety: Call from main thread only
 * Allocations: NONE (allocation-free)
 */
EngineSimResult EngineSimUpdate(
    EngineSimHandle handle,
    double deltaTime
);

// ============================================================================
// AUDIO RENDERING (Hot Path - Called from Audio Thread)
// ============================================================================

/**
 * Renders audio samples to the provided buffer.
 * This is the CRITICAL PATH function called from the hardware audio callback.
 *
 * PERFORMANCE REQUIREMENTS:
 * - MUST complete in < 5ms for 256 frames @ 48kHz
 * - MUST be allocation-free (no GC pressure)
 * - Buffer must be pre-allocated by caller
 *
 * @param handle Simulator handle
 * @param buffer Output buffer for interleaved float samples (L, R, L, R...)
 *               Range: -1.0 to +1.0
 * @param frames Number of frames to render (NOT samples)
 *               For stereo: buffer size = frames * 2 * sizeof(float)
 * @param outSamplesWritten Pointer to receive actual samples written (can be NULL)
 * @return ESIM_SUCCESS on success, error code otherwise
 *
 * Thread Safety: Call from audio thread only
 * Allocations: NONE (allocation-free, real-time safe)
 *
 * Example: Render(handle, buf, 256, NULL) fills 512 floats (256 L/R pairs)
 *
 * NOTE: This is for SYNCHRONOUS rendering only (no audio thread).
 *       When using the audio thread (EngineSimStartAudioThread), use
 *       EngineSimReadAudioBuffer() instead to read from the audio buffer.
 */
EngineSimResult EngineSimRender(
    EngineSimHandle handle,
    float* buffer,
    int32_t frames,
    int32_t* outSamplesWritten
);

/**
 * Reads audio samples from the internal audio buffer (asynchronous mode).
 * Use this when the audio thread is running (EngineSimStartAudioThread).
 * This function does NOT render audio - it only reads from the buffer
 * that the audio thread is continuously filling.
 *
 * PERFORMANCE REQUIREMENTS:
 * - MUST complete in < 2ms for 256 frames @ 48kHz
 * - MUST be allocation-free (no GC pressure)
 * - Buffer must be pre-allocated by caller
 *
 * @param handle Simulator handle
 * @param buffer Output buffer for interleaved float samples (L, R, L, R...)
 *               Range: -1.0 to +1.0
 * @param frames Number of frames to read (NOT samples)
 *               For stereo: buffer size = frames * 2 * sizeof(float)
 * @param outSamplesRead Pointer to receive actual samples read (can be NULL)
 * @return ESIM_SUCCESS on success, error code otherwise
 *
 * Thread Safety: Call from main thread only (audio thread writes to buffer)
 * Allocations: NONE (allocation-free, real-time safe)
 *
 * NOTE: This function may return fewer samples than requested if the audio
 *       buffer doesn't have enough data yet. This is normal - the audio thread
 *       is continuously filling the buffer in the background.
 */
EngineSimResult EngineSimReadAudioBuffer(
    EngineSimHandle handle,
    float* buffer,
    int32_t frames,
    int32_t* outSamplesRead
);

/**
 * Waits for the audio rendering thread to finish processing the current frame.
 * This ensures the audio buffer is fully populated before reading, preventing
 * race conditions and underruns.
 *
 * @param handle Simulator handle
 * @return ESIM_SUCCESS on success, error code otherwise
 *
 * Thread Safety: Call from main thread only
 * Synchronization: Uses condition variable to wait for audio thread
 * Allocations: NONE
 */
EngineSimResult EngineSimWaitForAudioFrame(
    EngineSimHandle handle
);

// ============================================================================
// DIAGNOSTICS & TELEMETRY (Optional)
// ============================================================================

/**
 * Retrieves current engine statistics.
 * Useful for UI/debugging but not required for audio generation.
 *
 * @param handle Simulator handle
 * @param outStats Pointer to receive statistics
 * @return ESIM_SUCCESS on success, error code otherwise
 *
 * Thread Safety: Safe to call from any thread
 * Allocations: NONE
 */
EngineSimResult EngineSimGetStats(
    EngineSimHandle handle,
    EngineSimStats* outStats
);

/**
 * Gets the last error message (UTF-8 string).
 * Valid until next API call on the same handle.
 *
 * @param handle Simulator handle
 * @return Error message string, or NULL if no error
 *
 * Thread Safety: Safe to call from any thread
 * Allocations: NONE (returns internal buffer)
 */
const char* EngineSimGetLastError(
    EngineSimHandle handle
);

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * Returns the version string of the mock engine-sim library.
 * Format: "mock-engine-sim/1.0.0"
 *
 * @return Version string (static buffer)
 */
const char* EngineSimGetVersion(void);

/**
 * Validates a configuration structure.
 *
 * @param config Configuration to validate
 * @return ESIM_SUCCESS if valid, error code otherwise
 */
EngineSimResult EngineSimValidateConfig(
    const EngineSimConfig* config
);

/**
 * Loads an impulse response for exhaust audio processing.
 * Must be called after loading a script and before starting audio.
 *
 * @param handle Simulator handle
 * @param exhaustIndex Exhaust system index (0-based)
 * @param impulseData Pointer to int16 impulse response data
 * @param sampleCount Number of samples in impulse data
 * @param volume Volume scale factor (0.0 - 1.0)
 * @return ESIM_SUCCESS on success, error code otherwise
 *
 * Thread Safety: Call from main thread after script load, before audio thread start
 * Allocations: Loads impulse response data into synthesizer
 *
 * Note: Caller is responsible for managing impulse data lifetime
 */
EngineSimResult EngineSimLoadImpulseResponse(
    EngineSimHandle handle,
    int exhaustIndex,
    const int16_t* impulseData,
    int sampleCount,
    float volume
);

/**
 * Sets the target RPM for the engine.
 * When set, the engine will use internal RPM control to reach this target.
 *
 * @param handle Simulator handle
 * @param rpm Target RPM (0-20000)
 * @return ESIM_SUCCESS on success, error code otherwise
 *
 * Thread Safety: Safe to call from any thread
 * Allocations: NONE (allocation-free)
 */
EngineSimResult EngineSimSetRPM(
    EngineSimHandle handle,
    double rpm
);

#ifdef __cplusplus
}
#endif

#endif /* MOCK_ENGINE_SIM_H */