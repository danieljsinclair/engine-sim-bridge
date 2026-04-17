#ifndef ATG_ENGINE_SIM_BRIDGE_H
#define ATG_ENGINE_SIM_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

// Forward declaration for C++ logging interface
#ifdef __cplusplus
class ILogging;
#endif

// ============================================================================
// ENGINE-SIM HEADLESS BRIDGE API
// C-style API for .NET P/Invoke integration
//
// Design Goals:
// 1. Zero-allocation in render path (Render function)
// 2. Persistent state management across callbacks
// 3. 48kHz sample rate support
// 4. Thread-safe for single-threaded audio callback model
// ============================================================================

// Opaque handle to simulator instance
typedef void* EngineSimHandle;

// Result codes
typedef enum {
    ESIM_SUCCESS = 0,
    ESIM_ERROR_INVALID_HANDLE = -1,
    ESIM_ERROR_NOT_INITIALIZED = -2,
    ESIM_ERROR_LOAD_FAILED = -3,
    ESIM_ERROR_INVALID_PARAMETER = -4,
    ESIM_ERROR_AUDIO_BUFFER = -5,
    ESIM_ERROR_SCRIPT_COMPILATION = -6
} EngineSimResult;

// Configuration structure for initialization
typedef struct {
    // Audio configuration
    int32_t sampleRate;              // Target sample rate (e.g., 48000)
    int32_t inputBufferSize;         // Internal buffer size (1024 recommended)
    int32_t audioBufferSize;         // Ring buffer size (96000 = 2s @ 48kHz)

    // Simulation parameters
    int32_t simulationFrequency;     // Hz (10000 recommended)
    int32_t fluidSimulationSteps;    // Substeps per physics step (8 recommended)
    double targetSynthesizerLatency; // seconds (0.05 = 50ms recommended)

    // Audio DSP parameters
    float volume;                    // Master volume (0.0 - 2.0)
    float convolutionLevel;          // Convolution mix (0.0 - 1.0)
    float airNoise;                  // Air intake noise (0.0 - 2.0)

    // Sine wave mode (for testing/development without engine script)
    int32_t sineMode;                 // 1 = sine wave mode, 0 = normal physics mode

} EngineSimConfig;

// Runtime statistics (optional, for diagnostics)
typedef struct {
    double currentRPM;
    double currentLoad;              // 0.0 - 1.0
    double exhaustFlow;              // m^3/s
    double manifoldPressure;         // Pa
    int32_t activeChannels;
    double processingTimeMs;         // Last frame processing time
} EngineSimStats;

// ============================================================================
// LIFECYCLE FUNCTIONS
// ============================================================================

/**
 * Creates a new simulator instance with specified configuration.
 *
 * Initializes the simulator with the given configuration. Does NOT load
 * an engine script - call EngineSimLoadScript() separately after Create().
 *
 * @param config Pointer to configuration structure
 * @param outHandle Pointer to receive the simulator handle
 * @return ESIM_SUCCESS on success, error code otherwise
 *
 * Thread Safety: Safe to call from any thread
 * Allocations: Yes (one-time)
 *
 * Example: EngineSimCreate(&config, &handle)
 */
EngineSimResult EngineSimCreate(
    const EngineSimConfig* config,
    EngineSimHandle* outHandle
);

/**
 * Loads an engine script into the simulator instance.
 *
 * Compiles and loads the specified .mr script file, which contains the engine
 * configuration. Must be called after EngineSimCreate() and before any
 * simulation or rendering functions.
 *
 * @param handle Simulator handle
 * @param scriptPath Absolute path to .mr file (UTF-8)
 * @param assetBasePath Optional base path for assets (WAV files)
 * @return ESIM_SUCCESS on success, error code otherwise
 *
 * Thread Safety: Must be called from main thread
 * Allocations: Yes (script compilation)
 *
 * Example: EngineSimLoadScript(handle, "/path/to/assets/engines/subaru.mr", "/path/to/assets")
 */
EngineSimResult EngineSimLoadScript(
    EngineSimHandle handle,
    const char* scriptPath,
    const char* assetBasePath
);

/**
 * Sets the logging interface for the simulator instance.
 * The bridge will use this for all debug and diagnostic output.
 *
 * @param handle Simulator handle
 * @param logger Pointer to ILogging implementation (caller retains ownership)
 * @return ESIM_SUCCESS on success, error code otherwise
 *
 * Thread Safety: Safe to call from any thread before audio callbacks start
 * Allocations: None
 *
 * Example: EngineSimSetLogging(handle, &myLogger);
 */
EngineSimResult EngineSimSetLogging(
    EngineSimHandle handle,
    ILogging* logger
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
 * Renders audio on-demand - runs simulation and generates audio in the callback.
 * This is the synchronous pull model - audio is generated when the audio
 * callback requests it, rather than being pre-rendered in a background thread.
 *
 * @param handle Simulator handle
 * @param buffer Output buffer for interleaved float stereo samples (L, R, L, R...)
 * @param frames Number of frames to render
 * @param outFramesWritten Pointer to receive actual frames written
 * @return ESIM_SUCCESS on success
 */
EngineSimResult EngineSimRenderOnDemand(
    EngineSimHandle handle,
    float* buffer,
    int32_t frames,
    int32_t* outFramesWritten
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

// ============================================================================
// DIAGNOSTICS & TELEMETRY
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
 * Returns the version string of the engine-sim library.
 * Format: "engine-sim-bridge/1.0.0"
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
 * Loads a pre-compiled engine preset from a JSON file.
 * Use this on iOS where Piranha scripting is unavailable.
 *
 * The preset file must be generated by the macOS preset compiler tool
 * (engine-sim-preset-compiler). It contains serialized engine/vehicle/
 * transmission parameters in JSON format.
 *
 * @param handle Simulator handle (must have been created with EngineSimCreate)
 * @param presetPath Path to the JSON preset file
 * @return ESIM_SUCCESS on success, error code otherwise
 *
 * Thread Safety: Must be called from main thread
 * Allocations: Yes (engine construction)
 *
 * Example: EngineSimLoadPreset(handle, "/path/to/honda_trx520.json")
 */
EngineSimResult EngineSimLoadPreset(
    EngineSimHandle handle,
    const char* presetPath
);

#ifdef __cplusplus
}
#endif

// ============================================================================
// SHARED AUDIO UTILITIES (DRY principle)
// Used by both bridge and mock implementations
// ============================================================================

#ifdef __cplusplus

#include <cstring>

namespace EngineSimAudio {

/**
 * Converts mono int16 samples to stereo float32 (interleaved).
 * Uses correct scale factor: 1.0/32768.0 (NOT 0.5/32768.0).
 * This matches the audio hardware full-scale range.
 */
inline void convertInt16ToStereoFloat(const int16_t* input, float* output, int32_t frameCount) {
    constexpr float scale = 1.0f / 32768.0f;
    for (int32_t i = 0; i < frameCount; ++i) {
        const float sample = static_cast<float>(input[i]) * scale;
        output[i * 2] = sample;     // Left channel
        output[i * 2 + 1] = sample; // Right channel
    }
}

/**
 * Converts mono int16 samples to stereo float32 with clipping protection.
 * Uses correct scale factor: 1.0/32768.0
 */
inline void convertInt16ToStereoFloatClipped(const int16_t* input, float* output, int32_t frameCount) {
    constexpr float scale = 1.0f / 32768.0f;
    for (int32_t i = 0; i < frameCount; ++i) {
        float sample = static_cast<float>(input[i]) * scale;
        if (sample > 1.0f) sample = 1.0f;
        if (sample < -1.0f) sample = -1.0f;
        output[i * 2] = sample;
        output[i * 2 + 1] = sample;
    }
}

/**
 * Fills a stereo float buffer with silence (zeros).
 * This is a DRY utility used throughout the codebase for buffer underruns.
 * Assumes interleaved stereo format (2 channels per frame).
 *
 * @param buffer Pointer to the buffer to fill
 * @param frames Number of frames to fill (each frame = 2 samples)
 */
inline void fillSilence(float* buffer, int32_t frames) {
    std::memset(buffer, 0, frames * 2 * sizeof(float));
}

} // namespace EngineSimAudio
#endif

#endif /* ATG_ENGINE_SIM_BRIDGE_H */
