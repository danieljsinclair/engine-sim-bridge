// IAudioBuffer.h - Audio buffer interface
// Strategy pattern - abstracts audio generation behavior
// SRP: Single responsibility for audio generation
// OCP: New strategies can be added without modifying core code
// DI: Strategy is injected via IAudioBufferFactory
//
// Phase E: All methods take ISimulator* instead of EngineSimHandle/EngineSimAPI
// Phase F: Moved to engine-sim-bridge submodule

#ifndef IAUDIO_BUFFER_H
#define IAUDIO_BUFFER_H

#include <memory>
#include <string>
#include <atomic>

#include "hardware/AudioTypes.h"
#include "common/ILogging.h"

class ISimulator;

namespace telemetry { class ITelemetryWriter; }

// AudioBufferConfig — Configuration for IAudioBuffer implementations
// Note: No inline defaults - intentionally populated from ISimulatorConfig by caller (SimulationLoop)
// This two-stage init is necessary because the buffer implementation doesn't own the config; it receives values from upstream
struct AudioBufferConfig {
    int channels;
    double synthLatency;  // Target synthesis latency (from ISimulatorConfig)
};

// Audio mode enumeration
enum class AudioMode {
    Threaded,    // Cursor-chasing mode with separate audio thread
    SyncPull     // Lock-step mode where simulation advances with audio playback
};

/**
 * IAudioBuffer - Interface for audio generation approaches
 *
 * Each implementation provides a different approach to generating audio:
 * - ThreadedStrategy: Cursor-chasing mode with separate audio thread
 * - SyncPullStrategy: Lock-step mode where simulation advances with audio playback
 *
 * SRP: Each buffer has single responsibility for audio generation approach
 * OCP: New buffers extend this interface without modification
 */
class IAudioBuffer {
public:
    virtual ~IAudioBuffer() = default;

    // === Core Buffer Methods ===

    virtual const char* getName() const = 0;
    virtual bool isEnabled() const = 0;

    /**
     * Check if the buffer is currently playing audio.
     * Thread-safe: can be called from any thread (e.g. audio callback).
     */
    virtual bool isPlaying() const = 0;

    /**
     * Render audio to the provided buffer.
     * Called by the real-time audio callback.
     */
    virtual bool render(AudioBufferView& buffer) = 0;

    /**
     * Add frames to the buffer's internal storage.
     * Called by the main simulation loop.
     */
    virtual bool AddFrames(float* buffer, int frameCount) = 0;

    // === Lifecycle Methods ===

    virtual bool initialize(const AudioBufferConfig& config, int sampleRate) = 0;
    virtual void prepareBuffer() = 0;

    virtual bool startPlayback(ISimulator* simulator) = 0;
    virtual void stopPlayback(ISimulator* simulator) = 0;
    virtual void resetBufferAfterWarmup() = 0;

    // === Buffer-Specific Methods ===

    virtual bool shouldDrainDuringWarmup() const = 0;

    /**
     * Fill internal buffer with audio from the simulator.
     * Called by the main simulation loop each iteration.
     * ThreadedStrategy implementation: reads from simulator via readAudioBuffer, applies cursor chasing.
     * SyncPullStrategy implementation: no-op (audio is generated on-demand in render callback).
     */
    virtual void fillBufferFromEngine(ISimulator* simulator, int defaultFramesPerUpdate) = 0;

    virtual std::string getModeString() const = 0;
    virtual void reset() = 0;

    virtual void updateSimulation(ISimulator* simulator, double deltaTimeMs) = 0;
};

/**
 * Factory for creating audio buffers.
 * OCP: New buffers can be added by extending the factory.
 */
class IAudioBufferFactory {
public:
    static std::unique_ptr<IAudioBuffer> createBuffer(
        AudioMode mode,
        ILogging* logger = nullptr,
        telemetry::ITelemetryWriter* telemetry = nullptr
    );
};

#endif // IAUDIO_BUFFER_H
