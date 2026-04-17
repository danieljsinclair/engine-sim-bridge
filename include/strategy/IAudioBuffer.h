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

// Simulation configuration
struct AudioStrategyConfig {
    int sampleRate;
    int channels;
};

// Audio mode enumeration
enum class AudioMode {
    Threaded,    // Cursor-chasing mode with separate audio thread
    SyncPull     // Lock-step mode where simulation advances with audio playback
};

/**
 * IAudioBuffer - Interface for audio generation strategies
 *
 * Each strategy implements a different approach to generating audio:
 * - ThreadedStrategy: Cursor-chasing mode with separate audio thread
 * - SyncPullStrategy: Lock-step mode where simulation advances with audio playback
 *
 * SRP: Strategy has single responsibility for audio generation approach
 * OCP: New strategies extend this interface without modification
 */
class IAudioBuffer {
public:
    virtual ~IAudioBuffer() = default;

    // === Core Strategy Methods ===

    virtual const char* getName() const = 0;
    virtual bool isEnabled() const = 0;

    /**
     * Check if the strategy is currently playing audio.
     * Thread-safe: can be called from any thread (e.g. audio callback).
     */
    virtual bool isPlaying() const = 0;

    /**
     * Render audio to the provided buffer.
     * Called by the real-time audio callback.
     */
    virtual bool render(AudioBufferDescriptor& buffer) = 0;

    /**
     * Add frames to the strategy's internal buffer.
     * Called by the main simulation loop.
     */
    virtual bool AddFrames(float* buffer, int frameCount) = 0;

    // === Lifecycle Methods ===

    virtual bool initialize(const AudioStrategyConfig& config) = 0;
    virtual void prepareBuffer() = 0;

    virtual bool startPlayback(ISimulator* simulator) = 0;
    virtual void stopPlayback(ISimulator* simulator) = 0;
    virtual void resetBufferAfterWarmup() = 0;

    // === Strategy-Specific Methods ===

    virtual bool shouldDrainDuringWarmup() const = 0;

    /**
     * Fill internal buffer with audio from the simulator.
     * Called by the main simulation loop each iteration.
     * ThreadedStrategy: reads from simulator via readAudioBuffer, applies cursor chasing.
     * SyncPullStrategy: no-op (audio is generated on-demand in render callback).
     */
    virtual void fillBufferFromEngine(ISimulator* simulator, int defaultFramesPerUpdate) = 0;

    virtual std::string getModeString() const = 0;
    virtual void reset() = 0;

    virtual void updateSimulation(ISimulator* simulator, double deltaTimeMs) = 0;
};

/**
 * Factory for creating audio strategies.
 * OCP: New strategies can be added by extending the factory.
 */
class IAudioBufferFactory {
public:
    static std::unique_ptr<IAudioBuffer> createStrategy(
        AudioMode mode,
        ILogging* logger = nullptr,
        telemetry::ITelemetryWriter* telemetry = nullptr
    );
};

#endif // IAUDIO_BUFFER_H
