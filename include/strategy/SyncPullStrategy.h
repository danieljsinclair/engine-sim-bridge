// SyncPullStrategy.h - Lock-step audio strategy
// Implements synchronous audio generation where simulation advances with audio playback
// SRP: Single responsibility - only implements synchronous lock-step rendering
// OCP: New strategies can be added without modifying existing code
// DIP: Depends on abstractions (ISimulator), not concrete implementations
// Owns its own state: AudioState, Diagnostics
// Phase F: Moved to engine-sim-bridge submodule

#ifndef SYNC_PULL_STRATEGY_H
#define SYNC_PULL_STRATEGY_H

#include <string>
#include <atomic>
#include <chrono>
#include <memory>

#include "strategy/IAudioBuffer.h"
#include "strategy/AudioState.h"
#include "strategy/Diagnostics.h"
#include "telemetry/ITelemetryProvider.h"
#include "common/ILogging.h"
#include "simulator/EngineSimTypes.h"

class ISimulator;

/**
 * SyncPullStrategy - Lock-step audio generation strategy
 *
 * Engine simulation advances in sync with audio playback.
 * Audio is rendered on-demand synchronously from the simulator via ISimulator.
 * No separate audio thread is needed.
 *
 * Owns its own state: AudioState, Diagnostics.
 */
class SyncPullStrategy : public IAudioBuffer {
public:
    explicit SyncPullStrategy(ILogging* logger = nullptr,
                              telemetry::ITelemetryWriter* telemetry = nullptr);

    const char* getName() const override;
    bool isEnabled() const override;
    bool isPlaying() const override;
    bool shouldDrainDuringWarmup() const override;
    void fillBufferFromEngine(ISimulator* simulator, int defaultFramesPerUpdate) override;

    bool render(AudioBufferView& buffer) override;
    bool AddFrames(float* buffer, int frameCount) override;

    // Lifecycle Methods
    bool initialize(const AudioBufferConfig& config, int sampleRate) override;
    void prepareBuffer() override;
    bool startPlayback(ISimulator* simulator) override;
    void stopPlayback(ISimulator* simulator) override;
    void swapSimulator(ISimulator* newSimulator) override;
    void resetBufferAfterWarmup() override;
    void updateSimulation(ISimulator* simulator, double deltaTimeMs) override;

    void reset() override;
    std::string getModeString() const override;

    // Direct diagnostics access (for internal use and testing)
    const Diagnostics& diagnostics() const { return diagnostics_; }

private:
    // Logger: always non-null (defaults to ConsoleLogger if not injected)
    std::unique_ptr<ILogging> defaultLogger_;
    ILogging* logger_;

    // Telemetry: always non-null (defaults to NullTelemetryWriter if not injected)
    std::unique_ptr<telemetry::ITelemetryWriter> defaultTelemetry_;
    telemetry::ITelemetryWriter* telemetry_;

    // Owned state
    AudioState audioState_;
    Diagnostics diagnostics_;

    // Shutdown flag: set in stopPlayback() so render() can bail out quickly
    std::atomic<bool> shuttingDown_{false};

    // Simulator reference (set during startPlayback)
    ISimulator* simulator_ = nullptr;

    // Sample rate (for update() retry calculations)
    int sampleRate_ = 0;

    // Throughput timing
    std::chrono::steady_clock::time_point lastThroughputTime_;

    // Retry render: advance simulation and retry up to maxRetries times
    bool retryRender(float* dst, int offset, int framesNeeded,
                     int& framesWritten, int maxRetries);

    // render helpers
    int renderChunked(float* dst, int framesToGenerate);
    bool attemptRender(float* dst, int offset, int framesNeeded, int32_t& framesWritten);
    void applyCrossfade(float* dst, int framesRendered);
    void fillRemainingSilence(float* dst, int framesRendered, int framesToGenerate, int remainingFrames);
    void resetFrameRender(int framesToGenerate, int framesRendered, const float* dst, std::chrono::high_resolution_clock::time_point callbackStart);
    void updateTelemetry();

    // Crossfade state for hot-swap (prevents clicks/pops)
    float lastLeftSample_ = 0.0f;
    float lastRightSample_ = 0.0f;
    int crossfadeSamplesRemaining_ = 0;
    static constexpr int CROSSFADE_SAMPLES = 176; // ~4ms at 44100Hz
};

#endif // SYNC_PULL_STRATEGY_H
