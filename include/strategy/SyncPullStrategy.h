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

    bool render(AudioBufferDescriptor& buffer) override;
    bool AddFrames(float* buffer, int frameCount) override;

    // Lifecycle Methods
    bool initialize(const AudioStrategyConfig& config) override;
    void prepareBuffer() override;
    bool startPlayback(ISimulator* simulator) override;
    void stopPlayback(ISimulator* simulator) override;
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

    // Throughput timing
    std::chrono::steady_clock::time_point lastThroughputTime_;
};

#endif // SYNC_PULL_STRATEGY_H
