// ThreadedStrategy.h - Cursor-chasing audio strategy
// SRP: Single responsibility - only implements threaded cursor-chasing rendering
// OCP: New strategies can be added without modifying existing code
// DIP: Depends on abstractions, not concrete implementations
// Owns its own state: AudioState, Diagnostics, CircularBuffer
// Phase F: Moved to engine-sim-bridge submodule

#ifndef THREADED_STRATEGY_H
#define THREADED_STRATEGY_H

#include <string>
#include <memory>
#include <chrono>

#include "strategy/IAudioBuffer.h"
#include "common/CircularBuffer.h"
#include "strategy/AudioState.h"
#include "strategy/Diagnostics.h"
#include "telemetry/ITelemetryProvider.h"
#include "common/ILogging.h"

class ISimulator;

/**
 * ThreadedStrategy - Cursor-chasing audio strategy
 *
 * Audio is generated in the main loop, written to an internal circular buffer,
 * and read by the real-time audio callback at playback cursor position.
 * Maintains ~100ms lead between generation and playback.
 *
 * Owns its own state: AudioState, Diagnostics, CircularBuffer.
 */
class ThreadedStrategy : public IAudioBuffer {
public:
    explicit ThreadedStrategy(ILogging* logger = nullptr,
                              telemetry::ITelemetryWriter* telemetry = nullptr);

    // IAudioBuffer Implementation
    const char* getName() const override;
    bool isEnabled() const override;
    bool isPlaying() const override;

    bool render(AudioBufferView& buffer) override;
    bool AddFrames(float* buffer, int frameCount) override;

    // Lifecycle Methods
    bool initialize(const AudioStrategyConfig& config) override;
    void prepareBuffer() override;
    bool startPlayback(ISimulator* simulator) override;
    void stopPlayback(ISimulator* simulator) override;
    void resetBufferAfterWarmup() override;
    void updateSimulation(ISimulator* simulator, double deltaTimeMs) override;

    // Strategy-Specific Methods
    bool shouldDrainDuringWarmup() const override;
    void fillBufferFromEngine(ISimulator* simulator, int defaultFramesPerUpdate) override;
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
    CircularBuffer circularBuffer_;

    // Internal underrun tracking
    int underrunCount_ = 0;

    // Simulator reference (set during startPlayback)
    ISimulator* simulator_ = nullptr;

    // Throughput timing
    std::chrono::steady_clock::time_point lastThroughputTime_;

    int getAvailableFrames() const;
    void updateDiagnostics(int availableFrames, int framesRequested);
    void publishAudioDiagnostics(int underrunCount, double bufferHealthPct);
    void publishAudioTiming();
};

#endif // THREADED_STRATEGY_H
