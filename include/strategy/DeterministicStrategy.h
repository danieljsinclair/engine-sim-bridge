// DeterministicStrategy.h - Fixed-timestep headless audio strategy
//
// The determinism mode's audio strategy: the simulation advances ON THE LOOP
// THREAD at the fixed 1/60 s update interval, with no audio callback thread
// and no wall-clock dependence. In the default sync-pull mode the physics is
// advanced by the AUDIO callback thread (SyncPullStrategy::retryRender calls
// simulator->update(1/sampleRate) with retry counts that depend on buffer
// readiness), while the input side advances on the loop thread's own clock —
// two threads at incommensurate wall-driven rates. Their interleaving varies
// run to run, which is the live-gate cadence race. This strategy removes the
// second clock: updateSimulation() forwards the loop's fixed dt straight to
// simulator->update(), and render() is silence (audio is not the product of a
// deterministic replay/gate run; pair with NullAudioHardwareProvider so no
// callback thread exists at all).

#ifndef DETERMINISTIC_STRATEGY_H
#define DETERMINISTIC_STRATEGY_H

#include "strategy/IAudioBuffer.h"

#include <atomic>

class ISimulator;

class DeterministicStrategy : public IAudioBuffer {
public:
    DeterministicStrategy(ILogging* logger = nullptr,
                          telemetry::ITelemetryWriter* telemetry = nullptr);

    // === IAudioBuffer ===
    const char* getName() const override;
    bool isEnabled() const override;
    bool isPlaying() const override;

    // Silence only. A deterministic run has no audio output; the pair is that
    // this method must NEVER advance the simulator (that is the sync-pull
    // cadence race this strategy exists to remove).
    bool render(AudioBufferView& buffer) override;

    bool AddFrames(float* buffer, int frameCount) override;

    bool initialize(const AudioBufferConfig& config, int sampleRate) override;
    void prepareBuffer() override;

    // Mirrors SyncPullStrategy: playback state only — does NOT call
    // simulator->start() (no synthesizer thread in a deterministic run).
    bool startPlayback(ISimulator* simulator) override;
    void stopPlayback(ISimulator* simulator) override;
    void swapSimulator(ISimulator* newSimulator) override;
    void resetBufferAfterWarmup() override;
    bool shouldDrainDuringWarmup() const override;

    void fillBufferFromEngine(ISimulator* simulator, int defaultFramesPerUpdate) override;

    std::string getModeString() const override;
    void reset() override;

    // THE contract: advance the simulation by exactly deltaTimeMs/1000 on the
    // calling (loop) thread. One call, one update, exact dt — no retries, no
    // sample-rate fractions, no clock reads.
    void updateSimulation(ISimulator* simulator, double deltaTimeMs) override;

private:
    ILogging* logger_;
    telemetry::ITelemetryWriter* telemetry_;
    std::atomic<bool> isPlaying_{false};
    std::atomic<ISimulator*> simulator_{nullptr};
};

#endif // DETERMINISTIC_STRATEGY_H
