// DeterministicStrategyTests.cpp - The determinism contract of the headless
// audio strategy.
//
// The business value under test is the removal of the live-gate cadence race:
// in sync-pull mode the physics is advanced by the audio callback thread
// (simulator->update(1/sampleRate) with retry-dependent call counts) while the
// input side runs on the loop thread's clock — the interleaving of two
// wall-driven threads. DeterministicStrategy must make the simulation advance
// ONLY from updateSimulation() with the loop's exact fixed dt, and render()
// must never touch the simulator. These are the two invariants that make a
// replay run reproducible; everything else about the strategy is mode
// identity.

#include "strategy/DeterministicStrategy.h"
#include "strategy/IAudioBuffer.h"
#include "simulator/ISimulator.h"
#include "simulator/EngineSimTypes.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace {

// Records every simulator entry point the audio framework can reach. The
// determinism contract is partly NEGATIVE: after render(), nothing here may
// have been called.
class RecordingSimulator : public ISimulator {
public:
    bool create(const ISimulatorConfig&, ILogging*, telemetry::ITelemetryWriter*) override { return true; }
    void destroy() override {}
    std::string getLastError() const override { return ""; }
    const char* getName() const override { return "RecordingSimulator"; }

    void update(double dt) override {
        updateDts.push_back(dt);
    }

    EngineSimStats getStats() const override { return EngineSimStats{}; }
    void setThrottle(double) override {}
    bool renderOnDemand(float*, int32_t, int32_t*) override {
        renderOnDemandCalls++;
        return true;
    }
    bool readAudioBuffer(float*, int32_t, int32_t*) override {
        readAudioBufferCalls++;
        return true;
    }
    bool start() override {
        startCalls++;
        return true;
    }
    void stop() override { stopCalls++; }
    int getSimulationFrequency() const override { return 44100; }

    std::vector<double> updateDts;
    int renderOnDemandCalls = 0;
    int readAudioBufferCalls = 0;
    int startCalls = 0;
    int stopCalls = 0;
};

TEST(DeterministicStrategyTest, UpdateSimulationForwardsExactFixedDt) {
    DeterministicStrategy strategy;
    RecordingSimulator sim;

    // One loop tick: 1/60 s expressed in ms, exactly as SimulationLoop::step
    // passes it (updateInterval() * 1000).
    strategy.updateSimulation(&sim, (1.0 / 60.0) * 1000.0);

    ASSERT_EQ(sim.updateDts.size(), 1u);
    EXPECT_NEAR(sim.updateDts[0], 1.0 / 60.0, 1e-12);
}

TEST(DeterministicStrategyTest, UpdateSimulationIsIdempotentPerTickNoRetries) {
    DeterministicStrategy strategy;
    RecordingSimulator sim;

    // The sync-pull race lives in RETRY-driven update bursts; a fixed number
    // of loop ticks must produce exactly that many updates — never more.
    for (int tick = 0; tick < 120; ++tick) {
        strategy.updateSimulation(&sim, (1.0 / 60.0) * 1000.0);
    }

    ASSERT_EQ(sim.updateDts.size(), 120u);
    for (double dt : sim.updateDts) {
        EXPECT_NEAR(dt, 1.0 / 60.0, 1e-12);
    }
}

TEST(DeterministicStrategyTest, RenderFillsSilenceAndNeverTouchesSimulator) {
    DeterministicStrategy strategy;
    RecordingSimulator sim;

    constexpr int32_t kFrames = 64;
    constexpr int32_t kChannels = 2;
    alignas(float) std::byte storage[static_cast<size_t>(kFrames) * kChannels * sizeof(float)];
    auto* samples = reinterpret_cast<float*>(storage);
    for (size_t i = 0; i < kFrames * kChannels; ++i) samples[i] = 0.5f;  // non-silent seed

    AudioBufferView buffer{samples, kFrames, kChannels};
    ASSERT_TRUE(strategy.render(buffer));

    for (size_t i = 0; i < kFrames * kChannels; ++i) {
        EXPECT_EQ(samples[i], 0.0f) << "sample " << i << " not silenced";
    }
    // The negative contract: rendering must not advance or consume the
    // simulation in any way.
    EXPECT_TRUE(sim.updateDts.empty());
    EXPECT_EQ(sim.renderOnDemandCalls, 0);
    EXPECT_EQ(sim.readAudioBufferCalls, 0);
    EXPECT_EQ(sim.startCalls, 0);
    EXPECT_EQ(sim.stopCalls, 0);
}

TEST(DeterministicStrategyTest, StartPlaybackDoesNotStartSimulatorThread) {
    // SyncPullStrategy (the live default) deliberately does not call
    // simulator->start(); the deterministic strategy must mirror that — the
    // engine-sim synthesizer thread is a wall-clocked thread a deterministic
    // run must not own.
    DeterministicStrategy strategy;
    RecordingSimulator sim;

    EXPECT_TRUE(strategy.startPlayback(&sim));
    EXPECT_TRUE(strategy.isPlaying());
    EXPECT_EQ(sim.startCalls, 0);

    strategy.stopPlayback(&sim);
    EXPECT_FALSE(strategy.isPlaying());
}

TEST(DeterministicStrategyTest, FactoryRoutesDeterministicMode) {
    auto strategy = IAudioBufferFactory::createBuffer(AudioMode::Deterministic);
    ASSERT_TRUE(strategy != nullptr);
    EXPECT_EQ(strategy->getName(), "DeterministicStrategy");
    EXPECT_EQ(strategy->getModeString(), "DETERMINISTIC");
}

} // namespace
