// SineWaveFrequencyTest.cpp - Pins the sine generator frequency at 440 Hz.
//
// DEFECT (fixed): SineSimulator::simulateStep_() derived the pitch from a fake
// idle RPM as `rpm / 6.0` (~800 / 6 = ~133 Hz), which never matched the
// documented "--sine: Generate 440Hz sine wave test tone". The help text and
// CLI11 description both say 440 Hz (git history: present since the first
// CLI11 refactor), so the generator was corrected to emit 440 Hz directly.
//
// This test renders a buffer from a SineSimulator and asserts the dominant
// frequency is 440 Hz (within a tight tolerance). It MUST FAIL on the old
// 133 Hz code and PASS on the corrected generator.

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <numeric>
#include <algorithm>

#include "simulator/BridgeSimulator.h"
#include "simulator/SineSimulator.h"
#include "simulator/SineEngine.h"
#include "simulator/SineVehicle.h"
#include "simulator/SineTransmission.h"

namespace {
// Estimate the dominant frequency of a mono float signal via the autocorrelation
// lag of the first peak after zero-lag. Robust for a clean sine.
double estimateFrequency(const std::vector<float>& mono, int sampleRate) {
    if (mono.size() < 4) return 0.0;

    // Remove DC.
    double mean = std::accumulate(mono.begin(), mono.end(), 0.0) / mono.size();
    std::vector<double> x(mono.size());
    for (size_t i = 0; i < mono.size(); ++i) x[i] = mono[i] - mean;

    // Autocorrelation.
    size_t n = x.size();
    std::vector<double> acf(n / 2, 0.0);
    for (size_t lag = 1; lag < acf.size(); ++lag) {
        double sum = 0.0;
        for (size_t i = 0; i + lag < n; ++i) sum += x[i] * x[i + lag];
        acf[lag] = sum;
    }

    // Find the first positive peak after the initial decay (skip lag 0).
    // The fundamental period is the lag of the first local maximum past the
    // first zero-crossing of the autocorrelation.
    size_t searchStart = 2;
    if (searchStart >= acf.size()) return 0.0;

    // Find the maximum in the autocorrelation (the strongest periodic lag).
    size_t bestLag = searchStart;
    double bestVal = acf[searchStart];
    for (size_t lag = searchStart; lag < acf.size(); ++lag) {
        if (acf[lag] > bestVal) {
            bestVal = acf[lag];
            bestLag = lag;
        }
    }
    if (bestVal <= 0.0) return 0.0;

    return static_cast<double>(sampleRate) / static_cast<double>(bestLag);
}

// Build a SineSimulator wrapped in a BridgeSimulator, ready to render.
struct SineFixture {
    void setUp() {
        auto sineSim = std::make_unique<SineSimulator>();
        Simulator::Parameters simParams;
        simParams.systemType = Simulator::SystemType::NsvOptimized;
        sineSim->initialize(simParams);
        sineSim->setSimulationFrequency(EngineSimDefaults::SIMULATION_FREQUENCY);
        sineSim->setFluidSimulationSteps(EngineSimDefaults::FLUID_SIMULATION_STEPS);
        sineSim->setTargetSynthesizerLatency(EngineSimDefaults::TARGET_SYNTH_LATENCY);
        sineSim->loadSimulation(new SineEngine(), new SineVehicle(), new SineTransmission());
        sim_ = std::make_unique<BridgeSimulator>(std::move(sineSim));

        ISimulatorConfig config;
        config.sampleRate = EngineSimDefaults::SAMPLE_RATE;
        config.simulationFrequency = EngineSimDefaults::SIMULATION_FREQUENCY;
        bool created = sim_->create(config, nullptr, nullptr);
        ASSERT_TRUE(created) << "BridgeSimulator::create() failed";
        sim_->update(0.01);  // advance so the synth has sine data
    }

    void tearDown() {
        if (sim_) sim_->destroy();
    }

    // Render `frames` stereo float samples, return the left channel.
    // Calls `checker` (a void function/lambda) with the rendered left channel;
    // this keeps ASSERT_* macros in a void context (gtest requirement).
    template <typename Check>
    void renderAndCheck(int frames, Check checker) {
        std::vector<float> buffer(static_cast<size_t>(frames) * 2, 0.0f);
        int32_t written = 0;
        bool ok = sim_->renderOnDemand(buffer.data(), frames, &written);
        ASSERT_TRUE(ok);
        ASSERT_GT(written, 0);
        int n = std::min(frames, written);
        std::vector<float> left(n);
        for (int i = 0; i < n; ++i) left[i] = buffer[i * 2];
        checker(left);
    }

    std::unique_ptr<BridgeSimulator> sim_;
};
}  // namespace

// ============================================================================
// The sine generator must produce 440 Hz (the documented --sine frequency).
// ============================================================================
TEST(SineWaveFrequency, Generates440Hz) {
    SineFixture f;
    f.setUp();

    // Render ~0.5s of audio at 44100 Hz — enough cycles for a precise estimate.
    constexpr int frames = 22050;
    f.renderAndCheck(frames, [](const std::vector<float>& left) {
        // The output must not be silence.
        float maxAbs = 0.0f;
        for (float s : left) maxAbs = std::max(maxAbs, std::abs(s));
        EXPECT_GT(maxAbs, 0.01f) << "Sine output is silent";

        double freq = estimateFrequency(left, EngineSimDefaults::SAMPLE_RATE);
        double tolerance = 440.0 * 0.05;  // 5% tolerance (22 Hz)
        EXPECT_NEAR(freq, 440.0, tolerance)
            << "Sine generator frequency " << freq << " Hz is not the documented 440 Hz";
    });

    f.tearDown();
}
