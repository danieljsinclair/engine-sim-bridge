// SineWaveRegressionTests.cpp - TDD regression test for buffer aliasing bug
//
// BUG: SineWaveSimulator::renderOnDemand() writes int16 data into the float
// output buffer, then reads int16 values from the SAME buffer while converting
// to float. The float writes (4 bytes) overlap and corrupt subsequent int16
// reads (2 bytes), producing random noise instead of a clean sine wave.
//
// This test MUST FAIL with the buggy code and PASS once the aliasing bug is
// fixed (by using a separate int16 conversion buffer).

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <numeric>

#include "simulator/BridgeSimulator.h"
#include "simulator/SineSimulator.h"

// ============================================================================
// Helper: Compute smoothness metric for a sequence of float samples.
//
// For a clean sine wave, adjacent sample differences are small and consistent.
// For random noise, adjacent differences are large and erratic.
//
// Returns the maximum absolute difference between adjacent samples,
// normalised by the full range (max - min) of the signal. For a sine wave
// sampled at reasonable rates this ratio is well below 1.0 (typically < 0.15).
// For noise it approaches 1.0 because adjacent samples jump across the full
// range.
// ============================================================================
static double computeMaxAdjacentJumpRatio(const std::vector<float>& samples) {
    if (samples.size() < 2) return 0.0;

    float minVal = *std::min_element(samples.begin(), samples.end());
    float maxVal = *std::max_element(samples.begin(), samples.end());
    float range = maxVal - minVal;

    if (range < 1e-6f) return 0.0;  // Flat signal (silence or DC)

    float maxJump = 0.0f;
    for (size_t i = 1; i < samples.size(); ++i) {
        float jump = std::abs(samples[i] - samples[i - 1]);
        if (jump > maxJump) maxJump = jump;
    }

    return static_cast<double>(maxJump) / static_cast<double>(range);
}

// ============================================================================
// Helper: Compute the standard deviation of adjacent sample differences.
//
// A clean sine wave has a LOW standard deviation of differences (smooth,
// predictable changes). Noise has a HIGH standard deviation (random jumps).
// ============================================================================
static double computeDifferenceStdDev(const std::vector<float>& samples) {
    if (samples.size() < 2) return 0.0;

    std::vector<double> diffs;
    diffs.reserve(samples.size() - 1);
    for (size_t i = 1; i < samples.size(); ++i) {
        diffs.push_back(static_cast<double>(samples[i] - samples[i - 1]));
    }

    double mean = std::accumulate(diffs.begin(), diffs.end(), 0.0) / diffs.size();
    double sqSum = std::accumulate(diffs.begin(), diffs.end(), 0.0,
        [mean](double acc, double d) { return acc + (d - mean) * (d - mean); });

    return std::sqrt(sqSum / diffs.size());
}

// ============================================================================
// Test fixture: Creates a SineWaveSimulator ready for rendering.
// ============================================================================
class SineWaveRenderTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto sineSim = std::make_unique<SineSimulator>();
        Simulator::Parameters simParams;
        simParams.systemType = Simulator::SystemType::NsvOptimized;
        sineSim->initialize(simParams);
        sineSim->setSimulationFrequency(EngineSimDefaults::SIMULATION_FREQUENCY);
        sineSim->setFluidSimulationSteps(EngineSimDefaults::FLUID_SIMULATION_STEPS);
        sineSim->setTargetSynthesizerLatency(EngineSimDefaults::TARGET_SYNTH_LATENCY);
        sineSim->loadSimulation(nullptr, nullptr, nullptr);
        simulator_ = std::make_unique<BridgeSimulator>(std::move(sineSim));

        // Use default constructor to get proper defaults from EngineSimDefaults
        ISimulatorConfig config;
        config.sampleRate = EngineSimDefaults::SAMPLE_RATE;
        config.simulationFrequency = EngineSimDefaults::SIMULATION_FREQUENCY;

        bool created = simulator_->create(config, nullptr, nullptr);
        ASSERT_TRUE(created) << "BridgeSimulator::create() failed";

        // Advance simulation to populate synthesizer with sine samples.
        // Without update(), renderOnDemand() has no data to render.
        simulator_->update(0.01);  // 10ms of simulation
    }

    void TearDown() override {
        if (simulator_) {
            simulator_->destroy();
        }
    }

    std::unique_ptr<BridgeSimulator> simulator_;
};

// ============================================================================
// REGRESSION TEST: Buffer aliasing produces noise, not sine wave
//
// This test proves the int16/float aliasing bug in renderOnDemand().
// The bug corrupts every sample after index 0 by overwriting int16 source
// data with float destination data, producing random noise.
//
// Assertion strategy:
// 1. Render a buffer of stereo audio (L/R interleaved)
// 2. Extract the left channel
// 3. Check that the waveform is smooth (not noisy) using two metrics:
//    a. Max adjacent jump ratio: must be < 0.25 (sine waves are smooth)
//    b. Difference std deviation: must be small relative to the signal range
// ============================================================================
TEST_F(SineWaveRenderTest, RenderOnDemand_ProducesSmoothSineWaveNotNoise) {
    // Arrange: Allocate a stereo float buffer (L, R interleaved)
    const int32_t frames = 256;
    const int totalSamples = frames * 2;  // stereo
    std::vector<float> buffer(totalSamples, 0.0f);

    // Act: Render audio
    int32_t written = 0;
    bool result = simulator_->renderOnDemand(buffer.data(), frames, &written);

    // Assert: renderOnDemand must succeed and write frames
    ASSERT_TRUE(result) << "renderOnDemand() returned false";
    ASSERT_GT(written, 0) << "renderOnDemand() wrote 0 frames";

    // Extract left channel for analysis
    std::vector<float> leftChannel(frames);
    for (int32_t i = 0; i < frames; ++i) {
        leftChannel[i] = buffer[i * 2];
    }

    // Check 1: Output is not silence (sine wave has amplitude)
    float maxAbs = 0.0f;
    for (float s : leftChannel) {
        maxAbs = std::max(maxAbs, std::abs(s));
    }
    EXPECT_GT(maxAbs, 0.01f)
        << "Output is silence (all near-zero). Expected a sine wave with amplitude.";

    // Check 2: Smoothness - max adjacent jump ratio.
    // For a sine wave at 133Hz (800RPM/6), sampled at 48kHz, each sample
    // advances by 133/SAMPLE_RATE * 2pi ~ 0.019 radians. The max delta between
    // adjacent samples of a sine wave of amplitude A is approximately:
    //   A * 2 * sin(pi * freq / sampleRate) ~ A * 0.0174
    // So the jump ratio (maxJump/range) is ~ 0.0174/2 ~ 0.0087 for a clean sine.
    // With some margin for synthesizer artifacts, 0.25 is very generous.
    // Noise will produce ratios close to 1.0.
    double jumpRatio = computeMaxAdjacentJumpRatio(leftChannel);
    EXPECT_LT(jumpRatio, 0.25)
        << "Waveform has large adjacent-sample jumps (ratio=" << jumpRatio
        << "), characteristic of noise, not a smooth sine wave. "
        << "This indicates the buffer aliasing bug is present.";

    // Check 3: Smoothness - difference standard deviation relative to range.
    // A smooth sine wave has a predictable, small std deviation of differences.
    // Noise has a large, erratic std deviation.
    float minVal = *std::min_element(leftChannel.begin(), leftChannel.end());
    float maxVal = *std::max_element(leftChannel.begin(), leftChannel.end());
    float range = maxVal - minVal;

    if (range > 1e-6f) {
        double diffStdDev = computeDifferenceStdDev(leftChannel);
        double relativeStdDev = diffStdDev / static_cast<double>(range);

        // For a clean sine wave at 133Hz/48kHz, the relative std dev of
        // differences is approximately 0.007 (very smooth).
        // We use a generous threshold of 0.15 to tolerate synthesizer artifacts.
        // Noise produces values near 0.5-0.6.
        EXPECT_LT(relativeStdDev, 0.15)
            << "Waveform difference std dev is too high (relative=" << relativeStdDev
            << "), indicating noise rather than a smooth sine wave. "
            << "This indicates the buffer aliasing bug is present.";
    }
}

// ============================================================================
// REGRESSION TEST: Specific aliasing signature - sample corruption pattern
//
// This test directly detects the aliasing pattern: with the bug, the float
// write at buffer[i*2] overwrites bytes 0-3, which includes int16Buffer[0]
// AND int16Buffer[1]. So when i=1 reads int16Buffer[1], it reads corrupted
// data. We verify that the first few samples are NOT corrupted in this way
// by checking they are consistent with sine wave expectations.
// ============================================================================
TEST_F(SineWaveRenderTest, RenderOnDemand_FirstSamplesAreNotAliased) {
    // Arrange
    const int32_t frames = 64;
    const int totalSamples = frames * 2;
    std::vector<float> buffer(totalSamples, 0.0f);

    // Act
    int32_t written = 0;
    ASSERT_TRUE(simulator_->renderOnDemand(buffer.data(), frames, &written));
    ASSERT_GT(written, 2) << "Need at least 3 frames to check aliasing";

    // Assert: With the aliasing bug, sample[1] reads corrupted int16 data.
    // In a clean sine wave, consecutive samples should have small, consistent
    // differences. If sample[1] is corrupted, the difference |s[1]-s[0]| will
    // be dramatically different from |s[2]-s[1]| (if s[2] were valid) or
    // from the expected sine wave delta.
    //
    // A simpler approach: check that the first 10 samples all fall within
    // a reasonable range and don't contain sudden discontinuities.

    std::vector<float> leftChannel(static_cast<size_t>(written));
    for (int32_t i = 0; i < written; ++i) {
        leftChannel[i] = buffer[i * 2];
    }

    // Compute expected maximum delta for a sine wave at the simulator's RPM.
    // SineEngine default: speedControl=0, RPM = 800, frequency = 800/6 ~ 133Hz
    // At 44100Hz, angular step per sample = 2*pi*133/44100 ~ 0.019 rad
    // Max amplitude ~ 28000/32768 ~ 0.854
    // Max delta between adjacent samples ~ 0.854 * 2 * sin(0.0087) ~ 0.0148
    // With generous tolerance for synthesizer interpolation: 0.15
    const float maxExpectedDelta = 0.15f;

    for (size_t i = 1; i < leftChannel.size(); ++i) {
        float delta = std::abs(leftChannel[i] - leftChannel[i - 1]);
        EXPECT_LT(delta, maxExpectedDelta)
            << "Sample[" << i << "]=" << leftChannel[i]
            << " differs from sample[" << (i - 1) << "]=" << leftChannel[i - 1]
            << " by " << delta << ", exceeding expected max delta " << maxExpectedDelta
            << ". This is the signature of the buffer aliasing bug.";
    }
}

// ============================================================================
// REGRESSION TEST: Stereo channels should be identical (mono source)
//
// SineWaveSimulator writes the same sample to left and right channels.
// With the aliasing bug, corruption patterns may differ between channels
// because the float writes at odd positions partially overlap differently.
// ============================================================================
TEST_F(SineWaveRenderTest, RenderOnDemand_StereoChannelsAreIdentical) {
    // Arrange
    const int32_t frames = 128;
    const int totalSamples = frames * 2;
    std::vector<float> buffer(totalSamples, 0.0f);

    // Act
    int32_t written = 0;
    ASSERT_TRUE(simulator_->renderOnDemand(buffer.data(), frames, &written));
    ASSERT_GT(written, 0);

    // Assert: Left and right channels should be identical for every frame
    for (int32_t i = 0; i < written; ++i) {
        EXPECT_FLOAT_EQ(buffer[i * 2], buffer[i * 2 + 1])
            << "Frame " << i << ": left=" << buffer[i * 2]
            << " right=" << buffer[i * 2 + 1]
            << " differ. Stereo channels from mono source should be identical.";
    }
}
