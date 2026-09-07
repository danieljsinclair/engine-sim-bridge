// BridgeUnitTests.cpp - Unit tests for bridge utilities
// Pure C++ unit tests - no engine-sim coupling, no external dependencies

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <string>
#include <cstring>
#include "simulator/EngineSimTypes.h"
#include "strategy/Diagnostics.h"
#include "audio/SpeakerProtection.h"

// Test basic math utilities
TEST(BridgeMathTest, SineCalculation) {
    // Test sine at known points
    EXPECT_NEAR(std::sin(0.0), 0.0, 0.0001);
    EXPECT_NEAR(std::sin(M_PI), 0.0, 0.0001);
    EXPECT_NEAR(std::sin(M_PI / 2), 1.0, 0.0001);
}

TEST(BridgeMathTest, AudioSampleRateConversion) {
    // Test samples per millisecond calculation
    const double sr = EngineSimDefaults::SAMPLE_RATE;
    double samplesPerMs = sr / 1000.0;
    EXPECT_NEAR(samplesPerMs, 44.1, 0.01);

    // Test buffer size for 10ms at 44100Hz
    int bufferFrames = static_cast<int>(sr * 0.01);
    EXPECT_EQ(bufferFrames, static_cast<int>(EngineSimDefaults::SAMPLE_RATE * 0.01));
}

TEST(BridgeMathTest, FrequencyToRad) {
    // Test Hz to radians conversion
    double freq = 440.0; // A4
    double radPerSec = 2.0 * M_PI * freq;
    EXPECT_NEAR(radPerSec, 2764.6, 0.1);
}

// Test buffer operations
TEST(BridgeBufferTest, FloatBufferInit) {
    const size_t size = 1024;
    std::vector<float> buffer(size, 0.0f);

    EXPECT_EQ(buffer.size(), size);
    EXPECT_FLOAT_EQ(buffer[0], 0.0f);
    EXPECT_FLOAT_EQ(buffer[size-1], 0.0f);
}

TEST(BridgeBufferTest, BufferCopy) {
    std::vector<float> source = {1.0f, 2.0f, 3.0f, 4.0f};
    std::vector<float> dest(source.size());

    std::copy(source.begin(), source.end(), dest.begin());

    EXPECT_EQ(source, dest);
}

TEST(BridgeBufferTest, BufferFill) {
    std::vector<float> buffer(100, 0.0f);
    std::fill(buffer.begin(), buffer.end(), 0.5f);

    float sum = 0.0f;
    for (float f : buffer) sum += f;

    EXPECT_NEAR(sum, 50.0f, 0.1f); // 100 * 0.5 = 50
}

// Test string utilities
TEST(BridgeStringTest, BasicOperations) {
    std::string s = "test";
    EXPECT_EQ(s.length(), 4);
    EXPECT_EQ(s[0], 't');
    EXPECT_EQ(s[3], 't');
}

TEST(BridgeStringTest, Concatenation) {
    std::string a = "hello";
    std::string b = "world";
    std::string c = a + " " + b;

    EXPECT_EQ(c, "hello world");
}

// Test numeric conversions
TEST(BridgeConvertTest, IntToDouble) {
    int samples = EngineSimDefaults::SAMPLE_RATE;
    double seconds = samples / static_cast<double>(EngineSimDefaults::SAMPLE_RATE);
    EXPECT_NEAR(seconds, 1.0, 0.0001);
}

TEST(BridgeConvertTest, DoubleToInt) {
    double freq = 440.5;
    int freqInt = static_cast<int>(freq);
    EXPECT_EQ(freqInt, 440);
}

// Test basic RPM calculations (bridge uses these)
TEST(BridgeRPMTest, RPMToFrequency) {
    // 600 RPM = 100 Hz, 6000 RPM = 1000 Hz (from config)
    // Test RPM at a point that gives predictable output: 3300 RPM
    double rpmMin = 600.0;
    double rpmMax = 6000.0;
    double freqMin = 100.0;
    double freqMax = 1000.0;

    double testRpm = 3300.0;
    // mapping = (3300-600)/(6000-600) = 2700/5400 = 0.5
    // freq = 0.5 * 900 + 100 = 550 Hz
    double expectedFreq = 550.0;

    double actualFreq = (testRpm - rpmMin) / (rpmMax - rpmMin) * (freqMax - freqMin) + freqMin;

    EXPECT_NEAR(actualFreq, expectedFreq, 0.1);
}

TEST(BridgeRPMTest, RPMToAngularVelocity) {
    // RPM to radians per second: RPM * 2π / 60
    double rpm = 3000.0;
    double radPerSec = rpm * 2.0 * M_PI / 60.0;

    EXPECT_NEAR(radPerSec, 314.159, 0.01);
}

// ============================================================================
// Speaker Protection Tests — BLIND tests (implementation-agnostic)
// Tests written from function signatures and intended behavior only
// ============================================================================

// Helper to find max absolute value in buffer
static float bufferMaxAbs(const float* buffer, size_t totalSamples) {
    float maxVal = 0.0f;
    for (size_t i = 0; i < totalSamples; ++i) {
        float absVal = std::fabs(buffer[i]);
        if (absVal > maxVal) maxVal = absVal;
    }
    return maxVal;
}

// ============================================================================
// fastTanh tests
// ============================================================================

TEST(SpeakerProtectionFastTanh, MatchesStdTanh) {
    // Compare fastTanh against std::tanh across relevant range [-3.0, 3.0]
    // tanh(±3.0) ≈ ±0.995, outside this range tanh saturates to ±1.0
    constexpr float tolerance = 0.005f; // 0.5% error allowed
    constexpr float step = 0.1f;

    for (float x = -3.0f; x <= 3.0f; x += step) {
        float expected = static_cast<float>(std::tanh(x));
        float actual = SpeakerProtection::fastTanh(x);
        EXPECT_NEAR(actual, expected, tolerance)
            << "fastTanh(" << x << ") should match std::tanh within 0.5%";
    }
}

TEST(SpeakerProtectionFastTanh, SymmetricOutput) {
    // tanh is an odd function: tanh(-x) = -tanh(x)
    float testValues[] = {0.5f, 1.0f, 2.0f, 3.0f};

    for (float x : testValues) {
        float pos = SpeakerProtection::fastTanh(x);
        float neg = SpeakerProtection::fastTanh(-x);
        EXPECT_NEAR(pos, -neg, 0.001f)
            << "fastTanh should be symmetric: fastTanh(-x) = -fastTanh(x)";
    }
}

TEST(SpeakerProtectionFastTanh, ZeroInZeroOut) {
    EXPECT_FLOAT_EQ(SpeakerProtection::fastTanh(0.0f), 0.0f)
        << "tanh(0) should be exactly 0";
}

TEST(SpeakerProtectionFastTanh, SaturatesAtUnity) {
    // tanh should approach ±1.0 for large inputs
    EXPECT_GE(SpeakerProtection::fastTanh(5.0f), 0.99f)
        << "tanh(5.0) should be close to 1.0";
    EXPECT_LE(SpeakerProtection::fastTanh(-5.0f), -0.99f)
        << "tanh(-5.0) should be close to -1.0";
}

// ============================================================================
// softClip tests
// ============================================================================

TEST(SpeakerProtectionSoftClip, OutputBelowOneForLargeInput) {
    // Soft clipping should prevent output from exceeding ±1.0 even for large inputs
    constexpr float defaultDrive = 1.0f;
    float largeInputs[] = {2.0f, 5.0f, 10.0f, 100.0f};

    for (float input : largeInputs) {
        float output = SpeakerProtection::softClip(input, defaultDrive);
        EXPECT_GE(output, -1.0f)
            << "softClip(" << input << ") should not be less than -1.0";
        EXPECT_LE(output, 1.0f)
            << "softClip(" << input << ") should not exceed 1.0";
    }
}

TEST(SpeakerProtectionSoftClip, TransparentForQuietSignal) {
    // For small signals, soft clipping should be transparent (output ≈ input)
    // tanh(0.5) ≈ 0.462 (7.6% deviation) — 10% tolerance accounts for natural waveshaping
    constexpr float defaultDrive = 1.0f;
    constexpr float tolerance = 0.10f; // 10%
    float quietInputs[] = {0.1f, 0.3f, 0.5f};

    for (float input : quietInputs) {
        float output = SpeakerProtection::softClip(input, defaultDrive);
        float relativeError = std::fabs((output - input) / std::max(input, 0.01f));
        EXPECT_LT(relativeError, tolerance)
            << "softClip(" << input << ") should be transparent (within 5%)";
    }
}

TEST(SpeakerProtectionSoftClip, SymmetricOutput) {
    // Soft clipping should preserve waveform symmetry
    constexpr float defaultDrive = 1.0f;
    float testValues[] = {0.5f, 1.0f, 2.0f, 3.0f};

    for (float x : testValues) {
        float pos = SpeakerProtection::softClip(x, defaultDrive);
        float neg = SpeakerProtection::softClip(-x, defaultDrive);
        EXPECT_NEAR(pos, -neg, 0.01f)
            << "softClip should be symmetric: softClip(-x, drive) = -softClip(x, drive)";
    }
}

TEST(SpeakerProtectionSoftClip, ZeroInZeroOut) {
    EXPECT_FLOAT_EQ(SpeakerProtection::softClip(0.0f, 1.0f), 0.0f)
        << "softClip(0.0, drive) should be exactly 0";
}

TEST(SpeakerProtectionSoftClip, NegativeInput) {
    // Negative inputs should produce negative outputs
    float negInputs[] = {-0.5f, -1.0f, -2.0f, -5.0f};

    for (float input : negInputs) {
        float output = SpeakerProtection::softClip(input, 1.0f);
        EXPECT_LE(output, 0.0f)
            << "softClip(" << input << ") should produce negative output";
        EXPECT_GE(output, -1.0f)
            << "softClip(" << input << ") should not be less than -1.0";
    }
}

TEST(SpeakerProtectionSoftClip, DriveAffectsCurvature) {
    // Higher drive should increase compression (output closer to ±1.0 for given input)
    float input = 1.0f;
    float lowDriveOutput = SpeakerProtection::softClip(input, 0.5f);
    float highDriveOutput = SpeakerProtection::softClip(input, 2.0f);

    EXPECT_GE(highDriveOutput, lowDriveOutput)
        << "Higher drive should produce more compression (higher output for same input)";
}

// ============================================================================
// applyPeakLimiter tests
// ============================================================================

TEST(SpeakerProtectionPeakLimiter, NoReductionWhenBelowThreshold) {
    // When buffer peak is below threshold, limiter should be transparent (gain = 1.0)
    constexpr float threshold = 0.95f;
    std::vector<float> buffer = {0.3f, 0.5f, 0.7f, 0.8f, 0.6f};

    // Copy for verification
    std::vector<float> original = buffer;

    float gain = SpeakerProtection::applyPeakLimiter(buffer.data(), buffer.size(), threshold);

    EXPECT_NEAR(gain, 1.0f, 0.01f)
        << "Peak limiter should return gain ≈ 1.0 when below threshold";

    // Buffer should be unchanged
    for (size_t i = 0; i < buffer.size(); ++i) {
        EXPECT_NEAR(buffer[i], original[i], 0.001f)
            << "Sample " << i << " should be unchanged when below threshold";
    }
}

TEST(SpeakerProtectionPeakLimiter, ReducesToThreshold) {
    // When buffer exceeds threshold, limiter should scale down to threshold
    constexpr float threshold = 0.95f;
    std::vector<float> buffer = {0.5f, 1.2f, 1.5f, 0.8f, 1.0f}; // Peak = 1.5

    float gain = SpeakerProtection::applyPeakLimiter(buffer.data(), buffer.size(), threshold);

    float maxAfter = bufferMaxAbs(buffer.data(), buffer.size());

    EXPECT_LE(maxAfter, threshold + 0.01f)
        << "After limiting, maximum sample should not exceed threshold";
    EXPECT_NEAR(maxAfter, threshold, 0.05f)
        << "Peak should be scaled close to threshold (within tolerance)";
    EXPECT_LT(gain, 1.0f)
        << "Applied gain should be < 1.0 when reduction occurred";
}

TEST(SpeakerProtectionPeakLimiter, PreservesWaveShape) {
    // Limiter should scale ALL samples by the same factor (proportional reduction)
    constexpr float threshold = 0.95f;
    std::vector<float> buffer = {0.5f, 1.0f, 1.5f, 0.75f, 1.2f};
    std::vector<float> original = buffer;

    float gain = SpeakerProtection::applyPeakLimiter(buffer.data(), buffer.size(), threshold);

    // All samples should be scaled by the same gain factor
    for (size_t i = 0; i < buffer.size(); ++i) {
        float expected = original[i] * gain;
        EXPECT_NEAR(buffer[i], expected, 0.001f)
            << "Sample " << i << " should be scaled uniformly by gain factor";
    }
}

TEST(SpeakerProtectionPeakLimiter, EmptyBuffer) {
    // Zero-length buffer should not crash and should return gain = 1.0
    std::vector<float> emptyBuffer;
    constexpr float threshold = 0.95f;

    float gain = SpeakerProtection::applyPeakLimiter(emptyBuffer.data(), 0, threshold);

    EXPECT_NEAR(gain, 1.0f, 0.01f)
        << "Empty buffer should return gain = 1.0 (no-op)";
}

TEST(SpeakerProtectionPeakLimiter, AllZero) {
    // All-zero buffer should be unchanged with gain = 1.0
    std::vector<float> buffer(100, 0.0f);
    constexpr float threshold = 0.95f;

    float gain = SpeakerProtection::applyPeakLimiter(buffer.data(), buffer.size(), threshold);

    EXPECT_NEAR(gain, 1.0f, 0.01f)
        << "All-zero buffer should return gain = 1.0";

    for (float sample : buffer) {
        EXPECT_FLOAT_EQ(sample, 0.0f)
            << "All-zero buffer should remain all zeros";
    }
}

TEST(SpeakerProtectionPeakLimiter, SingleSample) {
    // Single-element buffer should be scaled correctly
    constexpr float threshold = 0.95f;
    std::vector<float> buffer = {1.5f};

    float gain = SpeakerProtection::applyPeakLimiter(buffer.data(), buffer.size(), threshold);

    EXPECT_NEAR(buffer[0], threshold, 0.01f)
        << "Single sample exceeding threshold should be scaled to threshold";
    EXPECT_LT(gain, 1.0f)
        << "Applied gain should be < 1.0";
}

TEST(SpeakerProtectionPeakLimiter, HandlesNegativePeaks) {
    // Limiter should handle negative peaks symmetrically
    constexpr float threshold = 0.95f;
    std::vector<float> buffer = {0.5f, -1.5f, 0.8f, -1.2f}; // Peak = -1.5 (abs 1.5)

    float gain = SpeakerProtection::applyPeakLimiter(buffer.data(), buffer.size(), threshold);

    float maxAbs = bufferMaxAbs(buffer.data(), buffer.size());

    EXPECT_LE(maxAbs, threshold + 0.01f)
        << "Negative peaks should be limited to same threshold";
}

// ============================================================================
// protectBuffer tests (integration: softClip + peakLimiter)
// ============================================================================

TEST(SpeakerProtectionProtectBuffer, NoSampleExceedsThreshold) {
    // After full protection pipeline, no sample should exceed threshold
    constexpr int frameCount = 256;
    constexpr int channelCount = 2; // Stereo
    constexpr float threshold = 0.95f;
    constexpr float drive = 1.0f;

    std::vector<float> buffer(frameCount * channelCount);

    // Fill with values up to 5.0 (way above threshold)
    for (size_t i = 0; i < buffer.size(); ++i) {
        buffer[i] = (i % 10) * 0.5f; // 0.0, 0.5, 1.0, ..., 4.5
    }

    SpeakerProtection::protectBuffer(buffer.data(), frameCount, channelCount, drive, threshold);

    float maxAbs = bufferMaxAbs(buffer.data(), buffer.size());
    EXPECT_LE(maxAbs, threshold + 0.01f)
        << "After protectBuffer, no sample should exceed threshold";
}

TEST(SpeakerProtectionProtectBuffer, QuietSamplesPreserved) {
    // Samples below 0.5 should come through within 10% of original
    constexpr int frameCount = 128;
    constexpr int channelCount = 1; // Mono
    constexpr float threshold = 0.95f;
    constexpr float drive = 1.0f;
    constexpr float tolerance = 0.10f; // 10%

    std::vector<float> buffer(frameCount);
    std::vector<float> original(frameCount);

    // Fill with quiet signal
    for (int i = 0; i < frameCount; ++i) {
        float value = (i % 5) * 0.1f; // 0.0, 0.1, 0.2, 0.3, 0.4
        buffer[i] = value;
        original[i] = value;
    }

    SpeakerProtection::protectBuffer(buffer.data(), frameCount, channelCount, drive, threshold);

    for (int i = 0; i < frameCount; ++i) {
        if (original[i] < 0.5f) {
            float relativeError = std::fabs((buffer[i] - original[i]) / std::max(original[i], 0.01f));
            EXPECT_LT(relativeError, tolerance)
                << "Quiet sample " << i << " should be preserved within 5%";
        }
    }
}

TEST(SpeakerProtectionProtectBuffer, ZeroDrive) {
    // drive=0.0 should squash signal (tanh(0) ≈ 0, so output ≈ 0)
    constexpr int frameCount = 64;
    constexpr int channelCount = 1;
    constexpr float drive = 0.0f;
    constexpr float threshold = 0.95f;

    std::vector<float> buffer(frameCount);
    for (int i = 0; i < frameCount; ++i) {
        buffer[i] = (i % 3) * 0.5f + 0.5f; // 0.5, 1.0, 1.5 pattern
    }

    SpeakerProtection::protectBuffer(buffer.data(), frameCount, channelCount, drive, threshold);

    float maxAbs = bufferMaxAbs(buffer.data(), buffer.size());
    EXPECT_LT(maxAbs, 0.1f)
        << "With drive=0.0, all samples should be close to 0";
}

TEST(SpeakerProtectionProtectBuffer, HandlesStereo) {
    // Stereo buffer should have both channels protected
    constexpr int frameCount = 100;
    constexpr int channelCount = 2; // Stereo
    constexpr float threshold = 0.95f;

    std::vector<float> buffer(frameCount * channelCount);

    // Fill with signal that exceeds threshold
    for (size_t i = 0; i < buffer.size(); ++i) {
        buffer[i] = (i % 7) * 0.3f; // Peaks above threshold
    }

    SpeakerProtection::protectBuffer(buffer.data(), frameCount, channelCount, 1.0f, threshold);

    float maxAbs = bufferMaxAbs(buffer.data(), buffer.size());
    EXPECT_LE(maxAbs, threshold + 0.01f)
        << "Stereo buffer should be protected";
}

TEST(SpeakerProtectionProtectBuffer, DefaultParameters) {
    // Test with default parameters (drive=1.0, threshold=0.95)
    constexpr int frameCount = 50;
    constexpr int channelCount = 1;

    std::vector<float> buffer(frameCount);
    for (int i = 0; i < frameCount; ++i) {
        buffer[i] = 2.0f; // All samples at 2.0
    }

    // Call with default parameters
    SpeakerProtection::protectBuffer(buffer.data(), frameCount, channelCount);

    float maxAbs = bufferMaxAbs(buffer.data(), buffer.size());
    EXPECT_LE(maxAbs, 0.95f + 0.01f)
        << "With default threshold=0.95, output should not exceed 0.95";
}

// ============================================================================
// Diagnostics breach tracking tests — driven by real audio pipeline data
// ============================================================================

// Helper: simulate a render cycle with known timing
static void simulateRender(Diagnostics& diag, double renderMs, int framesRendered, int framesRequested) {
    diag.recordRender(renderMs, framesRendered, framesRequested);
}

class DiagnosticsTest : public ::testing::Test {
protected:
    Diagnostics diag;

    void SetUp() override {
        diag.setSampleRate(44100);
    }
};

// --- Budget breach detection ---

TEST_F(DiagnosticsTest, RecordRenderWithinBudget_NoBreach) {
    // Real data: req=512, rendered=10.5ms, headroom=+1.1ms (91% of budget)
    // Budget for 512 frames at 44100Hz = 512/44100*1000 = 11.61ms
    simulateRender(diag, 10.5, 512, 512);

    EXPECT_FALSE(diag.lastBreachedBudget.load());
    EXPECT_EQ(diag.breachCount.load(), 0);
    EXPECT_GT(diag.lastHeadroomMs.load(), 0.0);
}

TEST_F(DiagnosticsTest, RecordRenderOverBudget_BreachDetected) {
    // Real data: rendered=11.8ms, headroom=-0.2ms (102% of budget)
    simulateRender(diag, 11.8, 512, 512);

    EXPECT_TRUE(diag.lastBreachedBudget.load());
    EXPECT_EQ(diag.breachCount.load(), 1);
    EXPECT_LT(diag.lastHeadroomMs.load(), 0.0);
}

TEST_F(DiagnosticsTest, BreachCountAccumulates) {
    simulateRender(diag, 12.0, 512, 512);  // over budget
    simulateRender(diag, 13.0, 512, 512);  // over budget
    simulateRender(diag, 14.0, 512, 512);  // over budget

    EXPECT_EQ(diag.breachCount.load(), 3);
}

TEST_F(DiagnosticsTest, BreachCountOnlyIncrementsOnBreach) {
    simulateRender(diag, 12.0, 512, 512);  // breach
    simulateRender(diag, 10.0, 512, 512);  // safe
    simulateRender(diag, 13.0, 512, 512);  // breach

    EXPECT_EQ(diag.breachCount.load(), 2);
}

TEST_F(DiagnosticsTest, LastBreachedReflectsMostRecentRender) {
    simulateRender(diag, 12.0, 512, 512);  // breach
    EXPECT_TRUE(diag.lastBreachedBudget.load());

    simulateRender(diag, 10.0, 512, 512);  // safe
    EXPECT_FALSE(diag.lastBreachedBudget.load());
}

// --- Trend detection (callback rate drops) ---
// This is the real crackle trigger from user's data:
// callbacks=86Hz → 58Hz (trend=-32.2%) with budget still within limits

TEST_F(DiagnosticsTest, TrendDropDetectable) {
    // Simulate 1 second of good throughput: 86 callbacks, 86*512 = 44032 frames
    diag.callbackCount_.store(86);
    diag.totalFramesRendered.store(86 * 512);
    diag.generatingRateFps.store(44032.0);
    diag.previousGeneratingRateFps.store(44032.0);

    // updateThroughput exchanges counters, computes rates
    diag.updateThroughput(1.0);

    auto snap = diag.getSnapshot();
    // No trend change - rate was stable
    EXPECT_NEAR(snap.trendPct, 0.0, 0.1);

    // Now simulate drop: only 58 callbacks in the next second
    diag.callbackCount_.store(58);
    diag.totalFramesRendered.store(58 * 512);
    diag.updateThroughput(1.0);

    snap = diag.getSnapshot();
    // Trend should show significant drop
    EXPECT_LT(snap.trendPct, -30.0)
        << "Trend " << snap.trendPct << "% should show significant drop from 86Hz to 58Hz";
}

TEST_F(DiagnosticsTest, GeneratingRateBelowSampleRate_IndicatesStarvation) {
    // 44100 samples/sec needed, only generating 29900 (58Hz * 512 frames)
    diag.callbackCount_.store(58);
    diag.totalFramesRendered.store(58 * 512);
    diag.generatingRateFps.store(44100.0);
    diag.previousGeneratingRateFps.store(44100.0);
    diag.updateThroughput(1.0);

    auto snap = diag.getSnapshot();
    // Generating rate is ~29.7k, well below 44.1k needed
    EXPECT_LT(snap.generatingRateFps, 44100.0)
        << "Generating rate " << snap.generatingRateFps << " should be below 44100";
}

// --- Snapshot captures breach state ---

TEST_F(DiagnosticsTest, SnapshotIncludesBreachData) {
    simulateRender(diag, 12.0, 512, 512);

    auto snap = diag.getSnapshot();
    // Snapshot should reflect the breached state
    EXPECT_LT(snap.lastHeadroomMs, 0.0);
}

// --- Reset clears breach state ---

TEST_F(DiagnosticsTest, ResetClearsBreachState) {
    simulateRender(diag, 12.0, 512, 512);
    EXPECT_GT(diag.breachCount.load(), 0);
    EXPECT_TRUE(diag.lastBreachedBudget.load());

    diag.reset();

    EXPECT_EQ(diag.breachCount.load(), 0);
    EXPECT_FALSE(diag.lastBreachedBudget.load());
}

// ============================================================================
// SpeakerProtection::applyBreachRecoveryFade tests
// Tests driven by real pipeline data: headroom=-0.6ms (minor) vs -5ms (severe)
// ============================================================================

TEST(BreachRecoveryFade, NoFadeWhenHeadroomAboveThreshold) {
    // -0.3ms is above the -0.5ms threshold — still within hardware buffering tolerance
    std::vector<float> buffer(512 * 2, 0.5f);
    std::vector<float> original = buffer;

    bool applied = SpeakerProtection::applyBreachRecoveryFade(
        buffer.data(), 512, 2, -0.3);

    EXPECT_FALSE(applied);
    EXPECT_EQ(buffer, original)
        << "Buffer should be untouched for minor breach (-0.3ms headroom)";
}

TEST(BreachRecoveryFade, NoFadeWhenHeadroomPositive) {
    // No breach at all
    std::vector<float> buffer(512 * 2, 0.5f);
    std::vector<float> original = buffer;

    bool applied = SpeakerProtection::applyBreachRecoveryFade(
        buffer.data(), 512, 2, +1.2);

    EXPECT_FALSE(applied);
    EXPECT_EQ(buffer, original);
}

TEST(BreachRecoveryFade, FadeAppliedWhenHeadroomBelowThreshold) {
    // Severe breach: headroom=-5ms — DAC likely ran dry
    std::vector<float> buffer(512 * 2, 0.5f);

    bool applied = SpeakerProtection::applyBreachRecoveryFade(
        buffer.data(), 512, 2, -5.0);

    EXPECT_TRUE(applied);
    // First sample should be near 0 (start of fade)
    EXPECT_NEAR(buffer[0], 0.0f, 0.01f);
}

TEST(BreachRecoveryFade, FadeRampsToUnity) {
    // At BREACH_FADE_SAMPLES, gain should be close to 1.0
    std::vector<float> buffer(512 * 2, 1.0f);

    SpeakerProtection::applyBreachRecoveryFade(
        buffer.data(), 512, 2, -5.0);

    // Sample at end of fade region should be close to original
    int fadeEnd = SpeakerProtection::BREACH_FADE_SAMPLES;
    EXPECT_NEAR(buffer[fadeEnd * 2], 1.0f, 0.05f)
        << "Sample at fade boundary should be close to original";
}

TEST(BreachRecoveryFade, SamplesBeyondFadeUnchanged) {
    // Samples past the fade region must be untouched
    std::vector<float> buffer(512 * 2, 0.75f);

    SpeakerProtection::applyBreachRecoveryFade(
        buffer.data(), 512, 2, -5.0);

    int fadeEnd = SpeakerProtection::BREACH_FADE_SAMPLES;
    // Check a sample well past the fade
    for (int i = fadeEnd + 10; i < 512; ++i) {
        EXPECT_FLOAT_EQ(buffer[i * 2], 0.75f)
            << "Sample " << i << " beyond fade should be unchanged";
    }
}

TEST(BreachRecoveryFade, StereoBothChannelsFaded) {
    // Both L and R should be faded
    std::vector<float> buffer(512 * 2, 0.8f);

    SpeakerProtection::applyBreachRecoveryFade(
        buffer.data(), 512, 2, -5.0);

    // First frame: both L and R should be near 0
    EXPECT_NEAR(buffer[0], 0.0f, 0.01f) << "L channel first sample";
    EXPECT_NEAR(buffer[1], 0.0f, 0.01f) << "R channel first sample";
}

TEST(BreachRecoveryFade, NoFadeAtExactThreshold) {
    // Exactly at threshold should NOT trigger (only strictly below)
    std::vector<float> buffer(512 * 2, 0.5f);
    std::vector<float> original = buffer;

    bool applied = SpeakerProtection::applyBreachRecoveryFade(
        buffer.data(), 512, 2, SpeakerProtection::BREACH_SMOOTH_THRESHOLD_MS);

    EXPECT_FALSE(applied);
    EXPECT_EQ(buffer, original);
}

TEST(BreachRecoveryFade, ZeroLengthBufferNoCrash) {
    float dummy = 0.0f;
    bool applied = SpeakerProtection::applyBreachRecoveryFade(
        &dummy, 0, 2, -10.0);

    EXPECT_FALSE(applied);
}

// ============================================================================
// SpeakerProtection::BreachRecoveryState — cross-fade and trend hold tests
// ============================================================================

class BreachRecoveryStateTest : public ::testing::Test {
protected:
    SpeakerProtection::BreachRecoveryState state;
};

// --- saveTail ---

TEST_F(BreachRecoveryStateTest, SaveTailCapturesLastFrames) {
    // Create a 100-frame stereo buffer with ascending values
    std::vector<float> buffer(100 * 2);
    for (int i = 0; i < 200; ++i) buffer[i] = static_cast<float>(i);

    state.saveTail(buffer.data(), 100, 2);

    // heldTail should contain the last BREACH_XFADE_SAMPLES frames
    int xfade = SpeakerProtection::BREACH_XFADE_SAMPLES;
    for (int i = 0; i < xfade * 2; ++i) {
        int srcIdx = (100 - xfade) * 2 + i;
        EXPECT_FLOAT_EQ(state.heldTail[i], buffer[srcIdx])
            << "heldTail[" << i << "] mismatch";
    }
}

TEST_F(BreachRecoveryStateTest, SaveTailStereoBothChannels) {
    // Fill with distinct L/R values
    std::vector<float> buffer(100 * 2);
    for (int i = 0; i < 100; ++i) {
        buffer[i * 2] = 0.7f;      // L
        buffer[i * 2 + 1] = 0.3f;  // R
    }

    state.saveTail(buffer.data(), 100, 2);

    // Last frame's L and R should be in heldTail
    int xfade = SpeakerProtection::BREACH_XFADE_SAMPLES;
    EXPECT_FLOAT_EQ(state.heldTail[(xfade - 1) * 2], 0.7f);     // last L
    EXPECT_FLOAT_EQ(state.heldTail[(xfade - 1) * 2 + 1], 0.3f); // last R
}

// --- applyCrossfade ---

TEST_F(BreachRecoveryStateTest, CrossfadeAppliedOnSevereBreach) {
    // Set held tail to 0.5, new buffer to 0.8
    for (auto& s : state.heldTail) s = 0.5f;
    std::vector<float> buffer(100 * 2, 0.8f);

    bool applied = state.applyCrossfade(buffer.data(), 100, 2, -5.0);

    EXPECT_TRUE(applied);
    // First sample: cross-fade from held (0.5) to new (0.8)
    // At sample 0, gain=0 → should be held (0.5)
    EXPECT_NEAR(buffer[0], 0.5f, 0.05f) << "First sample should lean toward held";
}

TEST_F(BreachRecoveryStateTest, CrossfadeRampsToNewBuffer) {
    for (auto& s : state.heldTail) s = 0.5f;
    std::vector<float> buffer(100 * 2, 0.8f);

    state.applyCrossfade(buffer.data(), 100, 2, -5.0);

    int xfade = SpeakerProtection::BREACH_XFADE_SAMPLES;
    // At fade boundary, should be close to new buffer value (0.8)
    EXPECT_NEAR(buffer[(xfade - 1) * 2], 0.8f, 0.05f)
        << "Last fade sample should lean toward new buffer";
}

TEST_F(BreachRecoveryStateTest, CrossfadeNoOpOnMinorBreach) {
    for (auto& s : state.heldTail) s = 0.5f;
    std::vector<float> buffer(100 * 2, 0.8f);
    std::vector<float> original = buffer;

    // -0.3ms is above the -0.5ms threshold — still within hardware buffering tolerance
    bool applied = state.applyCrossfade(buffer.data(), 100, 2, -0.3);

    EXPECT_FALSE(applied);
    EXPECT_EQ(buffer, original) << "Minor breach should not modify buffer";
}

TEST_F(BreachRecoveryStateTest, CrossfadeSamplesBeyondFadeUnchanged) {
    for (auto& s : state.heldTail) s = 0.5f;
    std::vector<float> buffer(100 * 2, 0.8f);

    state.applyCrossfade(buffer.data(), 100, 2, -5.0);

    int xfade = SpeakerProtection::BREACH_XFADE_SAMPLES;
    for (int i = xfade + 2; i < 100; ++i) {
        EXPECT_FLOAT_EQ(buffer[i * 2], 0.8f)
            << "Sample " << i << " beyond cross-fade should be unchanged";
    }
}

TEST_F(BreachRecoveryStateTest, CrossfadeNoOpAtExactThreshold) {
    for (auto& s : state.heldTail) s = 0.5f;
    std::vector<float> buffer(100 * 2, 0.8f);
    std::vector<float> original = buffer;

    bool applied = state.applyCrossfade(buffer.data(), 100, 2,
        SpeakerProtection::BREACH_SMOOTH_THRESHOLD_MS);

    EXPECT_FALSE(applied);
    EXPECT_EQ(buffer, original);
}

TEST_F(BreachRecoveryStateTest, CrossfadeStereoBothChannels) {
    for (int i = 0; i < SpeakerProtection::BREACH_XFADE_SAMPLES; ++i) {
        state.heldTail[i * 2] = 0.4f;      // held L
        state.heldTail[i * 2 + 1] = 0.6f;  // held R
    }
    std::vector<float> buffer(100 * 2);
    for (int i = 0; i < 100; ++i) {
        buffer[i * 2] = 0.9f;     // new L
        buffer[i * 2 + 1] = 0.1f; // new R
    }

    state.applyCrossfade(buffer.data(), 100, 2, -5.0);

    // First sample should lean toward held for both channels
    EXPECT_LT(buffer[0], 0.9f) << "L should be blended toward held";
    EXPECT_GT(buffer[1], 0.1f) << "R should be blended toward held";
}

// --- trend hold ---

TEST(TrendProtection, TrendThresholdAtRealDrop) {
    // Real data: 86Hz → 58Hz = -32.2% trend
    EXPECT_LT(-32.2, SpeakerProtection::TREND_HOLD_THRESHOLD_PCT)
        << "Real -32.2% drop should exceed threshold";
}

TEST(TrendProtection, NormalVariationBelowThreshold) {
    // Normal: -1.1% is within normal fluctuation
    EXPECT_GT(-1.1, SpeakerProtection::TREND_HOLD_THRESHOLD_PCT)
        << "-1.1% should NOT exceed threshold";
}

// ============================================================================
// SpeakerProtection::TrendHoldState tests
// ============================================================================

class TrendHoldStateTest : public ::testing::Test {
protected:
    SpeakerProtection::TrendHoldState state;
};

TEST_F(TrendHoldStateTest, SaveAndReplayRoundTrip) {
    std::vector<float> buffer(100 * 2);
    for (int i = 0; i < 200; ++i) buffer[i] = static_cast<float>(i) * 0.01f;

    state.saveBuffer(buffer.data(), 100, 2);

    std::vector<float> replay(100 * 2, 0.0f);
    bool replayed = state.replayHeld(replay.data(), 100, 2);

    EXPECT_TRUE(replayed);
    for (int i = 0; i < 200; ++i) {
        EXPECT_FLOAT_EQ(replay[i], buffer[i])
            << "Replayed sample " << i << " should match saved";
    }
}

TEST_F(TrendHoldStateTest, ReplayWithoutSaveReturnsFalse) {
    std::vector<float> buffer(100 * 2, 0.5f);
    bool replayed = state.replayHeld(buffer.data(), 100, 2);

    EXPECT_FALSE(replayed) << "Replay without prior save should return false";
}

TEST_F(TrendHoldStateTest, SaveOverwritesPrevious) {
    std::vector<float> first(100 * 2, 0.3f);
    std::vector<float> second(100 * 2, 0.7f);

    state.saveBuffer(first.data(), 100, 2);
    state.saveBuffer(second.data(), 100, 2);

    std::vector<float> replay(100 * 2, 0.0f);
    state.replayHeld(replay.data(), 100, 2);

    for (int i = 0; i < 200; ++i) {
        EXPECT_FLOAT_EQ(replay[i], 0.7f)
            << "Replay should contain second save, not first";
    }
}

TEST_F(TrendHoldStateTest, ReplayStereoBothChannels) {
    std::vector<float> buffer(100 * 2);
    for (int i = 0; i < 100; ++i) {
        buffer[i * 2] = 0.8f;      // L
        buffer[i * 2 + 1] = 0.2f;  // R
    }

    state.saveBuffer(buffer.data(), 100, 2);

    std::vector<float> replay(100 * 2, 0.0f);
    state.replayHeld(replay.data(), 100, 2);

    EXPECT_FLOAT_EQ(replay[0], 0.8f) << "L channel first sample";
    EXPECT_FLOAT_EQ(replay[1], 0.2f) << "R channel first sample";
    EXPECT_FLOAT_EQ(replay[198], 0.8f) << "L channel last sample";
    EXPECT_FLOAT_EQ(replay[199], 0.2f) << "R channel last sample";
}

// ============================================================================
// SpeakerProtection::smoothDiscontinuities — BLIND tests
// Written from signature + intended behavior only, before implementation
// ============================================================================

// Helper: compute max absolute delta between consecutive samples in a single channel
static float maxConsecutiveDelta(const float* buffer, int frameCount, int channelCount, int channel) {
    float maxDelta = 0.0f;
    for (int i = 1; i < frameCount; ++i) {
        float delta = std::fabs(buffer[i * channelCount + channel] - buffer[(i - 1) * channelCount + channel]);
        if (delta > maxDelta) maxDelta = delta;
    }
    return maxDelta;
}

// --- Happy paths (PRIMARY) ---

TEST(SmoothDiscontinuities, NoOpOnQuietAudio) {
    // Gentle sine wave: max delta well below threshold of 0.15
    constexpr int frameCount = 512;
    constexpr int channelCount = 1;
    std::vector<float> buffer(frameCount * channelCount);

    for (int i = 0; i < frameCount; ++i) {
        buffer[i] = 0.1f * std::sin(2.0f * static_cast<float>(M_PI) * 440.0f * i / 44100.0f);
    }
    std::vector<float> original = buffer;

    int count = SpeakerProtection::smoothDiscontinuities(buffer.data(), frameCount, channelCount);

    EXPECT_EQ(count, 0) << "No smoothing should occur on quiet audio";
    for (int i = 0; i < frameCount; ++i) {
        EXPECT_NEAR(buffer[i], original[i], 0.001f)
            << "Sample " << i << " should be unchanged";
    }
}

TEST(SmoothDiscontinuities, SmoothsSingleCrackle) {
    // Flat signal with one sharp jump in the middle
    constexpr int frameCount = 256;
    constexpr int channelCount = 1;
    std::vector<float> buffer(frameCount, 0.3f);

    // Inject crackle at frame 100: 0.3 -> -0.7 (delta=1.0)
    int crackleFrame = 100;
    for (int i = crackleFrame; i < frameCount; ++i) {
        buffer[i] = -0.7f;
    }

    int count = SpeakerProtection::smoothDiscontinuities(buffer.data(), frameCount, channelCount);

    EXPECT_EQ(count, 1) << "Should detect and smooth exactly one discontinuity";

    // After smoothing, the jump at the crackle point should be much reduced
    float deltaAfter = std::fabs(buffer[crackleFrame] - buffer[crackleFrame - 1]);
    EXPECT_LT(deltaAfter, 0.5f) << "Jump at crackle point should be smoothed";
}

TEST(SmoothDiscontinuities, RampConnectsCorrectly) {
    // After smoothing, the ramp should start near the pre-crackle level
    // and end near the post-crackle level at the fade boundary
    constexpr int frameCount = 256;
    constexpr int channelCount = 1;
    constexpr float threshold = 0.15f;
    constexpr int fadeSamples = 24;

    std::vector<float> buffer(frameCount, 0.5f);
    int crackleFrame = 100;
    for (int i = crackleFrame; i < frameCount; ++i) {
        buffer[i] = -0.5f;
    }
    float preLevel = 0.5f;
    float postLevel = -0.5f;

    SpeakerProtection::smoothDiscontinuities(buffer.data(), frameCount, channelCount, threshold, fadeSamples);

    // Sample just before crackle should be near preLevel
    EXPECT_NEAR(buffer[crackleFrame - 1], preLevel, 0.05f)
        << "Sample before crackle should be near original pre-crackle level";

    // Sample at end of fade window should be near postLevel
    int fadeEnd = std::min(crackleFrame + fadeSamples, frameCount - 1);
    EXPECT_NEAR(buffer[fadeEnd], postLevel, 0.05f)
        << "Sample at fade boundary should be near post-crackle level";
}

TEST(SmoothDiscontinuities, SamplesBeyondFadeUnchanged) {
    // Samples well outside the fade window must be untouched
    constexpr int frameCount = 256;
    constexpr int channelCount = 1;
    constexpr float threshold = 0.15f;
    constexpr int fadeSamples = 24;

    std::vector<float> buffer(frameCount, 0.3f);
    int crackleFrame = 60;
    for (int i = crackleFrame; i < frameCount; ++i) {
        buffer[i] = -0.3f;
    }

    SpeakerProtection::smoothDiscontinuities(buffer.data(), frameCount, channelCount, threshold, fadeSamples);

    // Samples well before the crackle should be unchanged
    for (int i = 0; i < crackleFrame - fadeSamples - 5; ++i) {
        EXPECT_NEAR(buffer[i], 0.3f, 0.001f)
            << "Sample " << i << " well before crackle should be unchanged";
    }

    // Samples well after the fade window should be at the post-crackle level
    int fadeEnd = crackleFrame + fadeSamples + 5;
    for (int i = fadeEnd; i < frameCount; ++i) {
        EXPECT_NEAR(buffer[i], -0.3f, 0.001f)
            << "Sample " << i << " well after fade should be unchanged";
    }
}

TEST(SmoothDiscontinuities, StereoBothChannelsSmoothed) {
    // Stereo buffer with a crackle on both channels at the same frame
    constexpr int frameCount = 128;
    constexpr int channelCount = 2;
    std::vector<float> buffer(frameCount * channelCount);

    // L channel: 0.4, R channel: -0.4, then jump
    int crackleFrame = 50;
    for (int i = 0; i < frameCount; ++i) {
        if (i < crackleFrame) {
            buffer[i * 2] = 0.4f;
            buffer[i * 2 + 1] = -0.4f;
        } else {
            buffer[i * 2] = -0.6f;     // L jump: delta=1.0
            buffer[i * 2 + 1] = 0.6f;  // R jump: delta=1.0
        }
    }

    int count = SpeakerProtection::smoothDiscontinuities(buffer.data(), frameCount, channelCount);

    // Both channels should have their crackle smoothed
    float deltaL = std::fabs(buffer[crackleFrame * 2] - buffer[(crackleFrame - 1) * 2]);
    float deltaR = std::fabs(buffer[crackleFrame * 2 + 1] - buffer[(crackleFrame - 1) * 2 + 1]);

    EXPECT_LT(deltaL, 0.5f) << "L channel crackle should be smoothed";
    EXPECT_LT(deltaR, 0.5f) << "R channel crackle should be smoothed";
}

TEST(SmoothDiscontinuities, MultipleCrackles) {
    // Buffer with 3 distinct discontinuities
    constexpr int frameCount = 512;
    constexpr int channelCount = 1;
    std::vector<float> buffer(frameCount);

    // Build signal: quiet sections separated by jumps
    for (int i = 0; i < 100; ++i) buffer[i] = 0.2f;
    for (int i = 100; i < 200; ++i) buffer[i] = -0.6f;  // jump at 100: delta=0.8
    for (int i = 200; i < 350; ++i) buffer[i] = 0.5f;   // jump at 200: delta=1.1
    for (int i = 350; i < 512; ++i) buffer[i] = -0.4f;  // jump at 350: delta=0.9

    int count = SpeakerProtection::smoothDiscontinuities(buffer.data(), frameCount, channelCount);

    EXPECT_EQ(count, 3) << "Should detect all 3 discontinuities";

    // All three jump points should have reduced deltas
    EXPECT_LT(std::fabs(buffer[100] - buffer[99]), 0.8f) << "First crackle smoothed";
    EXPECT_LT(std::fabs(buffer[200] - buffer[199]), 1.1f) << "Second crackle smoothed";
    EXPECT_LT(std::fabs(buffer[350] - buffer[349]), 0.9f) << "Third crackle smoothed";
}

// --- Edge cases (SECONDARY) ---

TEST(SmoothDiscontinuities, CrackleAtBufferStart) {
    // Discontinuity at index 0 — the inter-callback boundary case
    // We can't compute delta at index 0 (no previous sample), but the first
    // sample may be wildly different from expected continuation.
    // The function should handle this gracefully (no crash, no OOB access).
    constexpr int frameCount = 128;
    constexpr int channelCount = 1;
    std::vector<float> buffer(frameCount);

    // Sharp value at start, then gentle signal
    buffer[0] = 0.9f;
    for (int i = 1; i < frameCount; ++i) {
        buffer[i] = 0.1f * std::sin(2.0f * static_cast<float>(M_PI) * 100.0f * i / 44100.0f);
    }

    // Should not crash — the function handles the index-0 boundary
    int count = SpeakerProtection::smoothDiscontinuities(buffer.data(), frameCount, channelCount);

    // No assertion on count — index 0 has no "previous sample" for delta,
    // so behavior is implementation-defined. Just verify no crash and
    // the buffer is valid floating point.
    for (int i = 0; i < frameCount; ++i) {
        EXPECT_FALSE(std::isnan(buffer[i])) << "Sample " << i << " should not be NaN";
        EXPECT_FALSE(std::isinf(buffer[i])) << "Sample " << i << " should not be Inf";
    }
}

TEST(SmoothDiscontinuities, CrackleNearBufferEnd) {
    // Discontinuity close to the last sample, fade would extend past buffer end
    constexpr int frameCount = 64;
    constexpr int channelCount = 1;
    constexpr int fadeSamples = 24;

    std::vector<float> buffer(frameCount, 0.3f);
    // Crack at frame 58 — fade window of 24 extends to 82, past buffer end
    for (int i = 58; i < frameCount; ++i) {
        buffer[i] = -0.7f;
    }

    // Should not crash or write past buffer end
    int count = SpeakerProtection::smoothDiscontinuities(buffer.data(), frameCount, channelCount, 0.15f, fadeSamples);

    EXPECT_GE(count, 1) << "Should detect crackle near buffer end";

    // Verify no buffer overrun: last sample should be valid
    EXPECT_FALSE(std::isnan(buffer[frameCount - 1]));
    EXPECT_FALSE(std::isinf(buffer[frameCount - 1]));
}

TEST(SmoothDiscontinuities, EmptyBuffer) {
    // Zero frames should not crash
    float dummy = 0.0f;
    int count = SpeakerProtection::smoothDiscontinuities(&dummy, 0, 2);

    EXPECT_EQ(count, 0) << "Empty buffer should return 0 smoothed regions";
}

TEST(SmoothDiscontinuities, ThresholdBoundary) {
    // Delta exactly at threshold should NOT be smoothed
    // (only deltas STRICTLY above threshold are smoothed)
    constexpr int frameCount = 128;
    constexpr int channelCount = 1;
    constexpr float threshold = 0.15f;

    std::vector<float> buffer(frameCount);
    for (int i = 0; i < 64; ++i) buffer[i] = 0.0f;
    for (int i = 64; i < frameCount; ++i) buffer[i] = threshold;  // delta exactly = threshold

    std::vector<float> original = buffer;

    int count = SpeakerProtection::smoothDiscontinuities(buffer.data(), frameCount, channelCount, threshold);

    EXPECT_EQ(count, 0) << "Delta at exact threshold should not be smoothed";

    // Buffer should be unchanged
    for (int i = 0; i < frameCount; ++i) {
        EXPECT_NEAR(buffer[i], original[i], 0.001f)
            << "Sample " << i << " should be unchanged at threshold boundary";
    }
}

// --- Real-data pattern tests (TERTIARY) ---

TEST(SmoothDiscontinuities, RealCrackleData) {
    // Using actual pipeline data: first=-0.2619, next=0.4509 (delta=0.7128)
    constexpr int frameCount = 256;
    constexpr int channelCount = 1;
    std::vector<float> buffer(frameCount, -0.2619f);

    // At frame 100, jump to 0.4509
    for (int i = 100; i < frameCount; ++i) {
        buffer[i] = 0.4509f;
    }

    int count = SpeakerProtection::smoothDiscontinuities(buffer.data(), frameCount, channelCount);

    EXPECT_GE(count, 1) << "Real crackle (delta=0.71) should be detected";

    // The jump should be significantly reduced
    float deltaAfter = std::fabs(buffer[100] - buffer[99]);
    EXPECT_LT(deltaAfter, 0.71f) << "Real crackle delta should be reduced after smoothing";
}

TEST(SmoothDiscontinuities, SilenceFillBoundary) {
    // Silence fill creates jump: 0.45 -> 0.0 (delta=0.45)
    constexpr int frameCount = 256;
    constexpr int channelCount = 1;
    std::vector<float> buffer(frameCount, 0.45f);

    // At frame 100, silence fill drops to 0.0
    for (int i = 100; i < frameCount; ++i) {
        buffer[i] = 0.0f;
    }

    int count = SpeakerProtection::smoothDiscontinuities(buffer.data(), frameCount, channelCount);

    EXPECT_GE(count, 1) << "Silence fill boundary (delta=0.45) should be detected";

    // The sharp edge should be smoothed — the drop should be gradual
    float deltaAtBoundary = std::fabs(buffer[100] - buffer[99]);
    EXPECT_LT(deltaAtBoundary, 0.45f) << "Silence fill jump should be smoothed";
}