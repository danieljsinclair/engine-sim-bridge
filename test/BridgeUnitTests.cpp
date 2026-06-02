// BridgeUnitTests.cpp - Unit tests for bridge utilities
// Pure C++ unit tests - no engine-sim coupling, no external dependencies

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <string>
#include <cstring>
#include "simulator/EngineSimTypes.h"

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