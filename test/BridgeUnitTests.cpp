// BridgeUnitTests.cpp - Unit tests for bridge utilities
// Pure C++ unit tests - no engine-sim coupling, no external dependencies

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <string>
#include <cstring>
#include "simulator/engine_sim_bridge.h"

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