// WavWriterTest.cpp - Unit tests for the WavWriter (the --output backend).
//
// Proves the WavWriter produces a valid 16-bit PCM WAV: correct RIFF header,
// the sample-rate/channels encoded in the fmt chunk, and a data chunk whose
// byte size matches frames*channels*2 (i.e. the duration relationship the
// --output smoke test asserts). Also proves float32->int16 conversion clamps
// and scales correctly.

#include <gtest/gtest.h>
#include "common/wav_writer.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

namespace {
std::string tempPath(const std::string& name) {
    return (std::filesystem::temp_directory_path() / name).string();
}

// Read the first 44 bytes of a WAV and parse the fmt + data headers.
struct WavHeader {
    uint32_t sampleRate = 0;
    uint16_t channels = 0;
    uint16_t bitsPerSample = 0;
    uint32_t dataSize = 0;
    bool valid = false;
};

WavHeader parseHeader(const std::string& path) {
    WavHeader h;
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return h;
    uint8_t buf[44];
    size_t n = std::fread(buf, 1, 44, f);
    std::fclose(f);
    if (n < 44) return h;
    if (std::memcmp(buf, "RIFF", 4) != 0) return h;
    if (std::memcmp(buf + 8, "WAVE", 4) != 0) return h;
    if (std::memcmp(buf + 12, "fmt ", 4) != 0) return h;
    h.channels = buf[22] | (static_cast<uint16_t>(buf[23]) << 8);
    h.sampleRate = buf[24] | (static_cast<uint32_t>(buf[25]) << 8) |
                   (static_cast<uint32_t>(buf[26]) << 16) |
                   (static_cast<uint32_t>(buf[27]) << 24);
    h.bitsPerSample = buf[34] | (static_cast<uint16_t>(buf[35]) << 8);
    if (std::memcmp(buf + 36, "data", 4) != 0) return h;
    h.dataSize = buf[40] | (static_cast<uint32_t>(buf[41]) << 8) |
                 (static_cast<uint32_t>(buf[42]) << 16) |
                 (static_cast<uint32_t>(buf[43]) << 24);
    h.valid = true;
    return h;
}

int16_t readSample(const std::string& path, size_t frame, int channel, int channels) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return 0;
    long offset = 44 + static_cast<long>((frame * channels + channel) * 2);
    std::fseek(f, offset, SEEK_SET);
    uint8_t buf[2];
    size_t n = std::fread(buf, 1, 2, f);
    std::fclose(f);
    if (n < 2) return 0;
    return static_cast<int16_t>(buf[0] | (static_cast<uint16_t>(buf[1]) << 8));
}
}  // namespace

// ============================================================================
// WavWriter produces a valid WAV header with the requested format.
// ============================================================================
TEST(WavWriter, WritesValidHeader) {
    const std::string path = tempPath("wav_writer_header.wav");
    std::filesystem::remove(path);

    WavWriter writer;
    ASSERT_TRUE(writer.open(path, 44100, 2));
    // Write a tiny bit of data so the data chunk is non-empty.
    constexpr int frames = 100;
    std::array<float, frames * 2> silence{};  // interleaved stereo, all zero
    EXPECT_GT(writer.writeFrames(silence.data(), frames), 0u);
    writer.close();

    WavHeader h = parseHeader(path);
    EXPECT_TRUE(h.valid) << "WAV header must be valid RIFF/WAVE/PCM";
    EXPECT_EQ(h.sampleRate, 44100u);
    EXPECT_EQ(h.channels, 2u);
    EXPECT_EQ(h.bitsPerSample, 16u);
    EXPECT_EQ(h.dataSize, static_cast<uint32_t>(frames * 2 * 2));

    std::filesystem::remove(path);
}

// ============================================================================
// data chunk size == frames * channels * 2  — the duration relationship.
// A 1-second mono 44100 Hz tone must be 44100*1*2 = 88200 bytes of data.
// ============================================================================
TEST(WavWriter, DataSizeMatchesDuration) {
    const std::string path = tempPath("wav_writer_duration.wav");
    std::filesystem::remove(path);

    constexpr uint32_t rate = 44100;
    constexpr uint32_t channels = 1;
    constexpr int seconds = 1;
    constexpr int frames = static_cast<int>(rate * seconds);

    WavWriter writer;
    ASSERT_TRUE(writer.open(path, rate, channels));

    // Generate a 440 Hz sine in a float buffer.
    std::vector<float> buf(static_cast<size_t>(frames) * channels);
    for (int i = 0; i < frames; ++i) {
        buf[i] = static_cast<float>(std::sin(2.0 * M_PI * 440.0 * i / rate)) * 0.9;
    }
    EXPECT_GT(writer.writeFrames(buf.data(), frames), 0u);
    writer.close();

    WavHeader h = parseHeader(path);
    EXPECT_TRUE(h.valid);
    EXPECT_EQ(h.dataSize, static_cast<uint32_t>(frames * channels * 2));

    std::filesystem::remove(path);
}

// ============================================================================
// float32 [-1, 1] maps to int16 with correct scale and clamping.
// ============================================================================
TEST(WavWriter, FloatToInt16Scaling) {
    const std::string path = tempPath("wav_writer_scale.wav");
    std::filesystem::remove(path);

    WavWriter writer;
    ASSERT_TRUE(writer.open(path, 8000, 1));

    // One frame each: +1.0, -1.0, 0.0, and an out-of-range +2.0 (must clamp).
    float samples[4] = {1.0f, -1.0f, 0.0f, 2.0f};
    EXPECT_GT(writer.writeFrames(samples, 4), 0u);
    writer.close();

    // +1.0 -> 32767 (max via *32767 scaling); -1.0 -> -32767. The convention
    // x*32767 keeps the mapping symmetric and avoids the -32768 asymmetry.
    EXPECT_EQ(readSample(path, 0, 0, 1), 32767);
    EXPECT_EQ(readSample(path, 1, 0, 1), -32767);
    EXPECT_EQ(readSample(path, 2, 0, 1), 0);        // 0.0 -> 0
    EXPECT_EQ(readSample(path, 3, 0, 1), 32767);    // +2.0 clamps to max int16

    std::filesystem::remove(path);
}

// ============================================================================
// Writing zero frames is a no-op (no crash, no data).
// ============================================================================
TEST(WavWriter, ZeroFramesIsNoOp) {
    const std::string path = tempPath("wav_writer_zero.wav");
    std::filesystem::remove(path);

    WavWriter writer;
    ASSERT_TRUE(writer.open(path, 44100, 2));
    float dummy[2] = {0.0f, 0.0f};
    EXPECT_EQ(writer.writeFrames(dummy, 0), 0u);
    writer.close();

    WavHeader h = parseHeader(path);
    EXPECT_TRUE(h.valid);
    EXPECT_EQ(h.dataSize, 0u);

    std::filesystem::remove(path);
}
