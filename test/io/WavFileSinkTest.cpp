// WavFileSinkTest.cpp - Contract tests for WavFileSink (the --output capture sink)
//
// WavFileSink is what makes `--output <path>` produce a file at all: the render
// callback hands it every rendered frame and it streams them out as 16-bit PCM
// WAV. These tests assert the OBSERVABLE contract a consumer depends on:
//   - a finalized file is a structurally valid RIFF/WAVE PCM header carrying the
//     sample rate, channel count and data size actually written
//   - the samples read back are the samples handed in (scaled to int16)
//   - out-of-range and NaN input clips/mutes instead of wrapping into noise
//   - a zero-frame run still yields a valid (empty) WAV, not a truncated file
//   - an unopenable path reports isOpen()==false rather than throwing
//   - finalize() is idempotent, and post-finalize writes are ignored
//
// Header fields are decoded from the raw bytes on purpose — asserting against a
// hand-parsed header is what proves the file is readable by an external tool,
// which re-serialising through our own writer would not.

#include "io/WavFileSink.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

// Decoded subset of a canonical 44-byte RIFF/WAVE PCM header.
struct WavHeader {
    std::string riffTag;
    std::string waveTag;
    std::string fmtTag;
    std::string dataTag;
    uint32_t riffSize = 0;
    uint32_t fmtSize = 0;
    uint16_t formatTag = 0;
    uint16_t channels = 0;
    uint32_t sampleRate = 0;
    uint32_t byteRate = 0;
    uint16_t blockAlign = 0;
    uint16_t bitsPerSample = 0;
    uint32_t dataSize = 0;
};

std::vector<unsigned char> readAllBytes(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    return std::vector<unsigned char>(std::istreambuf_iterator<char>(in),
                                      std::istreambuf_iterator<char>());
}

uint32_t decodeU32(const std::vector<unsigned char>& bytes, size_t offset) {
    return static_cast<uint32_t>(bytes[offset])
         | (static_cast<uint32_t>(bytes[offset + 1]) << 8)
         | (static_cast<uint32_t>(bytes[offset + 2]) << 16)
         | (static_cast<uint32_t>(bytes[offset + 3]) << 24);
}

uint16_t decodeU16(const std::vector<unsigned char>& bytes, size_t offset) {
    return static_cast<uint16_t>(static_cast<uint16_t>(bytes[offset])
         | (static_cast<uint16_t>(bytes[offset + 1]) << 8));
}

std::string decodeTag(const std::vector<unsigned char>& bytes, size_t offset) {
    return std::string(reinterpret_cast<const char*>(bytes.data() + offset), 4);
}

WavHeader decodeHeader(const std::vector<unsigned char>& bytes) {
    WavHeader header;
    header.riffTag = decodeTag(bytes, 0);
    header.riffSize = decodeU32(bytes, 4);
    header.waveTag = decodeTag(bytes, 8);
    header.fmtTag = decodeTag(bytes, 12);
    header.fmtSize = decodeU32(bytes, 16);
    header.formatTag = decodeU16(bytes, 20);
    header.channels = decodeU16(bytes, 22);
    header.sampleRate = decodeU32(bytes, 24);
    header.byteRate = decodeU32(bytes, 28);
    header.blockAlign = decodeU16(bytes, 32);
    header.bitsPerSample = decodeU16(bytes, 34);
    header.dataTag = decodeTag(bytes, 36);
    header.dataSize = decodeU32(bytes, 40);
    return header;
}

int16_t decodeSample(const std::vector<unsigned char>& bytes, size_t sampleIndex) {
    return static_cast<int16_t>(decodeU16(bytes, 44 + sampleIndex * 2));
}

constexpr uint32_t HEADER_BYTES = 44;
constexpr int SAMPLE_RATE = 44100;
constexpr int STEREO = 2;

class WavFileSinkTest : public ::testing::Test {
protected:
    void SetUp() override {
        path_ = (std::filesystem::temp_directory_path()
                 / ("wavfilesink_test_" + std::to_string(::testing::UnitTest::GetInstance()
                        ->current_test_info()->line() ) + ".wav")).string();
        std::filesystem::remove(path_);
    }

    void TearDown() override {
        std::filesystem::remove(path_);
    }

    std::string path_;
};

TEST_F(WavFileSinkTest, FinalizedFileCarriesTheFormatItWasGiven) {
    const std::vector<float> frames = {0.0f, 0.0f, 0.25f, -0.25f, 0.5f, -0.5f};
    constexpr int frameCount = 3;

    {
        io::WavFileSink sink(path_, SAMPLE_RATE, STEREO);
        ASSERT_TRUE(sink.isOpen());
        sink.writeFrames(frames.data(), frameCount, STEREO);
        EXPECT_TRUE(sink.finalize());
        EXPECT_EQ(sink.framesWritten(), 3u);
    }

    const auto bytes = readAllBytes(path_);
    const WavHeader header = decodeHeader(bytes);

    EXPECT_EQ(header.riffTag, "RIFF");
    EXPECT_EQ(header.waveTag, "WAVE");
    EXPECT_EQ(header.fmtTag, "fmt ");
    EXPECT_EQ(header.dataTag, "data");
    EXPECT_EQ(header.fmtSize, 16u);
    EXPECT_EQ(header.formatTag, 1u);              // PCM
    EXPECT_EQ(header.channels, STEREO);
    EXPECT_EQ(header.sampleRate, static_cast<uint32_t>(SAMPLE_RATE));
    EXPECT_EQ(header.bitsPerSample, 16u);
    EXPECT_EQ(header.blockAlign, STEREO * 2);
    EXPECT_EQ(header.byteRate, static_cast<uint32_t>(SAMPLE_RATE * STEREO * 2));

    // Sizes must describe the payload actually present, or players read garbage.
    const uint32_t expectedData = frameCount * STEREO * 2;
    EXPECT_EQ(header.dataSize, expectedData);
    EXPECT_EQ(header.riffSize, HEADER_BYTES - 8 + expectedData);
    EXPECT_EQ(bytes.size(), HEADER_BYTES + expectedData);
}

TEST_F(WavFileSinkTest, SamplesReadBackMatchWhatWasWritten) {
    // Exact-representable values so the int16 conversion is checkable.
    const std::vector<float> frames = {1.0f, -1.0f, 0.5f, -0.5f, 0.0f, 0.0f};

    {
        io::WavFileSink sink(path_, SAMPLE_RATE, STEREO);
        sink.writeFrames(frames.data(), 3, STEREO);
        ASSERT_TRUE(sink.finalize());
    }

    const auto bytes = readAllBytes(path_);
    EXPECT_EQ(decodeSample(bytes, 0), 32767);
    EXPECT_EQ(decodeSample(bytes, 1), -32767);
    EXPECT_EQ(decodeSample(bytes, 2), 16384);   // 0.5 * 32767, rounded
    EXPECT_EQ(decodeSample(bytes, 3), -16384);
    EXPECT_EQ(decodeSample(bytes, 4), 0);
    EXPECT_EQ(decodeSample(bytes, 5), 0);
}

TEST_F(WavFileSinkTest, OutOfRangeAndNaNSamplesClipInsteadOfWrapping) {
    // An over-driven render must clip to the rails; a NaN must not become an
    // arbitrary loud sample. Both would otherwise be audible corruption.
    const std::vector<float> frames = {
        4.0f, -4.0f,
        std::nanf(""), 0.0f
    };

    {
        io::WavFileSink sink(path_, SAMPLE_RATE, STEREO);
        sink.writeFrames(frames.data(), 2, STEREO);
        ASSERT_TRUE(sink.finalize());
    }

    const auto bytes = readAllBytes(path_);
    EXPECT_EQ(decodeSample(bytes, 0), 32767);
    EXPECT_EQ(decodeSample(bytes, 1), -32767);
    EXPECT_EQ(decodeSample(bytes, 2), 0);       // NaN -> silence
}

TEST_F(WavFileSinkTest, ZeroFrameCaptureStillProducesAValidEmptyWav) {
    {
        io::WavFileSink sink(path_, SAMPLE_RATE, STEREO);
        ASSERT_TRUE(sink.isOpen());
        EXPECT_TRUE(sink.finalize());
        EXPECT_EQ(sink.framesWritten(), 0u);
    }

    const auto bytes = readAllBytes(path_);
    ASSERT_EQ(bytes.size(), HEADER_BYTES);

    const WavHeader header = decodeHeader(bytes);
    EXPECT_EQ(header.riffTag, "RIFF");
    EXPECT_EQ(header.dataSize, 0u);
    EXPECT_EQ(header.riffSize, HEADER_BYTES - 8);
}

TEST_F(WavFileSinkTest, UnopenablePathReportsNotOpenRatherThanThrowing) {
    // Boundary input: the path comes from a CLI flag, so a bad one is a user
    // error the caller reports — not an exception from the constructor.
    const std::string bad = "/nonexistent_directory_wavfilesink/out.wav";

    io::WavFileSink sink(bad, SAMPLE_RATE, STEREO);

    EXPECT_FALSE(sink.isOpen());
    EXPECT_FALSE(sink.finalize());
    EXPECT_FALSE(std::filesystem::exists(bad));
}

TEST_F(WavFileSinkTest, NonPositiveFormatIsRejectedAsUnopened) {
    io::WavFileSink zeroRate(path_, 0, STEREO);
    EXPECT_FALSE(zeroRate.isOpen());

    io::WavFileSink zeroChannels(path_, SAMPLE_RATE, 0);
    EXPECT_FALSE(zeroChannels.isOpen());
}

TEST_F(WavFileSinkTest, FinalizeIsIdempotentAndLaterWritesAreIgnored) {
    const std::vector<float> frames = {0.5f, 0.5f};
    const std::vector<float> after = {1.0f, 1.0f};

    io::WavFileSink sink(path_, SAMPLE_RATE, STEREO);
    sink.writeFrames(frames.data(), 1, STEREO);

    EXPECT_TRUE(sink.finalize());
    EXPECT_TRUE(sink.finalize());   // second call repeats the result, no re-write

    // A late render callback after teardown must not corrupt the closed file.
    sink.writeFrames(after.data(), 1, STEREO);
    EXPECT_EQ(sink.framesWritten(), 1u);

    const auto bytes = readAllBytes(path_);
    EXPECT_EQ(bytes.size(), HEADER_BYTES + 1 * STEREO * 2);
    EXPECT_EQ(decodeHeader(bytes).dataSize, static_cast<uint32_t>(STEREO * 2));
}

TEST_F(WavFileSinkTest, MismatchedChannelCountIsRejectedRatherThanMisinterleaved) {
    // Writing 1-channel data into a stereo file would silently halve the
    // playback rate of everything after it; the sink must refuse instead.
    const std::vector<float> mono = {0.5f, 0.5f, 0.5f};

    io::WavFileSink sink(path_, SAMPLE_RATE, STEREO);
    sink.writeFrames(mono.data(), 3, 1);

    EXPECT_EQ(sink.framesWritten(), 0u);
    ASSERT_TRUE(sink.finalize());
    EXPECT_EQ(readAllBytes(path_).size(), HEADER_BYTES);
}

TEST_F(WavFileSinkTest, DestructorFinalizesAnAbandonedCapture) {
    // An exception path may destroy the session without an explicit finalize();
    // the partial capture must still be a readable file.
    const std::vector<float> frames = {0.25f, -0.25f};
    {
        io::WavFileSink sink(path_, SAMPLE_RATE, STEREO);
        sink.writeFrames(frames.data(), 1, STEREO);
    }

    const auto bytes = readAllBytes(path_);
    ASSERT_EQ(bytes.size(), HEADER_BYTES + STEREO * 2);
    EXPECT_EQ(decodeHeader(bytes).dataSize, static_cast<uint32_t>(STEREO * 2));
}

TEST_F(WavFileSinkTest, SuccessiveWritesAppendInOrder) {
    const std::vector<float> first = {0.25f, 0.25f};
    const std::vector<float> second = {-0.75f, -0.75f};

    {
        io::WavFileSink sink(path_, SAMPLE_RATE, STEREO);
        sink.writeFrames(first.data(), 1, STEREO);
        sink.writeFrames(second.data(), 1, STEREO);
        ASSERT_TRUE(sink.finalize());
        EXPECT_EQ(sink.framesWritten(), 2u);
    }

    const auto bytes = readAllBytes(path_);
    EXPECT_EQ(decodeSample(bytes, 0), 8192);
    EXPECT_EQ(decodeSample(bytes, 2), -24575);
    EXPECT_EQ(decodeHeader(bytes).dataSize, static_cast<uint32_t>(2 * STEREO * 2));
}

} // namespace
