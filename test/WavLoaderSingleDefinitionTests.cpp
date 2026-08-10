// WavLoaderSingleDefinitionTests.cpp - Guard against the duplicate-WavLoader trap.
//
// BACKGROUND (a real bug this pins, not a hypothetical):
// The bridge used to carry its own "declaration-only mirror" of WavLoader at
// engine-sim-bridge/include/common/wav_loader.h, alongside the real one at
// engine-sim/include/wav_loader.h. The two drifted: engine-sim's Result gained
// an `int channels` field, the mirror never did. Same class name, two layouts
// (40 vs 32 bytes), but only ONE definition of WavLoader::load in the program
// (engine-sim/src/wav_loader.cpp, built against the 40-byte layout).
//
// Bridge TUs therefore called a 40-byte-layout function through a 32-byte-layout
// view: they read `valid` at the wrong offset and ran ~std::string on a shifted
// `this`. Result: "malloc: pointer being freed was not allocated" and SIGABRT
// across 30 tests, plus impulse-response filenames that came back empty or
// missing their first byte ("s/sound-library/..." instead of "es/...").
//
// Both headers also shared the include guard ATG_ENGINE_SIM_WAV_LOADER_H, so
// whichever was included first silently suppressed the other — meaning the
// failure depended on include order and could appear "fixed" in one build and
// return in the next. That is exactly the kind of trap that needs a test.
//
// WHY THESE ASSERTIONS: a bare sizeof()==40 check would be brittle (padding and
// std::vector size are platform/ABI dependent) and, worse, would NOT actually
// catch the bug — it pins a number, not an agreement between caller and callee.
// The real invariant is that THIS translation unit's view of Result matches the
// view held by the TU that defines WavLoader::load. So we verify it
// behaviourally: call the real load() across the link boundary and confirm the
// fields we read back are the ones it wrote. If a duplicate header with a
// different layout is reintroduced, the field reads land at the wrong offsets
// and these tests fail (or trip the sanitiser) instead of silently corrupting
// the heap at runtime.

#include "wav_loader.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

// Minimal, valid 16-bit PCM mono WAV written by hand, so the test owns its
// fixture and cannot be broken by the shipped sound library being reorganised.
class TinyWavFixture {
public:
    TinyWavFixture() {
        path_ = std::filesystem::temp_directory_path() /
                ("es_wavloader_odr_" + std::to_string(::getpid()) + ".wav");
        writeWav();
    }
    ~TinyWavFixture() {
        std::error_code ec;
        std::filesystem::remove(path_, ec);
    }
    TinyWavFixture(const TinyWavFixture&) = delete;
    TinyWavFixture& operator=(const TinyWavFixture&) = delete;

    const std::string path() const { return path_.string(); }
    static constexpr uint32_t kSampleRate = 44100;
    static constexpr uint16_t kChannels = 1;
    static constexpr uint32_t kFrames = 64;

private:
    void writeWav() const {
        const uint16_t bitsPerSample = 16;
        const uint16_t blockAlign = kChannels * bitsPerSample / 8;
        const uint32_t byteRate = kSampleRate * blockAlign;
        const uint32_t dataBytes = kFrames * blockAlign;
        const uint32_t riffSize = 36 + dataBytes;

        std::ofstream out(path_, std::ios::binary);
        auto u32 = [&out](uint32_t v) { out.write(reinterpret_cast<const char*>(&v), 4); };
        auto u16 = [&out](uint16_t v) { out.write(reinterpret_cast<const char*>(&v), 2); };

        out.write("RIFF", 4);  u32(riffSize);      out.write("WAVE", 4);
        out.write("fmt ", 4);  u32(16);            u16(1); // PCM
        u16(kChannels);        u32(kSampleRate);   u32(byteRate);
        u16(blockAlign);       u16(bitsPerSample);
        out.write("data", 4);  u32(dataBytes);

        for (uint32_t i = 0; i < kFrames; ++i) {
            u16(static_cast<uint16_t>(static_cast<int16_t>(i * 128)));
        }
    }

    std::filesystem::path path_;
};

} // namespace

// The load path across the link boundary. If this TU's Result layout disagrees
// with the defining TU's, `valid` is read from the wrong offset and this fails
// — which is precisely the corruption that used to abort the suite.
TEST(WavLoaderSingleDefinitionTest, LoadReportsSuccessAcrossTheLinkBoundary) {
    const TinyWavFixture wav;

    const WavLoader::Result result = WavLoader::load(wav.path());

    EXPECT_TRUE(result.valid)
        << "load() reported failure for a valid WAV — if a duplicate wav_loader.h "
           "with a different Result layout was reintroduced, 'valid' is being read "
           "at the wrong offset";
}

// Every scalar field must survive the return trip. Reading the CORRECT values
// (not just non-zero ones) is what proves caller and callee agree on offsets.
TEST(WavLoaderSingleDefinitionTest, ScalarFieldsSurviveTheReturnTrip) {
    const TinyWavFixture wav;

    const WavLoader::Result result = WavLoader::load(wav.path());

    ASSERT_TRUE(result.valid);
    EXPECT_EQ(result.sampleRate, static_cast<int>(TinyWavFixture::kSampleRate));
    EXPECT_EQ(result.channels, static_cast<int>(TinyWavFixture::kChannels))
        << "the 'channels' field is the one the deleted bridge mirror lacked — a "
           "wrong value here means two Result layouts are live again";
}

// The std::vector member must come back intact and destruct cleanly. The old
// bug destroyed a shifted std::string/vector, so this is the direct guard
// against the "pointer being freed was not allocated" abort.
TEST(WavLoaderSingleDefinitionTest, SampleBufferIsIntactAndDestructsCleanly) {
    const TinyWavFixture wav;

    {
        const WavLoader::Result result = WavLoader::load(wav.path());

        ASSERT_TRUE(result.valid);
        EXPECT_EQ(result.getSampleCount(),
                  static_cast<size_t>(TinyWavFixture::kFrames) * TinyWavFixture::kChannels);
        ASSERT_NE(result.getData(), nullptr);
    }
    // Leaving the scope runs ~Result. A layout disagreement aborts here.
    SUCCEED() << "Result destructed without heap corruption";
}

// A missing file must report failure rather than throw or corrupt — the error
// path also returns a Result by value, so it exercises the same layout contract.
TEST(WavLoaderSingleDefinitionTest, MissingFileReportsInvalidWithoutThrowing) {
    const auto absent = std::filesystem::temp_directory_path() /
        ("es_wavloader_absent_" + std::to_string(::getpid()) + ".wav");
    ASSERT_FALSE(std::filesystem::exists(absent));

    EXPECT_NO_THROW({
        const WavLoader::Result result = WavLoader::load(absent.string());
        EXPECT_FALSE(result.valid);
    });
}
