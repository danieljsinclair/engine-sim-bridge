// ScriptLoadHelpersLoadImpulseResponsesTests.cpp - Behavior tests for IR loading
//
// Pins the contract of ScriptLoadHelpers::loadImpulseResponses around its
// decomposition (cpp:S3776/S134): candidates from audioFileCandidates are
// probed in priority order, a missing reference fails fast with three ASSET
// error diagnostics naming the primary candidate, and a successful load
// reports the winning candidate and its sample count. Exhaust systems without
// an impulse response are skipped, not failures.
//
// Uses the real production objects (Simulator, Engine, ExhaustSystem,
// ImpulseResponse, WavLoader) with temp-directory assets; only the logger is
// replaced, with a recording fake, so diagnostics stay assertable.

#include "simulator/ScriptLoadHelpers.h"
#include "common/ILogging.h"

#include "engine.h"
#include "exhaust_system.h"
#include "impulse_response.h"
#include "simulator/SineSimulator.h"
#include "synthesizer.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <vector>

#include <unistd.h>

namespace fs = std::filesystem;

namespace {

// Unique-per-process scratch directory, removed on scope exit.
struct TempDir {
    fs::path path;

    TempDir() {
        static int counter = 0;
        path = fs::temp_directory_path() /
            ("ir_load_test_" + std::to_string(::getpid()) + "_" +
             std::to_string(++counter));
        fs::create_directories(path);
    }
    ~TempDir() {
        std::error_code ec;
        fs::remove_all(path, ec);
    }
};

// Recording logger: captures (mask, message) pairs for assertion.
class RecordingLogger : public ILogging {
public:
    struct Record {
        uint32_t mask;
        std::string message;
    };

    void setMask(uint32_t) override {}
    uint32_t getMask() const override { return LogMask::ALL; }

    const std::vector<Record>& records() const { return records_; }

    size_t countWithLevel(uint32_t level) const {
        size_t count = 0;
        for (const Record& record : records_) {
            if ((record.mask & level) != 0) ++count;
        }
        return count;
    }

    bool anyMessageContains(uint32_t level, const std::string& needle) const {
        for (const Record& record : records_) {
            if ((record.mask & level) != 0 &&
                record.message.find(needle) != std::string::npos) {
                return true;
            }
        }
        return false;
    }

protected:
    void _write(uint32_t mask, const std::string& msg) override {
        records_.push_back({mask, msg});
    }

private:
    std::vector<Record> records_;
};

#pragma pack(push, 1)
struct MonoWavHeader {
    char riff[4] = {'R', 'I', 'F', 'F'};
    uint32_t chunkSize = 36;
    char wave[4] = {'W', 'A', 'V', 'E'};
    char fmt[4] = {'f', 'm', 't', ' '};
    uint32_t subchunk1Size = 16;
    uint16_t audioFormat = 1;
    uint16_t numChannels = 1;
    uint32_t sampleRate = 44100;
    uint32_t byteRate = 44100 * 1 * 2;
    uint16_t blockAlign = 2;
    uint16_t bitsPerSample = 16;
    char data[4] = {'d', 'a', 't', 'a'};
    uint32_t subchunk2Size = 0;
};
#pragma pack(pop)

// Write a minimal valid mono 16-bit PCM wav with `sampleCount` frames.
void writeMonoWav(const fs::path& file, uint32_t sampleCount) {
    fs::create_directories(file.parent_path());
    MonoWavHeader header;
    header.subchunk2Size = sampleCount * 2;
    header.chunkSize = 36 + header.subchunk2Size;

    std::ofstream out(file, std::ios::binary);
    out.write(reinterpret_cast<const char*>(&header), sizeof(header));
    for (uint32_t i = 0; i < sampleCount; ++i) {
        const int16_t sample = static_cast<int16_t>(200);
        out.write(reinterpret_cast<const char*>(&sample), sizeof(sample));
    }
}

} // namespace

class LoadImpulseResponsesTest : public ::testing::Test {
protected:
    void SetUp() override {
        // One synthesizer channel so initializeImpulseResponse has a target.
        Synthesizer::Parameters synthParams;
        simulator_.synthesizer().initialize(synthParams);
    }

    void TearDown() override {
        // engine-sim idiom: destroy() before destruction (the destructor
        // asserts the arrays were released).
        engine_.destroy();
    }

    // Minimal real engine: no cylinders, one exhaust system whose impulse
    // response references `reference`. The impulse response must outlive the
    // exhaust system, so it is a fixture member.
    Engine& engineWithImpulseReference(const std::string& reference) {
        Engine::Parameters params{};
        params.exhaustSystemCount = 1;
        engine_.initialize(params);

        impulse_.initialize(reference, 0.5);
        ExhaustSystem::Parameters exhaust{};
        exhaust.impulseResponse = &impulse_;
        engine_.getExhaustSystem(0)->initialize(exhaust);
        return engine_;
    }

    // Same engine but the exhaust system has no impulse response attached.
    Engine& engineWithoutImpulseResponse() {
        Engine::Parameters params{};
        params.exhaustSystemCount = 1;
        engine_.initialize(params);

        ExhaustSystem::Parameters exhaust{};
        exhaust.impulseResponse = nullptr;
        engine_.getExhaustSystem(0)->initialize(exhaust);
        return engine_;
    }

    SineSimulator simulator_;
    Engine engine_;
    ImpulseResponse impulse_;
    RecordingLogger logger_;
};

// (a) A null engine is rejected outright, with no diagnostics.
TEST(LoadImpulseResponsesGuardTest, NullEngine_ReturnsFalseWithoutDiagnostics) {
    SineSimulator simulator;
    RecordingLogger logger;

    EXPECT_FALSE(ScriptLoadHelpers::loadImpulseResponses(
        &simulator, nullptr, "/tmp", &logger));
    EXPECT_TRUE(logger.records().empty());
}

// (b) A loadable reference anchored against the script family's asset base
//     succeeds and reports the winning candidate with its sample count. The
//     reference is written engine-sim-root relative ("es/sound-library/...");
//     the base carries the family's own sound-library copy, which must win
//     over every search-root probe.
TEST_F(LoadImpulseResponsesTest, LoadsAnchoredCandidateFromAssetBase) {
    TempDir temp;
    const std::string base = (temp.path / "es_new").string();
    const fs::path anchored = temp.path / "es_new" / "sound-library" / "smooth" / "ir.wav";
    writeMonoWav(anchored, 64);

    Engine& engine = engineWithImpulseReference("es/sound-library/smooth/ir.wav");

    EXPECT_TRUE(ScriptLoadHelpers::loadImpulseResponses(
        &simulator_, &engine, base, &logger_));

    ASSERT_EQ(logger_.records().size(), 1u);
    EXPECT_NE(logger_.records()[0].mask & LogMask::INFO, 0u);
    EXPECT_EQ(logger_.countWithLevel(LogMask::ERROR), 0u);
    EXPECT_TRUE(logger_.anyMessageContains(LogMask::INFO, anchored.string()))
        << "info log must name the anchored candidate, got: "
        << logger_.records()[0].message;
    EXPECT_TRUE(logger_.anyMessageContains(LogMask::INFO, "64 samples"))
        << "info log must report the loaded sample count, got: "
        << logger_.records()[0].message;
}

// (c) A reference no probe can load fails fast: exactly three ASSET error
//     diagnostics naming the primary candidate, the asset base and the
//     reference as written, and the pipe-joined list of candidates tried.
//     The engine is not reported loaded (no info line).
TEST_F(LoadImpulseResponsesTest, MissingReference_FailsFastWithAssetDiagnostics) {
    TempDir temp;
    const std::string base = (temp.path / "es_new").string();
    fs::create_directories(temp.path / "es_new" / "sound-library" / "none");

    Engine& engine = engineWithImpulseReference("es/sound-library/none/missing.wav");

    EXPECT_FALSE(ScriptLoadHelpers::loadImpulseResponses(
        &simulator_, &engine, base, &logger_));

    ASSERT_EQ(logger_.countWithLevel(LogMask::ERROR), 3u)
        << "failure must log primary, asset-base and candidates-tried errors";
    EXPECT_EQ(logger_.countWithLevel(LogMask::INFO), 0u);

    EXPECT_TRUE(logger_.anyMessageContains(LogMask::ERROR, "Failed to load required audio file"));
    EXPECT_TRUE(logger_.anyMessageContains(LogMask::ERROR, "missing.wav"))
        << "the failure must name the primary candidate";
    EXPECT_TRUE(logger_.anyMessageContains(LogMask::ERROR, "asset base: " + base));
    EXPECT_TRUE(logger_.anyMessageContains(LogMask::ERROR, "from script: es/sound-library/none/missing.wav"));
    EXPECT_TRUE(logger_.anyMessageContains(LogMask::ERROR, "Candidates tried:"));
    EXPECT_TRUE(logger_.anyMessageContains(LogMask::ERROR, " | "))
        << "candidates must be pipe-joined";
    for (const auto& record : logger_.records()) {
        EXPECT_NE(record.mask & LogMask::ASSET, 0u) << record.message;
    }
}

// (d) An exhaust system without an impulse response is skipped, not a
//     failure: the load succeeds silently.
TEST_F(LoadImpulseResponsesTest, ExhaustWithoutImpulseResponse_SucceedsSilently) {
    TempDir temp;
    Engine& engine = engineWithoutImpulseResponse();

    EXPECT_TRUE(ScriptLoadHelpers::loadImpulseResponses(
        &simulator_, &engine, (temp.path / "es_new").string(), &logger_));
    EXPECT_TRUE(logger_.records().empty());
}
