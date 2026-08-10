// PresetEngineFactoryTests.cpp - Error-path coverage for the preset loaders
//
// PresetEngineFactory is the production entry point for loading JSON presets.
// These tests pin the documented structured-error contract: loaders NEVER throw
// on bad external input — they return a PresetLoadResult whose success() is
// false and whose error string explains the failure. We assert the contract
// (success()==false + non-empty error), never the word-for-word message.
//
// Reachable zero-coverage paths exercised here:
//   - loadFromJson(validPtr, n)                         -> delegation path
//   - loadFromFile(nonexistent)                         -> missing-file branch
//   - loadFromFile(relativePath)                        -> !is_absolute() branch
//   - loadFromString("{}")                              -> no-engine structured return
//   - loadFromString(malformed)                         -> JSON parse error catch
//
// NOT tested (documented findings):
//   - The `catch (const PresetException&)` branch in loadFromString is
//     effectively dead — PresetValidationException is never thrown anywhere and
//     plain PresetException is never thrown directly, so the only subtype that
//     could reach it (PresetDeserializationException) is caught by the preceding
//     catch.
//   - The loadFromJson null/empty guard: a null pointer from an internal caller
//     is a programmer error (should-never-happen), not anticipated external
//     input, so per exception-specificity guidance the guard should be an
//     assert/remove rather than a graceful return. The branch is left untested
//     (FLAGGED for assert/remove in a refactor) instead of pinning the current
//     graceful behavior.

#include "simulator/PresetEngineFactory.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <unistd.h>

// --- loadFromJson: the delegation path (entire function was 0-hit) ------------

// Non-empty valid-JSON pointer delegates into loadFromString. "{}" parses but
// has no engine section -> success() is false via the structured-return path,
// proving the delegation path was taken.
TEST(PresetEngineFactoryTest, LoadFromJsonDelegatesParsedContent) {
    const std::string json = "{}";
    const auto result = PresetEngineFactory::loadFromJson(json.data(), json.size());
    // Delegated + parsed; no engine section -> structured failure, not a throw.
    EXPECT_FALSE(result.success());
    EXPECT_FALSE(result.error.empty());
}

// --- loadFromFile: the missing-file and relative-path branches ----------------

// A nonexistent file -> structured error, no throw. (loadFromFile was hit only
// on the happy path; the !file.is_open() branch was zero-hit.)
TEST(PresetEngineFactoryTest, LoadFromFileMissingReturnsStructuredError) {
    const auto nonexistent = std::filesystem::temp_directory_path() /
        ("es_bridge_nope_" + std::to_string(::getpid()) + ".json");
    ASSERT_FALSE(std::filesystem::exists(nonexistent));

    const auto result = PresetEngineFactory::loadFromFile(nonexistent.string());
    EXPECT_FALSE(result.success());
    EXPECT_FALSE(result.error.empty());
    EXPECT_EQ(result.engine, nullptr);
}

// A relative path that does not exist exercises the !is_absolute() normalization
// branch and then falls through to the missing-file branch.
TEST(PresetEngineFactoryTest, LoadFromFileRelativeMissingNormalizesThenErrors) {
    const std::string relative = "definitely_not_a_real_preset_zzz.json";
    ASSERT_FALSE(std::filesystem::exists(relative));

    const auto result = PresetEngineFactory::loadFromFile(relative);
    EXPECT_FALSE(result.success());
    EXPECT_FALSE(result.error.empty());
}

// --- loadFromString: structured return without throwing -----------------------

// "{}" parses successfully but the preset has no engine section. The loader
// must return a structured failure (success()==false), NOT throw. This is the
// normal-return path (distinct from the catch blocks).
TEST(PresetEngineFactoryTest, LoadFromStringWithNoEngineSectionReturnsStructuredError) {
    const auto result = PresetEngineFactory::loadFromString("{}", "<test>");
    EXPECT_FALSE(result.success());
    EXPECT_FALSE(result.error.empty());
    EXPECT_EQ(result.engine, nullptr);
}

// Malformed JSON -> the loader catches the parse failure and returns a
// structured error (success()==false), never throws to the caller.
TEST(PresetEngineFactoryTest, LoadFromStringMalformedJsonReturnsStructuredError) {
    const auto result = PresetEngineFactory::loadFromString("{not valid json", "<test>");
    EXPECT_FALSE(result.success());
    EXPECT_FALSE(result.error.empty());
    EXPECT_EQ(result.engine, nullptr);
}

// --- resolveAfterfireWavPaths: glob expansion + the empty-result contract -----
//
// This is the seam the CLI's --afterfire-wav depends on. An empty result is NOT
// merely "fall back to the default": for a NON-EMPTY request it is the signal
// BridgeSimulator::configureAfterfire turns into a hard failure, so the
// distinction between "matched files" and "matched nothing" is load-bearing.
// Tests build their own fixture tree in a temp dir rather than depending on the
// shipped sound library, so they cannot break when assets are re-organised.

namespace {

// Temp directory containing a few named WAV-ish files, removed on destruction.
class AfterfireWavFixture {
public:
    AfterfireWavFixture() {
        dir_ = std::filesystem::temp_directory_path() /
               ("es_afterfire_glob_" + std::to_string(::getpid()));
        std::filesystem::create_directories(dir_);
        for (const char* name : {"pop_01.wav", "pop_02.wav", "pop_03.wav", "other.wav"}) {
            std::ofstream(dir_ / name) << "RIFF";
        }
    }
    ~AfterfireWavFixture() {
        std::error_code ec;
        std::filesystem::remove_all(dir_, ec);
    }
    AfterfireWavFixture(const AfterfireWavFixture&) = delete;
    AfterfireWavFixture& operator=(const AfterfireWavFixture&) = delete;

    std::string pattern(const std::string& leaf) const {
        return (dir_ / leaf).string();
    }

private:
    std::filesystem::path dir_;
};

} // namespace

// A glob against a real directory expands to every match — the behaviour the
// CLI relies on to offer a randomised pool of pop samples.
TEST(AfterfireWavPathTest, GlobExpandsToAllMatchingFiles) {
    const AfterfireWavFixture fixture;

    const auto paths = resolveAfterfireWavPaths(fixture.pattern("pop_*.wav"));

    EXPECT_EQ(paths.size(), 3u) << "pop_01/02/03 should match, other.wav should not";
}

// A glob that matches nothing yields an empty list. This is what lets the
// bridge fail fast instead of silently substituting the default IR.
TEST(AfterfireWavPathTest, NonMatchingGlobYieldsEmptyResult) {
    const AfterfireWavFixture fixture;

    EXPECT_TRUE(resolveAfterfireWavPaths(fixture.pattern("nope_*.wav")).empty());
}

// A literal path that exists resolves to exactly itself.
TEST(AfterfireWavPathTest, LiteralExistingPathResolvesToOneFile) {
    const AfterfireWavFixture fixture;

    const auto paths = resolveAfterfireWavPaths(fixture.pattern("pop_01.wav"));

    ASSERT_EQ(paths.size(), 1u);
    EXPECT_NE(paths.front().find("pop_01.wav"), std::string::npos);
}

// A literal path that does NOT exist is also an empty result, so a typo'd
// filename fails fast on the same code path as a non-matching glob.
TEST(AfterfireWavPathTest, LiteralMissingPathYieldsEmptyResult) {
    const AfterfireWavFixture fixture;

    EXPECT_TRUE(resolveAfterfireWavPaths(fixture.pattern("absent.wav")).empty());
}

// An empty request is the documented "use the engine default" sentinel and must
// stay empty WITHOUT touching the filesystem.
TEST(AfterfireWavPathTest, EmptyPathYieldsEmptyResult) {
    EXPECT_TRUE(resolveAfterfireWavPaths("").empty());
}

// A glob whose DIRECTORY does not exist must not throw — it reports no matches.
// This was the pre-fix production state (the CLI handed the bridge a path under
// a nonexistent build/ directory), so it stays pinned as a boundary case.
TEST(AfterfireWavPathTest, GlobInMissingDirectoryYieldsEmptyResultWithoutThrowing) {
    EXPECT_NO_THROW({
        EXPECT_TRUE(resolveAfterfireWavPaths("/no/such/dir/anywhere/pop_*.wav").empty());
    });
}
