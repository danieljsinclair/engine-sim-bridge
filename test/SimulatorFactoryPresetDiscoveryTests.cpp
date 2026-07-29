// SimulatorFactoryPresetDiscoveryTests.cpp
//
// Behavior tests for SimulatorFactory::discoverPresets / discoverPresetPaths.
//
// These functions sit on a genuine filesystem boundary: they scan a directory
// for .json preset files. We exercise them against a self-contained temp dir
// (std::filesystem::temp_directory_path) populated with tiny hand-written stubs
// — no reliance on repo fixtures. Each test pins one rule of the documented
// contract:
//   - .json files are discovered, sorted ascending by full path;
//   - non-.json files are ignored;
//   - a nonexistent directory yields an empty result;
//   - discoverPresetPaths scans a directory AND resolves currentIndex to the
//     position of a separate currentFullPath when it is among the results
//     (default otherwise). The dir and the file are distinct arguments —
//     passing a file to the scan would be a caller bug, not this API's concern.

#include "simulator/SimulatorFactory.h"

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

namespace {

// RAII helper: creates a uniquely-named temp dir for the test and removes it on
// destruction so tests never leave debris behind.
class TempPresetDir {
public:
    TempPresetDir() {
        auto base = std::filesystem::temp_directory_path();
        // mkdtemp-style uniqueness via pid + a counter would do, but
        // temp_directory_path + a random-ish stem is portable and sufficient.
        template_ = base / ("es_bridge_test_" + std::to_string(::getpid()) +
                            "_" + std::to_string(counter_++));
        std::filesystem::create_directories(template_);
    }
    ~TempPresetDir() {
        std::error_code ec;
        std::filesystem::remove_all(template_, ec);
    }
    TempPresetDir(const TempPresetDir&) = delete;
    TempPresetDir& operator=(const TempPresetDir&) = delete;

    const std::filesystem::path& path() const { return template_; }

    // Create an empty (but valid-placeholder) file with the given leaf name.
    void writeStub(const std::string& leafName) const {
        std::ofstream(template_ / leafName) << "{}";
    }

private:
    std::filesystem::path template_;
    static unsigned long long counter_;
};

unsigned long long TempPresetDir::counter_ = 0;

}  // namespace

// discoverPresets returns the .json files, sorted ascending by full path.
TEST(SimulatorFactoryPresetDiscoveryTest, DiscoversJsonFilesSorted) {
    TempPresetDir dir;
    dir.writeStub("zebra.json");
    dir.writeStub("alpha.json");
    dir.writeStub("mike.json");

    const auto presets = SimulatorFactory::discoverPresets(dir.path().string());

    ASSERT_EQ(presets.size(), 3u);
    // Sorted: alpha < mike < zebra
    EXPECT_EQ(std::filesystem::path(presets[0]).filename(), "alpha.json");
    EXPECT_EQ(std::filesystem::path(presets[1]).filename(), "mike.json");
    EXPECT_EQ(std::filesystem::path(presets[2]).filename(), "zebra.json");
}

// Non-.json entries (other extensions, subdirectories) are ignored.
TEST(SimulatorFactoryPresetDiscoveryTest, IgnoresNonJsonEntries) {
    TempPresetDir dir;
    dir.writeStub("keep.json");
    dir.writeStub("skip.txt");
    dir.writeStub("skip.md");
    dir.writeStub("no_ext");

    // A nested directory, even if named *.json, must not be reported.
    std::filesystem::create_directories(dir.path() / "subdir.json");

    const auto presets = SimulatorFactory::discoverPresets(dir.path().string());

    ASSERT_EQ(presets.size(), 1u);
    EXPECT_EQ(std::filesystem::path(presets[0]).filename(), "keep.json");
}

// A nonexistent directory yields an empty result (no throw).
TEST(SimulatorFactoryPresetDiscoveryTest, NonexistentDirYieldsEmpty) {
    const auto nonexistent = std::filesystem::temp_directory_path() /
        ("es_bridge_nope_" + std::to_string(::getpid()));
    ASSERT_FALSE(std::filesystem::exists(nonexistent));
    EXPECT_TRUE(SimulatorFactory::discoverPresets(nonexistent.string()).empty());
}

// An empty (but existing) directory yields an empty result.
TEST(SimulatorFactoryPresetDiscoveryTest, EmptyDirYieldsEmpty) {
    TempPresetDir dir;
    EXPECT_TRUE(SimulatorFactory::discoverPresets(dir.path().string()).empty());
}

// discoverPresetPaths reports every preset (shortName + fullPath) for the
// directory passed in. This is the shape the sole production caller uses
// (CLIMain passes a preset *directory* and an empty currentFullPath).
TEST(SimulatorFactoryPresetDiscoveryTest, ReportsEveryPresetShortNameAndFullPath) {
    TempPresetDir dir;
    dir.writeStub("alpha.json");
    dir.writeStub("beta.json");
    dir.writeStub("gamma.json");

    const auto result = SimulatorFactory::discoverPresetPaths(dir.path().string(), /*currentFullPath=*/{});

    ASSERT_EQ(result.presets.size(), 3u);
    EXPECT_EQ(result.presets[0].shortName, "alpha");
    EXPECT_EQ(result.presets[1].shortName, "beta");
    EXPECT_EQ(result.presets[2].shortName, "gamma");
    EXPECT_EQ(result.presets[0].fullPath, (dir.path() / "alpha.json").string());
    EXPECT_EQ(result.presets[2].fullPath, (dir.path() / "gamma.json").string());
}

// When no current selection is given (empty currentFullPath — the production
// call shape), no fullPath matches and currentIndex remains at its default (0).
TEST(SimulatorFactoryPresetDiscoveryTest, CurrentIndexDefaultWhenNoCurrentSelection) {
    TempPresetDir dir;
    dir.writeStub("alpha.json");
    dir.writeStub("beta.json");

    const auto result = SimulatorFactory::discoverPresetPaths(dir.path().string(), /*currentFullPath=*/{});

    ASSERT_EQ(result.presets.size(), 2u);
    EXPECT_EQ(result.currentIndex, 0u);
}

// currentIndex resolves to the position of currentFullPath among the discovered
// presets. This is the behavior the split signature now makes possible: the
// directory is scanned, and a SEPARATE full file path is matched. (Under the
// old single-arg API this could never resolve — the arg was the directory, and
// passing the file path instead threw.)
TEST(SimulatorFactoryPresetDiscoveryTest, CurrentIndexResolvesToCurrentFullPath) {
    TempPresetDir dir;
    dir.writeStub("alpha.json");
    dir.writeStub("beta.json");
    dir.writeStub("gamma.json");

    // beta.json sorts to index 1 (alpha=0, beta=1, gamma=2).
    const auto betaPath = (dir.path() / "beta.json").string();
    const auto result = SimulatorFactory::discoverPresetPaths(dir.path().string(), betaPath);

    ASSERT_EQ(result.presets.size(), 3u);
    EXPECT_EQ(result.currentIndex, 1u);
}

// A currentFullPath that is not among the discovered presets (different
// directory, typo, etc.) leaves currentIndex at its default rather than
// matching anything spurious.
TEST(SimulatorFactoryPresetDiscoveryTest, CurrentIndexDefaultWhenFullPathNotAmongResults) {
    TempPresetDir dir;
    dir.writeStub("alpha.json");
    dir.writeStub("beta.json");

    const auto result = SimulatorFactory::discoverPresetPaths(dir.path().string(), "/nonexistent/delta.json");

    ASSERT_EQ(result.presets.size(), 2u);
    EXPECT_EQ(result.currentIndex, 0u);
}
