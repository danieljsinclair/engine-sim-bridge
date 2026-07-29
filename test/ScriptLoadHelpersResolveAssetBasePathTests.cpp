// ScriptLoadHelpersResolveAssetBasePathTests.cpp
//
// resolveAssetBasePath (ScriptLoadHelpers.h:49-128) resolves the platform-specific
// base directory that directly contains (or leads to) "sound-library/". It is pure
// logic over std::filesystem::exists — a genuine filesystem boundary — so each test
// builds a real directory layout under a scoped temp dir and asserts the documented
// return contract for one branch. No mocks; real filesystem, real function.
//
// Branches pinned (one scenario per test, SRP):
//   - explicit override accepted: candidate contains sound-library/        -> as-is
//   - explicit override accepted: valid engine-sim root (es/+assets/)       -> <root>/es
//   - explicit override accepted: split-root parent (es/+engine-sim/)       -> <parent>/engine-sim/es
//   - explicit override rejected: real path, no valid sound base            -> upward search runs
//   - scriptPath with /assets/: root/es has sound-library                   -> <root>/es
//   - scriptPath with /assets/: root/es has no sound-library                -> root
//   - JSON preset upward search: direct root found                          -> <root>/es
//   - JSON preset upward search: split root found                           -> <parent>/engine-sim/es
//   - JSON preset upward search: iOS flat bundle (sound-library/ directly)  -> that dir
//   - JSON preset upward search: nothing found within the climb window      -> fallback to parent
//
// NOT tested: the override "looks like a path" guard (L71-72) is an internal
// detail; we exercise it through the realistic inputs above, not in isolation.

#include "simulator/ScriptLoadHelpers.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <unistd.h>

namespace {

// RAII temp dir (mirrors the established TempPresetDir convention in
// SimulatorFactoryPresetDiscoveryTests): unique per process + counter, removed
// on destruction so tests leave no debris.
class ScopedTempTree {
public:
    ScopedTempTree() {
        root_ = std::filesystem::temp_directory_path() /
                ("es_script_helpers_test_" + std::to_string(::getpid()) +
                 "_" + std::to_string(counter_++));
        std::filesystem::create_directories(root_);
    }
    ~ScopedTempTree() {
        std::error_code ec;
        std::filesystem::remove_all(root_, ec);
    }
    ScopedTempTree(const ScopedTempTree&) = delete;
    ScopedTempTree& operator=(const ScopedTempTree&) = delete;

    const std::filesystem::path& root() const { return root_; }

    // mkdir -p relative to root; returns the full path.
    std::filesystem::path makeDir(const std::string& rel) const {
        auto p = root_ / rel;
        std::filesystem::create_directories(p);
        return p;
    }

    // Create an empty placeholder file (existence is all the function checks).
    void touch(const std::string& relPath) const {
        auto p = root_ / relPath;
        std::filesystem::create_directories(p.parent_path());
        std::ofstream(p) << "";
    }

private:
    std::filesystem::path root_;
    static unsigned long long counter_;
};

unsigned long long ScopedTempTree::counter_ = 0;

}  // namespace

// --- Explicit override accepted: candidate directly contains sound-library/ ----
// Returns the candidate unchanged (the "es/ directory" case in the doc comment).
TEST(ResolveAssetBasePathTest, ExplicitOverrideWithSoundLibraryReturnedAsIs) {
    ScopedTempTree t;
    auto esDir = t.makeDir("engine-sim/es");
    t.touch("engine-sim/es/sound-library/.keep");

    const auto result = ScriptLoadHelpers::resolveAssetBasePath(
        /*scriptPath*/ t.root().string() + "/engine-sim/es/main.mr",
        /*assetBasePath*/ esDir.string());
    EXPECT_EQ(result, esDir.string());
}

// --- Explicit override accepted: valid engine-sim root (has es/ and assets/) ---
// Returns <root>/es (the macOS dev layout where es/sound-library/ lives under es/).
TEST(ResolveAssetBasePathTest, ExplicitOverrideValidEngineSimRootReturnsRootEs) {
    ScopedTempTree t;
    t.makeDir("engine-sim/es");
    t.makeDir("engine-sim/assets");

    const auto result = ScriptLoadHelpers::resolveAssetBasePath(
        /*scriptPath*/ t.root().string() + "/engine-sim/main.mr",
        /*assetBasePath*/ (t.root() / "engine-sim").string());
    EXPECT_EQ(result, (t.root() / "engine-sim" / "es").string());
}

// --- Explicit override accepted: split-root parent ----------------------------
// Candidate has es/ locally + an engine-sim/ child with es/ and assets/.
// Returns <candidate>/engine-sim/es (the split layout).
TEST(ResolveAssetBasePathTest, ExplicitOverrideSplitRootReturnsEngineSimEs) {
    ScopedTempTree t;
    // parent has its own es/ AND an engine-sim/ submodule that is a valid root
    t.makeDir("parent/es");
    t.makeDir("parent/engine-sim/es");
    t.makeDir("parent/engine-sim/assets");

    const auto result = ScriptLoadHelpers::resolveAssetBasePath(
        /*scriptPath*/ t.root().string() + "/parent/main.mr",
        /*assetBasePath*/ (t.root() / "parent").string());
    EXPECT_EQ(result, (t.root() / "parent" / "engine-sim" / "es").string());
}

// --- Explicit override rejected: real path, no valid sound base -> upward search
// The override is a path that exists but matches none of the accept cases, so the
// function falls through to the scriptPath-based upward search. With a scriptPath
// containing /assets/ and root/es holding sound-library/, the search returns root/es.
TEST(ResolveAssetBasePathTest, InvalidOverrideFallsThroughToScriptPathSearch) {
    ScopedTempTree t;
    t.makeDir("engine-sim/assets");
    t.makeDir("engine-sim/es/sound-library");   // root/es has sound-library -> search returns es
    t.makeDir("bogus_override");                // exists but is not a valid sound base

    const auto result = ScriptLoadHelpers::resolveAssetBasePath(
        /*scriptPath*/ t.root().string() + "/engine-sim/assets/main.mr",
        /*assetBasePath*/ (t.root() / "bogus_override").string());
    EXPECT_EQ(result, (t.root() / "engine-sim" / "es").string());
}

// --- scriptPath with /assets/: root/es has sound-library/ -> <root>/es --------
TEST(ResolveAssetBasePathTest, ScriptPathAssetsRootEsWithSoundLibraryReturnsEs) {
    ScopedTempTree t;
    t.makeDir("engine-sim/assets");
    t.makeDir("engine-sim/es/sound-library");

    const auto result = ScriptLoadHelpers::resolveAssetBasePath(
        /*scriptPath*/ t.root().string() + "/engine-sim/assets/main.mr",
        /*assetBasePath*/ "");
    EXPECT_EQ(result, (t.root() / "engine-sim" / "es").string());
}

// --- scriptPath with /assets/: root/es has NO sound-library/ -> root ----------
// When root/es lacks sound-library/, the /assets/ branch returns root itself.
TEST(ResolveAssetBasePathTest, ScriptPathAssetsRootEsWithoutSoundLibraryReturnsRoot) {
    ScopedTempTree t;
    t.makeDir("engine-sim/assets");
    t.makeDir("engine-sim/es");   // present but no sound-library/

    const auto result = ScriptLoadHelpers::resolveAssetBasePath(
        /*scriptPath*/ t.root().string() + "/engine-sim/assets/main.mr",
        /*assetBasePath*/ "");
    EXPECT_EQ(result, (t.root() / "engine-sim").string());
}

// --- JSON preset upward search: direct root (es/ + assets/) found -------------
// No /assets/ in scriptPath, no override: climb from the script's parent. When a
// parent directory is a valid engine-sim root, return <that>/es.
TEST(ResolveAssetBasePathTest, UpwardSearchFindsDirectRoot) {
    ScopedTempTree t;
    t.makeDir("engine-sim/es");
    t.makeDir("engine-sim/assets");
    t.makeDir("engine-sim/presets");   // script lives here, two levels under the root

    const auto result = ScriptLoadHelpers::resolveAssetBasePath(
        /*scriptPath*/ t.root().string() + "/engine-sim/presets/myengine.json",
        /*assetBasePath*/ "");
    EXPECT_EQ(result, (t.root() / "engine-sim" / "es").string());
}

// --- JSON preset upward search: split root found ------------------------------
// A parent has es/ locally + an engine-sim/ child that is a valid root.
TEST(ResolveAssetBasePathTest, UpwardSearchFindsSplitRoot) {
    ScopedTempTree t;
    t.makeDir("parent/es");
    t.makeDir("parent/engine-sim/es");
    t.makeDir("parent/engine-sim/assets");
    t.makeDir("parent/presets");

    const auto result = ScriptLoadHelpers::resolveAssetBasePath(
        /*scriptPath*/ t.root().string() + "/parent/presets/myengine.json",
        /*assetBasePath*/ "");
    EXPECT_EQ(result, (t.root() / "parent" / "engine-sim" / "es").string());
}

// --- JSON preset upward search: iOS flat bundle (sound-library/ directly) -----
// No root layout found, but a parent directory directly contains sound-library/.
// Returns that directory as-is (the iOS bundle case).
TEST(ResolveAssetBasePathTest, UpwardSearchFindsFlatBundleSoundLibrary) {
    ScopedTempTree t;
    t.makeDir("bundle/sound-library");
    t.makeDir("bundle/presets");

    const auto result = ScriptLoadHelpers::resolveAssetBasePath(
        /*scriptPath*/ t.root().string() + "/bundle/presets/myengine.json",
        /*assetBasePath*/ "");
    EXPECT_EQ(result, (t.root() / "bundle").string());
}

// --- JSON preset upward search: nothing found -> fallback to script parent ----
// No valid layout anywhere in the climb window: returns the parent of the script.
TEST(ResolveAssetBasePathTest, UpwardSearchFindsNothingFallsBackToScriptParent) {
    ScopedTempTree t;
    t.makeDir("orphan/presets");   // no es/, no assets/, no sound-library anywhere up

    const auto result = ScriptLoadHelpers::resolveAssetBasePath(
        /*scriptPath*/ t.root().string() + "/orphan/presets/myengine.json",
        /*assetBasePath*/ "");
    EXPECT_EQ(result, (t.root() / "orphan" / "presets").string());
}
