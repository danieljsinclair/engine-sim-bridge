// AssetResolverTests.cpp - Behavior tests for exe-aware asset resolution
//
// Pins the documented contract of asset_resolver: relative references resolve
// against the executable's real (symlink-resolved) directory first, then its
// parent (the repository root for in-tree builds), then the current working
// directory. Audio references additionally probe the script family's asset
// base first — never CWD alone.
//
// Spec-driven: each test pins one rule from the contract, using temp
// directories and explicit search roots so behaviour does not depend on where
// the test binary happens to be launched from.

#include "common/AssetResolver.h"
#include "simulator/ScriptLoadHelpers.h"

#include <gtest/gtest.h>

#include <algorithm>
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
            ("asset_resolver_test_" + std::to_string(::getpid()) + "_" +
             std::to_string(++counter));
        fs::create_directories(path);
    }
    ~TempDir() {
        std::error_code ec;
        fs::remove_all(path, ec);
    }
};

void writeFile(const fs::path& file) {
    fs::create_directories(file.parent_path());
    std::ofstream out(file);
    out << "x";
}

bool containsPath(const std::vector<std::string>& candidates, const std::string& path) {
    return std::find(candidates.begin(), candidates.end(), path) != candidates.end();
}

} // namespace

// (a) The executable directory is a real, canonical directory: absolute,
//     existing, and idempotent under canonicalize() (symlinks and ".."
//     components introduced by the launch path are already resolved).
TEST(AssetResolverTest, ExecutableDirectoryIsCanonicalAndExists) {
    const fs::path exeDir = asset_resolver::executableDirectory();
    ASSERT_FALSE(exeDir.empty());
    EXPECT_TRUE(exeDir.is_absolute());
    ASSERT_TRUE(fs::exists(exeDir));

    std::error_code ec;
    EXPECT_TRUE(fs::is_directory(exeDir, ec));
    EXPECT_EQ(fs::canonical(exeDir, ec), exeDir) << "exe dir must be fully resolved";
}

// (b) Search roots follow the documented order: exe dir, its parent (repo
//     root), then cwd — with duplicates collapsed.
TEST(AssetResolverTest, SearchRootsFollowExeRepoCwdOrder) {
    const fs::path exeDir = asset_resolver::executableDirectory();
    const std::vector<fs::path> roots = asset_resolver::searchRoots();

    ASSERT_GE(roots.size(), 2u);
    EXPECT_TRUE(fs::equivalent(roots[0], exeDir));
    EXPECT_TRUE(fs::equivalent(roots[1], exeDir.parent_path()));

    std::error_code ec;
    const fs::path cwd = fs::current_path(ec);
    ASSERT_FALSE(ec);
    bool hasCwd = false;
    for (const auto& root : roots) {
        if (fs::equivalent(root, cwd)) hasCwd = true;
    }
    EXPECT_TRUE(hasCwd) << "cwd must remain a search root (legacy fallback)";

    for (size_t i = 0; i < roots.size(); ++i) {
        for (size_t j = i + 1; j < roots.size(); ++j) {
            EXPECT_FALSE(fs::equivalent(roots[i], roots[j]))
                << "duplicate roots: " << roots[i] << " vs " << roots[j];
        }
    }
}

// (c) Resolution prefers the first root that carries the file.
TEST(AssetResolverTest, ResolvePrefersFirstRoot) {
    TempDir a, b;
    writeFile(a.path / "asset.txt");
    writeFile(b.path / "asset.txt");

    const fs::path resolved = asset_resolver::resolve("asset.txt", {a.path, b.path});
    EXPECT_TRUE(fs::equivalent(resolved, a.path / "asset.txt"));
}

// (d) Resolution falls through to later roots when earlier ones miss.
TEST(AssetResolverTest, ResolveFallsThroughToLaterRoot) {
    TempDir a, b;
    writeFile(b.path / "asset.txt");

    const fs::path resolved = asset_resolver::resolve("asset.txt", {a.path, b.path});
    EXPECT_TRUE(fs::equivalent(resolved, b.path / "asset.txt"));
}

// (e) Absolute references pass through untouched.
TEST(AssetResolverTest, ResolvePassesAbsolutePathThrough) {
    TempDir t;
    writeFile(t.path / "asset.txt");
    const fs::path abs = t.path / "asset.txt";

    EXPECT_EQ(asset_resolver::resolve(abs).string(), abs.string());
    const std::vector<fs::path> probes = asset_resolver::candidatesFor(abs);
    ASSERT_EQ(probes.size(), 1u);
    EXPECT_EQ(probes[0].string(), abs.string());
}

// (f) A missing reference resolves to the first probe so callers can report a
//     concrete path (legacy behaviour: cwd-joined).
TEST(AssetResolverTest, ResolveMissingReturnsFirstProbe) {
    TempDir a, b;
    const fs::path resolved = asset_resolver::resolve("missing.txt", {a.path, b.path});
    EXPECT_EQ(resolved.string(), (a.path / "missing.txt").string());
}

// (g) Roots reached through symlinks still resolve (realpath handling: the
//     probe goes through the symlinked root to the real file).
TEST(AssetResolverTest, ResolveThroughSymlinkedRoot) {
    TempDir real, links;
    writeFile(real.path / "asset.txt");
    std::error_code ec;
    fs::create_directory_symlink(real.path, links.path / "alias", ec);
    ASSERT_FALSE(ec) << "symlink creation failed: " << ec.message();

    const fs::path resolved = asset_resolver::resolve("asset.txt", {links.path / "alias"});
    EXPECT_TRUE(fs::exists(resolved));
    EXPECT_TRUE(fs::equivalent(resolved, real.path / "asset.txt"));
}

// (h) "." and ".." components in the reference are normalized before probing.
TEST(AssetResolverTest, ResolveNormalizesDotComponents) {
    TempDir t;
    fs::create_directories(t.path / "sub");
    writeFile(t.path / "asset.txt");

    const fs::path resolved = asset_resolver::resolve("sub/../asset.txt", {t.path});
    EXPECT_EQ(resolved.string(), (t.path / "asset.txt").string());
}

// (i) Audio references anchor on the script family's asset base FIRST: an
//     "es/sound-library/..." reference joined against an es_new base yields
//     the es_new library copy — never the double-prefixed
//     "<base>/es/sound-library/..." path (the double-es bug).
TEST(AudioFileCandidatesTest, AnchorsOnAssetBaseBeforeRawJoin) {
    TempDir t;
    const std::string base = (t.path / "es_new").string();
    writeFile(t.path / "es_new" / "sound-library" / "smooth" / "smooth_39.wav");

    const std::vector<std::string> candidates =
        ScriptLoadHelpers::audioFileCandidates(base, "es/sound-library/smooth/smooth_39.wav");

    ASSERT_FALSE(candidates.empty());
    EXPECT_EQ(candidates[0], base + "/sound-library/smooth/smooth_39.wav");
    // The double-prefixed "<base>/es/sound-library/..." join may appear as a
    // later fallback probe, but must never be the first candidate.
    if (containsPath(candidates, base + "/es/sound-library/smooth/smooth_39.wav")) {
        EXPECT_NE(candidates[0], base + "/es/sound-library/smooth/smooth_39.wav");
    }
}

// (i-cont.) The anchored base candidate must point at a real file for both
//     script families (es and es_new layouts).
TEST(AudioFileCandidatesTest, AnchoredCandidateExistsForBothFamilies) {
    TempDir t;
    writeFile(t.path / "es" / "sound-library" / "smooth" / "a.wav");
    writeFile(t.path / "es_new" / "sound-library" / "smooth" / "a.wav");

    for (const char* family : {"es", "es_new"}) {
        const std::string base = (t.path / family).string();
        const auto candidates =
            ScriptLoadHelpers::audioFileCandidates(base, "es/sound-library/smooth/a.wav");
        ASSERT_GE(candidates.size(), 1u);
        EXPECT_TRUE(fs::exists(candidates[0])) << candidates[0];
    }
}

// (j) Exe-aware search roots are probed after the asset base — the reference
//     as written (engine-sim-root relative, e.g. "es/sound-library/...") and
//     the anchored form.
TEST(AudioFileCandidatesTest, IncludesExeAwareRootCandidates) {
    TempDir t;
    const std::string base = (t.path / "es_new").string();

    const auto candidates =
        ScriptLoadHelpers::audioFileCandidates(base, "es/sound-library/smooth/smooth_39.wav");

    for (const auto& root : asset_resolver::searchRoots()) {
        EXPECT_TRUE(containsPath(candidates, (root / "es/sound-library/smooth/smooth_39.wav").generic_string()))
            << "missing root-relative candidate for " << root;
        EXPECT_TRUE(containsPath(candidates, (root / "sound-library/smooth/smooth_39.wav").generic_string()))
            << "missing anchored root candidate for " << root;
    }
}

// (j-cont.) Bare library-relative references (no sound-library component)
//     probe the base's sound-library/ directory.
TEST(AudioFileCandidatesTest, BareReferenceProbesSoundLibraryUnderBase) {
    TempDir t;
    const std::string base = (t.path / "es").string();

    const auto candidates =
        ScriptLoadHelpers::audioFileCandidates(base, "smooth/smooth_39.wav");

    EXPECT_TRUE(containsPath(candidates, base + "/sound-library/smooth/smooth_39.wav"));
    EXPECT_EQ(candidates[0], base + "/smooth/smooth_39.wav");
}

// (j-cont.) Absolute audio references are returned as the single candidate.
TEST(AudioFileCandidatesTest, AbsoluteReferenceIsSingleCandidate) {
    const auto candidates = ScriptLoadHelpers::audioFileCandidates(
        "/tmp/base", "/var/folders/T/sound-library/new/x.wav");
    ASSERT_EQ(candidates.size(), 1u);
    EXPECT_EQ(candidates[0], "/var/folders/T/sound-library/new/x.wav");
}

// (k) Script paths resolve exe-aware: "es/engine_sim.mr" is found relative to
//     the executable's repository (the bridge ships es/ next to build/),
//     independent of the launch directory.
TEST(NormalizeScriptPathTest, ResolvesRelativeScriptExeAware) {
    const std::string normalized = ScriptLoadHelpers::normalizeScriptPath("es/engine_sim.mr");

    const fs::path path(normalized);
    EXPECT_TRUE(path.is_absolute());
    EXPECT_EQ(path.filename(), "engine_sim.mr");

    // In-tree layout: <bridge>/build/<test-bin> with es/ shipped next to build/.
    const fs::path exeRoot = asset_resolver::executableDirectory().parent_path();
    if (fs::exists(exeRoot / "es" / "engine_sim.mr")) {
        EXPECT_TRUE(fs::equivalent(path, exeRoot / "es" / "engine_sim.mr"))
            << "script must resolve via the exe's repo root, not cwd";
    } else {
        // Out-of-tree install: falls back to the cwd-joined legacy form.
        EXPECT_EQ(normalized, fs::absolute("es/engine_sim.mr").lexically_normal().string());
    }
}

// (k-cont.) Absolute script paths pass through (lexically normalized).
TEST(NormalizeScriptPathTest, AbsoluteScriptPathPassesThrough) {
    const std::string normalized = ScriptLoadHelpers::normalizeScriptPath("/tmp/x/../es/foo.mr");
    EXPECT_EQ(normalized, "/tmp/es/foo.mr");
}

// (k-cont.) A missing relative script resolves to the FIRST probe (exe-dir
//     joined) so downstream errors name a concrete path — mirroring
//     cli::ExecutablePath::resolveResource's best-effort fallback.
TEST(NormalizeScriptPathTest, MissingScriptResolvesToFirstProbe) {
    const std::string normalized =
        ScriptLoadHelpers::normalizeScriptPath("no_such_script_zz.mr");
    const fs::path firstProbe =
        asset_resolver::executableDirectory() / "no_such_script_zz.mr";
    EXPECT_EQ(normalized, firstProbe.lexically_normal().string());
}
