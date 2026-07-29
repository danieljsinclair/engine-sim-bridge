// PathNormalizerTests.cpp - Behavior tests for impulse-response path normalization
//
// Pins the documented contract of PathNormalizer::normalizeImpulseResponsePath:
// the function converts Piranha's CWD-relative impulse-response paths (e.g.
// "../../es/sound-library/X.wav") into a portable, anchor-relative form
// ("sound-library/X.wav") so that deserialization and serialization produce
// identical filenames regardless of where the engine was loaded from.
//
// Spec-driven: each test pins one rule from the contract. No assertions on
// internal string-walking mechanics.

#include "common/PathNormalizer.h"

#include <gtest/gtest.h>

#include <string>

// (a) Relative path: ".." / "." components are collapsed and a leading "es/"
//     prefix is stripped, yielding the portable "sound-library/X" form.
TEST(PathNormalizerTest, ResolvesRelativeDotsAndStripsEsPrefix) {
    // The canonical Piranha shape: two dirs up, into es/, into sound-library.
    const std::string input = "../../es/sound-library/ir_v8.wav";
    EXPECT_EQ(PathNormalizer::normalizeImpulseResponsePath(input),
              "sound-library/ir_v8.wav");
}

// (a-cont.) A single ".." component is removed the same way.
TEST(PathNormalizerTest, StripsSingleParentReference) {
    EXPECT_EQ(PathNormalizer::normalizeImpulseResponsePath("../sound-library/a.wav"),
              "sound-library/a.wav");
}

// (a-cont.) A "." (current-dir) component is dropped, not collapsed into the name.
TEST(PathNormalizerTest, DropsCurrentDirComponent) {
    EXPECT_EQ(PathNormalizer::normalizeImpulseResponsePath("./sound-library/a.wav"),
              "sound-library/a.wav");
}

// (b) Absolute path: everything up to and including the "sound-library" anchor
//     is discarded; the result is portable and relative from that anchor.
TEST(PathNormalizerTest, AbsolutePathKeepsSoundLibraryOnward) {
    const std::string input = "/Users/someone/assets/es/sound-library/ir.wav";
    EXPECT_EQ(PathNormalizer::normalizeImpulseResponsePath(input),
              "sound-library/ir.wav");
}

// (c) Absolute path with NO "sound-library" anchor: input is returned unchanged
//     so callers can detect that no portable form exists.
TEST(PathNormalizerTest, AbsolutePathWithoutAnchorReturnedUnchanged) {
    const std::string input = "/opt/assets/audio/sample.wav";
    EXPECT_EQ(PathNormalizer::normalizeImpulseResponsePath(input), input);
}

// (d) Empty input -> empty output (no anchor to synthesize).
TEST(PathNormalizerTest, EmptyInputYieldsEmptyOutput) {
    EXPECT_EQ(PathNormalizer::normalizeImpulseResponsePath(""), "");
}

// (e) Already-portable input passes through verbatim.
TEST(PathNormalizerTest, AlreadyPortableInputPassesThrough) {
    const std::string input = "sound-library/ir_v8.wav";
    EXPECT_EQ(PathNormalizer::normalizeImpulseResponsePath(input), input);
}
