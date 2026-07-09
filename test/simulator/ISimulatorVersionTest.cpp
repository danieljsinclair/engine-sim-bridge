// ISimulatorVersionTest.cpp - Contract test for ISimulator::getVersion()
//
// ISimulator.cpp is a 9-LOC dead-stripped file (0% coverage). The only public
// surface is the static getVersion() that returns the engine-sim build version
// string. These tests assert the OBSERVABLE contract: a well-formed, non-empty
// semantic version string. This is blind to the internal implementation
// (EngineSimGetVersion returns "1.0.0") — we assert the *form* of the result,
// not a hardcoded copy of the literal.

#include "simulator/ISimulator.h"

#include <gtest/gtest.h>

#include <cstring>
#include <string>

namespace {

// getVersion() returns a non-empty, null-terminated string.
TEST(ISimulatorVersionTest, GetVersionReturnsNonNullNonEmpty) {
    const char* version = ISimulator::getVersion();
    ASSERT_NE(version, nullptr);
    EXPECT_GT(std::strlen(version), 0u);
}

// getVersion() returns a dotted-version string of the form major[.minor[.patch]]
// (e.g. "1.0.0" or "1"). We assert the well-formedness contract, not the exact
// value, so the test stays green when the version is bumped.
TEST(ISimulatorVersionTest, GetVersionHasDottedNumericForm) {
    const std::string version = ISimulator::getVersion();
    ASSERT_FALSE(version.empty());

    // Every character must be a digit or a dot, and there must be at least one
    // digit so the string is a genuine version, not just separators.
    bool hasDigit = false;
    for (char c : version) {
        bool isDigit = c >= '0' && c <= '9';
        bool isDot = c == '.';
        EXPECT_TRUE(isDigit || isDot) << "Unexpected character in version: " << c;
        if (isDigit) hasDigit = true;
    }
    EXPECT_TRUE(hasDigit) << "Version string must contain at least one digit";
}

}  // namespace
