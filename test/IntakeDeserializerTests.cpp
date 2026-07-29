// IntakeDeserializerTests.cpp - Dual-key alias contract for intake presets
//
// IntakeDeserializer accepts two historical key spellings for the flow constants:
//   - idle circuit: "idleFlowK" (preferred) or legacy "idleFlowRate"
//   - input flow:   "inputFlowK" (preferred) or legacy "intakeFlowRate"
//
// These tests pin the documented dual-key alias contract against the REAL
// Intake production class (standalone-constructible, no engine context needed):
//   - the preferred key resolves to the intake's idle/input flow constant;
//   - when only the legacy alias is present, it resolves identically;
//   - when BOTH preferred + alias appear, the preferred key wins (alias ignored);
//   - when neither key of a pair is present, deserialization throws
//     PresetDeserializationException.
//
// We assert the exception TYPE (intent), never the word-for-word message.

#include "preset/IntakeDeserializer.h"

#include "intake.h"  // full Intake definition (header forward-declares it)

#include "common/JsonParser.h"
#include "common/PresetExceptions.h"

#include <gtest/gtest.h>

#include <string>

namespace {

// A complete intake JSON object with the two aliasable keys omitted, so each
// test can inject exactly the pair it wants to exercise. All non-alias fields
// required by IntakeDeserializer are present so the only failure mode under
// test is the alias contract.
const char* kBaseIntakeJson = R"({
    "plenumCrossSectionArea": 0.01,
    "runnerFlowRate": 0.005,
    "runnerLength": 0.1,
    "velocityDecay": 0.4,
    "idleThrottlePlatePosition": 0.975,
    "plenumVolume": 0.002,
    "molecularAfr": 12.5
})";

// Inject a JSON snippet of key:value pairs into the base object (after the
// opening brace) to build a tailored intake document per test.
std::string buildIntakeJson(const std::string& pairs) {
    std::string base = kBaseIntakeJson;
    // Insert just after the opening brace.
    auto pos = base.find('{');
    return base.substr(0, pos + 1) + pairs + "," + base.substr(pos + 1);
}

// Deserialize the given JSON into a fresh standalone Intake and return it.
Intake deserializeFresh(const std::string& json) {
    Intake intake;
    json::JsonValue root = json::parse(json);
    IntakeDeserializer::deserialize(root, &intake);
    return intake;
}

constexpr double kPreferredIdle = 2.5e-6;
constexpr double kLegacyIdle = 9.9e-6;    // deliberately distinct from preferred
constexpr double kPreferredInput = 3.0e-6;
constexpr double kLegacyInput = 8.8e-6;   // deliberately distinct from preferred

}  // namespace

// Preferred idle key resolves to the intake's idle flow constant.
TEST(IntakeDeserializerTest, PreferredIdleKeyResolves) {
    auto intake = deserializeFresh(
        buildIntakeJson("\"idleFlowK\": 2.5e-6, \"inputFlowK\": 3.0e-6"));
    EXPECT_DOUBLE_EQ(intake.getIdleFlowK(), kPreferredIdle);
    EXPECT_DOUBLE_EQ(intake.getInputFlowK(), kPreferredInput);
}

// Legacy idle alias ("idleFlowRate") resolves when the preferred key is absent.
TEST(IntakeDeserializerTest, LegacyIdleAliasResolvesWhenPreferredAbsent) {
    auto intake = deserializeFresh(
        buildIntakeJson("\"idleFlowRate\": 9.9e-6, \"inputFlowK\": 3.0e-6"));
    EXPECT_DOUBLE_EQ(intake.getIdleFlowK(), kLegacyIdle);
}

// Legacy input alias ("intakeFlowRate") resolves when the preferred key is absent.
TEST(IntakeDeserializerTest, LegacyInputAliasResolvesWhenPreferredAbsent) {
    auto intake = deserializeFresh(
        buildIntakeJson("\"idleFlowK\": 2.5e-6, \"intakeFlowRate\": 8.8e-6"));
    EXPECT_DOUBLE_EQ(intake.getInputFlowK(), kLegacyInput);
}

// When BOTH preferred + legacy idle keys appear, the preferred wins and the
// legacy alias is ignored.
TEST(IntakeDeserializerTest, PreferredIdleWinsOverLegacyAlias) {
    auto intake = deserializeFresh(
        buildIntakeJson("\"idleFlowK\": 2.5e-6, \"idleFlowRate\": 9.9e-6, \"inputFlowK\": 3.0e-6"));
    EXPECT_DOUBLE_EQ(intake.getIdleFlowK(), kPreferredIdle);
}

// When BOTH preferred + legacy input keys appear, the preferred wins and the
// legacy alias is ignored.
TEST(IntakeDeserializerTest, PreferredInputWinsOverLegacyAlias) {
    auto intake = deserializeFresh(
        buildIntakeJson("\"idleFlowK\": 2.5e-6, \"inputFlowK\": 3.0e-6, \"intakeFlowRate\": 8.8e-6"));
    EXPECT_DOUBLE_EQ(intake.getInputFlowK(), kPreferredInput);
}

// Neither idle key present -> throws PresetDeserializationException.
TEST(IntakeDeserializerTest, MissingIdleKeyPairThrows) {
    EXPECT_THROW(
        deserializeFresh(buildIntakeJson("\"inputFlowK\": 3.0e-6")),
        PresetDeserializationException);
}

// Neither input key present -> throws PresetDeserializationException.
TEST(IntakeDeserializerTest, MissingInputKeyPairThrows) {
    EXPECT_THROW(
        deserializeFresh(buildIntakeJson("\"idleFlowK\": 2.5e-6")),
        PresetDeserializationException);
}
