// JsonParserTests.cpp - Behavior tests for the hand-rolled JSON parser
//
// JsonParser is a recursive-descent parser with zero direct unit tests, yet
// every preset load runs through it. These tests pin its core contract:
//   - each JSON value type parses with the correct type and value;
//   - numbers (int, double, negative, exponent) parse to the right magnitude;
//   - nested object/array structures are reconstructed faithfully;
//   - leading whitespace is tolerated;
//   - malformed input (unterminated string, bad bool/null literal, missing
//     closing delimiters) is rejected with PresetDeserializationException.
//
// We assert the exception TYPE (intent) and the structural outcome, never the
// word-for-word message. No mocks: the real `json::parse` is exercised.

#include "common/JsonParser.h"
#include "common/PresetExceptions.h"

#include <gtest/gtest.h>

#include <string>

using json::JsonValue;
using json::parse;

// --- Value types: each parses to the correct type and value -------------------

TEST(JsonParserTest, ParsesEmptyObject) {
    auto v = parse("{}");
    ASSERT_TRUE(v.isObject());
    EXPECT_EQ(v.size(), 0u);
}

TEST(JsonParserTest, ParsesEmptyArray) {
    auto v = parse("[]");
    ASSERT_TRUE(v.isArray());
    EXPECT_EQ(v.size(), 0u);
}

TEST(JsonParserTest, ParsesString) {
    auto v = parse("\"hello\"");
    ASSERT_TRUE(v.isString());
    EXPECT_EQ(v.asString(), "hello");
}

TEST(JsonParserTest, ParsesTrueBoolean) {
    auto v = parse("true");
    ASSERT_TRUE(v.isBool());
    EXPECT_TRUE(v.asBool());
}

TEST(JsonParserTest, ParsesFalseBoolean) {
    auto v = parse("false");
    ASSERT_TRUE(v.isBool());
    EXPECT_FALSE(v.asBool());
}

TEST(JsonParserTest, ParsesNull) {
    auto v = parse("null");
    EXPECT_TRUE(v.isNull());
}

// --- Numbers: integer, fractional, negative, exponent ------------------------

TEST(JsonParserTest, ParsesInteger) {
    auto v = parse("42");
    ASSERT_TRUE(v.isNumber());
    EXPECT_DOUBLE_EQ(v.asNumber(), 42.0);
    EXPECT_EQ(v.asInt(), 42);
}

TEST(JsonParserTest, ParsesDouble) {
    auto v = parse("3.14159");
    ASSERT_TRUE(v.isNumber());
    EXPECT_DOUBLE_EQ(v.asNumber(), 3.14159);
}

TEST(JsonParserTest, ParsesNegativeNumber) {
    auto v = parse("-17.5");
    ASSERT_TRUE(v.isNumber());
    EXPECT_DOUBLE_EQ(v.asNumber(), -17.5);
}

TEST(JsonParserTest, ParsesExponentNumber) {
    auto v = parse("2.5e-6");
    ASSERT_TRUE(v.isNumber());
    EXPECT_DOUBLE_EQ(v.asNumber(), 2.5e-6);
}

// --- Structure: object keys, array elements, nesting -------------------------

TEST(JsonParserTest, ParsesObjectWithStringAndNumberFields) {
    auto v = parse(R"({"name": "v8", "cylinders": 8})");
    ASSERT_TRUE(v.isObject());
    ASSERT_TRUE(v["name"].isString());
    EXPECT_EQ(v["name"].asString(), "v8");
    ASSERT_TRUE(v["cylinders"].isNumber());
    EXPECT_EQ(v["cylinders"].asInt(), 8);
}

TEST(JsonParserTest, ParsesArrayWithMixedElementTypes) {
    auto v = parse("[1, \"two\", true, null]");
    ASSERT_TRUE(v.isArray());
    ASSERT_EQ(v.size(), 4u);
    const auto& arr = v.asArray();
    EXPECT_EQ(arr[0].asInt(), 1);
    EXPECT_EQ(arr[1].asString(), "two");
    EXPECT_TRUE(arr[2].asBool());
    EXPECT_TRUE(arr[3].isNull());
}

TEST(JsonParserTest, ParsesNestedObjectInsideArray) {
    auto v = parse(R"([{"a": 1}, {"b": 2}])");
    ASSERT_TRUE(v.isArray());
    ASSERT_EQ(v.size(), 2u);
    const auto& arr = v.asArray();
    EXPECT_EQ(arr[0]["a"].asInt(), 1);
    EXPECT_EQ(arr[1]["b"].asInt(), 2);
}

TEST(JsonParserTest, ParsesDeeplyNestedStructure) {
    // Object containing array containing object — the shape preset files use.
    auto v = parse(R"({"engines": [{"name": "i4", "config": {"cylinders": 4}}]})");
    ASSERT_TRUE(v.isObject());
    const auto& engines = v["engines"];
    ASSERT_TRUE(engines.isArray());
    ASSERT_EQ(engines.size(), 1u);
    const auto& arr = engines.asArray();
    EXPECT_EQ(arr[0]["name"].asString(), "i4");
    EXPECT_EQ(arr[0]["config"]["cylinders"].asInt(), 4);
}

TEST(JsonParserTest, HasReportsPresenceAndAbsenceOfKey) {
    auto v = parse(R"({"present": 1})");
    EXPECT_TRUE(v.has("present"));
    EXPECT_FALSE(v.has("absent"));
}

// --- Whitespace tolerance ----------------------------------------------------

TEST(JsonParserTest, LeadingAndTrailingWhitespaceTolerated) {
    auto v = parse("   42   \n");
    ASSERT_TRUE(v.isNumber());
    EXPECT_DOUBLE_EQ(v.asNumber(), 42.0);
}

// --- String escapes (subset the parser documents) ----------------------------

TEST(JsonParserTest, ParsesCommonStringEscapes) {
    auto v = parse("\"line1\\nline2\\ttab\"");
    ASSERT_TRUE(v.isString());
    EXPECT_EQ(v.asString(), "line1\nline2\ttab");
}

// --- Malformed input: rejected with PresetDeserializationException -----------
// Intent assertions only — we check the type, not the message text.

TEST(JsonParserTest, UnterminatedStringThrows) {
    EXPECT_THROW(parse("\"unterminated"), PresetDeserializationException);
}

TEST(JsonParserTest, InvalidBooleanLiteralThrows) {
    // "tru" is not a complete boolean literal.
    EXPECT_THROW(parse("tru"), PresetDeserializationException);
}

TEST(JsonParserTest, InvalidNullLiteralThrows) {
    // "nul" is not a complete null literal.
    EXPECT_THROW(parse("nul"), PresetDeserializationException);
}

TEST(JsonParserTest, UnterminatedObjectThrows) {
    EXPECT_THROW(parse("{\"a\": 1"), PresetDeserializationException);
}

TEST(JsonParserTest, UnterminatedArrayThrows) {
    EXPECT_THROW(parse("[1, 2, 3"), PresetDeserializationException);
}

TEST(JsonParserTest, UnexpectedLeadingCharacterThrows) {
    // 'x' is not a valid start of any JSON value.
    EXPECT_THROW(parse("xyz"), PresetDeserializationException);
}

TEST(JsonParserTest, EmptyInputThrows) {
    EXPECT_THROW(parse(""), PresetDeserializationException);
}

TEST(JsonParserTest, ExceptionIsARuntimeErrorForBackwardsCompat) {
    // The header documents std::runtime_error; PresetDeserializationException
    // derives from it. Callers catching std::runtime_error must still work.
    try {
        parse("tru");
        FAIL() << "expected throw";
    } catch (const std::runtime_error&) {
        SUCCEED();
    } catch (...) {
        FAIL() << "threw something other than std::runtime_error";
    }
}
