// LiveTelemetryParserTest.cpp - Tests for vehicle-sim JSON telemetry parser

#include <gtest/gtest.h>
#include "io/LiveTelemetryParser.h"

using namespace input;

// MARK: - Happy Path Tests

TEST(LiveTelemetryParserTest, ValidFullSignal)
{
    UpstreamSignal signal = LiveTelemetryParser::parse(
        "{\"throttle\":45.2,\"speed\":80.5,\"acceleration\":0.3,\"brake\":10.0}");

    EXPECT_TRUE(signal.isValid);
    EXPECT_DOUBLE_EQ(signal.throttleFraction, 0.452);
    EXPECT_DOUBLE_EQ(signal.speedKmh, 80.5);
    EXPECT_DOUBLE_EQ(signal.accelerationG, 0.3);
    EXPECT_DOUBLE_EQ(signal.brakeFraction, 0.1);
}

TEST(LiveTelemetryParserTest, ExplicitTimestamp)
{
    UpstreamSignal signal = LiveTelemetryParser::parse(
        "{\"throttle\":50.0,\"speed\":100.0}", 1715260800000ULL);

    EXPECT_TRUE(signal.isValid);
    EXPECT_EQ(signal.timestampUtcMs, 1715260800000ULL);
    EXPECT_DOUBLE_EQ(signal.throttleFraction, 0.5);
    EXPECT_DOUBLE_EQ(signal.speedKmh, 100.0);
}

TEST(LiveTelemetryParserTest, PartialFields_MissingBrakeAndAccel)
{
    UpstreamSignal signal = LiveTelemetryParser::parse(
        "{\"throttle\":30.0,\"speed\":60.0}");

    EXPECT_TRUE(signal.isValid);
    EXPECT_DOUBLE_EQ(signal.throttleFraction, 0.3);
    EXPECT_DOUBLE_EQ(signal.speedKmh, 60.0);
    // Missing fields default to 0.0
    EXPECT_DOUBLE_EQ(signal.accelerationG, 0.0);
    EXPECT_DOUBLE_EQ(signal.brakeFraction, 0.0);
}

TEST(LiveTelemetryParserTest, OnlyThrottle)
{
    UpstreamSignal signal = LiveTelemetryParser::parse("{\"throttle\":75.0}");

    EXPECT_TRUE(signal.isValid);
    EXPECT_DOUBLE_EQ(signal.throttleFraction, 0.75);
    EXPECT_DOUBLE_EQ(signal.speedKmh, 0.0);
}

TEST(LiveTelemetryParserTest, ZeroValues)
{
    UpstreamSignal signal = LiveTelemetryParser::parse(
        "{\"throttle\":0.0,\"speed\":0.0,\"acceleration\":0.0,\"brake\":0.0}");

    EXPECT_TRUE(signal.isValid);
    EXPECT_DOUBLE_EQ(signal.throttleFraction, 0.0);
    EXPECT_DOUBLE_EQ(signal.speedKmh, 0.0);
    EXPECT_DOUBLE_EQ(signal.accelerationG, 0.0);
    EXPECT_DOUBLE_EQ(signal.brakeFraction, 0.0);
}

// MARK: - Clamping Tests

TEST(LiveTelemetryParserTest, ThrottleClampedAbove100)
{
    UpstreamSignal signal = LiveTelemetryParser::parse("{\"throttle\":150.0}");
    EXPECT_TRUE(signal.isValid);
    EXPECT_DOUBLE_EQ(signal.throttleFraction, 1.0); // clamped to 1.0
}

TEST(LiveTelemetryParserTest, ThrottleClampedBelow0)
{
    UpstreamSignal signal = LiveTelemetryParser::parse("{\"throttle\":-10.0}");
    EXPECT_TRUE(signal.isValid);
    EXPECT_DOUBLE_EQ(signal.throttleFraction, 0.0); // clamped to 0.0
}

TEST(LiveTelemetryParserTest, BrakeClampedAbove100)
{
    UpstreamSignal signal = LiveTelemetryParser::parse("{\"brake\":200.0}");
    EXPECT_TRUE(signal.isValid);
    EXPECT_DOUBLE_EQ(signal.brakeFraction, 1.0);
}

TEST(LiveTelemetryParserTest, SpeedNegativeClampedToZero)
{
    UpstreamSignal signal = LiveTelemetryParser::parse("{\"speed\":-50.0}");
    EXPECT_TRUE(signal.isValid);
    EXPECT_DOUBLE_EQ(signal.speedKmh, 0.0); // negative clamped to 0
}

// MARK: - Edge Case: Decimal Precision

TEST(LiveTelemetryParserTest, DecimalPrecision)
{
    UpstreamSignal signal = LiveTelemetryParser::parse(
        "{\"throttle\":33.333,\"speed\":120.456}");

    EXPECT_TRUE(signal.isValid);
    EXPECT_NEAR(signal.throttleFraction, 0.33333, 1e-5);
    EXPECT_NEAR(signal.speedKmh, 120.456, 1e-5);
}

// MARK: - Tolerance: Example-based test

TEST(LiveTelemetryParserTest, ExampleBasedTolerance)
{
    // This is functionally the same as the full-signal test; retained as an
    // explicit example that pins the conversion contract for new contributors.
    UpstreamSignal signal = LiveTelemetryParser::parse(
        "{\"throttle\":45.2,\"speed\":80.5,\"acceleration\":0.3,\"brake\":10.0}");

    EXPECT_TRUE(signal.isValid);
    EXPECT_NEAR(signal.throttleFraction, 0.452, 0.001);
    EXPECT_NEAR(signal.speedKmh, 80.5, 0.001);
}

// MARK: - Robustness Tests

TEST(LiveTelemetryParserTest, EmptyObject_ReturnsValidZeroed)
{
    UpstreamSignal signal = LiveTelemetryParser::parse("{}");

    EXPECT_TRUE(signal.isValid);
    EXPECT_DOUBLE_EQ(signal.throttleFraction, 0.0);
    EXPECT_DOUBLE_EQ(signal.speedKmh, 0.0);
    EXPECT_DOUBLE_EQ(signal.accelerationG, 0.0);
    EXPECT_DOUBLE_EQ(signal.brakeFraction, 0.0);
}

TEST(LiveTelemetryParserTest, IgnoresUnknownFields)
{
    UpstreamSignal signal = LiveTelemetryParser::parse(
        "{\"throttle\":40.0,\"rpm\":3000,\"load\":0.8,\"speed\":90.0}");

    EXPECT_TRUE(signal.isValid);
    EXPECT_DOUBLE_EQ(signal.throttleFraction, 0.4);
    EXPECT_DOUBLE_EQ(signal.speedKmh, 90.0);
}

TEST(LiveTelemetryParserTest, NegativeSpeedValues)
{
    UpstreamSignal signal = LiveTelemetryParser::parse(
        "{\"acceleration\":-1.5,\"throttle\":20.0}");

    EXPECT_TRUE(signal.isValid);
    EXPECT_DOUBLE_EQ(signal.accelerationG, -1.5); // acceleration CAN be negative
    EXPECT_DOUBLE_EQ(signal.throttleFraction, 0.2);
}

TEST(LiveTelemetryParserTest, ScientificNotationValues)
{
    UpstreamSignal signal = LiveTelemetryParser::parse(
        "{\"throttle\":1.0,\"acceleration\":1.5e-1}");

    EXPECT_TRUE(signal.isValid);
    EXPECT_NEAR(signal.accelerationG, 0.15, 1e-5);
}

// MARK: - Format Variation Tests

TEST(LiveTelemetryParserTest, NoSpaces)
{
    UpstreamSignal signal = LiveTelemetryParser::parse(
        "{\"throttle\":60.0,\"speed\":120.0}");

    EXPECT_TRUE(signal.isValid);
    EXPECT_DOUBLE_EQ(signal.throttleFraction, 0.6);
    EXPECT_DOUBLE_EQ(signal.speedKmh, 120.0);
}

TEST(LiveTelemetryParserTest, ExtraWhitespace)
{
    UpstreamSignal signal = LiveTelemetryParser::parse(
        "{  \"throttle\" :  60.0 ,  \"speed\"  :  120.0  }");

    EXPECT_TRUE(signal.isValid);
    EXPECT_DOUBLE_EQ(signal.throttleFraction, 0.6);
    EXPECT_DOUBLE_EQ(signal.speedKmh, 120.0);
}

TEST(LiveTelemetryParserTest, FieldsInDifferentOrder)
{
    UpstreamSignal signal = LiveTelemetryParser::parse(
        "{\"speed\":100.0,\"throttle\":50.0,\"brake\":5.0}");

    EXPECT_TRUE(signal.isValid);
    EXPECT_DOUBLE_EQ(signal.throttleFraction, 0.5);
    EXPECT_DOUBLE_EQ(signal.speedKmh, 100.0);
    EXPECT_DOUBLE_EQ(signal.brakeFraction, 0.05);
}

// MARK: - parseNumber / extractDoubleRaw error paths via public parse()

// Exponent parsing (covers positive, negative, E/e variants)
TEST(LiveTelemetryParserTest, Parse_ScientificNotation_SpeedAndThrottle)
{
    UpstreamSignal signal = LiveTelemetryParser::parse(
        "{\"throttle\":45.2,\"speed\":1.5e3}");
    EXPECT_TRUE(signal.isValid);
    EXPECT_DOUBLE_EQ(signal.throttleFraction, 0.452);
    EXPECT_DOUBLE_EQ(signal.speedKmh, 1500.0);
}

TEST(LiveTelemetryParserTest, Parse_ScientificNotation_NegativeExponent)
{
    UpstreamSignal signal = LiveTelemetryParser::parse(
        "{\"speed\":2.5e-1}");
    EXPECT_TRUE(signal.isValid);
    EXPECT_DOUBLE_EQ(signal.speedKmh, 0.25);
}

// Malformed numbers: parse() treats invalid values as 0.0 via extractDouble's default
TEST(LiveTelemetryParserTest, Parse_NaNField_DefaultsToZero)
{
    UpstreamSignal signal = LiveTelemetryParser::parse(
        "{\"throttle\":NaN}");
    EXPECT_TRUE(signal.isValid);
    EXPECT_DOUBLE_EQ(signal.throttleFraction, 0.0);
}

TEST(LiveTelemetryParserTest, Parse_InfinityField_DefaultsToZero)
{
    UpstreamSignal signal = LiveTelemetryParser::parse(
        "{\"speed\":Infinity}");
    EXPECT_TRUE(signal.isValid);
    EXPECT_DOUBLE_EQ(signal.speedKmh, 0.0);
}

TEST(LiveTelemetryParserTest, Parse_ExponentWithoutDigits_DefaultsToZero)
{
    UpstreamSignal signal = LiveTelemetryParser::parse(
        "{\"throttle\":1.0e}");
    EXPECT_TRUE(signal.isValid);
    EXPECT_DOUBLE_EQ(signal.throttleFraction, 0.0);
}
