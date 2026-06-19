// CsvTelemetryParserTest.cpp
//
// Tests for the shared CSV parser used by both ReplayTelemetryProvider and
// LiveTelemetryProvider.

#include <gtest/gtest.h>
#include <input/CsvTelemetryParser.h>

#include <string>

using namespace input;

// ============================================================================
// Header parsing
// ============================================================================

class CsvTelemetryParserTest : public ::testing::Test {
protected:
    CsvTelemetryParser parser;
};

TEST_F(CsvTelemetryParserTest, ParseHeader_RecognisesStandardColumns) {
    std::string error;
    EXPECT_TRUE(parser.parseHeader("time_s,throttle_pct,road_speed_kmh,gear,clutch_pct", error));
    EXPECT_EQ(parser.header().colTime, 0);
    EXPECT_EQ(parser.header().colThrottle, 1);
    EXPECT_EQ(parser.header().colRoad, 2);
    EXPECT_EQ(parser.header().colGear, 3);
    EXPECT_EQ(parser.header().colClutch, 4);
    EXPECT_FALSE(parser.header().timeInMs);
}

TEST_F(CsvTelemetryParserTest, ParseHeader_RecognisesTimestampMs) {
    std::string error;
    EXPECT_TRUE(parser.parseHeader("timestamp_ms,throttle_pct,road_speed_kmh", error));
    EXPECT_EQ(parser.header().colTime, 0);
    EXPECT_TRUE(parser.header().timeInMs);
}

TEST_F(CsvTelemetryParserTest, ParseHeader_RecognisesGearSelector) {
    std::string error;
    EXPECT_TRUE(parser.parseHeader("time_s,throttle_pct,gear_selector", error));
    EXPECT_EQ(parser.header().colGearSelector, 2);
}

TEST_F(CsvTelemetryParserTest, ParseHeader_MissingTimeColumn_Fails) {
    std::string error;
    EXPECT_FALSE(parser.parseHeader("throttle_pct,road_speed_kmh", error));
    EXPECT_FALSE(error.empty());
}

TEST_F(CsvTelemetryParserTest, ParseHeader_RawCanFormat_Fails) {
    std::string error;
    EXPECT_FALSE(parser.parseHeader("can_id,data_hex,time_s", error));
    EXPECT_FALSE(error.empty());
}

TEST_F(CsvTelemetryParserTest, ParseHeader_EmptyLine_Fails) {
    std::string error;
    EXPECT_FALSE(parser.parseHeader("", error));
    EXPECT_FALSE(error.empty());
}

TEST_F(CsvTelemetryParserTest, ParseHeader_CaseInsensitive) {
    std::string error;
    EXPECT_TRUE(parser.parseHeader("TIME_S,THROTTLE_PCT,ROAD_SPEED_KMH", error));
    EXPECT_EQ(parser.header().colTime, 0);
    EXPECT_EQ(parser.header().colThrottle, 1);
    EXPECT_EQ(parser.header().colRoad, 2);
}

// ============================================================================
// Row parsing
// ============================================================================

TEST_F(CsvTelemetryParserTest, ParseRow_AllFields) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader("time_s,throttle_pct,road_speed_kmh,gear,clutch_pct,gear_selector", error));

    CsvSample sample;
    EXPECT_TRUE(parser.parseRow("1.5,75.0,120.0,3,50.0,D", 1.0, sample, error));
    EXPECT_DOUBLE_EQ(sample.timeS, 1.5);
    EXPECT_DOUBLE_EQ(sample.throttle, 0.75);
    EXPECT_DOUBLE_EQ(sample.roadSpeedKmh, 120.0);
    EXPECT_EQ(sample.gear, 3);
    EXPECT_DOUBLE_EQ(sample.clutchPct, 0.5);
    EXPECT_EQ(sample.gearSelector, "D");
}

TEST_F(CsvTelemetryParserTest, ParseRow_ThrottlesOutsideRange_AreClamped) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader("time_s,throttle_pct", error));

    CsvSample s1;
    EXPECT_TRUE(parser.parseRow("0.0,150.0", 1.0, s1, error));
    EXPECT_DOUBLE_EQ(s1.throttle, 1.0);

    CsvSample s2;
    EXPECT_TRUE(parser.parseRow("0.0,-10.0", 1.0, s2, error));
    EXPECT_DOUBLE_EQ(s2.throttle, 0.0);
}

TEST_F(CsvTelemetryParserTest, ParseRow_NegativeRoadSpeed_KeepsSentinel) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader("time_s,road_speed_kmh", error));

    CsvSample sample;
    EXPECT_TRUE(parser.parseRow("0.0,-1.0", 1.0, sample, error));
    EXPECT_DOUBLE_EQ(sample.roadSpeedKmh, -2.0);  // sentinel unchanged
}

TEST_F(CsvTelemetryParserTest, ParseRow_EmptyFields_KeepDefaults) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader("time_s,throttle_pct,road_speed_kmh,gear,clutch_pct", error));

    CsvSample sample;
    EXPECT_TRUE(parser.parseRow("2.0,,,,", 1.0, sample, error));
    EXPECT_DOUBLE_EQ(sample.timeS, 2.0);
    EXPECT_DOUBLE_EQ(sample.throttle, 0.0);       // default
    EXPECT_DOUBLE_EQ(sample.roadSpeedKmh, -2.0);  // sentinel
    EXPECT_EQ(sample.gear, -1);                   // sentinel
    EXPECT_DOUBLE_EQ(sample.clutchPct, -1.0);     // sentinel
}

TEST_F(CsvTelemetryParserTest, ParseRow_MsTimestamp_DividedBy1000) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader("timestamp_ms,throttle_pct", error));

    CsvSample sample;
    EXPECT_TRUE(parser.parseRow("5000,50.0", 1000.0, sample, error));
    EXPECT_DOUBLE_EQ(sample.timeS, 5.0);
}

TEST_F(CsvTelemetryParserTest, ParseRow_InvalidTime_SkipsRow) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader("time_s,throttle_pct", error));

    CsvSample sample;
    EXPECT_FALSE(parser.parseRow("abc,50.0", 1.0, sample, error));
}

TEST_F(CsvTelemetryParserTest, ParseRow_EmptyLine_ReturnsFalse) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader("time_s,throttle_pct", error));

    CsvSample sample;
    EXPECT_FALSE(parser.parseRow("", 1.0, sample, error));
    EXPECT_FALSE(parser.parseRow("   ", 1.0, sample, error));
}
