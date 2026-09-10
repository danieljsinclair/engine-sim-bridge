// TcpCsvFrameParserTests.cpp
//
// Port of EngineSimApp/Tests/CsvTelemetryParserTests.swift onto the live-schema
// bridge parser (TcpCsvFrameParser). Every Swift test method maps 1:1; the
// Swift multi-row `parse(csv)` conveniences become header + parseRow pairs.

#include <gtest/gtest.h>

#include "io/TcpCsvFrameParser.h"

using input::TcpCsvColumnMap;
using input::TcpCsvFrameParser;

// ---- Happy path -------------------------------------------------------------

TEST(TcpCsvFrameParserTests, ParsesTaskSchema) {
    // Standard vehicle-sim CSV with all channels (Swift testParsesTaskSchema).
    const char* kHeader = "timestamp_ms,vehicle_id,speed_kmh,throttle_percent,brake_percent,acceleration_g";
    const auto map = TcpCsvFrameParser::tryHeader(kHeader);
    ASSERT_TRUE(map.has_value());

    const auto first = TcpCsvFrameParser::parseRow("1000,carA,60.0,40.0,0.0,0.3", map.value());
    const auto second = TcpCsvFrameParser::parseRow("2000,carA,80.0,55.0,10.0,0.2", map.value());
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());

    EXPECT_DOUBLE_EQ(first->timestampMs, 1000.0);
    EXPECT_DOUBLE_EQ(first->speedKmh.value(), 60.0);
    EXPECT_DOUBLE_EQ(first->throttle.value(), 40.0);
    EXPECT_DOUBLE_EQ(first->brake.value(), 0.0);
    EXPECT_DOUBLE_EQ(second->throttle.value(), 55.0);
}

TEST(TcpCsvFrameParserTests, ParsesSampleDriveSchemaTolerantToEmptyCells) {
    // Mirrors vehicle-sim/telemetry/sampleDrive.csv: sparse columns. The first
    // row has a timestamp but no other channels -> still parsed (Swift
    // testParsesSampleDriveSchemaTolerantToEmptyCells).
    const char* kHeader = "timestamp_utc_ms,throttle_pct,speed_kmh,acceleration_g,brake_pct";
    const auto map = TcpCsvFrameParser::tryHeader(kHeader);
    ASSERT_TRUE(map.has_value());

    const auto sparse = TcpCsvFrameParser::parseRow("1781472526915,,,,,,,", map.value());
    const auto full = TcpCsvFrameParser::parseRow("1781472527015,30.0,80.5,0.3,0.0", map.value());
    ASSERT_TRUE(sparse.has_value());
    ASSERT_TRUE(full.has_value());

    EXPECT_DOUBLE_EQ(full->throttle.value(), 30.0);
    EXPECT_DOUBLE_EQ(full->speedKmh.value(), 80.5);
    EXPECT_FALSE(sparse->throttle.has_value());
}

// ---- Edge cases -------------------------------------------------------------

TEST(TcpCsvFrameParserTests, ReturnsNulloptWhenNoTimestampColumn) {
    // Cannot pace the stream without a timestamp column (precondition).
    EXPECT_FALSE(TcpCsvFrameParser::tryHeader("throttle_pct,speed_kmh").has_value());
}

TEST(TcpCsvFrameParserTests, SkipsRowsWithUnparseableTimestamp) {
    const auto map = TcpCsvFrameParser::tryHeader("timestamp_ms,throttle_percent");
    ASSERT_TRUE(map.has_value());
    // Corrupt row skipped (nullopt), valid rows continue.
    EXPECT_FALSE(TcpCsvFrameParser::parseRow("not_a_number,40.0", map.value()).has_value());
    const auto good = TcpCsvFrameParser::parseRow("2000,55.0", map.value());
    ASSERT_TRUE(good.has_value());
    EXPECT_DOUBLE_EQ(good->timestampMs, 2000.0);
}

// ---- Column map variants ------------------------------------------------------

TEST(TcpCsvFrameParserTests, RejectsMissingTimestampColumn) {
    EXPECT_FALSE(TcpCsvFrameParser::tryHeader("vehicle_id,throttle_pct,speed_kmh").has_value());
}

TEST(TcpCsvFrameParserTests, AcceptsTimestampMsVariant) {
    const auto map = TcpCsvFrameParser::tryHeader(
        "timestamp_ms,speed_kmh,throttle_percent,brake_percent,acceleration_g");
    ASSERT_TRUE(map.has_value());
    EXPECT_EQ(map->timestamp, 0);
    EXPECT_EQ(map->speed, 1);
    EXPECT_EQ(map->throttle, 2);
}

TEST(TcpCsvFrameParserTests, AcceptsTimestampUtcMsVariant) {
    const auto map = TcpCsvFrameParser::tryHeader("timestamp_utc_ms,throttle_pct,speed_kmh");
    ASSERT_TRUE(map.has_value());
    EXPECT_EQ(map->timestamp, 0);
    EXPECT_EQ(map->throttle, 1);
    EXPECT_EQ(map->speed, 2);
}

TEST(TcpCsvFrameParserTests, HandlesBothThrottleVariants) {
    const auto map1 = TcpCsvFrameParser::tryHeader("timestamp_ms,throttle_pct");
    ASSERT_TRUE(map1.has_value());
    EXPECT_EQ(map1->throttle, 1);

    const auto map2 = TcpCsvFrameParser::tryHeader("timestamp_ms,throttle_percent");
    ASSERT_TRUE(map2.has_value());
    EXPECT_EQ(map2->throttle, 1);
}

// ---- Row parsing ------------------------------------------------------------

TEST(TcpCsvFrameParserTests, ParsesEmptyCellAsNullopt) {
    TcpCsvColumnMap map;
    map.timestamp = 0;
    map.throttle = 1;
    const auto row = TcpCsvFrameParser::parseRow("1000,", map);
    ASSERT_TRUE(row.has_value());
    EXPECT_DOUBLE_EQ(row->timestampMs, 1000.0);
    EXPECT_FALSE(row->throttle.has_value());  // empty cell -> absent
}

TEST(TcpCsvFrameParserTests, ParsesWhitespacePaddedCells) {
    TcpCsvColumnMap map;
    map.timestamp = 0;
    map.throttle = 1;
    const auto row = TcpCsvFrameParser::parseRow(" 1000 , 50.0 ", map);
    ASSERT_TRUE(row.has_value());
    EXPECT_DOUBLE_EQ(row->timestampMs, 1000.0);
    EXPECT_DOUBLE_EQ(row->throttle.value(), 50.0);
}

// ---- Header probing -----------------------------------------------------------

TEST(TcpCsvFrameParserTests, TryHeaderReturnsMapForValidHeader) {
    const auto map = TcpCsvFrameParser::tryHeader("timestamp_ms,throttle_pct");
    ASSERT_TRUE(map.has_value());
    EXPECT_EQ(map->timestamp, 0);
    EXPECT_EQ(map->throttle, 1);
}

TEST(TcpCsvFrameParserTests, TryHeaderReturnsNulloptForInvalidHeader) {
    // Non-CSV line -> the stream client keeps probing.
    EXPECT_FALSE(TcpCsvFrameParser::tryHeader("WELCOME TO VEHICLE SIM").has_value());
}

TEST(TcpCsvFrameParserTests, TryHeaderToleratesTrailingSpaces) {
    // Real CSV may carry trailing whitespace/newlines.
    const auto map = TcpCsvFrameParser::tryHeader("timestamp_ms,throttle_pct,\n  ");
    ASSERT_TRUE(map.has_value());
    EXPECT_EQ(map->timestamp, 0);
    EXPECT_EQ(map->throttle, 1);
}
