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

TEST_F(CsvTelemetryParserTest, ParseRow_NegativeRoadSpeedIsPreserved) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader("time_s,road_speed_kmh", error));

    CsvSample sample;
    // Reverse driving is a real CSV state (em-dinner.csv 'R' rows at -3.2 km/h).
    // The parser must preserve the negative speed rather than collapsing it to
    // the -2.0 sentinel, so downstream reverse coercion can tell genuine reverse
    // (speed < 0, honoured) from a standstill/forward 'R' (coerced to P/N).
    EXPECT_TRUE(parser.parseRow("0.0,-1.0", 1.0, sample, error));
    EXPECT_DOUBLE_EQ(sample.roadSpeedKmh, -1.0);
    EXPECT_TRUE(parser.parseRow("1.0,-3.2", 1.0, sample, error));
    EXPECT_DOUBLE_EQ(sample.roadSpeedKmh, -3.2);
    // A blank/unparseable road column still leaves the -2.0 sentinel intact.
    EXPECT_TRUE(parser.parseRow("2.0,", 1.0, sample, error));
    EXPECT_DOUBLE_EQ(sample.roadSpeedKmh, -2.0);
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

// ============================================================================
// Motor-torque column (MATCH mode) — the recorded motor/engine torque that the
// torque-feedback drivetrain injects at the transmission input. Surfacing it
// from the CSV is what makes MATCH mode deterministic (the recorded torque is
// literally what produced the recorded speed). Aliases motor_torque / torque_nm
// are accepted; a blank/absent column leaves the 0.0 default (a true no-op when
// injected onto the rotating mass in FREE/PIN).
// ============================================================================

TEST_F(CsvTelemetryParserTest, ParseHeader_RecognisesMotorTorqueColumn) {
    std::string error;
    EXPECT_TRUE(parser.parseHeader("time_s,throttle_pct,road_speed_kmh,motor_torque_nm", error));
    EXPECT_EQ(parser.header().colMotorTorque, 3);
}

TEST_F(CsvTelemetryParserTest, ParseHeader_RecognisesMotorTorqueAliases) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader("time_s,motor_torque", error));
    EXPECT_EQ(parser.header().colMotorTorque, 1);

    CsvTelemetryParser aliasParser;
    ASSERT_TRUE(aliasParser.parseHeader("time_s,torque_nm", error));
    EXPECT_EQ(aliasParser.header().colMotorTorque, 1);
}

TEST_F(CsvTelemetryParserTest, ParseRow_SurfacesPositiveMotorTorque) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader("time_s,throttle_pct,road_speed_kmh,motor_torque_nm", error));

    CsvSample sample;
    ASSERT_TRUE(parser.parseRow("1.0,60.0,50.0,1840.5", 1.0, sample, error));
    EXPECT_DOUBLE_EQ(sample.motorTorqueNm, 1840.5);
}

TEST_F(CsvTelemetryParserTest, ParseRow_SurfacesNegativeRegenTorque) {
    // Regen/braking torque is negative; it must pass through verbatim (not
    // clamped to 0) so MATCH mode can decelerate the vehicle too.
    std::string error;
    ASSERT_TRUE(parser.parseHeader("time_s,motor_torque_nm", error));

    CsvSample sample;
    ASSERT_TRUE(parser.parseRow("2.0,-950.25", 1.0, sample, error));
    EXPECT_DOUBLE_EQ(sample.motorTorqueNm, -950.25);
}

TEST_F(CsvTelemetryParserTest, ParseRow_MissingTorqueColumn_LeavesDefaultZero) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader("time_s,throttle_pct,road_speed_kmh", error));

    CsvSample sample;
    ASSERT_TRUE(parser.parseRow("1.0,50.0,30.0", 1.0, sample, error));
    EXPECT_DOUBLE_EQ(sample.motorTorqueNm, 0.0);  // no column -> no-op injection
}

// motor_torque_nm column (MATCH mode input). Must be recognised in the header
// and surfaced on the decoded sample so the twin can inject it.
TEST_F(CsvTelemetryParserTest, ParseRow_SurfacesMotorTorqueNm) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader("time_s,throttle_pct,road_speed_kmh,motor_torque_nm", error));
    ASSERT_EQ(parser.header().colMotorTorque, 3);

    CsvSample sample;
    ASSERT_TRUE(parser.parseRow("1.5,75.0,120.0,1850.0", 1.0, sample, error));
    EXPECT_DOUBLE_EQ(sample.motorTorqueNm, 1850.0);
}

TEST_F(CsvTelemetryParserTest, ParseRow_NegativeRegenTorqueIsPreserved) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader("time_s,motor_torque_nm", error));
    CsvSample sample;
    ASSERT_TRUE(parser.parseRow("2.0,-960.5", 1.0, sample, error));
    EXPECT_DOUBLE_EQ(sample.motorTorqueNm, -960.5);
}
