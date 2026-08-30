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

// ============================================================================
// Brake-light column (binary pedal proxy from the vehicle-sim --stdout-csv
// pipe): recognised in the header, decoded tri-state (1/0/absent), and the
// retired brake_percent / brake_pedal_state aliases are NOT consumed.
// ============================================================================

TEST_F(CsvTelemetryParserTest, ParseHeader_RecognisesBrakeLightColumn) {
    std::string error;
    EXPECT_TRUE(parser.parseHeader("time_s,throttle_pct,brake_light", error));
    EXPECT_EQ(parser.header().colBrakeLight, 2);
}

TEST_F(CsvTelemetryParserTest, ParseRow_BrakeLightOnOffBlank) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader("time_s,brake_light", error));

    CsvSample on;
    EXPECT_TRUE(parser.parseRow("1.0,1", 1.0, on, error));
    EXPECT_TRUE(on.brakeLight.has_value() && *on.brakeLight);

    CsvSample off;
    EXPECT_TRUE(parser.parseRow("2.0,0", 1.0, off, error));
    EXPECT_TRUE(off.brakeLight.has_value() && !*off.brakeLight);

    CsvSample blank;
    EXPECT_TRUE(parser.parseRow("3.0,", 1.0, blank, error));
    EXPECT_FALSE(blank.brakeLight.has_value());  // blank keeps the nullopt default
}

TEST_F(CsvTelemetryParserTest, ParseRow_BrakeLightGarbageValue_IsAbsent) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader("time_s,brake_light", error));

    CsvSample sample;
    EXPECT_TRUE(parser.parseRow("1.0,x", 1.0, sample, error));
    EXPECT_FALSE(sample.brakeLight.has_value());  // unparseable => absent, never a guess
}

// Regression (owner decision 2): captures recorded with the OLD 13-column
// vehicle-sim schema (brake_percent) must still parse — the retired column is
// ignored via the unknown-column path and column-index alignment holds.
TEST_F(CsvTelemetryParserTest, OldSchemaWithBrakePercentColumn_StillParses) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader(
        "timestamp_ms,vehicle_id,speed_kmh,throttle_percent,brake_percent,"
        "acceleration_g,steering_angle_deg,motor_rpm,motor_hv_voltage,"
        "motor_hv_current,motor_torque_nm,gear_selector,dbc_signal_count",
        error));

    CsvSample sample;
    EXPECT_TRUE(parser.parseRow(
        "1000,tesla,50.00,30.00,25.00,0.15,2.50,3000.00,400.00,60.00,450.00,D,10",
        1000.0, sample, error));
    EXPECT_FALSE(sample.brakeLight.has_value());  // nothing consumed the old column
    EXPECT_DOUBLE_EQ(sample.timeS, 1.0);          // column alignment did not shift
    EXPECT_DOUBLE_EQ(sample.throttle, 0.30);
}

TEST_F(CsvTelemetryParserTest, ParseHeader_BrakePercentNoLongerConsumed) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader("time_s,brake_percent", error));
    EXPECT_EQ(parser.header().colBrakeLight, -1);  // old alias dropped, not kept

    CsvSample sample;
    EXPECT_TRUE(parser.parseRow("1.0,100", 1.0, sample, error));
    EXPECT_FALSE(sample.brakeLight.has_value());   // populated cell ignored
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

// Every accepted millisecond-timestamp spelling maps the time column AND sets
// the ms unit flag (callers then divide by 1000 via timeDivisor).
TEST_F(CsvTelemetryParserTest, ParseHeader_MillisecondTimeAliases_SetUnitFlag) {
    for (const char* alias : {"timestamp_utc_ms", "timestamp_ms", "ts_ms"}) {
        CsvTelemetryParser msParser;
        std::string error;
        ASSERT_TRUE(msParser.parseHeader(std::string(alias) + ",throttle_pct", error)) << alias;
        EXPECT_EQ(msParser.header().colTime, 0) << alias;
        EXPECT_TRUE(msParser.header().timeInMs) << alias;
    }
}

// The seconds-family spellings map the time column without the ms flag.
TEST_F(CsvTelemetryParserTest, ParseHeader_SecondsTimeAliases_KeepSecondUnits) {
    for (const char* alias : {"time_s", "time", "t", "timecode"}) {
        CsvTelemetryParser sParser;
        std::string error;
        ASSERT_TRUE(sParser.parseHeader(std::string(alias) + ",throttle_pct", error)) << alias;
        EXPECT_EQ(sParser.header().colTime, 0) << alias;
        EXPECT_FALSE(sParser.header().timeInMs) << alias;
    }
}

// Compact spellings from hand-written capture dialects map to the same
// decoded columns as their long forms.
TEST_F(CsvTelemetryParserTest, ParseHeader_CompactSignalAliases) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader(
        "time_s,throttle,road_speed,clutch,gearselector,brakelight", error));
    EXPECT_EQ(parser.header().colThrottle, 1);
    EXPECT_EQ(parser.header().colRoad, 2);
    EXPECT_EQ(parser.header().colClutch, 3);
    EXPECT_EQ(parser.header().colGearSelector, 4);
    EXPECT_EQ(parser.header().colBrakeLight, 5);
}

// Two time columns: the LAST one wins the index and the ms flag is sticky
// (an ms spelling anywhere in the header keeps the capture on ms units).
TEST_F(CsvTelemetryParserTest, ParseHeader_DuplicateTimeColumn_LastWinsMsFlagSticky) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader("time_s,timestamp_ms", error));
    EXPECT_EQ(parser.header().colTime, 1);
    EXPECT_TRUE(parser.header().timeInMs);

    CsvTelemetryParser reversed;
    ASSERT_TRUE(reversed.parseHeader("timestamp_ms,time_s", error));
    EXPECT_EQ(reversed.header().colTime, 1);
    EXPECT_TRUE(reversed.header().timeInMs);
}

// Column names tolerate surrounding whitespace (tabs and spaces) — the
// capture dialects pad-align their headers.
TEST_F(CsvTelemetryParserTest, ParseHeader_TrimsWhitespaceAroundColumnNames) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader("  Time_S ,\tThrottle_PCT\t, speed_kmh  ", error));
    EXPECT_EQ(parser.header().colTime, 0);
    EXPECT_EQ(parser.header().colThrottle, 1);
    EXPECT_EQ(parser.header().colRoad, 2);
}

// Raw-CAN rejection needs BOTH markers: a capture that merely mentions can_id
// (or data_hex) alongside decoded columns is not a raw capture.
TEST_F(CsvTelemetryParserTest, ParseHeader_SingleCanMarker_IsNotRawCan) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader("can_id,time_s", error));
    EXPECT_EQ(parser.header().colTime, 1);

    CsvTelemetryParser hexOnly;
    ASSERT_TRUE(hexOnly.parseHeader("data_hex,time_s", error));
    EXPECT_EQ(hexOnly.header().colTime, 1);
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
