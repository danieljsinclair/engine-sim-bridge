// CsvTelemetryParserTest.cpp
//
// Tests for the shared CSV parser used by both ReplayTelemetryProvider and
// LiveTelemetryProvider.

#include <gtest/gtest.h>
#include <input/CsvTelemetryParser.h>

#include <cstdio>
#include <cstring>
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
// Out-of-range / epoch-scale timestamp rejection.
//
// Trailing rows whose timestamp is epoch-scale (~1.79e12 s) must be skipped
// INSTANTLY (one row at a time, no wall-clock delay), but the formerly
// per-row stderr WARNING must be collapsed into EXACTLY ONE summary line at
// end-of-input. This guards the user-observed "every rejected row spams a
// separate stderr line" regression.
// ============================================================================

TEST_F(CsvTelemetryParserTest, ParseRow_EpochScaleTimestamps_AreSkippedAndSummarisedOnce) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader("time_s,throttle_pct", error));

    // A few legitimate rows, then several epoch-scale trailing rows.
    CsvSample good;
    EXPECT_TRUE(parser.parseRow("1.5,60.0", 1.0, good, error));
    EXPECT_DOUBLE_EQ(good.timeS, 1.5);

    // 5 epoch-scale stragglers — each must be rejected immediately.
    constexpr int kOutliers = 5;
    for (int i = 0; i < kOutliers; ++i) {
        CsvSample bad;
        // 1.786961013730e12 s >> 1e7 threshold.
        EXPECT_FALSE(parser.parseRow("1786961013730.0,0.0", 1.0, bad, error))
            << "epoch-scale row " << i << " must be skipped";
    }

    // Capture stderr and confirm exactly ONE summary line is emitted.
    std::fflush(stderr);
    std::string captured;
    testing::internal::CaptureStderr();
    parser.emitRejectionSummary();
    captured = testing::internal::GetCapturedStderr();
    std::fflush(stderr);

    // A single line, carrying the total count.
    EXPECT_EQ(captured.empty(), false);
    const size_t lines = std::count(captured.begin(), captured.end(), '\n');
    EXPECT_EQ(lines, 1u) << "expected exactly one summary line, got:\n" << captured;
    EXPECT_NE(captured.find("[CsvTelemetryParser] INFO: skipped 5 row(s)"), std::string::npos)
        << "summary line missing count/format:\n" << captured;
}

TEST_F(CsvTelemetryParserTest, EmitRejectionSummary_PrintsNothingWhenNoOutliers) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader("time_s,throttle_pct", error));

    CsvSample good;
    EXPECT_TRUE(parser.parseRow("1.5,60.0", 1.0, good, error));

    std::fflush(stderr);
    testing::internal::CaptureStderr();
    parser.emitRejectionSummary();
    const std::string captured = testing::internal::GetCapturedStderr();
    std::fflush(stderr);

    EXPECT_TRUE(captured.empty())
        << "no summary expected when zero rows rejected, got:\n" << captured;
}

// Epoch-scale timestamp_ms (e.g. vehicle-sim emits Unix epoch milliseconds such
// as 1786538088200) must be REBASED to 0-based seconds, NOT rejected. A bare
// epoch value would normalise to ~1.79e9 s and trip the >1e7 outlier guard,
// silently dropping the ENTIRE stream — which is exactly the "live telemetry
// doesn't track the CSV" bug when piping vehicle-sim's --stdout-csv into
// --live-telemetry. The first row anchors t=0; subsequent rows are relative.
TEST_F(CsvTelemetryParserTest, ParseRow_EpochMillisecondTimestamps_AreRebasedToZero) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader("timestamp_ms,throttle_pct,road_speed_kmh", error));
    EXPECT_TRUE(parser.header().timeInMs);

    // First row anchors t=0 at the epoch value 1786538088200.
    CsvSample first;
    ASSERT_TRUE(parser.parseRow("1786538088200,55.0,0.0", 1000.0, first, error));
    EXPECT_DOUBLE_EQ(first.timeS, 0.0);

    // A later row 1000 ms later must be t=1.0 s, not rejected.
    CsvSample later;
    ASSERT_TRUE(parser.parseRow("1786538089200,60.0,30.0", 1000.0, later, error));
    EXPECT_DOUBLE_EQ(later.timeS, 1.0);
    EXPECT_DOUBLE_EQ(later.roadSpeedKmh, 30.0);

    // No rows rejected -> rejection summary prints nothing.
    std::fflush(stderr);
    testing::internal::CaptureStderr();
    parser.emitRejectionSummary();
    const std::string captured = testing::internal::GetCapturedStderr();
    std::fflush(stderr);
    EXPECT_TRUE(captured.empty())
        << "no summary expected for rebaseable epoch-ms stream, got:\n" << captured;
}

// A relative (non-epoch) timestamp_ms column must still parse normally via the
// timeDivisor path (not be mistaken for epoch-scale).
TEST_F(CsvTelemetryParserTest, ParseRow_RelativeMillisecondTimestamps_ParsedViaDivisor) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader("timestamp_ms,throttle_pct", error));
    EXPECT_TRUE(parser.header().timeInMs);

    CsvSample s;
    ASSERT_TRUE(parser.parseRow("1500.0,75.0", 1000.0, s, error));
    EXPECT_DOUBLE_EQ(s.timeS, 1.5);
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

// ============================================================================
// Regression: vehicle-sim capture schema aliases.
//
// vehicle-sim captures (e.g. roadtest_2026-08-17-1056.csv) use the header names
// throttle_gas_pct / vehicle_speed_kmh / clutch_pressure. These were previously
// unrecognised, so throttle decoded to 0 and road speed fell back to the -2.0
// dyno-off sentinel — the sim idled forever. This must never silently regress.
// ============================================================================

TEST_F(CsvTelemetryParserTest, ParseHeader_RecognisesVehicleSimSchemaAliases) {
    std::string error;
    EXPECT_TRUE(parser.parseHeader(
        "time_s,throttle_gas_pct,vehicle_speed_kmh,clutch_pressure", error));
    EXPECT_EQ(parser.header().colThrottle, 1);
    EXPECT_EQ(parser.header().colRoad, 2);
    EXPECT_EQ(parser.header().colClutch, 3);
}

TEST_F(CsvTelemetryParserTest, ParseRow_VehicleSimSchemaDecodesRealValues) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader(
        "time_s,throttle_gas_pct,vehicle_speed_kmh,clutch_pressure", error));

    CsvSample sample;
    ASSERT_TRUE(parser.parseRow("1.0,80.0,100.0,0.0", 1.0, sample, error));
    EXPECT_DOUBLE_EQ(sample.timeS, 1.0);
    EXPECT_DOUBLE_EQ(sample.throttle, 0.80);          // NOT the 0.0 silent default
    EXPECT_DOUBLE_EQ(sample.roadSpeedKmh, 100.0);      // NOT the -2.0 dyno sentinel
    EXPECT_DOUBLE_EQ(sample.clutchPct, 0.0);
}

TEST_F(CsvTelemetryParserTest, ParseRow_VehicleSimSchemaDecodesClutchPressure) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader(
        "time_s,throttle_gas_pct,vehicle_speed_kmh,clutch_pressure", error));

    CsvSample sample;
    ASSERT_TRUE(parser.parseRow("2.0,30.0,55.0,40.0", 1.0, sample, error));
    EXPECT_DOUBLE_EQ(sample.throttle, 0.30);
    EXPECT_DOUBLE_EQ(sample.roadSpeedKmh, 55.0);
    EXPECT_DOUBLE_EQ(sample.clutchPct, 0.40);         // 40% -> 0.40
}

// ============================================================================
// Steering-angle column (steering_angle_deg): vehicle-sim's decoded
// SCCM_steeringAngle (CAN 0x129 / 297, signed degrees). Recognised in the
// real --stdout-csv header, decoded as an optional signed double (blank or
// garbage = absent, mirroring the brake-light tri-state), and simply not
// consumed when the feed has no such column (non-DBC sources unchanged).
// The fixture header + rows below are REAL vehicle-sim output rows from the
// ReadingPickup_2026-08-31_001408 capture (embedded, no file reads).
// ============================================================================

namespace {
// The exact 13-column header vehicle-sim --stdout-csv emits (CsvRowFormatter).
const char* const kVehicleSimHeader =
    "timestamp_ms,vehicle_id,speed_kmh,throttle_percent,brake_light,"
    "acceleration_g,steering_angle_deg,motor_rpm,motor_hv_voltage,"
    "motor_hv_current,motor_torque_nm,gear_selector,dbc_signal_count";
}  // namespace

TEST_F(CsvTelemetryParserTest, ParseHeader_RecognisesSteeringAngleColumn) {
    std::string error;
    EXPECT_TRUE(parser.parseHeader(kVehicleSimHeader, error));
    EXPECT_EQ(parser.header().colSteering, 6);  // steering_angle_deg position
}

TEST_F(CsvTelemetryParserTest, ParseRow_SteeringAngleDecodesRealRows) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader(kVehicleSimHeader, error));

    // Real capture rows: early partial row (steering only), cruise row, and
    // the capture's largest angle under way.
    CsvSample early;
    ASSERT_TRUE(parser.parseRow(
        "1788131648475,tesla,,,,,1.60,,,,,,1", 0.001, early, error));
    ASSERT_TRUE(early.steeringAngleDeg.has_value());
    EXPECT_DOUBLE_EQ(*early.steeringAngleDeg, 1.60);

    CsvSample underWay;
    ASSERT_TRUE(parser.parseRow(
        "1788131648574,tesla,95.60,0.00,,,1.80,,,,,D,4", 0.001, underWay, error));
    ASSERT_TRUE(underWay.steeringAngleDeg.has_value());
    EXPECT_DOUBLE_EQ(*underWay.steeringAngleDeg, 1.80);

    CsvSample turning;
    ASSERT_TRUE(parser.parseRow(
        "1788131684325,tesla,106.00,31.60,0,,3.90,,,,360.00,D,6", 0.001, turning, error));
    ASSERT_TRUE(turning.steeringAngleDeg.has_value());
    EXPECT_DOUBLE_EQ(*turning.steeringAngleDeg, 3.90);
}

// The signal is signed (DBC range [-819.2, 819.1]); negative (left-hand)
// angles must decode with their sign intact — the display reads direction
// from it.
TEST_F(CsvTelemetryParserTest, ParseRow_SteeringAngleNegativeDecodesSigned) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader("time_s,steering_angle_deg", error));

    CsvSample sample;
    ASSERT_TRUE(parser.parseRow("1.0,-12.50", 1.0, sample, error));
    ASSERT_TRUE(sample.steeringAngleDeg.has_value());
    EXPECT_DOUBLE_EQ(*sample.steeringAngleDeg, -12.50);
}

// Blank (not reported this row) and unparseable cells are ABSENT — nullopt,
// not zero. Zero-vs-absent is a real distinction: the renderer must show
// nothing when the feed carries no steering.
TEST_F(CsvTelemetryParserTest, ParseRow_SteeringAngleBlankAndGarbageAreAbsent) {
    std::string error;
    ASSERT_TRUE(parser.parseHeader("time_s,steering_angle_deg", error));

    CsvSample blank;
    ASSERT_TRUE(parser.parseRow("1.0,", 1.0, blank, error));
    EXPECT_FALSE(blank.steeringAngleDeg.has_value());

    CsvSample garbage;
    ASSERT_TRUE(parser.parseRow("2.0,abc", 1.0, garbage, error));
    EXPECT_FALSE(garbage.steeringAngleDeg.has_value());
}

// Degraded contract: a feed WITHOUT the column (legacy captures, non-DBC
// sources) parses exactly as before — the column index stays -1 and the
// sample stays absent. No behavior change for steering-less feeds.
TEST_F(CsvTelemetryParserTest, ParseHeader_WithoutSteeringColumn_Unchanged) {
    std::string error;
    EXPECT_TRUE(parser.parseHeader("time_s,throttle_pct,road_speed_kmh", error));
    EXPECT_EQ(parser.header().colSteering, -1);

    CsvSample sample;
    ASSERT_TRUE(parser.parseRow("1.0,50.0,80.0", 1.0, sample, error));
    EXPECT_FALSE(sample.steeringAngleDeg.has_value());
}
