// CsvTelemetryParserCharacterizationTest.cpp - Characterization net for
// CsvTelemetryParser ahead of the planned sonar refactor of parseRow
// (cpp:S3776, cognitive complexity 40 -> helper decomposition into
// tokenize/validate/field-mapping phases) and emitRejectionSummary
// (cpp:S5145, tainted 'this' leaking into fprintf via rejectedOutlierRows_).
//
// SCOPE: pins the CURRENT observable behaviour through the real public API
// (parseHeader + parseRow + emitRejectionSummary, no mocks, no production
// edits). Every test is a green-phase characterization test against
// master @ 3b09dce: it asserts what the code DOES today so the refactor must
// keep doing it. Behaviour must be preserved exactly (owner directive) —
// parsing outcomes, accepted/rejected rows, values, error states.
//
// NOT duplicated here (already pinned by CsvTelemetryParserTest):
//   - standard column recognition, case-insensitivity, whitespace trimming
//     of header names, ms/seconds alias families, compact aliases
//   - throttle clamping, negative road speed, blank-field defaults
//   - epoch-scale rejection + single summary line, epoch-ms rebasing
//   - motor torque decode, steering decode/tri-state, retired brake_percent
//   - blank settle rows vs populated rows (engineDataPresent basics)
//
// Pinned HERE (the behaviours an SRP split most easily breaks):
//   header phase:
//     - minimal valid header is the time column alone
//     - whitespace-only header line fails like an empty one
//     - missing-time diagnosis WINS over raw-CAN diagnosis (check order)
//     - raw-CAN markers match case-insensitively (both markers required)
//     - duplicate non-time column: LAST occurrence wins the index
//     - the full alias table, data-driven, incl. ms-flag polarity
//     - re-parsing a header REPLACES the previous column map (no stale
//       indices survive into the second stream)
//   row gate + short-circuit semantics:
//     - a row shorter than the time column is rejected (time is the gate)
//     - rejected rows (blank / unparseable time / outlier) leave the OUT
//       sample byte-identical to its previous content
//     - garbage in every engine field still accepts the row (time gate is
//       the ONLY reject gate after the time parse; each field degrades to
//       its default independently)
//     - trailing extra columns beyond the header are ignored
//     - rows missing trailing columns keep the field defaults
//     - '#vs-...' hint lines are skipped WITHOUT counting as outliers and
//       without stalling the parser (next row still surfaces)
//   numeric semantics (stod/stoi prefix parse):
//     - trailing junk after a valid number is accepted with the prefix
//       value (time, throttle, gear) — a strict from_chars swap breaks this
//     - gear decimals truncate (stoi prefix); out-of-range integers and
//       doubles fall back to defaults without rejecting the row
//   field-decode asymmetries (a uniform "clamp all fields" helper breaks):
//     - clutch: negative value is NOT parsed (stays -1 sentinel, no engine
//       data) while >100 clamps to 1.0 — opposite of throttle clamping
//     - steering alone does NOT mark engineDataPresent (display-only)
//     - brake_light out-of-domain integer (e.g. 2) marks engineDataPresent
//       but leaves the value nullopt
//     - gear_selector accepts ANY non-empty string verbatim (no PRNDL
//       validation); a whitespace-only cell is treated as absent
//   time handling:
//     - epoch threshold is 1e12 ms with >= semantics: 999999999999 ms is
//       an outlier-rejected relative timestamp, 1000000000000 ms rebases t=0
//     - epoch rows preserve the RAW epoch ms in timeMs; relative/time_s
//       rows leave timeMs at -1
//     - the >1e7 s outlier guard is strict: exactly 1e7 accepted
//     - non-monotonic epoch rows yield NEGATIVE timeS (no monotonicity fix-up)
//     - arbitrarily huge epoch-ms values are accepted (epoch path has no
//       upper bound; the outlier guard only guards the relative path)
//     - a stream may mix epoch and relative ms rows; the anchor set by the
//       first epoch row persists across relative rows
//     - re-parsing a header re-anchors the epoch base AND clears the
//       outlier counter (parser instance is reusable across streams)
//   S5145 path (emitRejectionSummary / fprintf of rejectedOutlierRows_):
//     - the count includes ONLY >1e7 outlier rejects — blank lines, hint
//       lines, short rows and unparseable-time rows do NOT increment it
//     - the summary is NOT one-shot: every call while the count is non-zero
//       prints; it goes silent only after a re-parseHeader resets the count

#include <gtest/gtest.h>
#include <input/CsvTelemetryParser.h>

#include <cstdio>
#include <string>
#include <vector>

namespace {

using input::CsvSample;
using input::CsvTelemetryParser;

class CsvTelemetryParserCharacterizationTest : public ::testing::Test {
protected:
    CsvTelemetryParser parser;
    std::string error;

    // Parse a header, asserting success, to keep row tests focused.
    void requireHeader(const std::string& headerLine) {
        ASSERT_TRUE(parser.parseHeader(headerLine, error)) << "header: " << headerLine;
    }

    // Captures stderr emitted while fn runs (C-level stderr, like the
    // production fprintf). Convention matches CsvTelemetryParserTest.
    template <typename Fn>
    std::string captureStderr(Fn&& fn) {
        std::fflush(stderr);
        testing::internal::CaptureStderr();
        fn();
        std::fflush(stderr);
        const std::string captured = testing::internal::GetCapturedStderr();
        return captured;
    }

    static std::string toLower(std::string s) {
        for (char& c : s) {
            if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
        }
        return s;
    }
};

// ===========================================================================
// Header phase
// ===========================================================================

TEST_F(CsvTelemetryParserCharacterizationTest, MinimalTimeOnlyHeaderIsValid) {
    requireHeader("time_s");
    EXPECT_EQ(parser.header().colTime, 0);
    EXPECT_EQ(parser.header().colThrottle, -1);
    EXPECT_EQ(parser.header().colRoad, -1);

    // A time-only header still accepts rows: a bare timestamp is a
    // timecoded blank, not an operating point.
    CsvSample s;
    EXPECT_TRUE(parser.parseRow("1.5", 1.0, s, error));
    EXPECT_DOUBLE_EQ(s.timeS, 1.5);
    EXPECT_FALSE(s.engineDataPresent);
}

TEST_F(CsvTelemetryParserCharacterizationTest, WhitespaceOnlyHeaderLineFails) {
    EXPECT_FALSE(parser.parseHeader("   ", error));
    EXPECT_FALSE(error.empty());
}

// The colTime check runs BEFORE the raw-CAN check: a capture missing the
// time column is diagnosed as "missing time" even when it is also raw CAN.
// Which diagnosis the user sees is observable contract.
TEST_F(CsvTelemetryParserCharacterizationTest, MissingTimeDiagnosisWinsOverRawCan) {
    EXPECT_FALSE(parser.parseHeader("can_id,data_hex", error));
    EXPECT_FALSE(error.empty());
    const std::string folded = toLower(error);
    EXPECT_NE(folded.find("time"), std::string::npos)
        << "expected the missing-time diagnosis, got: " << error;
}

TEST_F(CsvTelemetryParserCharacterizationTest, RawCanMarkersMatchCaseInsensitive) {
    EXPECT_FALSE(parser.parseHeader("CAN_ID,DATA_HEX,TIME_S", error));
    EXPECT_FALSE(error.empty());
    const std::string folded = toLower(error);
    EXPECT_NE(folded.find("can"), std::string::npos)
        << "expected the raw-CAN diagnosis, got: " << error;
}

TEST_F(CsvTelemetryParserCharacterizationTest, DuplicateNonTimeColumnLastOccurrenceWins) {
    requireHeader("time_s,throttle,throttle_pct");
    EXPECT_EQ(parser.header().colThrottle, 2);  // later duplicate overwrites

    CsvSample s;
    EXPECT_TRUE(parser.parseRow("1.0,99.0,25.0", 1.0, s, error));
    EXPECT_DOUBLE_EQ(s.throttle, 0.25);  // read from column 2, not column 1
}

// The full alias registry, data-driven: every spelling maps its column; the
// ms flag is set ONLY by the millisecond family. A refactor of the alias
// table (the likely S3776 helper extraction) must keep every row intact.
TEST_F(CsvTelemetryParserCharacterizationTest, FullAliasTableMapsEverySpelling) {
    // The ms family maps the time column (index 0 here) AND sets the flag.
    for (const char* alias : {"timestamp_utc_ms", "timestamp_ms", "ts_ms"}) {
        SCOPED_TRACE(alias);
        CsvTelemetryParser p;
        std::string err;
        ASSERT_TRUE(p.parseHeader(std::string(alias) + ",throttle_pct", err));
        EXPECT_EQ(p.header().colTime, 0);
        EXPECT_TRUE(p.header().timeInMs);
    }

    struct SecondsOrEngine {
        const char* label;
        int input::CsvHeader::*column;
        std::vector<const char*> aliases;
    };
    const std::vector<SecondsOrEngine> columns = {
        {"time_s", &input::CsvHeader::colTime,
         {"time_s", "time", "t", "timecode"}},
        {"throttle", &input::CsvHeader::colThrottle,
         {"throttle_pct", "throttle_percent", "throttle", "throttle_gas_pct"}},
        {"road", &input::CsvHeader::colRoad,
         {"road_speed_kmh", "road_speed", "speed_kmh", "speed", "vehicle_speed_kmh"}},
        {"gear", &input::CsvHeader::colGear, {"gear"}},
        {"gear_selector", &input::CsvHeader::colGearSelector,
         {"gear_selector", "gearselector"}},
        {"clutch", &input::CsvHeader::colClutch,
         {"clutch_pct", "clutch", "clutch_pressure"}},
        {"motor_torque", &input::CsvHeader::colMotorTorque,
         {"motor_torque_nm", "motor_torque", "torque_nm"}},
        {"brake_light", &input::CsvHeader::colBrakeLight,
         {"brake_light", "brakelight"}},
        {"steering", &input::CsvHeader::colSteering,
         {"steering_angle_deg", "steering_angle"}},
    };
    for (const auto& col : columns) {
        for (const char* alias : col.aliases) {
            SCOPED_TRACE(std::string(col.label) + " alias '" + alias + "'");
            CsvTelemetryParser p;
            std::string err;
            ASSERT_TRUE(p.parseHeader(std::string("time_s,") + alias, err)) << alias;
            EXPECT_EQ(p.header().*col.column, 1) << alias;
            // No non-ms spelling sets the ms flag; the time_s in column 0 is
            // itself a seconds spelling, so the flag must stay false.
            EXPECT_FALSE(p.header().timeInMs) << alias;
        }
    }
}

// Re-parsing a header must fully replace the previous column map: indices
// from the first stream must not leak into rows of the second.
TEST_F(CsvTelemetryParserCharacterizationTest, ReparseReplacesPreviousColumnMap) {
    requireHeader("time_s,throttle_pct");
    ASSERT_TRUE(parser.parseHeader("time_s", error));
    EXPECT_EQ(parser.header().colThrottle, -1);  // stale index gone

    CsvSample s;
    // A populated second cell is now an unknown trailing column: ignored.
    EXPECT_TRUE(parser.parseRow("1.0,50.0", 1.0, s, error));
    EXPECT_DOUBLE_EQ(s.throttle, 0.0);
    EXPECT_FALSE(s.engineDataPresent);
}

// ===========================================================================
// Row gate + short-circuit semantics
// ===========================================================================

// The time field is the gate: a row that ends before the time column has no
// parseable time and is rejected outright.
TEST_F(CsvTelemetryParserCharacterizationTest, RowShorterThanTimeColumnIsRejected) {
    requireHeader("throttle_pct,time_s");  // colTime == 1
    CsvSample s;
    EXPECT_FALSE(parser.parseRow("50.0", 1.0, s, error));
}

// Rejected rows leave the OUT sample untouched (out is written only on
// success). A refactor that early-assigns partial state breaks this.
TEST_F(CsvTelemetryParserCharacterizationTest, RejectedRowsLeaveOutSampleUntouched) {
    requireHeader("time_s,throttle_pct,road_speed_kmh,gear,gear_selector,clutch_pct,"
                  "motor_torque_nm,brake_light,steering_angle_deg");

    CsvSample good;
    ASSERT_TRUE(parser.parseRow(
        "1.5,75.0,120.0,3,D,50.0,1840.5,1,-12.5", 1.0, good, error));
    ASSERT_TRUE(good.engineDataPresent);

    // Three distinct reject classes; the sample must survive all of them.
    const char* rejectedRows[] = {
        "",                    // blank line
        "#vs-start-from 30",   // hint line: no parseable time
        "1e8,0.0",             // outlier timestamp (> 1e7 s)
    };
    for (const char* row : rejectedRows) {
        SCOPED_TRACE(row);
        CsvSample candidate = good;
        EXPECT_FALSE(parser.parseRow(row, 1.0, candidate, error));
        EXPECT_DOUBLE_EQ(candidate.timeS, good.timeS);
        EXPECT_DOUBLE_EQ(candidate.throttle, good.throttle);
        EXPECT_DOUBLE_EQ(candidate.roadSpeedKmh, good.roadSpeedKmh);
        EXPECT_EQ(candidate.gear, good.gear);
        EXPECT_EQ(candidate.gearSelector, good.gearSelector);
        EXPECT_DOUBLE_EQ(candidate.clutchPct, good.clutchPct);
        EXPECT_DOUBLE_EQ(candidate.motorTorqueNm, good.motorTorqueNm);
        ASSERT_TRUE(candidate.brakeLight.has_value());
        EXPECT_TRUE(*candidate.brakeLight);
        ASSERT_TRUE(candidate.steeringAngleDeg.has_value());
        EXPECT_DOUBLE_EQ(*candidate.steeringAngleDeg, -12.5);
        EXPECT_TRUE(candidate.engineDataPresent);
        EXPECT_EQ(candidate.timeMs, good.timeMs);
    }
}

// The time gate is the ONLY row-level reject gate after the time parse:
// garbage in every engine field degrades each field to its default and the
// row is still accepted (engineDataPresent false).
TEST_F(CsvTelemetryParserCharacterizationTest, GarbageEngineFieldsStillAcceptRow) {
    requireHeader("time_s,throttle_pct,road_speed_kmh,gear,gear_selector,clutch_pct,"
                  "motor_torque_nm,brake_light,steering_angle_deg");

    CsvSample s;
    EXPECT_TRUE(parser.parseRow("1.5,x,y,z,,q,r,s", 1.0, s, error));
    EXPECT_DOUBLE_EQ(s.timeS, 1.5);
    EXPECT_DOUBLE_EQ(s.throttle, 0.0);
    EXPECT_DOUBLE_EQ(s.roadSpeedKmh, -2.0);
    EXPECT_EQ(s.gear, -1);
    EXPECT_EQ(s.gearSelector, "");
    EXPECT_DOUBLE_EQ(s.clutchPct, -1.0);
    EXPECT_DOUBLE_EQ(s.motorTorqueNm, 0.0);
    EXPECT_FALSE(s.brakeLight.has_value());
    EXPECT_FALSE(s.steeringAngleDeg.has_value());
    EXPECT_FALSE(s.engineDataPresent);
}

TEST_F(CsvTelemetryParserCharacterizationTest, TrailingExtraColumnsBeyondHeaderIgnored) {
    requireHeader("time_s,throttle_pct");
    CsvSample s;
    EXPECT_TRUE(parser.parseRow("1.0,50.0,999,junk", 1.0, s, error));
    EXPECT_DOUBLE_EQ(s.throttle, 0.5);
    EXPECT_DOUBLE_EQ(s.timeS, 1.0);
}

TEST_F(CsvTelemetryParserCharacterizationTest, RowMissingTrailingColumnsKeepsDefaults) {
    requireHeader("time_s,throttle_pct,road_speed_kmh");
    CsvSample s;
    EXPECT_TRUE(parser.parseRow("1.0,50.0", 1.0, s, error));
    EXPECT_DOUBLE_EQ(s.throttle, 0.5);
    EXPECT_DOUBLE_EQ(s.roadSpeedKmh, -2.0);  // column absent from this row
    EXPECT_TRUE(s.engineDataPresent);        // throttle still counts
}

// '#vs-...' hint lines (vehicle-sim skip hints) have no parseable time:
// parseRow rejects them, they do NOT count as outliers, and the parser is
// not stalled — the next row still surfaces.
TEST_F(CsvTelemetryParserCharacterizationTest, HintLineSkippedWithoutCountingOrStalling) {
    requireHeader("time_s,throttle_pct");
    CsvSample s;
    EXPECT_FALSE(parser.parseRow("#vs-start-from 30", 1.0, s, error));

    EXPECT_TRUE(parser.parseRow("1.0,50.0", 1.0, s, error));
    EXPECT_DOUBLE_EQ(s.throttle, 0.5);

    const std::string captured = captureStderr(
        [this] { parser.emitRejectionSummary(); });
    EXPECT_TRUE(captured.empty())
        << "hint lines must not be counted as outliers, got: " << captured;
}

// ===========================================================================
// Numeric semantics (stod/stoi prefix parsing)
// ===========================================================================

// Trailing junk after a valid numeric prefix is ACCEPTED with the prefix
// value — std::stod/stoi partial parse. A refactor onto strict from_chars
// would reject these rows; that is a behaviour change, not a refactor.
TEST_F(CsvTelemetryParserCharacterizationTest, NumericPrefixToleranceForTrailingJunk) {
    requireHeader("time_s,throttle_pct,gear");
    CsvSample s;
    EXPECT_TRUE(parser.parseRow("1.5xyz,75abc,3x", 1.0, s, error));
    EXPECT_DOUBLE_EQ(s.timeS, 1.5);
    EXPECT_DOUBLE_EQ(s.throttle, 0.75);
    EXPECT_EQ(s.gear, 3);
}

TEST_F(CsvTelemetryParserCharacterizationTest, GearDecimalValueTruncatesViaStoi) {
    requireHeader("time_s,gear");
    CsvSample s;
    EXPECT_TRUE(parser.parseRow("1.0,2.9", 1.0, s, error));
    EXPECT_EQ(s.gear, 2);  // stoi prefix parse, no rounding
}

TEST_F(CsvTelemetryParserCharacterizationTest, OutOfRangeNumbersFallBackToDefaults) {
    requireHeader("time_s,throttle_pct,gear");
    CsvSample s;
    // 99,999,999,999 overflows int (stoi throws out_of_range);
    // 1e999 overflows double (stod throws out_of_range).
    EXPECT_TRUE(parser.parseRow("1.0,1e999,99999999999", 1.0, s, error));
    EXPECT_DOUBLE_EQ(s.throttle, 0.0);
    EXPECT_EQ(s.gear, -1);
    EXPECT_FALSE(s.engineDataPresent);
}

TEST_F(CsvTelemetryParserCharacterizationTest, SignedAndExponentNotationAccepted) {
    requireHeader("time_s,throttle_pct");
    CsvSample s;
    EXPECT_TRUE(parser.parseRow("+1.0,+50.0", 1.0, s, error));
    EXPECT_DOUBLE_EQ(s.timeS, 1.0);
    EXPECT_DOUBLE_EQ(s.throttle, 0.5);

    CsvSample exp;
    EXPECT_TRUE(parser.parseRow("2.0,1e2", 1.0, exp, error));
    EXPECT_DOUBLE_EQ(exp.throttle, 1.0);  // 100 -> clamped to 1.0
}

// ===========================================================================
// Field-decode asymmetries — the behaviours a uniform decode helper breaks
// ===========================================================================

// Clutch is the anti-throttle: a negative value FAILS the v >= 0 guard and
// leaves the -1 sentinel (throttle would clamp to 0); >100 clamps to 1.0.
TEST_F(CsvTelemetryParserCharacterizationTest, ClutchNegativeIsNotParsedNotClamped) {
    requireHeader("time_s,clutch_pct");
    CsvSample s;
    EXPECT_TRUE(parser.parseRow("1.0,-5.0", 1.0, s, error));
    EXPECT_DOUBLE_EQ(s.clutchPct, -1.0);  // sentinel, NOT 0.0
    EXPECT_FALSE(s.engineDataPresent);    // the guard failed: no engine data

    CsvSample high;
    EXPECT_TRUE(parser.parseRow("2.0,150.0", 1.0, high, error));
    EXPECT_DOUBLE_EQ(high.clutchPct, 1.0);  // clamped
    EXPECT_TRUE(high.engineDataPresent);
}

// Steering is display-only: it never marks the row as an operating point,
// even when it is the only populated field besides time.
TEST_F(CsvTelemetryParserCharacterizationTest, SteeringOnlyRowIsNotEngineData) {
    requireHeader("time_s,steering_angle_deg");
    CsvSample s;
    EXPECT_TRUE(parser.parseRow("1.0,-12.5", 1.0, s, error));
    ASSERT_TRUE(s.steeringAngleDeg.has_value());
    EXPECT_DOUBLE_EQ(*s.steeringAngleDeg, -12.5);
    EXPECT_FALSE(s.engineDataPresent);
}

// An out-of-domain brake_light integer parses as an int, marks the row as
// engine data, but leaves the value absent (neither 1 nor 0). The stoi
// prefix means "1.9" reads as 1 (light ON).
TEST_F(CsvTelemetryParserCharacterizationTest, BrakeLightOutOfDomainMarksEngineDataWithoutValue) {
    requireHeader("time_s,brake_light");
    CsvSample s;
    EXPECT_TRUE(parser.parseRow("1.0,2", 1.0, s, error));
    EXPECT_FALSE(s.brakeLight.has_value());  // neither on nor off
    EXPECT_TRUE(s.engineDataPresent);        // ...but the row is telemetry

    CsvSample prefix;
    EXPECT_TRUE(parser.parseRow("2.0,1.9", 1.0, prefix, error));
    ASSERT_TRUE(prefix.brakeLight.has_value());
    EXPECT_TRUE(*prefix.brakeLight);
}

// The selector is a free-text cell: any non-empty trimmed string is stored
// verbatim (no PRNDL validation here); a whitespace-only cell is absent.
TEST_F(CsvTelemetryParserCharacterizationTest, GearSelectorAcceptsAnyNonEmptyString) {
    requireHeader("time_s,gear_selector,throttle_pct");
    CsvSample s;
    EXPECT_TRUE(parser.parseRow("1.0,X9,50.0", 1.0, s, error));
    EXPECT_EQ(s.gearSelector, "X9");
    EXPECT_TRUE(s.engineDataPresent);

    CsvSample blank;
    EXPECT_TRUE(parser.parseRow("2.0,  ,60.0", 1.0, blank, error));
    EXPECT_EQ(blank.gearSelector, "");
    EXPECT_TRUE(blank.engineDataPresent);  // throttle carries the row
}

// ===========================================================================
// Time handling: epoch threshold, raw timeMs, outlier guard, anchoring
// ===========================================================================

// The epoch threshold is 1e12 ms with >= semantics. One millisecond below
// it, the value takes the relative path and is outlier-rejected; at it, the
// value rebases to t=0. This pins kEpochMsThreshold through behaviour.
TEST_F(CsvTelemetryParserCharacterizationTest, EpochThresholdBoundaryAt1e12Ms) {
    requireHeader("timestamp_ms,throttle_pct");
    ASSERT_TRUE(parser.header().timeInMs);

    CsvSample below;
    EXPECT_FALSE(parser.parseRow("999999999999,10.0", 1000.0, below, error))
        << "999999999999 ms must take the relative path and be outlier-rejected";

    CsvSample at;
    EXPECT_TRUE(parser.parseRow("1000000000000,10.0", 1000.0, at, error))
        << "1e12 ms must take the epoch path";
    EXPECT_DOUBLE_EQ(at.timeS, 0.0);
    EXPECT_EQ(at.timeMs, 1000000000000);
}

// Epoch rows preserve the RAW epoch ms in timeMs (latency math downstream);
// relative-ms and time_s rows leave it at the -1 default.
TEST_F(CsvTelemetryParserCharacterizationTest, EpochRowsPreserveRawTimeMs) {
    requireHeader("timestamp_ms,throttle_pct");
    CsvSample epoch;
    EXPECT_TRUE(parser.parseRow("1786538088200,10.0", 1000.0, epoch, error));
    EXPECT_EQ(epoch.timeMs, 1786538088200);

    CsvSample relative;
    EXPECT_TRUE(parser.parseRow("1500,20.0", 1000.0, relative, error));
    EXPECT_EQ(relative.timeMs, -1);
    EXPECT_DOUBLE_EQ(relative.timeS, 1.5);

    CsvTelemetryParser secondsParser;
    std::string err;
    ASSERT_TRUE(secondsParser.parseHeader("time_s,throttle_pct", err));
    CsvSample seconds;
    EXPECT_TRUE(secondsParser.parseRow("2.5,30.0", 1.0, seconds, err));
    EXPECT_EQ(seconds.timeMs, -1);
}

// The outlier guard is strict (>): exactly 1e7 seconds is accepted.
TEST_F(CsvTelemetryParserCharacterizationTest, OutlierBoundaryStrictAt1e7Seconds) {
    requireHeader("time_s,throttle_pct");

    CsvSample at;
    EXPECT_TRUE(parser.parseRow("10000000", 1.0, at, error));
    EXPECT_DOUBLE_EQ(at.timeS, 1e7);

    CsvSample above;
    EXPECT_FALSE(parser.parseRow("10000000.5", 1.0, above, error));

    const std::string captured = captureStderr(
        [this] { parser.emitRejectionSummary(); });
    EXPECT_NE(captured.find("skipped 1 row(s)"), std::string::npos) << captured;
}

// No monotonicity fix-up: an epoch row earlier than the anchor yields a
// NEGATIVE timeS. The math is preserved verbatim.
TEST_F(CsvTelemetryParserCharacterizationTest, NonMonotonicEpochRowsYieldNegativeTime) {
    requireHeader("timestamp_ms,throttle_pct");
    CsvSample first;
    ASSERT_TRUE(parser.parseRow("1786538088200,10.0", 1000.0, first, error));
    EXPECT_DOUBLE_EQ(first.timeS, 0.0);

    CsvSample earlier;
    EXPECT_TRUE(parser.parseRow("1786538087700,20.0", 1000.0, earlier, error));
    EXPECT_DOUBLE_EQ(earlier.timeS, -0.5);
}

// The epoch path has no upper bound: arbitrarily huge epoch-ms values are
// accepted (the >1e7 guard only protects the relative path).
TEST_F(CsvTelemetryParserCharacterizationTest, HugeEpochMsValuesAreAccepted) {
    requireHeader("timestamp_ms,throttle_pct");
    CsvSample s;
    EXPECT_TRUE(parser.parseRow("5000000000000000,10.0", 1000.0, s, error));
    EXPECT_DOUBLE_EQ(s.timeS, 0.0);  // first epoch row anchors t=0

    CsvSample later;
    EXPECT_TRUE(parser.parseRow("5000000000001000,20.0", 1000.0, later, error));
    EXPECT_DOUBLE_EQ(later.timeS, 1.0);
}

// Streams may mix epoch and relative ms rows: the branch is chosen per row,
// and the anchor set by the first epoch row persists across relative rows.
TEST_F(CsvTelemetryParserCharacterizationTest, MixedEpochAndRelativeMsRowsShareTheAnchor) {
    requireHeader("timestamp_ms,throttle_pct");

    CsvSample epoch;
    ASSERT_TRUE(parser.parseRow("1786538088200,10.0", 1000.0, epoch, error));
    EXPECT_DOUBLE_EQ(epoch.timeS, 0.0);

    CsvSample relative;
    ASSERT_TRUE(parser.parseRow("1500,20.0", 1000.0, relative, error));
    EXPECT_DOUBLE_EQ(relative.timeS, 1.5);  // divisor path, not epoch math

    // A later epoch row still measures from the FIRST epoch row's anchor.
    CsvSample epochAgain;
    ASSERT_TRUE(parser.parseRow("1786538089200,30.0", 1000.0, epochAgain, error));
    EXPECT_DOUBLE_EQ(epochAgain.timeS, 1.0);
}

// Re-parsing a header re-anchors the epoch base: the first epoch row of the
// SECOND stream is t=0 again, not relative to the first stream's anchor.
TEST_F(CsvTelemetryParserCharacterizationTest, ReparseReanchorsEpochBase) {
    requireHeader("timestamp_ms,throttle_pct");

    CsvSample first;
    ASSERT_TRUE(parser.parseRow("1786538088200,10.0", 1000.0, first, error));
    EXPECT_DOUBLE_EQ(first.timeS, 0.0);
    CsvSample second;
    ASSERT_TRUE(parser.parseRow("1786538089200,20.0", 1000.0, second, error));
    EXPECT_DOUBLE_EQ(second.timeS, 1.0);

    // New stream: same header, different epoch base.
    ASSERT_TRUE(parser.parseHeader("timestamp_ms,throttle_pct", error));
    CsvSample restart;
    EXPECT_TRUE(parser.parseRow("1786538089700,30.0", 1000.0, restart, error));
    EXPECT_DOUBLE_EQ(restart.timeS, 0.0);  // would be 1.5 with a stale anchor
}

// The outlier counter is per-stream: re-parsing a header clears it, so the
// second stream's summary starts from zero.
TEST_F(CsvTelemetryParserCharacterizationTest, ReparseClearsOutlierCounter) {
    requireHeader("time_s,throttle_pct");
    CsvSample s;
    EXPECT_FALSE(parser.parseRow("20000000.0", 1.0, s, error));  // outlier
    EXPECT_FALSE(parser.parseRow("20000001.0", 1.0, s, error));  // outlier

    ASSERT_TRUE(parser.parseHeader("time_s,throttle_pct", error));
    const std::string captured = captureStderr(
        [this] { parser.emitRejectionSummary(); });
    EXPECT_TRUE(captured.empty())
        << "re-parsed header must reset the outlier count, got: " << captured;
}

// ===========================================================================
// S5145 path: emitRejectionSummary / rejectedOutlierRows_ -> fprintf
// ===========================================================================

// The counter counts ONLY >1e7 outlier rejects. Blank lines, hint lines,
// short rows and unparseable-time rows are also rejected but must NOT be
// reported — a refactor merging the reject paths would over-count.
TEST_F(CsvTelemetryParserCharacterizationTest, SummaryCountsOnlyOutlierRejects) {
    requireHeader("throttle_pct,time_s");  // colTime == 1

    CsvSample s;
    EXPECT_FALSE(parser.parseRow("", 1.0, s, error));               // blank
    EXPECT_FALSE(parser.parseRow("#vs-start-from 30", 1.0, s, error));  // hint
    EXPECT_FALSE(parser.parseRow("50.0,abc", 1.0, s, error));        // bad time
    EXPECT_FALSE(parser.parseRow("50.0", 1.0, s, error));           // short row
    // Exactly two outlier rejects.
    EXPECT_FALSE(parser.parseRow("0.0,20000000.0", 1.0, s, error));
    EXPECT_FALSE(parser.parseRow("0.0,20000001.0", 1.0, s, error));

    const std::string captured = captureStderr(
        [this] { parser.emitRejectionSummary(); });
    EXPECT_NE(captured.find("skipped 2 row(s)"), std::string::npos)
        << "only the two outlier rows may be counted, got: " << captured;
}

// The summary is not one-shot: every call while the count is non-zero
// prints. (The once-per-stream contract lives with the CALLER.)
TEST_F(CsvTelemetryParserCharacterizationTest, SummaryEmitsOnEveryCallUntilReset) {
    requireHeader("time_s,throttle_pct");
    CsvSample s;
    EXPECT_FALSE(parser.parseRow("20000000.0", 1.0, s, error));  // one outlier

    const std::string first = captureStderr(
        [this] { parser.emitRejectionSummary(); });
    EXPECT_NE(first.find("skipped 1 row(s)"), std::string::npos) << first;

    const std::string second = captureStderr(
        [this] { parser.emitRejectionSummary(); });
    EXPECT_NE(second.find("skipped 1 row(s)"), std::string::npos) << second;
}

// ===========================================================================
// engineDataPresent matrix: each engine field alone marks the row
// ===========================================================================

// Every engine-data field, alone with time, marks the row as an operating
// point (steering is the display-only exception, pinned above). A helper
// extraction that drops one field's engineData flag silently degrades the
// arrival prime.
TEST_F(CsvTelemetryParserCharacterizationTest, EachEngineFieldAloneMarksEngineData) {
    struct FieldRow {
        const char* header;
        const char* row;
    };
    const std::vector<FieldRow> cases = {
        {"time_s,throttle_pct", "1.0,50.0"},
        {"time_s,road_speed_kmh", "1.0,30.0"},
        {"time_s,gear", "1.0,3"},
        {"time_s,gear_selector", "1.0,D"},
        {"time_s,clutch_pct", "1.0,50.0"},
        {"time_s,motor_torque_nm", "1.0,100.0"},
        {"time_s,brake_light", "1.0,1"},
    };
    for (const auto& c : cases) {
        SCOPED_TRACE(std::string(c.header) + " <- " + c.row);
        CsvTelemetryParser p;
        std::string err;
        ASSERT_TRUE(p.parseHeader(c.header, err));
        CsvSample s;
        EXPECT_TRUE(p.parseRow(c.row, 1.0, s, err));
        EXPECT_TRUE(s.engineDataPresent);
    }
}

}  // namespace
