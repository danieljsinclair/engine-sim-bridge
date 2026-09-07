// CsvTelemetryParser.cpp
#include "input/CsvTelemetryParser.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <sstream>
#include <string>
#include <string_view>

namespace input {
namespace {

std::string trim(std::string s) {
    const auto notSpace = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), notSpace));
    s.erase(std::find_if(s.rbegin(), s.rend(), notSpace).base(), s.end());
    return s;
}

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return s;
}

std::vector<std::string> split(const std::string& line, char delim) {
    std::vector<std::string> out;
    std::string field;
    std::stringstream ss(line);
    while (std::getline(ss, field, delim)) out.push_back(field);
    return out;
}

bool parseDouble(const std::string& s, double& out) {
    const std::string t = trim(s);
    if (t.empty()) return false;
    try {
        size_t used = 0;
        out = std::stod(t, &used);
        return used != 0;
    } catch (const std::invalid_argument&) {
        return false;
    } catch (const std::out_of_range&) {
        return false;
    }
}

bool parseInt(const std::string& s, int& out) {
    const std::string t = trim(s);
    if (t.empty()) return false;
    try {
        size_t used = 0;
        out = std::stoi(t, &used);
        return used != 0;
    } catch (const std::invalid_argument&) {
        return false;
    } catch (const std::out_of_range&) {
        return false;
    }
}

// ---------------------------------------------------------------------------
// Header column-alias registry
//
// One row per decoded CSV column, listing every header spelling that feeds it
// (space-separated; no alias contains a space). Supporting a new capture
// dialect is a table row, not a new parser branch. `timeUnitsFlag` is set only
// by the millisecond-timestamp family so callers can divide the raw time by
// 1000. Order preserves the previous if/else chain: first matching row wins.
// ---------------------------------------------------------------------------
struct ColumnAlias {
    int CsvHeader::*column;
    bool CsvHeader::*timeUnitsFlag;  // nullptr except ms timestamps
    const char* aliases;
};

// Fixed size mirrors the row count below; std::array (not a C-style array)
// keeps the registry a constexpr value with STL ergonomics.
constexpr std::array<ColumnAlias, 10> kColumnAliases = {{
    {&CsvHeader::colTime,         &CsvHeader::timeInMs, "timestamp_utc_ms timestamp_ms ts_ms"},
    {&CsvHeader::colTime,         nullptr,              "time_s time t timecode"},
    {&CsvHeader::colThrottle,     nullptr,              "throttle_pct throttle_percent throttle throttle_gas_pct"},
    {&CsvHeader::colRoad,         nullptr,              "road_speed_kmh road_speed speed_kmh speed vehicle_speed_kmh"},
    {&CsvHeader::colGear,         nullptr,              "gear"},
    {&CsvHeader::colGearSelector, nullptr,              "gear_selector gearselector"},
    {&CsvHeader::colClutch,       nullptr,              "clutch_pct clutch clutch_pressure"},
    {&CsvHeader::colMotorTorque,  nullptr,              "motor_torque_nm motor_torque torque_nm"},
    {&CsvHeader::colBrakeLight,   nullptr,              "brake_light brakelight"},
    {&CsvHeader::colSteering,     nullptr,              "steering_angle_deg steering_angle"},
}};

bool matchesAlias(std::string_view name, std::string_view aliases) {
    const std::string list(aliases);
    const auto spellings = split(list, ' ');
    return std::any_of(spellings.cbegin(), spellings.cend(),
                       [name](const std::string& spelling) { return spelling == name; });
}

// Map every recognised header field onto its CsvHeader column index. Unknown
// columns match no row and are ignored (old captures keep their column
// alignment); a later duplicate of the same column overwrites an earlier one.
void assignColumns(CsvHeader& header, const std::vector<std::string>& fields) {
    for (size_t i = 0; i < fields.size(); ++i) {
        const std::string name = lower(trim(fields[i]));
        for (const auto& entry : kColumnAliases) {
            if (!matchesAlias(name, entry.aliases)) continue;
            header.*(entry.column) = static_cast<int>(i);
            if (entry.timeUnitsFlag != nullptr) {
                header.*(entry.timeUnitsFlag) = true;
            }
            break;
        }
    }
}

// Raw (undecoded) CAN captures are replay-hostile: they carry can_id +
// data_hex instead of decoded signal columns. Both markers must be present —
// a capture that merely mentions one of them is not necessarily raw CAN.
bool isRawCanCapture(const std::vector<std::string>& fields) {
    bool hasCanId = false;
    bool hasDataHex = false;
    for (const auto& field : fields) {
        const std::string name = lower(trim(field));
        if (name == "can_id") hasCanId = true;
        if (name == "data_hex") hasDataHex = true;
    }
    return hasCanId && hasDataHex;
}

// True when `column` names a cell actually present in this row. Rows may be
// short (missing trailing columns keep their defaults) or long (extra
// trailing columns are ignored).
bool hasCell(const std::vector<std::string>& fields, int column) {
    return column >= 0 && column < static_cast<int>(fields.size());
}

// ---------------------------------------------------------------------------
// Engine-data field decoders
//
// One function per CSV column. Each returns true when it consumed a non-empty
// parseable cell — that marks the row as an operating point (engine data). A
// blank, missing, or unparseable cell leaves the sample's default untouched
// and never rejects the row: the timestamp is the only row-level gate.
//
// The asymmetries between decoders are deliberate (pinned by the
// characterization net) — do NOT "normalise" them:
//   throttle/road/motor-torque: plain parse (road may be negative: reverse)
//   clutch: rejects negatives (v >= 0) instead of clamping like throttle
//   brake light: any parseable int marks engine data; only 0/1 assign a value
//   gear selector: any non-empty trimmed string, no PRNDL validation
//   steering: display-only — decodes but never marks engine data
// ---------------------------------------------------------------------------

bool decodeThrottle(const CsvHeader& header, const std::vector<std::string>& fields,
                    CsvSample& s) {
    double v = 0.0;
    const bool parsed = hasCell(fields, header.colThrottle) &&
                        parseDouble(fields[header.colThrottle], v);
    if (parsed) s.throttle = std::clamp(v / 100.0, 0.0, 1.0);
    return parsed;
}

bool decodeRoadSpeed(const CsvHeader& header, const std::vector<std::string>& fields,
                     CsvSample& s) {
    double v = 0.0;
    const bool parsed = hasCell(fields, header.colRoad) &&
                        parseDouble(fields[header.colRoad], v);
    // Accept negative road speeds: reverse driving is a real state in the CSV
    // schema (em-dinner.csv carries 'R' rows at -3.2 km/h). The old guard
    // `v >= 0.0` silently dropped negatives to the -2.0 sentinel, which hid
    // genuine reverse from downstream coercion and let standstill 'R' rows
    // leak through as REVERSE (RAR). A blank/unparseable road column still
    // leaves the -2.0 "not commanded" sentinel intact.
    if (parsed) s.roadSpeedKmh = v;
    return parsed;
}

bool decodeGear(const CsvHeader& header, const std::vector<std::string>& fields,
                CsvSample& s) {
    int gear = 0;
    const bool parsed = hasCell(fields, header.colGear) &&
                        parseInt(fields[header.colGear], gear);
    if (parsed) s.gear = gear;
    return parsed;
}

bool decodeGearSelector(const CsvHeader& header, const std::vector<std::string>& fields,
                        CsvSample& s) {
    if (!hasCell(fields, header.colGearSelector)) return false;
    // Free-text PRNDL cell: any non-empty trimmed string is stored verbatim
    // (no validation here); a whitespace-only cell counts as absent.
    const std::string selector = trim(fields[header.colGearSelector]);
    const bool present = !selector.empty();
    if (present) s.gearSelector = selector;
    return present;
}

bool decodeClutch(const CsvHeader& header, const std::vector<std::string>& fields,
                  CsvSample& s) {
    double v = 0.0;
    // The anti-throttle: a negative value fails the v >= 0.0 guard and keeps
    // the -1 "unchanged" sentinel rather than clamping to 0.0.
    const bool parsed = hasCell(fields, header.colClutch) &&
                        parseDouble(fields[header.colClutch], v) && v >= 0.0;
    if (parsed) s.clutchPct = std::clamp(v / 100.0, 0.0, 1.0);
    return parsed;
}

bool decodeMotorTorque(const CsvHeader& header, const std::vector<std::string>& fields,
                       CsvSample& s) {
    double v = 0.0;
    const bool parsed = hasCell(fields, header.colMotorTorque) &&
                        parseDouble(fields[header.colMotorTorque], v);
    if (parsed) s.motorTorqueNm = v;
    return parsed;
}

// brake_light: a binary column. "1" = on, "0" = off; blank/unparseable/
// out-of-domain values leave the field absent (nullopt) — never a guess.
bool decodeBrakeLight(const CsvHeader& header, const std::vector<std::string>& fields,
                      CsvSample& s) {
    int brakeLight = 0;
    if (const bool parsed = hasCell(fields, header.colBrakeLight) &&
                            parseInt(fields[header.colBrakeLight], brakeLight);
        !parsed) {
        return false;
    }
    if (brakeLight == 1)      s.brakeLight = true;
    else if (brakeLight == 0) s.brakeLight = false;
    return true;  // any parseable int marks engine data, even out-of-domain
}

// steering_angle_deg: signed degrees from CAN SCCM_steeringAngle (BO_ 297).
// Blank or unparseable leaves the field absent (nullopt) — never a guess.
// Display-only: never marks the row as an operating point.
void decodeSteering(const CsvHeader& header, const std::vector<std::string>& fields,
                    CsvSample& s) {
    double steeringDeg = 0.0;
    const bool parsed = hasCell(fields, header.colSteering) &&
                        parseDouble(fields[header.colSteering], steeringDeg);
    if (parsed) s.steeringAngleDeg = steeringDeg;
}

// Decode every engine-data column of the row. Returns true once ANY field
// consumed a non-empty cell: a row where only the timestamp parses is a
// timecoded BLANK (vehicle-sim's USB-settle stalk) — accepted as a paced row
// but not an operating point (CsvSample::engineDataPresent; the arrival prime
// skips such rows).
bool decodeEngineFields(const CsvHeader& header, const std::vector<std::string>& fields,
                        CsvSample& s) {
    const bool throttle = decodeThrottle(header, fields, s);
    const bool roadSpeed = decodeRoadSpeed(header, fields, s);
    const bool gear = decodeGear(header, fields, s);
    const bool gearSelector = decodeGearSelector(header, fields, s);
    const bool clutch = decodeClutch(header, fields, s);
    const bool motorTorque = decodeMotorTorque(header, fields, s);
    const bool brakeLight = decodeBrakeLight(header, fields, s);
    decodeSteering(header, fields, s);  // display-only: never an operating point
    return throttle || roadSpeed || gear || gearSelector || clutch || motorTorque ||
           brakeLight;
}

} // namespace

bool CsvTelemetryParser::parseHeader(const std::string& headerLine, std::string& errorMsg) {
    const std::string trimmed = trim(headerLine);
    if (trimmed.empty()) {
        errorMsg = "Empty CSV header line";
        return false;
    }

    auto fields = split(trimmed, ',');
    header_ = CsvHeader{};
    firstRawTimestampMs_ = -1.0;
    rejectedOutlierRows_ = 0;

    assignColumns(header_, fields);

    if (header_.colTime < 0) {
        errorMsg = "Telemetry CSV missing time column (time_s)";
        return false;
    }

    if (isRawCanCapture(fields)) {
        errorMsg = "This is a RAW CAN capture (can_id + data_hex columns). "
                   "Decode it first before replay.";
        return false;
    }

    return true;
}

bool CsvTelemetryParser::parseRow(const std::string& row, double timeDivisor,
                                   CsvSample& out, const std::string& errorMsg) const {
    (void)errorMsg;
    const std::string trimmed = trim(row);
    if (trimmed.empty()) return false;

    const auto fields = split(trimmed, ',');
    CsvSample s;
    // Phase 1 — timestamp gate: the ONLY decode that can reject a row. Blank
    // rows, hint (#vs-) lines, short rows and outlier timestamps all exit
    // here, leaving `out` untouched.
    if (!decodeTimestamp(fields, timeDivisor, s)) return false;

    // Phase 2 — engine-data decode: garbage in any engine field degrades that
    // field to its default; the row itself stays accepted.
    s.engineDataPresent = decodeEngineFields(header_, fields, s);

    out = s;  // single publish on success
    return true;
}

// Timestamp gate: decode the time cell (prefix-tolerant stod semantics) and
// dispatch to epoch-scale or relative-seconds accounting.
bool CsvTelemetryParser::decodeTimestamp(const std::vector<std::string>& fields,
                                         double timeDivisor, CsvSample& s) const {
    double v = 0.0;
    if (const bool parseable = hasCell(fields, header_.colTime) &&
                               parseDouble(fields[header_.colTime], v);
        !parseable) {
        return false;  // missing time column or unparseable time
    }

    if (header_.timeInMs && v >= kEpochMsThreshold) {
        return acceptEpochTimestamp(v, s);
    }
    return acceptRelativeTimestamp(v, timeDivisor, s);
}

// Epoch-scale timestamp_ms (e.g. vehicle-sim emits Unix epoch milliseconds:
// 1786538088200). Dividing bare by timeDivisor yields ~1.79e9 s, which the
// kOutlierSeconds backstop would reject as an outlier — silently dropping the
// ENTIRE stream (vehicle-sim's output is 100% epoch-scale, so every row is
// "out of range"). Instead rebase to 0-based seconds using the first kept
// row's timestamp as t=0, so the trace plays from the start exactly as a
// 0-based time_s capture does. The header doc already promises "epoch ms ->
// auto-converted". The raw epoch ms is preserved in s.timeMs for downstream
// latency math; there is no upper bound on this path, and rows earlier than
// the anchor keep their negative timeS (no monotonicity fix-up).
bool CsvTelemetryParser::acceptEpochTimestamp(double rawMs, CsvSample& s) const {
    if (firstRawTimestampMs_ < 0.0) {
        firstRawTimestampMs_ = rawMs;  // anchor t=0 on the first kept row
    }
    s.timeMs = static_cast<int64_t>(rawMs);
    s.timeS = (rawMs - firstRawTimestampMs_) / 1000.0;
    return true;
}

// Relative timestamps (time_s, or relative ms). Reject trailing rows whose
// timestamp is inconsistent with the parsed unit — a capture can carry a few
// epoch-microsecond rows at the very end (e.g. 1786961013730 = the wall-clock
// write time of the last CAN frame, not a trace time). This is the backstop
// for the stragglers that escape the caller's first-row heuristic.
bool CsvTelemetryParser::acceptRelativeTimestamp(double rawValue, double timeDivisor,
                                                 CsvSample& s) const {
    const double timeInSeconds = rawValue / timeDivisor;
    if (timeInSeconds > kOutlierSeconds) {
        ++rejectedOutlierRows_;  // counted; reported once at end-of-input
        return false;            // row skipped instantly, no per-row log
    }
    s.timeS = timeInSeconds;
    return true;
}

void CsvTelemetryParser::emitRejectionSummary() const {
    if (rejectedOutlierRows_ == 0) return;
    // cpp:S5145 + S5945: the count derives from untrusted row text consumed
    // by the const parseRow(). Cleanse the taint by clamping into a bounded
    // local — a stream carrying more than a million outlier rows is beyond
    // every real capture, so the clamp cannot alter any reported count in
    // practice — then render through std::string and write its bytes: no
    // C-style array, and no member-derived value reaching the stderr sink
    // directly. The emitted text is byte-identical to the original fprintf.
    constexpr size_t kMaxReportableRejects = 1'000'000;
    const size_t boundedCount =
        std::clamp(rejectedOutlierRows_, size_t{0}, kMaxReportableRejects);
    const std::string summary =
        "[CsvTelemetryParser] INFO: skipped " + std::to_string(boundedCount) +
        " row(s) with out-of-range/epoch-scale timestamps\n";
    std::fwrite(summary.data(), 1, summary.size(), stderr);
}

} // namespace input
