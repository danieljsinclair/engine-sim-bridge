// CsvTelemetryParser.cpp
#include "input/CsvTelemetryParser.h"

#include <algorithm>
#include <cctype>
#include <sstream>

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

constexpr ColumnAlias kColumnAliases[] = {
    {&CsvHeader::colTime,         &CsvHeader::timeInMs, "timestamp_utc_ms timestamp_ms ts_ms"},
    {&CsvHeader::colTime,         nullptr,              "time_s time t timecode"},
    {&CsvHeader::colThrottle,     nullptr,              "throttle_pct throttle_percent throttle"},
    {&CsvHeader::colRoad,         nullptr,              "road_speed_kmh road_speed speed_kmh speed"},
    {&CsvHeader::colGear,         nullptr,              "gear"},
    {&CsvHeader::colGearSelector, nullptr,              "gear_selector gearselector"},
    {&CsvHeader::colClutch,       nullptr,              "clutch_pct clutch"},
    {&CsvHeader::colMotorTorque,  nullptr,              "motor_torque_nm motor_torque torque_nm"},
    {&CsvHeader::colBrakeLight,   nullptr,              "brake_light brakelight"},
};

bool matchesAlias(const std::string& name, const std::string& aliases) {
    for (const auto& alias : split(aliases, ' ')) {
        if (alias == name) return true;
    }
    return false;
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

} // namespace

bool CsvTelemetryParser::parseHeader(const std::string& headerLine, std::string& errorMsg) {
    const std::string trimmed = trim(headerLine);
    if (trimmed.empty()) {
        errorMsg = "Empty CSV header line";
        return false;
    }

    auto fields = split(trimmed, ',');
    header_ = CsvHeader{};
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

    auto fields = split(trimmed, ',');
    CsvSample s;

    double v = 0.0;
    if (header_.colTime >= 0 && header_.colTime < static_cast<int>(fields.size()) &&
        parseDouble(fields[header_.colTime], v)) {
        s.timeS = v / timeDivisor;
    } else {
        return false;  // skip rows with unparseable time
    }

    if (header_.colThrottle >= 0 && header_.colThrottle < static_cast<int>(fields.size()) &&
        parseDouble(fields[header_.colThrottle], v)) {
        s.throttle = std::clamp(v / 100.0, 0.0, 1.0);
    }

    if (header_.colRoad >= 0 && header_.colRoad < static_cast<int>(fields.size()) &&
        parseDouble(fields[header_.colRoad], v) && v >= 0.0) {
        s.roadSpeedKmh = v;
    }

    if (int gi; header_.colGear >= 0 && header_.colGear < static_cast<int>(fields.size()) &&
        parseInt(fields[header_.colGear], gi)) {
        s.gear = gi;
    }

    if (header_.colGearSelector >= 0 && header_.colGearSelector < static_cast<int>(fields.size())) {
        s.gearSelector = trim(fields[header_.colGearSelector]);
    }

    if (header_.colClutch >= 0 && header_.colClutch < static_cast<int>(fields.size()) &&
        parseDouble(fields[header_.colClutch], v) && v >= 0.0) {
        s.clutchPct = std::clamp(v / 100.0, 0.0, 1.0);
    }

    if (header_.colMotorTorque >= 0 && header_.colMotorTorque < static_cast<int>(fields.size()) &&
        parseDouble(fields[header_.colMotorTorque], v)) {
        s.motorTorqueNm = v;
    }

    // brake_light: a binary column. "1" = on, "0" = off; blank/unparseable/
    // out-of-domain values leave the field absent (nullopt) — never a guess.
    if (header_.colBrakeLight >= 0 &&
        header_.colBrakeLight < static_cast<int>(fields.size())) {
        int brakeLight = 0;
        if (parseInt(fields[header_.colBrakeLight], brakeLight)) {
            if (brakeLight == 1)      s.brakeLight = true;
            else if (brakeLight == 0) s.brakeLight = false;
        }
    }

    out = s;
    return true;
}

} // namespace input
