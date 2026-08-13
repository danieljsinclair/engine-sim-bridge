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

} // namespace

bool CsvTelemetryParser::parseHeader(const std::string& headerLine, std::string& errorMsg) {
    const std::string trimmed = trim(headerLine);
    if (trimmed.empty()) {
        errorMsg = "Empty CSV header line";
        return false;
    }

    auto fields = split(trimmed, ',');
    header_ = CsvHeader{};

    for (size_t i = 0; i < fields.size(); ++i) {
        const std::string name = lower(trim(fields[i]));
        if (name == "timestamp_utc_ms" || name == "timestamp_ms" || name == "ts_ms") {
            header_.colTime = static_cast<int>(i);
            header_.timeInMs = true;
        } else if (name == "time_s" || name == "time" || name == "t" || name == "timecode") {
            header_.colTime = static_cast<int>(i);
        } else if (name == "throttle_pct" || name == "throttle" || name == "throttle_percent") {
            header_.colThrottle = static_cast<int>(i);
        } else if (name == "road_speed_kmh" || name == "road_speed" ||
                   name == "speed_kmh" || name == "speed") {
            header_.colRoad = static_cast<int>(i);
        } else if (name == "gear") {
            header_.colGear = static_cast<int>(i);
        } else if (name == "gear_selector" || name == "gearselector") {
            header_.colGearSelector = static_cast<int>(i);
        } else if (name == "clutch_pct" || name == "clutch") {
            header_.colClutch = static_cast<int>(i);
        } else if (name == "motor_torque_nm" || name == "motor_torque" || name == "torque_nm") {
            header_.colMotorTorque = static_cast<int>(i);
        } else if (name == "brake_percent" || name == "brake_pedal_state" ||
                   name == "brake_pedalstate" || name == "di_brakepedalstate") {
            header_.colBrakePedalState = static_cast<int>(i);
        }
    }

    if (header_.colTime < 0) {
        errorMsg = "Telemetry CSV missing time column (time_s)";
        return false;
    }

    // Detect raw CAN format (undecoded).
    bool hasCanId = false;
    bool hasDataHex = false;
    for (const auto& field : fields) {
        const std::string name = lower(trim(field));
        if (name == "can_id") hasCanId = true;
        if (name == "data_hex") hasDataHex = true;
    }
    if (hasCanId && hasDataHex) {
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

    // DI_brakePedalState enum (0=OFF, 1=ON, 2=INVALID). The decoded capture column
    // is integer-valued; an unparseable/blank value keeps the default (0=OFF).
    if (header_.colBrakePedalState >= 0 &&
        header_.colBrakePedalState < static_cast<int>(fields.size()) &&
        parseInt(fields[header_.colBrakePedalState], s.brakePedalState)) {
        // Clamp to the documented 2-bit enum domain so a stray large value cannot
        // be mistaken for a valid pedal state downstream.
        if (s.brakePedalState < 0 || s.brakePedalState > 2) s.brakePedalState = 2;
    }

    out = s;
    return true;
}

} // namespace input
