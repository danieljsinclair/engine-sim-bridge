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
        } else if (name == "throttle_pct" || name == "throttle" || name == "throttle_percent" ||
                   name == "throttle_gas_pct") {
            header_.colThrottle = static_cast<int>(i);
        } else if (name == "road_speed_kmh" || name == "road_speed" ||
                   name == "speed_kmh" || name == "speed" || name == "vehicle_speed_kmh") {
            header_.colRoad = static_cast<int>(i);
        } else if (name == "gear") {
            header_.colGear = static_cast<int>(i);
        } else if (name == "gear_selector" || name == "gearselector") {
            header_.colGearSelector = static_cast<int>(i);
        } else if (name == "clutch_pct" || name == "clutch" || name == "clutch_pressure") {
            header_.colClutch = static_cast<int>(i);
        } else if (name == "motor_torque_nm" || name == "motor_torque" || name == "torque_nm") {
            header_.colMotorTorque = static_cast<int>(i);
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
        // Reject trailing rows whose timestamp is inconsistent with the parsed
        // unit. A capture can carry a few epoch-microsecond rows at the very end
        // (e.g. 1786961013730 = the wall-clock write time of the last CAN frame,
        // not a trace time). Accepted as-is they normalise to ~1.79e12 s, which
        // makes durationS() return that and the replay runs free-run to timeout
        // instead of self-terminating at trace end (~424.2 s). 1e7 s is far above
        // any legitimate trace span (the longest captures are ~1000 s) and far
        // below any epoch-microsecond value, so it cleanly separates the two.
        // The first-row heuristic in the caller (firstTs > 1e6 -> /1e6) already
        // handles the bulk of the file; this is the backstop for the stragglers.
        const double timeInSeconds = v / timeDivisor;
        if (timeInSeconds > 1e7) {
            fprintf(stderr,
                "[CsvTelemetryParser] WARNING: rejecting row with out-of-range "
                "timestamp %.1f s (unit mismatch / epoch-scale trailing row)\n",
                timeInSeconds);
            return false;
        }
        s.timeS = timeInSeconds;
    } else {
        return false;  // skip rows with unparseable time
    }

    if (header_.colThrottle >= 0 && header_.colThrottle < static_cast<int>(fields.size()) &&
        parseDouble(fields[header_.colThrottle], v)) {
        s.throttle = std::clamp(v / 100.0, 0.0, 1.0);
    }

    if (header_.colRoad >= 0 && header_.colRoad < static_cast<int>(fields.size()) &&
        parseDouble(fields[header_.colRoad], v)) {
        // Accept negative road speeds: reverse driving is a real state in the CSV
        // schema (em-dinner.csv carries 'R' rows at -3.2 km/h). The old guard
        // `v >= 0.0` silently dropped negatives to the -2.0 sentinel, which hid
        // genuine reverse from downstream coercion and let standstill 'R' rows
        // leak through as REVERSE (RAR). A blank/unparseable road column still
        // leaves the -2.0 "not commanded" sentinel intact.
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

    out = s;
    return true;
}

} // namespace input
