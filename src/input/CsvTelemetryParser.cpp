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
    firstRawTimestampMs_ = -1.0;
    rejectedOutlierRows_ = 0;

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
        } else if (name == "brake_light" || name == "brakelight") {
            header_.colBrakeLight = static_cast<int>(i);
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
        // Epoch-scale timestamp_ms (e.g. vehicle-sim emits Unix epoch
        // milliseconds: 1786538088200). Dividing bare by timeDivisor yields
        // ~1.79e9 s, which the legacy >1e7 backstop below would reject as an
        // outlier — silently dropping the ENTIRE stream (vehicle-sim's output is
        // 100% epoch-scale, so every row is "out of range"). Detect epoch-scale
        // and rebase to 0-based seconds using the first row's timestamp as t=0,
        // so the trace plays from the start exactly as a 0-based time_s capture
        // does. The header doc already promises "epoch ms -> auto-converted".
        if (header_.timeInMs && v >= kEpochMsThreshold) {
            if (firstRawTimestampMs_ < 0.0) {
                firstRawTimestampMs_ = v;  // anchor t=0 on the first kept row
            }
            s.timeS = (v - firstRawTimestampMs_) / 1000.0;
        } else {
            // Relative timestamps (time_s, or relative ms). Reject trailing rows
            // whose timestamp is inconsistent with the parsed unit — a capture
            // can carry a few epoch-microsecond rows at the very end (e.g.
            // 1786961013730 = the wall-clock write time of the last CAN frame,
            // not a trace time). 1e7 s is far above any legitimate trace span
            // (the longest captures are ~1000 s) and far below any epoch value,
            // so it cleanly separates the two. This is the backstop for the
            // stragglers that escape the caller's first-row heuristic.
            const double timeInSeconds = v / timeDivisor;
            if (timeInSeconds > 1e7) {
                ++rejectedOutlierRows_;  // counted; reported once at end-of-input
                return false;            // row skipped instantly, no per-row log
            }
            s.timeS = timeInSeconds;
        }
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

void CsvTelemetryParser::emitRejectionSummary() const {
    if (rejectedOutlierRows_ == 0) return;
    fprintf(stderr,
        "[CsvTelemetryParser] INFO: skipped %zu row(s) with "
        "out-of-range/epoch-scale timestamps\n",
        rejectedOutlierRows_);
}

} // namespace input
