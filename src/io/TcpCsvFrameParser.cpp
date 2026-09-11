// TcpCsvFrameParser.cpp - Live-schema CSV frame parser (see TcpCsvFrameParser.h)
//
// Faithful port of the Swift CsvTelemetryParser logic, including the exact
// header aliases and the timestamp-missing precondition failure.

#include "io/TcpCsvFrameParser.h"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <string_view>

namespace input {

namespace {

// Column alias tables, first match wins (mirrors the Swift lookup order).
const std::array<const char* const, 2> kTimestampAliases = {"timestamp_ms", "timestamp_utc_ms"};
const std::array<const char* const, 2> kThrottleAliases = {"throttle_pct", "throttle_percent"};
const std::array<const char* const, 1> kSpeedAliases    = {"speed_kmh"};
const std::array<const char* const, 1> kAccelAliases    = {"acceleration_g"};
const std::array<const char* const, 2> kBrakeAliases    = {"brake_pct", "brake_percent"};

template <std::size_t N>
int firstIndexOf(const std::vector<std::string>& headers, const std::array<const char* const, N>& aliases) {
    for (const char* alias : aliases) {
        auto it = std::find(headers.begin(), headers.end(), alias);
        if (it != headers.end()) {
            return static_cast<int>(it - headers.begin());
        }
    }
    return -1;
}

std::string trimAscii(const std::string_view raw) {
    const char* ws = " \t\r\n\v\f";
    const auto begin = raw.find_first_not_of(ws);
    if (begin == std::string_view::npos) {
        return {};
    }
    const auto end = raw.find_last_not_of(ws);
    return std::string(raw.substr(begin, end - begin + 1));
}

} // namespace

std::string TcpCsvFrameParser::normalize(const std::string& raw) {
    std::string out = trimAscii(raw);
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    std::replace(out.begin(), out.end(), ' ', '_');
    std::replace(out.begin(), out.end(), '-', '_');
    return out;
}

std::vector<std::string> TcpCsvFrameParser::split(const std::string_view line) {
    std::vector<std::string> cols;
    std::size_t start = 0;
    while (true) {
        const std::size_t comma = line.find(',', start);
        if (comma == std::string_view::npos) {
            cols.push_back(std::string(line.substr(start)));
            break;
        }
        cols.push_back(std::string(line.substr(start, comma - start)));
        start = comma + 1;
    }
    return cols;
}

std::optional<double> TcpCsvFrameParser::doubleOrNil(const std::string& raw) {
    const std::string trimmed = trimAscii(raw);
    if (trimmed.empty()) {
        return std::nullopt;
    }
    char* end = nullptr;
    const double value = std::strtod(trimmed.c_str(), &end);
    if (end == trimmed.c_str()) {
        return std::nullopt;  // no conversion at all (Swift Double.init fails)
    }
    return value;
}

std::optional<TcpCsvColumnMap> TcpCsvFrameParser::tryHeader(const std::string& line) {
    std::vector<std::string> headers;
    headers.reserve(split(line).size());
    for (const std::string& col : split(line)) {
        headers.push_back(normalize(col));
    }

    TcpCsvColumnMap map;
    map.timestamp = firstIndexOf(headers, kTimestampAliases);
    if (map.timestamp < 0) {
        // No usable timestamp column: not a decodable header (the stream
        // cannot be paced without one) — a genuine precondition failure.
        return std::nullopt;
    }
    map.throttle = firstIndexOf(headers, kThrottleAliases);
    map.speed = firstIndexOf(headers, kSpeedAliases);
    map.acceleration = firstIndexOf(headers, kAccelAliases);
    map.brake = firstIndexOf(headers, kBrakeAliases);
    return map;
}

std::optional<TcpTelemetryFrame> TcpCsvFrameParser::parseRow(const std::string& line,
                                                             const TcpCsvColumnMap& map) {
    const std::vector<std::string> cols = split(line);
    auto cell = [&cols](int index) -> const std::string* {
        if (index < 0 || index >= static_cast<int>(cols.size())) {
            return nullptr;
        }
        return &cols[static_cast<std::size_t>(index)];
    };

    // A row with no usable timestamp cannot be paced/aligned; skip silently.
    const std::string* tsCell = cell(map.timestamp);
    const std::optional<double> ts = tsCell ? doubleOrNil(*tsCell) : std::nullopt;
    if (!ts.has_value()) {
        return std::nullopt;
    }

    TcpTelemetryFrame frame;
    frame.timestampMs = ts.value();
    if (const std::string* throttleCell = cell(map.throttle)) {
        frame.throttle = doubleOrNil(*throttleCell);
    }
    if (const std::string* speedCell = cell(map.speed)) {
        frame.speedKmh = doubleOrNil(*speedCell);
    }
    if (const std::string* accelCell = cell(map.acceleration)) {
        frame.accelerationG = doubleOrNil(*accelCell);
    }
    if (const std::string* brakeCell = cell(map.brake)) {
        frame.brake = doubleOrNil(*brakeCell);
    }
    return frame;
}

} // namespace input
