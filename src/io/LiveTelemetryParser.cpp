// LiveTelemetryParser.cpp - Parse vehicle-sim JSON telemetry into UpstreamSignal

#include "io/LiveTelemetryParser.h"
#include <cmath>

namespace input {

UpstreamSignal LiveTelemetryParser::parse(const std::string& json) {
    return parse(json, 0);
}

UpstreamSignal LiveTelemetryParser::parse(const std::string& json, uint64_t timestampUtcMs) {
    UpstreamSignal signal;
    signal.timestampUtcMs = timestampUtcMs;

    // Extract vehicle-sim fields (percentages and SI units)
    double throttlePercent = 0.0;
    double speedKmh = 0.0;
    double accelerationG = 0.0;
    double brakePercent = 0.0;

    extractDoubleRaw(json, "throttle", throttlePercent);
    extractDoubleRaw(json, "speed", speedKmh);
    extractDoubleRaw(json, "acceleration", accelerationG);
    extractDoubleRaw(json, "brake", brakePercent);

    // Convert vehicle-sim units to UpstreamSignal (fraction 0-1)
    signal.throttleFraction = throttlePercent / 100.0;
    signal.speedKmh = speedKmh;
    signal.accelerationG = accelerationG;
    signal.brakeFraction = brakePercent / 100.0;

    // Clamp to valid ranges
    signal.throttleFraction = std::max(0.0, std::min(1.0, signal.throttleFraction));
    signal.brakeFraction = std::max(0.0, std::min(1.0, signal.brakeFraction));
    signal.speedKmh = std::max(0.0, signal.speedKmh);

    signal.isValid = true;
    return signal;
}

double LiveTelemetryParser::extractDouble(const std::string& json, const char* key, double defaultValue) {
    double value = defaultValue;
    extractDoubleRaw(json, key, value);
    return value;
}

bool LiveTelemetryParser::extractDoubleRaw(const std::string& json, const char* key, double& out) {
    // Build the search pattern: "key"
    std::string searchKey = std::string("\"") + key + "\"";

    // Find the key in the JSON string
    size_t keyPos = json.find(searchKey);
    if (keyPos == std::string::npos) {
        return false;
    }

    // Move past the key and closing quote
    size_t i = keyPos + searchKey.size();

    // Skip whitespace
    i = skipWhitespace(json, i);

    // Expect colon
    if (i >= json.size() || json[i] != ':') {
        return false;
    }
    i++; // skip colon

    // Skip whitespace after colon
    i = skipWhitespace(json, i);

    // Parse the number
    return parseNumber(json, i, out);
}

size_t LiveTelemetryParser::skipWhitespace(const std::string& json, size_t i) {
    while (i < json.size() && (json[i] == ' ' || json[i] == '\t' || json[i] == '\n' || json[i] == '\r')) {
        i++;
    }
    return i;
}

bool LiveTelemetryParser::parseNumber(const std::string& json, size_t& i, double& out) {
    if (i >= json.size()) {
        return false;
    }

    size_t start = i;

    // Optional minus sign
    if (json[i] == '-') {
        i++;
    }

    // Must have at least one digit or dot
    if (i >= json.size()) {
        return false;
    }

    // Integer part (required — a number must have at least one digit)
    if (!parseIntegerPart(json, i)) {
        return false;
    }

    // Decimal part (optional fractional digits)
    if (i < json.size() && json[i] == '.') {
        i++;
        if (!parseDecimalPart(json, i)) {
            return false;
        }
    }

    // Exponent part (optional e/E with optional sign)
    if (i < json.size() && (json[i] == 'e' || json[i] == 'E')) {
        i++;
        if (i < json.size() && (json[i] == '+' || json[i] == '-')) {
            i++;
        }
        if (!parseExponentPart(json, i)) {
            return false;
        }
    }

    // Must have consumed at least one character for the number
    if (i == start) {
        return false;
    }

    // Use strtod on the substring for accurate parsing
    std::string numStr = json.substr(start, i - start);
    return convertStringToDouble(numStr, out);
}

bool LiveTelemetryParser::parseIntegerPart(const std::string& json, size_t& i) {
    if (i >= json.size() || json[i] < '0' || json[i] > '9') {
        return false;
    }
    while (i < json.size() && json[i] >= '0' && json[i] <= '9') {
        i++;
    }
    return true;
}

bool LiveTelemetryParser::parseDecimalPart(const std::string& json, size_t& i) {
    if (i >= json.size() || json[i] < '0' || json[i] > '9') {
        return false;
    }
    while (i < json.size() && json[i] >= '0' && json[i] <= '9') {
        i++;
    }
    return true;
}

bool LiveTelemetryParser::parseExponentPart(const std::string& json, size_t& i) {
    if (i >= json.size() || json[i] < '0' || json[i] > '9') {
        return false;
    }
    while (i < json.size() && json[i] >= '0' && json[i] <= '9') {
        i++;
    }
    return true;
}

bool LiveTelemetryParser::convertStringToDouble(std::string_view numStr, double& out) {
    char* endPtr = nullptr;
    double value = std::strtod(numStr.data(), &endPtr);

    if (endPtr == numStr.data()) {
        return false; // No conversion performed
    }

    // Reject NaN and infinity
    if (std::isnan(value) || std::isinf(value)) {
        return false;
    }

    out = value;
    return true;
}

} // namespace input
