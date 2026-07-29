// LiveTelemetryParser.h - Parse vehicle-sim JSON telemetry lines into UpstreamSignal
//
// Accepts the JSON format produced by VehicleSignalFormatter / VehicleSimulator:
//   {"throttle":45.2,"speed":80.5,"acceleration":0.3,"brake":0.0}
//
// All fields are optional. Missing fields default to 0.0 (or false for isValid).
// Parsing is lenient: extra fields are ignored, malformed lines yield isValid=false.
//
// Thread-safe: no mutable state. Each call to parse() is independent.
//
// Usage:
//   auto signal = input::LiveTelemetryParser::parse(jsonLine);
//   if (signal.isValid) { /* use signal.throttleFraction, signal.speedKmh, etc. */ }

#ifndef LIVE_TELEMETRY_PARSER_H
#define LIVE_TELEMETRY_PARSER_H

#include "io/UpstreamSignal.h"
#include <string>
#include <string_view>
#include <cstdlib>
#include <cstring>

namespace input {

class LiveTelemetryParser {
public:
    /// Parse a JSON telemetry line into an UpstreamSignal.
    /// Returns an UpstreamSignal with isValid=true on success, isValid=false on failure.
    /// Expected JSON keys: "throttle" (%), "speed" (km/h), "acceleration" (G), "brake" (%)
    /// All keys are optional; missing keys default to 0.0.
    static UpstreamSignal parse(const std::string& json);

    /// Parse with explicit timestamp (UTC ms).
    /// Useful when the caller has its own clock source.
    static UpstreamSignal parse(const std::string& json, uint64_t timestampUtcMs);

private:
    /// Minimal JSON value extractor for flat objects.
    /// Finds "key": number patterns in a flat JSON string.
    /// Returns defaultValue if key is not found or value is not a valid number.
    static double extractDouble(const std::string& json, const char* key, double defaultValue);

    /// Find a JSON string key and extract the double value that follows the colon.
    /// Handles: whitespace, negative numbers, decimals, scientific notation.
    /// Returns false if the key was not found or the value is not a valid number.
    static bool extractDoubleRaw(const std::string& json, const char* key, double& out);

    /// Skip whitespace starting at position i.
    static size_t skipWhitespace(const std::string& json, size_t i);

    /// Parse the integer part (digits) starting at position i. Advances i.
    static bool parseIntegerPart(const std::string& json, size_t& i);

    /// Parse the decimal part (.digits) starting at position i. Advances i.
    static bool parseDecimalPart(const std::string& json, size_t& i);

    /// Parse the exponent part (e±digits) starting at position i. Advances i.
    static bool parseExponentPart(const std::string& json, size_t& i);

    /// Convert a string to double, rejecting NaN and infinity.
    static bool convertStringToDouble(std::string_view numStr, double& out);

    /// Parse a JSON number starting at position i. Advances i past the number.
    static bool parseNumber(const std::string& json, size_t& i, double& out);
};

} // namespace input

#endif // LIVE_TELEMETRY_PARSER_H
