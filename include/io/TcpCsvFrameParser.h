// TcpCsvFrameParser.h - Live-schema CSV frame parser for the TCP stream
//
// Port of the app's CsvTelemetryParser.swift (live vehicle-sim TCP schema).
// Deliberately NOT input::CsvTelemetryParser: that one is the REPLAY schema
// (brake_percent retired, no acceleration_g column) and decodes into
// CsvSample for the replay twin. The live stream contract is:
//   timestamp_utc_ms | timestamp_ms, throttle_pct | throttle_percent,
//   speed_kmh, acceleration_g, brake_pct | brake_percent
// Header-driven (names, not positions) so either timestamp/throttle/brake
// variant is tolerated. Lenient: unknown columns ignored, empty/unparseable
// optional cells decode as absent, rows without a usable timestamp are
// skipped (a stream cannot be paced/aligned without one).
//
// Thread-safe: no mutable state; each call is independent (mirrors
// LiveTelemetryParser conventions).

#ifndef IO_TCP_CSV_FRAME_PARSER_H
#define IO_TCP_CSV_FRAME_PARSER_H

#include <optional>
#include <string>
#include <vector>

namespace input {

/// One decoded live-telemetry frame. Channels the schema's header did not
/// carry (or a blank/unparseable cell) decode as nullopt, mirroring the Swift
/// CsvTelemetryRow optionals.
struct TcpTelemetryFrame {
    double timestampMs = 0.0;
    std::optional<double> throttle;      // vehicle-sim percent (0-100)
    std::optional<double> speedKmh;      // km/h
    std::optional<double> accelerationG; // g
    std::optional<double> brake;         // vehicle-sim percent (0-100)
};

/// Column indices from a normalized header row. timestamp is guaranteed to be
/// a valid index whenever a TcpCsvColumnMap exists (nullopt return otherwise).
struct TcpCsvColumnMap {
    int timestamp = -1;
    int throttle = -1;
    int speed = -1;
    int acceleration = -1;
    int brake = -1;
};

class TcpCsvFrameParser {
public:
    /// Examine a candidate header line and, if it carries a usable timestamp
    /// column, return a column map. Returns nullopt for a line that is not a
    /// valid header, so a stream client can keep scanning past auth banners
    /// until the real header arrives.
    static std::optional<TcpCsvColumnMap> tryHeader(const std::string& line);

    /// Parse a single CSV data row against a precomputed map. Returns
    /// nullopt when the timestamp cell is missing or unparseable.
    static std::optional<TcpTelemetryFrame> parseRow(const std::string& line,
                                                     const TcpCsvColumnMap& map);

private:
    /// Split on commas (no quoting in the live schema).
    static std::vector<std::string> split(const std::string& line);

    /// Normalize a header cell: lowercase, trim, spaces/dashes to underscores.
    static std::string normalize(const std::string& raw);

    /// Trim ASCII whitespace and parse, or nullopt when empty/unparseable.
    static std::optional<double> doubleOrNil(const std::string& raw);
};

} // namespace input

#endif // IO_TCP_CSV_FRAME_PARSER_H
