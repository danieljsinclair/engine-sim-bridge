// CsvTelemetryParser.h
//
// Shared CSV header + row parser for telemetry providers.
// Extracted from ReplayTelemetryProvider::parseCsv() to avoid duplication
// between file-based replay and live-stdin providers.
//
// Recognised columns (any subset; unknown columns ignored):
//   time_s / timestamp_ms       relative seconds (or epoch ms -> auto-converted)
//   throttle_pct / throttle_percent / throttle  0..100 -> normalised to 0..1
//   road_speed_kmh / speed_kmh   km/h. Blank/negative = dyno OFF (RPM emergent)
//   gear            blank/-1 = unchanged; 0 = neutral; 1..8 = forward (set directly)
//   gear_selector   PRNDL string (P/R/N/D) — followed by the auto gearbox
//   clutch_pct      0..100 -> clutch pressure 0..1. Blank/-1 = unchanged
//   brake_light     1 = brake light on (pedal pressed), 0 = off. Blank/unparseable
//                  = absent. The retired brake_percent / brake_pedal_state
//                  columns are no longer consumed (they fall through to the
//                  unknown-column path so old captures still parse).

#ifndef INPUT_CSV_TELEMETRY_PARSER_H
#define INPUT_CSV_TELEMETRY_PARSER_H

#include <optional>
#include <string>
#include <vector>

namespace input {

// Parsed column indices from the CSV header.
struct CsvHeader {
    int colTime = -1;
    int colThrottle = -1;
    int colRoad = -1;
    int colGear = -1;
    int colClutch = -1;
    int colGearSelector = -1;
    int colMotorTorque = -1;
    int colBrakeLight = -1;  // brake_light: 1=on, 0=off
    bool timeInMs = false;
    bool rawCanFormat = false;  // true if can_id + data_hex columns detected
};

// One decoded telemetry sample.
struct CsvSample {
    double timeS = 0.0;
    double throttle = 0.0;           // 0..1
    double roadSpeedKmh = -2.0;      // -2 sentinel = not commanded (dyno off)
    int gear = -1;                   // -1 = unchanged; 0 = neutral; 1..8 = forward
    double clutchPct = -1.0;         // -1 = unchanged; 0..1
    std::string gearSelector;        // PRNDL string from vehicle-sim captures
    double motorTorqueNm = 0.0;      // Recorded motor/engine torque (Nm); -1 = missing
    std::optional<bool> brakeLight;  // brake light: true=on, false=off, nullopt=absent
};

class CsvTelemetryParser {
public:
    // Parse a header line and return column indices. Must be called once before
    // parseRow(). Returns false if the header is invalid (e.g. raw CAN format).
    bool parseHeader(const std::string& headerLine, std::string& errorMsg);

    // Parse a single data row into a CsvSample. timeDivisor converts ms->s when
    // the header indicated timestamp_ms. The raw time value from the CSV is stored
    // in out.timeS; callers that need zero-based time must subtract the first
    // timestamp themselves.
    bool parseRow(const std::string& row, double timeDivisor,
                  CsvSample& out, const std::string& errorMsg) const;

    // Access the parsed header info.
    const CsvHeader& header() const { return header_; }

    // Emit a single end-of-input summary of rows rejected for out-of-range
    // (epoch-scale) timestamps. Prints nothing when no rows were rejected.
    // Call exactly once per input stream (replay: after the parse loop;
    // live: once when EOF is detected).
    void emitRejectionSummary() const;

private:
    CsvHeader header_;
    // Count of rows skipped for out-of-range/epoch-scale timestamps. Mutable so
    // it can be incremented from the const parseRow() without changing its
    // signature (lower blast radius than dropping const).
    mutable size_t rejectedOutlierRows_ = 0;

    // First raw timestamp value seen (ms) for the current stream. Used to
    // rebase epoch-scale timestamp_ms columns to 0-based seconds. -1 means
    // "not yet seen". Reset in parseHeader() so each stream starts fresh.
    mutable double firstRawTimestampMs_ = -1.0;

    // timestamp_ms values at/above this are epoch-scale (e.g. vehicle-sim emits
    // Unix epoch milliseconds such as 1786538088200). 1e12 ms ~= 1e9 s ~= year
    // 2001, far beyond any relative-trace duration, so it cleanly separates
    // epoch timestamps from relative ones.
    static constexpr double kEpochMsThreshold = 1e12;
};

} // namespace input

#endif // INPUT_CSV_TELEMETRY_PARSER_H
