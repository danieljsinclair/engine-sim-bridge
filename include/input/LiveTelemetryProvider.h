// LiveTelemetryProvider.h
//
// Reads live decoded CSV telemetry from stdin, one row at a time.
// Designed for piping from vehicle-sim or a real-time telemetry source:
//
//   vehicle-sim --connect file:capture.csv --log-csv | engine-sim-cli --live-telemetry -
//
// Behaviour:
//   - Reads the CSV header from the first line of stdin.
//   - Each subsequent line is parsed as a telemetry sample.
//   - Between row arrivals, the last received sample is held (step-hold).
//   - EOF on stdin sets IsConnected() to false; no crash.
//   - --live-telemetry and --replay-telemetry are mutually exclusive (enforced
//     by CLI argument parsing, not this class).
//
// CSV format: same header-driven format as ReplayTelemetryProvider (time_s,
// throttle_pct, road_speed_kmh, gear, gear_selector, clutch_pct).
// Uses CsvTelemetryParser internally to avoid duplicating parsing logic.

#ifndef INPUT_LIVE_TELEMETRY_PROVIDER_H
#define INPUT_LIVE_TELEMETRY_PROVIDER_H

#include "io/IInputProvider.h"
#include "input/CsvTelemetryParser.h"

#include <istream>
#include <string>
#include <atomic>

namespace input {

class LiveTelemetryProvider : public IInputProvider {
public:
    // Production constructor: reads from std::cin.
    explicit LiveTelemetryProvider(std::string streamId = "-",
                                   bool autoStart = true);

    // Test constructor: reads from the provided istream.
    explicit LiveTelemetryProvider(std::istream& stream,
                                   bool autoStart = true);

    ~LiveTelemetryProvider() override;

    bool Initialize() override;
    void Shutdown() override;
    bool IsConnected() const override;
    EngineInput OnUpdateSimulation(double dt) override;
    void provideFeedback(const EngineSimStats& stats) override;
    std::string GetProviderName() const override { return "LiveTelemetry"; }
    std::string GetLastError() const override { return lastError_; }

private:
    // Try to read the next row from stdin. Returns true if a new sample was
    // parsed and is now held in currentSample_.
    bool tryReadNextRow();

    std::string streamId_;
    bool autoStart_;
    std::istream* stream_ = nullptr;  // non-owning; points to std::cin or test stream
    std::atomic<bool> connected_{false};
    std::string lastError_;
    CsvTelemetryParser csvParser_;
    bool headerParsed_ = false;
    CsvSample currentSample_{};
    bool hasSample_ = false;
    double elapsedS_ = 0.0;
    bool startFired_ = false;
    bool eofSeen_ = false;
};

} // namespace input

#endif // INPUT_LIVE_TELEMETRY_PROVIDER_H
