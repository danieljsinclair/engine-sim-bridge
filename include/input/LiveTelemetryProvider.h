// LiveTelemetryProvider.h - Live telemetry input provider for engine-sim
//
// Accepts UpstreamSignal from an external source (vehicle-sim BLE/OBD2 feed)
// via the thread-safe submitSignal() method, then delegates to VirtualIceInputProvider
// for twin-based gearbox/clutch processing.
//
// This provider wraps VirtualIceInputProvider to reuse its twin logic:
//   - Throttle smoothing
//   - Automatic gearbox shifting
//   - Clutch pressure modeling
//   - Engine RPM feedback for cranking transitions
//
// Usage:
//   auto provider = std::make_unique<LiveTelemetryProvider>(IceVehicleProfile::zf8hp45());
//   provider->Initialize();
//   provider->submitSignal(signal);  // called from telemetry thread
//   EngineInput input = provider->OnUpdateSimulation(dt);  // called from sim thread
//
// Thread safety: submitSignal() uses an atomic copy of UpstreamSignal so the
// telemetry feed thread and the simulation loop thread can run concurrently.

#ifndef LIVE_TELEMETRY_PROVIDER_H
#define LIVE_TELEMETRY_PROVIDER_H

#include "io/IInputProvider.h"
#include "io/UpstreamSignal.h"
#include "input/CsvTelemetryParser.h"
#include "input/VirtualIceInputProvider.h"
#include "twin/IceVehicleProfile.h"
#include "simulator/EngineSimTypes.h"

#include <atomic>
#include <istream>
#include <memory>
#include <string>
#include <vector>

namespace input {

class LiveTelemetryProvider : public IInputProvider {
public:
    /// Create a live telemetry provider with the given vehicle profile.
    /// The profile defines gear ratios, shift tables, and vehicle dynamics.
    explicit LiveTelemetryProvider(const twin::IceVehicleProfile& profile);

    /// Create a live telemetry provider that reads CSV from an input stream
    /// (e.g. stdin pipe from vehicle-sim --stdout-csv) and routes each sample
    /// through the VirtualIceTwin (gearbox/clutch/throttle + cranking). The
    /// autoStart flag is retained for callers; the twin owns the start lifecycle.
    ///
    /// liveStream=true (the engine-sim-cli --live-telemetry pipe) surfaces the
    /// LATEST row available in the stream every frame — no consumption pacing by
    /// recording timestamp. Pacing by recording time is correct for a seekable
    /// replay file but adds up to ~0.5s+ latency on a live pipe (the sim clock
    /// must "catch up" to each row's timestamp), so a live feed must echo the
    /// freshest sample immediately. liveStream=false keeps the deterministic
    /// timestamp-paced behaviour (used by unit tests).
    LiveTelemetryProvider(std::istream& stream, bool autoStart, bool liveStream = false);

    ~LiveTelemetryProvider() override;

    // IInputProvider lifecycle
    bool Initialize() override;
    void Shutdown() override;
    bool IsConnected() const override;

    // IInputProvider input queries
    EngineInput OnUpdateSimulation(double dt) override;
    std::string GetProviderName() const override;
    std::string GetLastError() const override;

    /// Submit a new upstream signal from the telemetry feed.
    /// Thread-safe: can be called from any thread (BLE/serial/network).
    /// Overwrites the previous signal (only the latest matters).
    void submitSignal(const UpstreamSignal& signal);

    /// Submit a signal with explicit timestamp (UTC ms).
    void submitSignal(const UpstreamSignal& signal, uint64_t timestampUtcMs);

    /// Forward gear selector changes to the twin (e.g., from UI).
    void setGearSelector(int selector);

    /// Forward ignition state to the twin.
    void setIgnition(bool on);

    /// Forward simulator RPM feedback to the twin for cranking transition.
    void provideFeedback(const EngineSimStats& stats) override;

    /// Reconfigure the twin's gearbox to match the loaded engine preset's
    /// transmission ratios. Forwards to the owned VirtualIceTwin so the live
    /// CSV-driven box shifts against the actual engine (e.g. a C63) instead of
    /// the default ZF profile. Mirrors ReplayTelemetryProvider/DemoInputProvider.
    void reconfigureProfile(const std::vector<double>& gearRatios,
                            double diffRatio, double tireRadiusM);

    /// Attach a gearbox diagnostic logger. Mirrors DemoInputProvider so
    /// --gearbox-log works identically for --live-telemetry and --connect-demo.
    void setGearboxLogger(twin::IGearboxLogger* logger);

    /// Get the current upstream signal (for diagnostics/debugging).
    UpstreamSignal getCurrentSignal() const;

    /// Skip CSV rows before this time (seconds). Mirrors ReplayTelemetryProvider
    /// so --start-from works for --live-telemetry as well as --replay-telemetry.
    void setStartFromS(double s);

private:
    // CSV stdin path helpers
    // simElapsedS is the current simulation elapsed seconds; used to pace row
    // consumption by recording timestamp so that 1s of sim time = 1s of recording.
    bool tryReadNextRow(double simElapsedS);

    /// Parse the CSV header (first non-blank line), once. False until a usable
    /// header is found (or EOF).
    bool ensureHeaderParsed();

    /// LIVE stdin path: surface the LATEST row read this frame (no timestamp
    /// pacing) — keeps a live feed responsive. Returns true if a sample was found.
    bool tryReadNextRowLive();

    /// Timestamp-paced path (replay file): surface the last row at/before the sim
    /// window. Returns true if a sample was found.
    bool tryReadNextRowPaced(double simElapsedS);

    /// Outcome of classifying one parsed row against the current sim window.
    enum class RowDisposition { Skip, Surface, Future };

    /// Classify a row vs the sim window: before --start-from (Skip), within the
    /// window (Surface), or ahead of sim time (Future). Sets baselineTimeS_ on
    /// the first surfaced row.
    RowDisposition classifyRow(const CsvSample& sample, double simElapsedS);

    /// True if a line is all whitespace.
    static bool isBlankLine(std::string_view s);

    /// Create + initialise twinProvider_ (shared by the CSV and JSON paths).
    bool initTwinProvider();

    /// Map the CSV gear_selector column to a GearSelector (default DRIVE).
    bridge::GearSelector csvGearSelector() const;

    /// Monotonic non-zero ms so the twin's telemetry-timeout guard never fires.
    uint64_t streamTimestampUtcMs() const;

    // JSON mode: external profile ref. CSV mode: owned profile + istream.
    twin::IceVehicleProfile ownedProfile_;          // used only in CSV mode
    const twin::IceVehicleProfile& profile_;        // points to ownedProfile_ or external
    std::unique_ptr<VirtualIceInputProvider> twinProvider_;
    std::atomic<UpstreamSignal> currentSignal_;
    std::atomic<bool> signalReceived_;
    std::atomic<bool> initialized_;
    std::string lastError_;

    /// Non-virtual cleanup. Called by destructor and Shutdown().
    void doShutdown();

    /// CSV stdin members (unused in JSON mode)
    std::istream* stream_ = nullptr;
    bool liveStream_ = false;  // true => surface latest row every frame (no pacing)
    CsvTelemetryParser csvParser_;
    CsvSample currentSample_{};
    bool hasSample_ = false;
    double elapsedS_ = 0.0;
    bool eofSeen_ = false;
    bool headerParsed_ = false;  // header parsed once; later calls read data rows only
    double startFromS_ = -1.0;   // skip CSV rows before this time (-1 = disabled)
    double baselineTimeS_ = -1.0;  // recording-time of the first consumed row (set on first success)
};

} // namespace input

#endif // LIVE_TELEMETRY_PROVIDER_H
