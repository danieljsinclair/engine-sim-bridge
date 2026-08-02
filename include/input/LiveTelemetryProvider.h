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
    LiveTelemetryProvider(std::istream& stream, bool autoStart);

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

    /// Get the current upstream signal (for diagnostics/debugging).
    UpstreamSignal getCurrentSignal() const;

private:
    // CSV stdin path helpers
    bool tryReadNextRow();

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

    // CSV stdin members (unused in JSON mode)
    std::istream* stream_ = nullptr;
    CsvTelemetryParser csvParser_;
    CsvSample currentSample_{};
    bool hasSample_ = false;
    double elapsedS_ = 0.0;
    bool eofSeen_ = false;
    bool headerParsed_ = false;  // header parsed once; later calls read data rows only
};

} // namespace input

#endif // LIVE_TELEMETRY_PROVIDER_H
