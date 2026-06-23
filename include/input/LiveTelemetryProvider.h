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
#include "input/VirtualIceInputProvider.h"
#include "twin/IceVehicleProfile.h"
#include "simulator/EngineSimTypes.h"

#include <atomic>
#include <memory>
#include <string>

namespace input {

class LiveTelemetryProvider : public IInputProvider {
public:
    /// Create a live telemetry provider with the given vehicle profile.
    /// The profile defines gear ratios, shift tables, and vehicle dynamics.
    explicit LiveTelemetryProvider(const twin::IceVehicleProfile& profile);

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

    /// Get the current upstream signal (for diagnostics/debugging).
    UpstreamSignal getCurrentSignal() const;

private:
    const twin::IceVehicleProfile& profile_;
    std::unique_ptr<VirtualIceInputProvider> twinProvider_;
    std::atomic<UpstreamSignal> currentSignal_;
    std::atomic<bool> signalReceived_;
    std::atomic<bool> initialized_;
    std::string lastError_;
};

} // namespace input

#endif // LIVE_TELEMETRY_PROVIDER_H
