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
#include "input/IReplayTimeline.h"
#include "input/VirtualIceInputProvider.h"
#include "twin/IceVehicleProfile.h"
#include "simulator/EngineSimTypes.h"

#include <atomic>
#include <deque>
#include <istream>
#include <memory>
#include <string>
#include <vector>

namespace input {

class LiveTelemetryProvider : public IInputProvider, public IReplayTimeline {
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

    /// Select the live clutch wheel-coupling strategy (FREE/PIN).
    void setWheelCouplingMode(twin::WheelCouplingMode mode);

    /// Select the coupling MODEL (how the clutch pressure is derived): clutch-map
    /// (default — declarative smooth governor, no binary relief), torque-converter
    /// (fluid coupling), or legacy (historical slip-lock + binary relief, A/B).
    void setCouplingModel(twin::CouplingModelKind kind);

    /// Bring the owned twin to RUNNING and settle the warm cruise basin BEFORE the
    /// first real frame — the warm-boot the live path was missing. Without it the
    /// twin + core start COLD and the first emitted frame blows massive negative
    /// exhaust flow (reversion). Call AFTER setWheelCouplingMode/setCouplingModel
    /// (the CLI sets them post-Initialize) so the twin primes with the chosen
    /// coupling. Shares the replay prime path via warmBootTwinToRunning() (DRY).
    /// Live has no parsed samples at Initialize time, so it seeds from a
    /// running-baseline (light throttle / ~10 km/h, DRIVE). No-op when the twin is
    /// absent or already warmed (idempotent).
    void warmBootToRunning();

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

    // IReplayTimeline — makes the live path share the replay warm-start prefix
    // in SimulationLoop (read only getStartFromS()>0; never durationS()), so
    // --live-telemetry --start-from primes the twin + core instead of cold-jumping.
    double durationS() const override { return -1.0; }   // unbounded live stream
    void setEndAtS(double s) override { endAtS_ = s; }
    double getStartFromS() const override { return startFromS_; }

private:
    // CSV stdin path helpers
    // simElapsedS is the current simulation elapsed seconds; used to pace row
    // consumption by recording timestamp so that 1s of sim time = 1s of recording.
    bool tryReadNextRow(double simElapsedS);

    /// Parse the CSV header (first non-blank line), once. False until a usable
    /// header is found (or EOF).
    bool ensureHeaderParsed();

    /// True if a line is all whitespace.
    static bool isBlankLine(std::string_view s);

    /// True if a parsed sample has no meaningful signals (both throttle=0 and
    /// roadSpeedKmh at the dyno-off sentinel). Such rows arrive when the CAN bus
    /// is still waking up (e.g. ~166 blank frames from a live USB pipe before
    /// real telemetry populates). They must be skipped so hasSample_ stays false
    /// and the twin waits in OFF for valid data.
    static bool isSampleBlank(const CsvSample& s);

    /// Create + initialise twinProvider_ (shared by the CSV and JSON paths).
    bool initTwinProvider();

    /// Map the CSV gear_selector column to a GearSelector (default DRIVE).
    bridge::GearSelector csvGearSelector() const;

    /// Monotonic non-zero ms so the twin's telemetry-timeout guard never fires.
    uint64_t streamTimestampUtcMs() const;

    /// Record a consumed row's (recording-relative time, road speed) into the
    /// speed-level state. A "level" is a contiguous run of rows whose road speed
    /// is within kLevelStepKmh; the level START (first row of the run) is the
    /// fixed anchor used by interpolatedSpeedKmh() so its fraction sweeps the
    /// whole ~0.2 s hold rather than tracking the sim clock (which would leave
    /// it ~0). Dyno-off sentinels form their own level.
    void updateCurrentSpeedLevel(double relT, double speedKmh);

    /// Continuous (de-quantized) road speed at the sim clock. The CSV road speed
    /// is a STAIRCASE (CAN-quantized ~0.8 km/h, each level held ~0.2 s); feeding
    /// the held value makes a hard-pinned/slip-locked RPM step in lockstep (the
    /// "rpm stepper"). This linearly interpolates between the current level START
    /// and the next level, by fractional recording time, so the feed ramps
    /// smoothly across the whole hold. Pure feed interpolation — no RPM filter.
    /// Falls back to the held level when no next level is known (true live feed
    /// whose next sample has not arrived) and passes dyno-off sentinels through.
    double interpolatedSpeedKmh(double simElapsedS) const;

    /// Two recorded samples are the SAME speed level when their road speeds differ
    /// by <= this. The CAN quantization step is ~0.8 km/h, so 0.1 cleanly separates
    /// real level changes from float jitter while never splitting a genuine level.
    static constexpr double kLevelStepKmh = 0.1;

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
    bool liveStream_ = false;  // retained for the ctor signature; the buffered scan unifies both paths
    CsvTelemetryParser csvParser_;
    CsvSample currentSample_{};
    bool hasSample_ = false;
    double elapsedS_ = 0.0;
    bool eofSeen_ = false;
    bool headerParsed_ = false;  // header parsed once; later calls read data rows only
    double startFromS_ = -1.0;   // skip CSV rows before this time (-1 = disabled)
    double endAtS_ = -1.0;       // stop at this time (-1 = play to end); IReplayTimeline
    double baselineTimeS_ = -1.0;  // recording-time of the first consumed row (set on first success)

    /// Read-ahead buffer of parsed rows NOT yet past the sim clock. The CSV stream
    /// is forward-only and getline is destructive, so to interpolate the road
    /// speed between the current level and the NEXT level (which lives in the
    /// near future, ~0.2 s ahead) the upcoming rows are buffered here rather than
    /// consumed-and-lost. Rows are popped to currentSample_ as the sim clock
    /// reaches them; the remaining tail holds the lookahead used to find the next
    /// speed level. Bounded by kLevelLookaheadS (~0.3 s of rows).
    std::deque<CsvSample> rowBuffer_;

    /// Speed-level state for interpolatedSpeedKmh(). curSpeedLevelT_ is the
    /// recording-relative time at which the CURRENT speed level started (fixed
    /// while the sim clock is inside the level); nextSpeedLevel* is the first
    /// level after it (looked ahead in rowBuffer_), or hasNextSpeedLevel_=false
    /// when unknown (true live feed, next sample not yet arrived -> hold).
    double curSpeedLevelT_ = 0.0;
    double curSpeedLevelKmh_ = 0.0;
    bool hasCurSpeedLevel_ = false;
    double nextSpeedLevelT_ = 0.0;
    double nextSpeedLevelKmh_ = 0.0;
    bool hasNextSpeedLevel_ = false;
};

} // namespace input

#endif // LIVE_TELEMETRY_PROVIDER_H
