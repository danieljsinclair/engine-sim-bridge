// ReplayTelemetryProvider.h
//
// Replays a timecoded telemetry CSV as the simulation's input source. Designed
// for bench-driving the simulator from a recorded log (real vehicle CSV) or a
// hand-authored script for spikes/diagnostics.
//
// CSV is header-driven. Recognised columns (any subset; unknown columns ignored):
//   time_s / timestamp_ms  relative seconds (or epoch ms -> auto-converted)
//   throttle_pct / throttle_percent / throttle  0..100 -> normalised to 0..1
//   road_speed_kmh / speed_kmh   km/h. Blank/negative = dyno OFF (RPM emergent)
//   gear            blank/-1 = unchanged; 0 = neutral; 1..8 = forward (set directly)
//   gear_selector   PRNDL string (P/R/N/D) — followed by the auto gearbox; in
//                   manual mode it drives engineInput.gearSelector (authoritative
//                   over the numeric gear column when both are present)
//   clutch_pct      0..100 -> clutch pressure 0..1. Blank/-1 = unchanged
//   brake_light     1 = brake light on, 0 = off, blank = absent (tri-state);
//                   surfaces on engineInput.brakeLight every frame
//
// autoStart fires starterButton once on the first frame so the CrankingController
// cranks the engine — EXCEPT for traces carrying start/stop opinion columns
// (brake_light / gear_selector): there VehicleStartController owns every start
// (it engages in SimulationLoop as soon as the opinion is seen), so the frame-0
// pulse is suppressed (non-auto path only; see OnUpdateSimulation). autoGearbox
// routes the CSV through a VirtualIceTwin (via VirtualIceInputProvider — the
// SAME twin the live / --live-telemetry path uses) that decides gears + the
// clutch coupling (--coupling-model / --wheel-coupling) from the CSV road
// speed. This mirrors the live path so the replay path exercises the SAME
// coupling code (the slider/coupling toggle is no longer a no-op on replay).
// Q (quit) + P (preset cycle) work during replay if a keyboard + session are
// wired via setKeyboardInput / setSession.

#ifndef INPUT_REPLAY_TELEMETRY_PROVIDER_H
#define INPUT_REPLAY_TELEMETRY_PROVIDER_H

#include "io/IInputProvider.h"
#include "simulator/EngineSimTypes.h"
#include "twin/IceVehicleProfile.h"
#include "twin/IGearboxLogger.h"
#include "twin/WheelCoupling.h"
#include "twin/CouplingModelSelector.h"
#include "twin/VirtualIceTwin.h"
#include "input/IKeyboardInput.h"
#include "input/CsvTelemetryParser.h"
#include "input/VirtualIceInputProvider.h"
#include "session/ISimulatorSession.h"

// IReplayTimeline — satisfied by this provider so the CLI's
// validateReplayTimeSlicing() can depend on the narrow interface, not this
// whole class. Only durationS()/setEndAtS() are promoted to virtual here.
#include "input/IReplayTimeline.h"
// IArrivalStatePrimer — instant --start-from seam: the loop asks the provider
// to synthesize the arrival state (twin warm-boot from the arrival row + clock
// anchor) and hold that row as the synthetic input while it settles the engine
// core at the arrival operating point. No pre-offset row is ever simulated.
#include "input/IArrivalStatePrimer.h"

#include <memory>
#include <string>
#include <vector>

namespace input {

class ReplayTelemetryProvider : public IInputProvider,
                                public IReplayTimeline,
                                public IArrivalStatePrimer {
public:
    explicit ReplayTelemetryProvider(std::string csvPath, bool autoStart = true,
                                     bool autoGearbox = false);
    ~ReplayTelemetryProvider() override;

    bool Initialize() override;
    void Shutdown() override {}
    bool IsConnected() const override { return connected_; }
    bool endAtReached() const override { return endAtReached_; }
    EngineInput OnUpdateSimulation(double dt) override;
    void provideFeedback(const EngineSimStats& stats) override;
    std::string GetProviderName() const override { return "ReplayTelemetry"; }
    std::string GetLastError() const override { return lastError_; }

    // Enable Q (quit) + P (preset cycle) during replay.
    void setKeyboardInput(IKeyboardInput* keyboard) { keyboard_ = keyboard; }
    void setSession(ISimulatorSession* session) { session_ = session; }

    // Total span of the parsed trace, in seconds (last sample time). 0 if empty.
    // virtual via IReplayTimeline (used by the CLI's time-slice validator).
    double durationS() const override;

    // Time slicing: skip samples before startFromS, stop at endAtS.
    // -1 = disabled (play full trace).
    // setEndAtS is virtual via IReplayTimeline (validator clamps it to play-to-end).
    void setStartFromS(double s) { startFromS_ = s; }
    double getStartFromS() const override { return startFromS_; }
    void setEndAtS(double s) override { endAtS_ = s; }

    // IArrivalStatePrimer — instant --start-from (see the interface header).
    // Prime: twin warm-boot seeded from the ARRIVAL row (first row at/after
    // the offset), replay clock anchored on that row, and the row HELD as the
    // constant synthetic input. Release: the clock resumes from the arrival
    // row and the trace plays from there. No-op when no offset is set.
    void primeArrivalState() override;
    void releaseArrivalHold() override;
    bool arrivalHoldActive() const { return arrivalHoldActive_; }

    // Current replay timestamp (absolute, from CSV). -1 before first sample.
    double currentTimestampS() const { return currentTimestampS_; }

    // Reconfigure the gearbox profile to match the ACTUAL engine preset's ratios.
    // Forwards to the owned VirtualIceTwin (auto-gearbox only) so the replay box
    // shifts against the real engine (e.g. a C63) instead of the default ZF
    // profile. Mirrors LiveTelemetryProvider.
    void reconfigureProfile(const std::vector<double>& gearRatios,
                            double diffRatio, double tireRadiusM);

    // Attach a gearbox-decision logger so the oracle (section D: parse per-frame
    // gear/rpm/mph) can validate the automatic box during replay. Forwards to the
    // owned VirtualIceTwin (no-op when auto-gearbox is disabled / twin null).
    void setGearboxLogger(twin::IGearboxLogger* logger);

    // Select the live clutch wheel-coupling strategy (FREE/PIN/TORQUE). Mirrors
    // LiveTelemetryProvider: forwarded to the twin so --wheel-coupling takes
    // effect in the replay DRIVE branch.
    void setWheelCouplingMode(twin::WheelCouplingMode mode);

    // PIN-coupling compliance tau in ms (--pin-tau-ms): 0 = rigid pin. Like
    // setWheelCouplingMode this is stored and re-forwarded when the owned
    // twin provider is created in Initialize() (the CLI sets it before).
    void setPinTauMs(double tauMs);

    // --effective-throttle / --torque-informed-gearbox configs (both DEFAULT
    // disabled = inert). Same store + re-forward contract as setPinTauMs.
    void setEffectiveThrottleConfig(const twin::EffectiveThrottleConfig& config);
    void setTorqueInformedGearboxConfig(const twin::TorqueInformedGearboxConfig& config);

    // Select the coupling MODEL (how the clutch pressure is derived):
    // clutch-map (declarative smooth governor), torque-converter (fluid
    // coupling), or legacy (historical slip-lock + binary relief, A/B). Mirrors
    // LiveTelemetryProvider: forwarded to the twin so --coupling-model takes
    // effect in the replay DRIVE branch.
    void setCouplingModel(twin::CouplingModelKind kind);

private:
    using Sample = CsvSample;

    bool parseCsv();
    void postProcessSamples();  // sort + even-spreading of identical timestamps

    // OnUpdateSimulation helpers — each owns one responsibility so
    // OnUpdateSimulation stays a flat orchestrator (cognitive-complexity +
    // nesting under the sonar thresholds).
    bool applyTimeSlicing(EngineInput& input, double dt);
    void buildBaseEngineInput(EngineInput& input, const Sample& s) const;
    // Route a DRIVE sample through the twin provider (VirtualIceInputProvider ->
    // VirtualIceTwin) — mirrors the live path. The twin owns gearbox/clutch/
    // coupling-model processing; the returned EngineInput carries throttle, gear,
    // clutchPressure, ignition, starter, selector, auto-mode, road-speed pin,
    // injected torque and diagnostics.
    EngineInput driveThroughTwin(const Sample& s, double dt, double roadSpeedKmh);
    // Feed the twin through its OFF->CRANKING->IDLE->RUNNING lifecycle once, so
    // the first replayed frame is already RUNNING with a forward gear available
    // (replay reproduces a running engine, not the cranking transient). The
    // selector comes from the first sample (default DRIVE) so a PARK/NEUTRAL
    // trace never reaches RUNNING. No-op when the twin is absent / no samples.
    void primeTwinToRunning();
    void handleNonAutoGearbox(EngineInput& input, const Sample& s) const;

    const Sample& sampleAt(double t) const;
    // First sample with timeS >= t (the ARRIVAL row for --start-from t).
    // sampleAt() floors (last row <= t); this ceils — the instant-skip
    // contract replays from the first row AT or AFTER the offset, never a
    // pre-offset one. Clamps to the last row when t is past the trace end.
    const Sample& firstSampleAtOrAfter(double t) const;

    void processKeyboardInput(EngineInput& input);
    void processReplayKey(int key, EngineInput& input);

    std::string csvPath_;
    bool autoStart_;
    bool autoGearbox_;
    bool connected_ = false;
    std::string lastError_;
    CsvTelemetryParser csvParser_;
    std::vector<Sample> samples_;
    double elapsedS_ = 0.0;
    bool startFired_ = false;
    twin::IceVehicleProfile gearboxProfile_;  // OWNED SEED for the twin (twin copies it)

    // True when the CSV header carries brake_light / gear_selector: the trace
    // expresses start/stop opinions, so the autoStart frame-0 pulse is
    // suppressed (VehicleStartController owns every start for such traces).
    bool traceCarriesStartStopOpinion_ = false;

    // The replay DRIVE branch routes through VirtualIceInputProvider ->
    // VirtualIceTwin (the SAME twin the live/Demo paths use) so --coupling-model /
    // --wheel-coupling take effect on replay. The provider owns the gearbox +
    // coupling strategy/model. Null when autoGearbox_ is false (manual replay has
    // no gearbox/clutch to drive; the CSV drives throttle/gear/clutch directly).
    // Created in Initialize().
    std::unique_ptr<VirtualIceInputProvider> twinProvider_;

    // Coupling selection forwarded from the CLI (--coupling-model / --wheel-coupling).
    // Seeded into the twin so the replay path exercises the SAME coupling code as
    // the live path. Defaults mirror the CLI defaults (pin + torque-converter).
    twin::WheelCouplingMode wheelCouplingMode_ = twin::WheelCouplingMode::Pin;
    twin::CouplingModelKind couplingModelKind_ = twin::CouplingModelKind::TorqueConverter;

    // PIN-coupling compliance tau in ms (--pin-tau-ms; 0 = rigid, the CLI
    // default). Stored so a pre-Initialize() set survives twin creation.
    double pinTauMs_ = 0.0;

    // Torque-feature configs (--effective-throttle / --torque-informed-gearbox;
    // defaults disabled). Stored so a pre-Initialize() set survives twin
    // creation, same as pinTauMs_.
    twin::EffectiveThrottleConfig effectiveThrottleConfig_;
    twin::TorqueInformedGearboxConfig torqueInformedGearboxConfig_;

    // Keyboard input support for Q/P during replay
    IKeyboardInput* keyboard_ = nullptr;
    ISimulatorSession* session_ = nullptr;
    bool ignitionOn_ = true;          // toggled by 'I' key during replay
    double startFromS_ = -1.0;        // skip samples before this time (-1 = disabled)
    double endAtS_ = -1.0;            // stop replay at this time (-1 = disabled)
    bool endAtReached_ = false;       // the --end-at bound fired (honest stop reason)
    double currentTimestampS_ = -1.0; // current absolute timestamp from CSV
    // Instant --start-from hold: while true, OnUpdateSimulation returns the
    // ARRIVAL row unchanged and the replay clock does not advance — the loop
    // settles the engine core at this constant operating point before the
    // first emitted frame (see IArrivalStatePrimer).
    bool arrivalHoldActive_ = false;
};

} // namespace input

#endif // INPUT_REPLAY_TELEMETRY_PROVIDER_H
