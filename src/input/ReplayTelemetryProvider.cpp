// ReplayTelemetryProvider.cpp
#include "input/ReplayTelemetryProvider.h"
#include "input/CsvGearCoercion.h"
#include "input/VirtualIceInputProvider.h"
#include "input/WarmBoot.h"
#include "twin/IceVehicleProfile.h"
#include "io/UpstreamSignal.h"
#include "simulator/GearConventions.h"
#include "simulator/EngineSimTypes.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <cstdio>

namespace input {
namespace {

// trim/lower are used by parseGearSelector; CsvTelemetryParser has its own.
std::string trim(std::string s) {
    const auto notSpace = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), notSpace));
    s.erase(std::find_if(s.rbegin(), s.rend(), notSpace).base(), s.end());
    return s;
}

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return s;
}

// Convert PRNDL string to GearSelector enum (case-insensitive). Unknown -> DRIVE.
bridge::GearSelector parseGearSelector(const std::string& s) {
    const std::string ls = lower(trim(s));
    if (ls == "p" || ls == "park") return bridge::GearSelector::PARK;
    if (ls == "r" || ls == "reverse") return bridge::GearSelector::REVERSE;
    if (ls == "n" || ls == "neutral") return bridge::GearSelector::NEUTRAL;
    return bridge::GearSelector::DRIVE;  // "d"/"drive"/unknown
}

} // namespace

ReplayTelemetryProvider::ReplayTelemetryProvider(std::string csvPath, bool autoStart,
                                                 bool autoGearbox)
    : csvPath_(std::move(csvPath)), autoStart_(autoStart), autoGearbox_(autoGearbox) {
    // The twin provider is created lazily in Initialize() (after the CSV is
    // parsed) so it always owns at least one valid trace. autoGearbox_ gates
    // whether a twin is created at all (a non-auto replay has no gearbox/clutch
    // to drive; the CSV drives throttle/gear/clutch directly).
    if (autoGearbox_) {
        // Seed the owned profile with the ZF8 default; reconfigureProfile() (called
        // from CLIMain once the real .mr transmission is loaded) replaces it with
        // the actual engine's ratios. The twin copies this profile.
        gearboxProfile_ = twin::IceVehicleProfile::zf8hp45();
    }
}

ReplayTelemetryProvider::~ReplayTelemetryProvider() = default;

void ReplayTelemetryProvider::provideFeedback(const EngineSimStats& stats) {
    if (twinProvider_) twinProvider_->provideFeedback(stats);
}

bool ReplayTelemetryProvider::Initialize() {
    if (!parseCsv()) return false;
    connected_ = !samples_.empty();
    if (!connected_) {
        lastError_ = "No telemetry rows parsed from " + csvPath_;
        return false;
    }
    // A trace carrying start/stop opinion columns (brake_light / gear_selector)
    // describes the full vehicle lifecycle, so the VehicleStartController must
    // own every start; see the autoStart pulse in OnUpdateSimulation.
    traceCarriesStartStopOpinion_ =
        csvParser_.header().colBrakeLight >= 0 || csvParser_.header().colGearSelector >= 0;
    // The DRIVE branch routes through VirtualIceInputProvider -> VirtualIceTwin —
    // the SAME twin the live/Demo paths use — so --coupling-model / --wheel-coupling
    // take effect on replay. Only created when the auto-gearbox is enabled.
    if (autoGearbox_) {
        twinProvider_ = std::make_unique<VirtualIceInputProvider>(gearboxProfile_);
        if (!twinProvider_->Initialize()) {
            lastError_ = "Failed to initialize twin provider: " + twinProvider_->GetLastError();
            twinProvider_.reset();
            return false;
        }
        twinProvider_->setWheelCouplingMode(wheelCouplingMode_);
        twinProvider_->setCouplingModel(couplingModelKind_);
        twinProvider_->setPinTauMs(pinTauMs_);
        twinProvider_->setEffectiveThrottleConfig(effectiveThrottleConfig_);
        twinProvider_->setTorqueInformedGearboxConfig(torqueInformedGearboxConfig_);
        // Bring the twin to RUNNING before the first replayed frame so DRIVE
        // traces expose a forward gear immediately (replay is a running engine,
        // not a cranking capture). PARK/NEUTRAL traces stay out of RUNNING.
        primeTwinToRunning();
    }
    return true;
}

double ReplayTelemetryProvider::durationS() const {
    return samples_.empty() ? 0.0 : samples_.back().timeS;
}

void ReplayTelemetryProvider::setGearboxLogger(twin::IGearboxLogger* logger) {
    // Forwards to the owned twin provider so the oracle (section D) can parse
    // per-frame gear/rpm/mph from the gearbox log during replay. No-op when the
    // auto-gearbox is disabled (twin provider null).
    if (twinProvider_) twinProvider_->setGearboxLogger(logger);
}

void ReplayTelemetryProvider::setWheelCouplingMode(twin::WheelCouplingMode mode) {
    wheelCouplingMode_ = mode;
    if (twinProvider_) twinProvider_->setWheelCouplingMode(mode);
}

void ReplayTelemetryProvider::setPinTauMs(double tauMs) {
    // Stored + re-forwarded: the CLI sets this BEFORE Initialize() (where the
    // twin provider is created), mirroring setWheelCouplingMode.
    pinTauMs_ = tauMs;
    if (twinProvider_) twinProvider_->setPinTauMs(pinTauMs_);
}

void ReplayTelemetryProvider::setEffectiveThrottleConfig(
    const twin::EffectiveThrottleConfig& config) {
    // Same store + re-forward contract as setPinTauMs.
    effectiveThrottleConfig_ = config;
    if (twinProvider_) twinProvider_->setEffectiveThrottleConfig(config);
}

void ReplayTelemetryProvider::setTorqueInformedGearboxConfig(
    const twin::TorqueInformedGearboxConfig& config) {
    torqueInformedGearboxConfig_ = config;
    if (twinProvider_) twinProvider_->setTorqueInformedGearboxConfig(config);
}

void ReplayTelemetryProvider::setCouplingModel(twin::CouplingModelKind kind) {
    couplingModelKind_ = kind;
    if (twinProvider_) twinProvider_->setCouplingModel(kind);
}

void ReplayTelemetryProvider::reconfigureProfile(const std::vector<double>& gearRatios,
                                                  double diffRatio, double tireRadiusM) {
    if (!twinProvider_) return;
    twinProvider_->reconfigureProfile(gearRatios, diffRatio, tireRadiusM);
}

bool ReplayTelemetryProvider::parseCsv() {
    std::ifstream in(csvPath_);
    if (!in.is_open()) {
        lastError_ = "Cannot open telemetry CSV: " + csvPath_;
        return false;
    }

    std::string line;
    bool headerParsed = false;
    double firstTs = -1.0;
    bool firstRow = true;

    while (std::getline(in, line)) {
        const std::string trimmed = trim(line);
        if (trimmed.empty()) continue;

        if (!headerParsed) {
            if (!csvParser_.parseHeader(trimmed, lastError_)) {
                return false;
            }
            headerParsed = true;
            continue;
        }

        CsvSample s;
        std::string parseError;
        double timeDivisor = csvParser_.header().timeInMs ? 1000.0 : 1.0;
        if (csvParser_.parseRow(trimmed, timeDivisor, s, parseError)) {
            if (firstRow) {
                firstTs = s.timeS;  // raw first timestamp (already divided)
                firstRow = false;
            }
            s.timeS = s.timeS - firstTs;  // normalise to zero-based
            samples_.push_back(s);
        }
    }

    if (samples_.empty()) {
        if (!headerParsed) lastError_ = "Empty telemetry CSV: " + csvPath_;
        csvParser_.emitRejectionSummary();
        return true;
    }
    csvParser_.emitRejectionSummary();
    postProcessSamples();
    return true;
}

void ReplayTelemetryProvider::postProcessSamples() {
    std::stable_sort(samples_.begin(), samples_.end(),
                     [](const Sample& a, const Sample& b) { return a.timeS < b.timeS; });
    // If all timestamps are identical (e.g. vehicle-sim capture tool wrote all
    // zeros), distribute evenly at a default sample rate so the replay advances.
    if (samples_.size() > 1 && samples_.front().timeS == samples_.back().timeS) {
        const double defaultHz = 446.0;  // measured CAN frame rate from FirstDrive capture
        for (size_t i = 0; i < samples_.size(); ++i) {
            samples_[i].timeS = static_cast<double>(i) / defaultHz;
        }
    }
}

const ReplayTelemetryProvider::Sample& ReplayTelemetryProvider::sampleAt(double t) const {
    static const Sample kNeutral{};
    if (samples_.empty()) return kNeutral;
    if (t <= samples_.front().timeS) return samples_.front();
    if (t >= samples_.back().timeS) return samples_.back();
    for (size_t i = 1; i < samples_.size(); ++i) {
        if (samples_[i].timeS > t) return samples_[i - 1];
    }
    return samples_.back();
}

const ReplayTelemetryProvider::Sample&
ReplayTelemetryProvider::firstSampleAtOrAfter(double t) const {
    static const Sample kNeutral{};
    if (samples_.empty()) return kNeutral;
    const auto it = std::lower_bound(samples_.begin(), samples_.end(), t,
                                     [](const Sample& a, double value) {
                                         return a.timeS < value;
                                     });
    return it == samples_.end() ? samples_.back() : *it;
}

EngineInput ReplayTelemetryProvider::OnUpdateSimulation(double dt) {
    EngineInput input;
    if (applyTimeSlicing(input, dt)) return input;

    // NOTE: --start-from is owned by SimulationLoop::run(): for file traces it
    // asks this provider (IArrivalStatePrimer) to PRIME the arrival state from
    // the first row at/after the offset and HOLD that row while the loop
    // settles the engine core at the constant arrival operating point; after
    // releaseArrivalHold() the rows emit from the arrival row onward. Rows
    // before the offset are NEVER sampled or simulated (the owner contract).
    // While the hold is active the arrival row is served directly (the frozen
    // clock would otherwise floor to the PRE-offset row when the offset falls
    // between rows).

    const Sample& s = arrivalHoldActive_ ? firstSampleAtOrAfter(startFromS_)
                                         : sampleAt(elapsedS_);
    currentTimestampS_ = s.timeS;
    buildBaseEngineInput(input, s);
    // Feed the RAW recorded road speed. The CSV road speed is a CAN staircase
    // (levels held ~0.2 s). Interpolating it into a smooth ramp made the replay
    // feed diverge from live's staircase, flipping a gear decision at t=118-120
    // and producing the opposite flow sign at the same gear/rpm/clutch. Feeding
    // raw means replay decisions == live decisions. s still supplies
    // throttle/selector/gear.
    const double roadSpeedKmh = s.roadSpeedKmh;

    if (autoGearbox_ && twinProvider_) {
        // Route the replay sample THROUGH the twin provider (the SAME twin the
        // live/Demo paths use) so --coupling-model / --wheel-coupling take effect
        // on replay. The twin owns the gearbox/clutch/coupling-model processing and
        // the cranking lifecycle. In PARK/NEUTRAL the twin disengages the clutch
        // and the engine free-revs; only in DRIVE does it couple + track road speed.
        input = driveThroughTwin(s, dt, roadSpeedKmh);
        // driveThroughTwin rebuilds `input` from the twin's fresh EngineInput,
        // whose replayTimestampS is the -1 default — re-stamp the CSV-relative
        // time set above or the presentation layer's [mm:ss.ms] clock vanishes.
        input.replayTimestampS = currentTimestampS_;
    } else {
        // Non-auto (e.g. rev-in-park): no gear forced, clutch disengaged, free-rev.
        handleNonAutoGearbox(input, s);
        // One-shot starter pulse on the first frame so the CrankingController
        // cranks. (The auto/twin path above lets the twin own the cranking
        // lifecycle — mirroring the live path — so the pulse is only needed here.)
        // Suppressed for traces that carry start/stop opinion columns (brake_light
        // / gear_selector): those captures describe the full vehicle lifecycle,
        // and VehicleStartController — which engages in SimulationLoop as soon as
        // the opinion is seen — owns every start. A frame-0 pulse on such a trace
        // would crank before the first real brake/gear event and then be cut
        // mid-run when the controller takes authority (start-then-die blip).
        // Traces WITHOUT those columns keep the pulse: the documented autoStart
        // behavior for hand-authored / old-schema CSVs, byte-for-byte unchanged.
        if (autoStart_ && !startFired_ && !traceCarriesStartStopOpinion_) {
            input.starterButton = true;
            startFired_ = true;
        }
    }

    processKeyboardInput(input);
    return input;
}

EngineInput ReplayTelemetryProvider::driveThroughTwin(const Sample& s, double dt,
                                                       double roadSpeedKmh) {
    input::UpstreamSignal signal;
    signal.throttleFraction = s.throttle;
    signal.speedKmh = roadSpeedKmh;
    signal.motorTorqueNm = s.motorTorqueNm;
    signal.brakeLight = s.brakeLight;
    signal.isValid = true;
    // The throttle below is the RECORDING's, not a scripted driver's: the
    // crank path must not synthesize a start character over it (startup
    // flare — see UpstreamSignal::traceDriven).
    signal.traceDriven = true;
    // Monotonic non-zero timestamp (the twin treats 0 as invalid + times out).
    signal.timestampUtcMs = std::max<uint64_t>(1, static_cast<uint64_t>(elapsedS_ * 1000.0));
    // The CSV stalk (D/R/N/P) drives the twin's selector. A missing/blank selector
    // defaults to DRIVE so the bench-driving run engages and exercises the coupling
    // model (mirrors the live CSV path's default-selector contract).
    //
    // Reverse coercion shared with the live path (CsvGearCoercion.h): a recorded
    // 'R' only keeps REVERSE while genuinely reversing; a standstill/
    // contradictory 'R' maps to PARK or NEUTRAL. Applied here so a replay of the
    // same capture selects the SAME gear the live path coerces to (previously the
    // replay path forwarded the raw selector and could select REVERSE where live
    // coerced to PARK/NEUTRAL).
    const bridge::GearSelector sel = coerceCsvReverseGear(
        s.gearSelector.empty() ? bridge::GearSelector::DRIVE
                               : parseGearSelector(s.gearSelector),
        s.roadSpeedKmh);
    twinProvider_->setGearSelector(static_cast<int>(sel));
    // Replay owns its ignition (ignitionOn_, default ON, 'I' toggles): forward
    // the LEVEL to the twin every frame. The twin defaults ignition OFF and
    // never self-starts (startStop contract), and SimulationLoop's
    // VehicleStartController does not engage on the replay path — without this
    // forward the replayed twin would sit OFF and never process throttle.
    twinProvider_->setIgnition(ignitionOn_);
    twinProvider_->setUpstreamSignal(signal);
    // VirtualIceInputProvider maps TwinOutput -> EngineInput (throttle, gear,
    // clutchPressure, ignition, starter, gearSelector, gearAutoMode, road speed
    // pin, injected torque, diagnostics) — identical to the live path.
    EngineInput input = twinProvider_->OnUpdateSimulation(dt);
    // Echo the brake light for display + the start/stop decision layer (the
    // twin does not consume it) — mirrors the live CSV path's echo so the
    // '-'/'B' overlay and the VehicleStartController see the same signal on
    // replay as on live.
    input.brakeLight = s.brakeLight;

    // Echo the steering angle for display (the twin does not consume it) — the
    // CSV steering_angle_deg column is parsed into Sample.steeringAngleDeg but
    // the replay path never surfaced it, so the console [Str: ...] readout was
    // always silent on replay benches. Verbatim pass-through, absent stays nullopt.
    input.steeringAngleDeg = s.steeringAngleDeg;

    // Selector-gate overlay. The twin owns an automatic gearbox and therefore
    // ALWAYS reports gearAutoMode=true + a gear, even for a PARK/NEUTRAL stalk.
    // But a replayed P/N stalk means the vehicle is NOT in an auto drive state,
    // so the presentation layer must see neutral + manual-style auto-mode off.
    // The replay provider is the authority on the PRNDL gate it replays, so it
    // overrides the twin here (DRIVE/REVERSE pass through unchanged).
    if (sel == bridge::GearSelector::PARK || sel == bridge::GearSelector::NEUTRAL) {
        input.gearAutoMode = false;
        input.gearAbsolute = 0;
        input.vehicleSpeedTargetKmh = -1.0;
    }
    return input;
}

void ReplayTelemetryProvider::primeTwinToRunning() {
    if (!twinProvider_ || samples_.empty()) return;
    // Replay reproduces a running engine, not the cranking transient: feed the
    // twin through OFF->CRANKING->IDLE->RUNNING once at Initialize() so the first
    // replayed frame already exposes a forward gear for DRIVE traces. The selector
    // is taken from the first sample (default DRIVE), so a PARK/NEUTRAL trace
    // never leaves IDLE — it is not "running". The prime result is discarded; only
    // the resulting RUNNING state matters.
    //
    // HANDOFF CONTRACT (twin/core agreement): these synthetic frames advance the
    // TWIN's state machine only — the engine-sim core is not stepped here (the
    // provider does not own it), so the core starts Stopped. Agreement is
    // restored on the first REAL replay frames, through the core's own physics:
    //   - at road speed the drivetrain drags the crank above the catch bar and
    //     CrankingController::step bump-starts Stopped -> Running (push-start);
    //   - at standstill (rpm ~ 0) the twin's RUNNING stall guard pulses the
    //     starter and the core cranks and catches for real.
    // Either way the core phase must be Running within the first replay second;
    // the driveability gate's NO_STOPPED_LATCH check enforces the same
    // invariant on the deterministic sweep legs.
    //
    // DRY: the OFF->RUNNING stepping + warm basin settle is shared with the live
    // path via warmBootTwinToRunning() so the two prime paths never diverge.
    const Sample& first = samples_.front();
    const bridge::GearSelector sel = first.gearSelector.empty()
        ? bridge::GearSelector::DRIVE : parseGearSelector(first.gearSelector);
    warmBootTwinToRunning(twinProvider_.get(), first.throttle, first.roadSpeedKmh,
                          static_cast<int>(sel));
}

bool ReplayTelemetryProvider::applyTimeSlicing(EngineInput& input, double dt) {
    // Instant --start-from hold: while the arrival hold is active the replay
    // clock is FROZEN on the arrival row (the loop settles the engine core at
    // this constant operating point — advancing here would replay later rows
    // during the settle, breaking the constant-input contract).
    if (!arrivalHoldActive_) {
        elapsedS_ += dt;
    }

    // Time slicing: stop at endAtS — emit an ignition-off frame and signal the
    // caller to return it immediately (no further processing this frame).
    if (endAtS_ >= 0.0 && elapsedS_ >= endAtS_) {
        if (session_) session_->stop();
        // The BOUND ended the run (not trace exhaustion): record it so the CLI
        // can print the honest stop reason instead of the full trace length.
        endAtReached_ = true;
        input.ignition = false;
        return true;
    }
    return false;
}

void ReplayTelemetryProvider::primeArrivalState() {
    // Instant --start-from: synthesize the provider-side arrival state from
    // the FIRST row at/after the offset — no pre-offset row is ever sampled.
    // No-op without an offset (from-0 runs keep the Initialize()-time prime
    // and byte-identical behavior).
    if (startFromS_ <= 0.0 || samples_.empty()) return;

    const Sample& arrival = firstSampleAtOrAfter(startFromS_);
    arrivalHoldActive_ = true;
    // Clock cold-jump onto the arrival row's TRUE timecode (mirrors the live
    // path's instant anchor): the first emitted frame reads [mm:ss] at the
    // offset, and the post-release clock advances from here.
    elapsedS_ = arrival.timeS;

    if (twinProvider_) {
        const auto sel = arrival.gearSelector.empty()
            ? bridge::GearSelector::DRIVE : parseGearSelector(arrival.gearSelector);
        // Shared OFF->RUNNING warm boot (DRY with the live path), seeded with
        // the ARRIVAL row's operating point rather than the trace's first row.
        warmBootTwinToRunning(twinProvider_.get(), arrival.throttle,
                              arrival.roadSpeedKmh, static_cast<int>(sel));

        // Arrival-seed settle: the shared warm boot settles the GENERIC cruise
        // basin (its fixed light-throttle/low-speed prime); hold the twin at
        // the ARRIVAL operating point instead so the gearbox/clutch/coupling
        // state converges on THAT point (e.g. arrives in g5/g6 at highway
        // speed, not the g4/g5 cruise basin). Twin-only stepping — no engine
        // core, no rows — so this costs microseconds; the core warms in the
        // loop's settle window (see SimulationLoop). Ends with a fresh crank
        // budget so the synthetic frames' discarded starter edges leave no
        // re-crank cooldown debt on the real run (same contract as
        // primeWarmUp).
        input::UpstreamSignal signal;
        signal.throttleFraction = arrival.throttle;
        signal.speedKmh = arrival.roadSpeedKmh;
        signal.isValid = true;
        signal.traceDriven = true;  // the arrival row IS the trace
        signal.timestampUtcMs = std::max<uint64_t>(
            1, static_cast<uint64_t>(startFromS_ * 1000.0));
        twinProvider_->setGearSelector(static_cast<int>(sel));
        twinProvider_->setIgnition(true);
        twinProvider_->setUpstreamSignal(signal);
        constexpr double kArrivalSeedDt = 0.05;
        constexpr int kArrivalSeedFrames = 200;  // 10s of twin time
        for (int i = 0; i < kArrivalSeedFrames; ++i) {
            twinProvider_->OnUpdateSimulation(kArrivalSeedDt);
        }
        twinProvider_->armFreshCrankBudget();
    }
}

void ReplayTelemetryProvider::releaseArrivalHold() {
    arrivalHoldActive_ = false;
}

void ReplayTelemetryProvider::buildBaseEngineInput(EngineInput& input, const Sample& s) const {
    input.replayTimestampS = currentTimestampS_;
    input.throttle = s.throttle;
    input.ignition = ignitionOn_;
    // Tri-state brake light, straight from the CSV (1/0/blank). Mirrors the
    // live provider's echo (LiveTelemetryProvider.cpp: input.brakeLight =
    // signal.brakeLight): SimulationLoop treats a PRESENT value as "telemetry
    // reported an opinion", which hands start/stop authority to
    // VehicleStartController. Absent column => nullopt propagates => the
    // old-schema behavior (no telemetry opinion) is unchanged.
    input.brakeLight = s.brakeLight;
    // Steering angle for display (the twin does not consume it). Surfaces the
    // CSV steering_angle_deg column on the non-auto path; absent stays nullopt.
    input.steeringAngleDeg = s.steeringAngleDeg;
}

void ReplayTelemetryProvider::handleNonAutoGearbox(EngineInput& input, const Sample& s) const {
    input.roadSpeedKmh = s.roadSpeedKmh;
    input.gearAbsolute = s.gear;
    // gear_selector (PRNDL string) is the authoritative driver command when the
    // capture carries it — vehicle-sim's decoded schema has NO numeric `gear`
    // column, so without this mapping engineInput.gearSelector (what
    // SimulationLoop::applyStartStopDecision reads) would stay NEUTRAL forever.
    // Mirrors LiveTelemetryProvider::csvGearSelector. CSVs without the column
    // (or with a blank cell) keep the numeric-gear behavior: 0 -> NEUTRAL,
    // 1..8 -> that gear, absent (-1) -> NEUTRAL — exactly the old behavior.
    if (!s.gearSelector.empty()) {
        input.gearSelector = static_cast<int>(parseGearSelector(s.gearSelector));
    } else {
        input.gearSelector = (s.gear >= 0) ? s.gear : 0;
    }
    input.clutchPressure = s.clutchPct;
    input.gearAutoMode = false;
}

void ReplayTelemetryProvider::processKeyboardInput(EngineInput& input) {
    if (!keyboard_) return;
    // Q (quit) + P (preset cycle) + S (starter) + I (ignition) during replay.
    int key;
    while ((key = keyboard_->getKey()) > 0) {
        processReplayKey(key, input);
    }
}

void ReplayTelemetryProvider::processReplayKey(int key, EngineInput& input) {
    if (key == 'q' || key == 'Q' || key == 27) {
        if (session_) session_->stop();
    } else if (key == 'p' || key == 'P') {
        input.presetCycle = true;
    } else if (key == 's' || key == 'S') {
        input.starterButton = true;
    } else if (key == 'i' || key == 'I') {
        ignitionOn_ = !ignitionOn_;
    }
}

} // namespace input
