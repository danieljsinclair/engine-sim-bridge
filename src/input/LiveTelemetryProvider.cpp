// LiveTelemetryProvider.cpp - Live telemetry input provider for engine-sim

#include "input/LiveTelemetryProvider.h"
#include "input/CsvGearCoercion.h"
#include "input/WarmBoot.h"
#include "common/PresetExceptions.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <string_view>

namespace input {


LiveTelemetryProvider::LiveTelemetryProvider(const twin::IceVehicleProfile& profile)
    : ownedProfile_(profile)
    , profile_(ownedProfile_)
    , signalReceived_(false)
    , initialized_(false) {
}

LiveTelemetryProvider::LiveTelemetryProvider(std::istream& stream, bool autoStart,
                                             std::function<bool()> streamDataReady)
    : ownedProfile_(twin::IceVehicleProfile::zf8hp45())
    , profile_(ownedProfile_)
    , initialized_(false)
    , stream_(&stream)
    , streamDataReady_(std::move(streamDataReady)) {
    (void)autoStart;
    // NOTE: zf8hp45 is ONLY a construction-time default. The LIVE path must have
    // its geometry supplied by the loaded .mr — CLIMain::reconfigureGearboxProviders
    // FAILS FAST (throws CliException) if the script lacks a transmission/vehicle
    // node, so this default never silently drives an engine for --live-telemetry.
    // The non-live paths (replay/demo) may legitimately keep it.
    // autoStart is retained in the signature for callers; the twin owns the
    // engine cranking lifecycle (OFF->CRANKING) once valid telemetry arrives.
    (void)autoStart;
}

LiveTelemetryProvider::~LiveTelemetryProvider() {
    doShutdown();
}

bool LiveTelemetryProvider::Initialize() {
    if (initialized_.load()) {
        lastError_ = "Already initialized";
        return false;
    }

    // Both the CSV stdin path and the JSON network path drive the twin; the twin
    // is mandatory — it owns gearbox/clutch/throttle processing and the cranking
    // lifecycle. The CSV sample becomes the twin's upstream signal.
    if (stream_) {
        hasSample_ = false;
        eofSeen_ = false;
        headerParsed_ = false;
        elapsedS_ = 0.0;
        liveOffsetAnchored_ = false;
        sourceSkipHintS_ = 0.0;
        streamAnchorTimeS_ = -1.0;
        rowBuffer_.clear();
        hasCurSpeedLevel_ = false;
        hasNextSpeedLevel_ = false;
    }

    if (!initTwinProvider()) {
        return false;
    }
    initialized_.store(true);
    return true;
}

bool LiveTelemetryProvider::initTwinProvider() {
    try {
        twinProvider_ = std::make_unique<VirtualIceInputProvider>(profile_);
        if (!twinProvider_->Initialize()) {
            lastError_ = "Failed to initialize twin provider: " + twinProvider_->GetLastError();
            twinProvider_.reset();
            return false;
        }
        // Re-apply any torque-feature configs that arrived before Initialize()
        // (defaults are inert no-ops on the twin).
        twinProvider_->setEffectiveThrottleConfig(effectiveThrottleConfig_);
        twinProvider_->setTorqueInformedGearboxConfig(torqueInformedGearboxConfig_);
        return true;
    } catch (const std::bad_alloc& e) {
        lastError_ = std::string("Out of memory creating twin provider: ") + e.what();
        return false;
    } catch (const PresetException& e) {
        lastError_ = std::string("Failed to create twin provider (preset error): ") + e.what();
        return false;
    } catch (const SimulatorException& e) {
        lastError_ = std::string("Failed to create twin provider (simulator error): ") + e.what();
        return false;
    }
}

void LiveTelemetryProvider::Shutdown() {
    doShutdown();
}

void LiveTelemetryProvider::doShutdown() {
    if (twinProvider_) {
        twinProvider_->Shutdown();
        twinProvider_.reset();
    }
    initialized_.store(false);
}

bool LiveTelemetryProvider::IsConnected() const {
    if (stream_) return initialized_.load() && !eofSeen_;
    return initialized_.load() && twinProvider_ != nullptr && signalReceived_.load();
}

EngineInput LiveTelemetryProvider::OnUpdateSimulation(double dt) {
    // CSV stdin path: route the latest CSV sample THROUGH the twin (mirrors the
    // JSON network path below). The twin owns gearbox/clutch/throttle processing
    // and the cranking lifecycle; the CSV sample becomes its upstream signal.
    if (stream_) {
        // Consume any #vs-start-from hint + the header BEFORE anchoring the
        // clock: the hint is only knowable from the stream, and the cold-jump
        // below must include it on the FIRST tick. Idempotent (headerParsed_
        // guard); the getline here is the same blocking read tryReadNextRow
        // would do later in this very tick, just earlier.
        (void)ensureHeaderParsed();
        // Instant start-from anchor: the display/replay clock cold-jumps to the
        // EFFECTIVE offset on the first tick so the first emitted row already
        // reads the TRUE recording-relative timecode. The offset is additive
        // with any source-side skip (#vs-start-from): vehicle-sim
        // --start-from 45 piped into cli --start-from 45 displays [01:30] —
        // the double-skipped position, per the owner's display rule. The
        // pre-window rows are discarded by tryReadNextRow (unpaced, no sim
        // stepping) — live never waits on prefix data.
        if (!liveOffsetAnchored_ && effectiveStartFromS() > 0.0) {
            elapsedS_ = effectiveStartFromS();
            liveOffsetAnchored_ = true;
        }
        elapsedS_ += dt;
        tryReadNextRow(elapsedS_);

        // --end-at bounds the live run the same way it bounds replay: once the
        // relative clock crosses the bound the stream is finished — report EOF
        // so the loop's IsConnected check exits cleanly ("stop at that
        // relative timecode or input end, whichever first"). The flag records
        // that the BOUND (not stream EOF) ended the run, so the CLI can print
        // the honest stop reason.
        if (endAtS_ >= 0.0 && elapsedS_ >= endAtS_) {
            eofSeen_ = true;
            endAtReached_ = true;
        }

        if (!initialized_.load() || !twinProvider_) {
            lastError_ = "Provider not initialized";
            return EngineInput{};
        }

        UpstreamSignal signal;
        // The live stream's throttle is the VEHICLE's, not a scripted
        // driver's: the crank path must not synthesize a start character
        // over it at a standstill attach (startup flare — see
        // UpstreamSignal::traceDriven). Covers both the warm-boot seed and
        // the sample branches below. (The iOS JSON submitSignal path keeps
        // the default — not this scenario.)
        signal.traceDriven = true;
        if (!hasSample_ && primed_ && startFromS_ > 0.0) {
            // Post-offset rows have not arrived yet (pipe still draining the
            // discarded prefix, or a paced source catching up). An invalid
            // empty signal would time the twin out to OFF and un-prime it, so
            // hold the warm-boot seed (a valid running-baseline signal) until
            // the first real row crosses the offset.
            signal.throttleFraction = primeSeedThrottle_;
            signal.speedKmh = primeSeedSpeedKmh_;
            signal.isValid = true;
            signal.timestampUtcMs = streamTimestampUtcMs();
        }
        if (hasSample_) {
            signal.throttleFraction = currentSample_.throttle;
            // Feed the RAW recorded road speed (CAN staircase, levels held ~0.2 s)
            // — identical to the replay path's s.roadSpeedKmh. Interpolating it into
            // a smooth ramp made the live feed diverge from replay's staircase,
            // flipping clutch decisions at t=118-120 and producing the opposite
            // exhaust-flow sign at the same gear/rpm/clutch. Feeding raw means
            // live decisions == replay decisions. The -2.0 dyno-off sentinel passes
            // through unchanged (it is the "not commanded" value the twin expects).
            signal.speedKmh = currentSample_.roadSpeedKmh;
            signal.motorTorqueNm = currentSample_.motorTorqueNm;
            // The start/stop decision layer (VehicleStartController, read by
            // StartStopInputAdapter via getCurrentSignal()) needs the brake
            // light and the resolved gear. The CSV path is the documented
            // vehicle-sim --stdout-csv pipe, so it must populate these here —
            // the JSON network path does the equivalent via submitSignal().
            signal.brakeLight = currentSample_.brakeLight;
            signal.gearSelector = csvGearSelector();
            // Steering angle for display (the twin does not consume it). The CSV
            // steering_angle_deg column is parsed into currentSample_ but the
            // stdin path never copied it into the signal — so the console
            // [Str: ...] readout was always silent on live-telemetry benches.
            // Absent column => nullopt propagates => non-DBC sources render nothing.
            signal.steeringAngleDeg = currentSample_.steeringAngleDeg;
            // The row is valid telemetry even when speed is blank (dyno off); a
            // non-zero timestamp keeps the twin's telemetry-timeout guard happy.
            signal.isValid = true;
            signal.timestampUtcMs = streamTimestampUtcMs();
        }

        // Mirror the JSON network path: publish the assembled signal so the
        // decision layer (which polls getCurrentSignal()) observes the same
        // brake/gear the twin processes. submitSignal() is the single seam for
        // feeding currentSignal_ on every input path.
        submitSignal(signal);

        twinProvider_->setUpstreamSignal(signal);
        // Reverse coercion: an 'R' stalk command is only honoured while the
        // vehicle is GENUINELY reversing (road speed clearly negative, below the
        // reverse-active threshold). A standstill 'R' (or a contradictory forward
        // 'R' — the car actually moving forward) must NOT select REVERSE gear:
        // at a standstill there is nothing to reverse into, and a forward 'R'
        // row is a corrupted/contradictory signal. Such rows map to PARK (true
        // standstill) or NEUTRAL (creeping forward), never REVERSE.
        //
        // This replaces the iter2 standstill-only coercion (|speed| < 1 km/h),
        // which leaked RAR frames: em-dinner.csv carries ~5k 'R' rows at
        // near-standstill negative speeds (-2..-1 km/h) that the 1 km/h window
        // failed to catch. Only a clearly-reversing speed (< kReverseActiveSpeedKmh)
        // keeps REVERSE.
        // Reverse coercion shared with the replay path (CsvGearCoercion.h):
        // a recorded 'R' only keeps REVERSE while genuinely reversing; a
        // standstill/contradictory 'R' maps to PARK or NEUTRAL. Extracted so
        // live and replay of the same capture select the SAME gear.
        bridge::GearSelector sel =
            coerceCsvReverseGear(csvGearSelector(), currentSample_.roadSpeedKmh);
        twinProvider_->setGearSelector(static_cast<int>(sel));
        EngineInput input = twinProvider_->OnUpdateSimulation(dt);
        // Surface the live sim/CSV elapsed time so each per-frame console line
        // carries a [mm:ss.ms] timecode the user can read back as --start-from.
        // The replay path sets this from currentTimestampS_; the live path tracks
        // elapsedS_ (anchored to startFromS_ above) but previously never surfaced
        // it, so --live-telemetry output had no timecode. elapsedS_ already
        // accounts for --start-from, so the timecode matches the seek origin.
        input.replayTimestampS = elapsedS_;
        // Echo the brake light for display (the twin does not consume it).
        input.brakeLight = signal.brakeLight;
        // Echo the steering angle for display (the twin does not consume it) —
        // the JSON network path does the equivalent at line ~292. Absent column
        // => nullopt propagates => non-DBC sources render nothing.
        input.steeringAngleDeg = signal.steeringAngleDeg;
        return input;
    }

    // JSON network path (master)
    EngineInput input{};

    if (!initialized_.load() || !twinProvider_) {
        lastError_ = "Provider not initialized";
        return input;
    }

    // Read the latest signal atomically and feed it to the twin
    UpstreamSignal signal = currentSignal_.load();
    twinProvider_->setUpstreamSignal(signal);

    // Relay the decoded gear to the twin. The twin learns the selector ONLY via
    // setGearSelector (its `selector_` member) — it does NOT read
    // signal.gearSelector — so without this the network path leaves selector_ at
    // its NEUTRAL default and emits engineInput.gearSelector == 0 every frame.
    // That kills gear-initiated instant starts (driveSelected stays false) on the
    // live JSON path, unlike the CSV path (line 141) which relays the gear. The
    // twin remains the single owner of the selector; we only feed it the signal
    // that submitSignal() already published above.
    twinProvider_->setGearSelector(static_cast<int>(signal.gearSelector));

    // Delegate to the twin for gearbox/clutch/throttle processing
    input = twinProvider_->OnUpdateSimulation(dt);

    // Echo the decoded gear into engineInput.gearSelector for the start/stop
    // decision site (SimulationLoop::applyStartStopDecision reads
    // state.engineInput.gearSelector to compute driveSelected). The twin only
    // echoes its `selector_` member back through VirtualIceTwin::update, and
    // that update early-returns (leaving gearSelector == NEUTRAL) when the
    // upstream signal carries timestampUtcMs == 0 — which is the norm on the
    // live JSON network path (submitSignal() is called without a timestamp). So
    // we cannot rely on the twin to surface the gear here; set it from the
    // decoded signal directly, mirroring the CSV path's "populate these here"
    // intent (line 127/141). This is the canonical gearSelector value the
    // decision layer must see — every frame, regardless of telemetry state.
    input.gearSelector = static_cast<int>(signal.gearSelector);

    // Echo the brake light for display (the twin does not consume it).
    input.brakeLight = signal.brakeLight;

    // Echo the steering angle for display (the twin does not consume it).
    input.steeringAngleDeg = signal.steeringAngleDeg;

    return input;
}

std::string LiveTelemetryProvider::GetProviderName() const {
    return "LiveTelemetryProvider";
}

std::string LiveTelemetryProvider::GetLastError() const {
    return lastError_;
}

void LiveTelemetryProvider::submitSignal(const UpstreamSignal& signal) {
    currentSignal_.store(signal, std::memory_order_seq_cst);
    signalReceived_.store(true, std::memory_order_seq_cst);
}

void LiveTelemetryProvider::submitSignal(const UpstreamSignal& signal, uint64_t timestampUtcMs) {
    UpstreamSignal timedSignal = signal;
    timedSignal.timestampUtcMs = timestampUtcMs;
    submitSignal(timedSignal);
}

void LiveTelemetryProvider::setGearSelector(int selector) {
    if (twinProvider_) {
        twinProvider_->setGearSelector(selector);
    }
}

void LiveTelemetryProvider::setIgnition(bool on) {
    if (twinProvider_) {
        twinProvider_->setIgnition(on);
    }
}

void LiveTelemetryProvider::setWheelCouplingMode(twin::WheelCouplingMode mode) {
    if (twinProvider_) {
        twinProvider_->setWheelCouplingMode(mode);
    }
}

void LiveTelemetryProvider::setPinTauMs(double tauMs) {
    // The CLI forwards --pin-tau-ms AFTER Initialize() (the same ordering as
    // setWheelCouplingMode), so the twin provider exists by this call.
    if (twinProvider_) {
        twinProvider_->setPinTauMs(tauMs);
    }
}

void LiveTelemetryProvider::setEffectiveThrottleConfig(
    const twin::EffectiveThrottleConfig& config) {
    // Stored + re-applied at twin-provider creation, so a pre-Initialize() set
    // also takes effect (superset of the setPinTauMs ordering contract).
    effectiveThrottleConfig_ = config;
    if (twinProvider_) {
        twinProvider_->setEffectiveThrottleConfig(config);
    }
}

void LiveTelemetryProvider::setTorqueInformedGearboxConfig(
    const twin::TorqueInformedGearboxConfig& config) {
    torqueInformedGearboxConfig_ = config;
    if (twinProvider_) {
        twinProvider_->setTorqueInformedGearboxConfig(config);
    }
}

void LiveTelemetryProvider::setCouplingModel(twin::CouplingModelKind kind) {
    if (twinProvider_) {
        twinProvider_->setCouplingModel(kind);
    }
}

void LiveTelemetryProvider::warmBootToRunning() {
    // Bring the twin to RUNNING (then settle the warm cruise basin) BEFORE the
    // first real frame, mirroring ReplayTelemetryProvider::primeTwinToRunning.
    // This is the warm-boot the live path was missing: without it the twin +
    // engine-sim core start COLD and the first emitted frame blows massive
    // negative exhaust flow (reversion). Live has no parsed samples at
    // Initialize() time (rows are streamed lazily), so we seed the synthetic
    // prime from a running-baseline (light throttle / ~10 km/h, DRIVE) — the same
    // attractor replay's first-sample seed lands in. Live gets NO loop-side
    // settle (an unseekable stream cannot be held at an operating point); the
    // prime plus the synthesised hold signal carry convergence until the first
    // real post-offset row. Idempotent (primeWarmUp guards).
    if (stream_ && twinProvider_) {
        constexpr double kSeedThrottle = 0.20;
        constexpr double kSeedSpeedKmh = 10.0;
        warmBootTwinToRunning(twinProvider_.get(), kSeedThrottle, kSeedSpeedKmh);
        // Record the seed so OnUpdateSimulation can synthesise a valid hold
        // signal between the instant start-from anchor and the first real
        // post-offset row (see OnUpdateSimulation's stream branch).
        primeSeedThrottle_ = kSeedThrottle;
        primeSeedSpeedKmh_ = kSeedSpeedKmh;
        primed_ = true;
    }
}

void LiveTelemetryProvider::provideFeedback(const EngineSimStats& stats) {
    if (twinProvider_) {
        twinProvider_->provideFeedback(stats);
    }
}

void LiveTelemetryProvider::reconfigureProfile(const std::vector<double>& gearRatios,
                                               double diffRatio, double tireRadiusM) {
    if (twinProvider_) {
        twinProvider_->reconfigureProfile(gearRatios, diffRatio, tireRadiusM);
    }
}

void LiveTelemetryProvider::setStartFromS(double s) {
    startFromS_ = s;
}

void LiveTelemetryProvider::setGearboxLogger(twin::IGearboxLogger* logger) {
    if (twinProvider_) {
        twinProvider_->setGearboxLogger(logger);
    }
}
UpstreamSignal LiveTelemetryProvider::getCurrentSignal() const {
    return currentSignal_.load(std::memory_order_seq_cst);
}

// ============================================================================
// CSV stdin path
// ============================================================================

bool LiveTelemetryProvider::ensureHeaderParsed() {
    // Parse the header once (the first non-blank line, after any hint lines).
    // Re-running a data row through parseHeader resets the header and returns
    // false — which (in an older form) froze currentSample_ at row #1 for the
    // whole run.
    if (headerParsed_) return true;
    std::string line;
    while (std::getline(*stream_, line)) {
        if (isBlankLine(line)) continue;
        if (tryParseSourceSkipHint(line)) continue;
        if (csvParser_.parseHeader(line, lastError_)) {
            headerParsed_ = true;
            return true;
        }
        return false;  // first non-hint non-blank line was not a usable header
    }
    eofSeen_ = true;  // no header before EOF
    return false;
}

bool LiveTelemetryProvider::tryParseSourceSkipHint(std::string_view line) {
    // In-band protocol from vehicle-sim's stdout-csv replay: when the SOURCE
    // skipped a --start-from prefix it emits "#vs-start-from <s>" once, before
    // the CSV header, declaring how much of the recording never reaches this
    // consumer. Without the hint the first delivered row looks like the
    // recording's t0 and the display timecode silently restarts at [00:00].
    constexpr std::string_view kPrefix = "#vs-start-from ";
    if (line.size() < kPrefix.size() || line.substr(0, kPrefix.size()) != kPrefix) {
        return false;
    }
    const std::string value(line.substr(kPrefix.size()));
    char* end = nullptr;
    const double seconds = std::strtod(value.c_str(), &end);
    const std::string_view rest(end);
    const auto* trailingJunk = std::find_if_not(rest.begin(), rest.end(),
                                                [](unsigned char c) { return std::isspace(c) != 0; });
    if (end == value.c_str() || trailingJunk != rest.end() || seconds < 0.0) {
        // Malformed hint: consume nothing further, treat as absent (the line
        // then fails header parsing; the retry loop steps past it and the run
        // degrades to the legacy local timecode rather than aborting).
        return false;
    }
    if (sourceSkipHintS_ == 0.0) sourceSkipHintS_ = seconds;  // first hint wins
    return true;
}

double LiveTelemetryProvider::effectiveStartFromS() const {
    return sourceSkipHintS_ + (startFromS_ > 0.0 ? startFromS_ : 0.0);
}

bool LiveTelemetryProvider::isBlankLine(std::string_view s) {
    const auto notSpace = [](unsigned char c) { return !std::isspace(c); };
    return std::find_if(s.begin(), s.end(), notSpace) == s.end();
}

bool LiveTelemetryProvider::isSampleBlank(const CsvSample& s) {
    // A row is real telemetry when ANY expected column carries a decoded value:
    // brake_light (0 or 1 — both are decoded), a PRNDL gear_selector string, a
    // gear number, a clutch value, a road speed off the dyno-off sentinel, a
    // non-zero throttle, or a non-zero motor torque. During CAN bus wake-up the
    // vehicle decodes brake_light first while throttle/speed are still blank, so
    // the old two-field test silently dropped those rows — and any brake presses
    // they carried — until throttle or speed finally decoded (measured on
    // gt-coldstart.csv: brake decodable from +2.24s, first row surviving the old
    // filter +5.95s). A row where NO column decoded (every field still at its
    // not-decoded default) is a blank wake-up frame and must still be skipped:
    // that skip is what anchors the live clock baseline at the first row with
    // real data, keeping the twin in OFF until telemetry exists.
    return !s.brakeLight.has_value()
        && s.gearSelector.empty()
        && s.gear < 0
        && s.clutchPct < 0.0
        && s.throttle == 0.0
        && s.roadSpeedKmh == -2.0
        && s.motorTorqueNm == 0.0;
}

void LiveTelemetryProvider::updateCurrentSpeedLevel(double relT, double speedKmh) {
    // A new level starts whenever the road speed steps outside the level tolerance.
    // The level START (relT) is recorded once and held until the next step, so
    // interpolatedSpeedKmh()'s fraction sweeps the full ~0.2 s hold instead of
    // tracking the sim clock (which would leave it ~0 — the adjacent-row trap).
    if (!hasCurSpeedLevel_ || std::abs(speedKmh - curSpeedLevelKmh_) > kLevelStepKmh) {
        curSpeedLevelT_ = relT;
        curSpeedLevelKmh_ = speedKmh;
        hasCurSpeedLevel_ = true;
    }
}

double LiveTelemetryProvider::interpolatedSpeedKmh(double simElapsedS) const {
    if (!hasCurSpeedLevel_) return currentSample_.roadSpeedKmh;   // no level seen yet
    if (curSpeedLevelKmh_ == -2.0) return curSpeedLevelKmh_;      // dyno-off sentinel
    // No next level known (true live feed whose next sample has not arrived) or
    // the next level is a dyno-off sentinel: hold the current level rather than
    // ramp into a non-commanded value.
    if (!hasNextSpeedLevel_ || nextSpeedLevelKmh_ == -2.0) return curSpeedLevelKmh_;
    if (nextSpeedLevelT_ <= curSpeedLevelT_) return curSpeedLevelKmh_;  // degenerate bracket
    double frac = (simElapsedS - curSpeedLevelT_) / (nextSpeedLevelT_ - curSpeedLevelT_);
    if (frac < 0.0) frac = 0.0;
    else if (frac > 1.0) frac = 1.0;
    return curSpeedLevelKmh_ + frac * (nextSpeedLevelKmh_ - curSpeedLevelKmh_);
}

// Buffered, timestamp-paced consumption (unifies the former live/paced paths).
// The CSV stream is forward-only and getline is destructive, so upcoming rows are
// staged in rowBuffer_ rather than consumed-and-lost. Each call:
//   1. Refills rowBuffer_ from the stream until its tail is ~kLevelLookaheadS
//      ahead of the sim clock (enough lookahead to see the NEXT speed level, so
//      the road-speed feed can be interpolated across the ~0.2 s CAN hold) or EOF.
//   2. Pops every buffered row at/below the sim clock into currentSample_
//      (selector/throttle/torque/gear — unchanged contract) and updates the
//      current speed-level start.
//   3. Scans the remaining (future) tail for the next speed level.
// Blank/malformed/pre-start-from rows are skipped. 1 s of sim time == 1 s of
// recording time (paced), and on a file-redirected capture the gearbox still
// walks the speed ramp row-by-row (the drain-to-EOF freeze cannot recur: only
// the lookahead tail is ever held, never the whole stream).
void LiveTelemetryProvider::refillRowBuffer(double simElapsedS) {
    constexpr double kLevelLookaheadS = 0.30;  // read-ahead horizon to find the next speed level
    const double timeDivisor = csvParser_.header().timeInMs ? 1000.0 : 1.0;
    while (true) {
        if (!rowBuffer_.empty() &&
            (rowBuffer_.back().timeS - baselineTimeS_) > simElapsedS + kLevelLookaheadS) {
            break;
        }
        // Live-pipe guard: a std::getline on an empty pipe PARKS this (loop)
        // thread until the writer emits the next row. While parked the loop
        // stops stepping the engine, the synthesizer input stops, and the
        // audio ring drains — surfacing as short reads and single-sample
        // waveform steps at device-buffer boundaries (the audible
        // thump/knock in sync-pull mode). With a readiness probe injected we
        // only read when a row is already waiting; otherwise return short and
        // let the sim continue on wall clock with the telemetry it holds.
        // Pipes deliver whole rows atomically (< PIPE_BUF), so "any bytes
        // ready" implies a complete getline without blocking.
        if (streamDataReady_ && !streamDataReady_()) {
            break;
        }
        std::string line;
        if (!std::getline(*stream_, line)) break;  // EOF / error: stop refilling
        if (isBlankLine(line)) continue;
        CsvSample sample;
        std::string parseError;
        if (!csvParser_.parseRow(line, timeDivisor, sample, parseError)) continue;  // malformed
        if (isSampleBlank(sample)) continue;
        // Anchor the recording clock on the FIRST parsed row. With a source
        // skip hint the recording's TRUE t0 is that row's epoch minus the
        // skipped prefix, so every relative time (display [mm:ss], --end-at,
        // row pacing) stays TRUE-recording-relative. streamAnchorTimeS_
        // separately keeps the first DELIVERED row's epoch: the own
        // --start-from window is measured from it, which is what makes a
        // stacked skip additive (source skip + own skip) rather than a max().
        if (streamAnchorTimeS_ < 0.0) {
            streamAnchorTimeS_ = sample.timeS;
            baselineTimeS_ = sample.timeS - sourceSkipHintS_;
        }
        // Instant start-from: discard pre-window rows UNPACED (pure I/O, no sim
        // stepping, no clock waits — the owner-mandated "no wall-clock waiting").
        // Measured from the first DELIVERED row, i.e. ON TOP of any source-side
        // skip. The twin is primed synthetically (warmBootToRunning) and the
        // engine bump-starts at the offset state; prefix data is never simulated.
        if (startFromS_ > 0.0 && (sample.timeS - streamAnchorTimeS_) < startFromS_) {
            continue;
        }
        rowBuffer_.push_back(sample);
    }
}

bool LiveTelemetryProvider::tryReadNextRow(double simElapsedS) {
    if (eofSeen_ || !stream_ || !ensureHeaderParsed()) return false;

    // 1) Refill the lookahead buffer until its tail is far enough ahead (or EOF).
    refillRowBuffer(simElapsedS);

    // 2) Advance currentSample_ through every row the sim clock has reached.
    bool found = false;
    while (!rowBuffer_.empty()) {
        const double relT = rowBuffer_.front().timeS - baselineTimeS_;
        if (relT > simElapsedS) break;
        currentSample_ = rowBuffer_.front();
        rowBuffer_.pop_front();
        hasSample_ = true;
        found = true;
        updateCurrentSpeedLevel(relT, currentSample_.roadSpeedKmh);
    }

    // 3) Find the next speed level in the (future) tail for interpolation.
    hasNextSpeedLevel_ = false;
    if (hasCurSpeedLevel_) {
        for (const CsvSample& s : rowBuffer_) {
            if (std::abs(s.roadSpeedKmh - curSpeedLevelKmh_) > kLevelStepKmh) {
                nextSpeedLevelT_ = s.timeS - baselineTimeS_;
                nextSpeedLevelKmh_ = s.roadSpeedKmh;
                hasNextSpeedLevel_ = true;
                break;
            }
        }
    }

    // EOF only when the stream AND the lookahead buffer are fully drained (a call
    // that surfaced a row stays "connected"; EOF is confirmed on a later call that
    // surfaces nothing) — mirrors the former live/paced contract.
    //
    // Owner decision 2026-08-30 ("EOF = immediate termination"): this now applies
    // to the LIVE pipe path (engine-sim-cli --live-telemetry, e.g. a finite
    // `vehicle-sim --stdout-csv | cli` capture) exactly as it always did to the
    // deterministic path. When there is no data left to drive the sim, the
    // provider disconnects and SimulationLoop::run() exits on !IsConnected() —
    // the whole CLI stops at capture end. NO grace window: a brake-initiated
    // start that is still mid-crank in the last ~0.5s of a capture is
    // consciously cut short (KISS chosen over crank audibility at the tail).
    // A genuinely open live stream (real car feed) never EOFs, so live
    // listening is unaffected; --end-at still bounds bounded runs.
    if (stream_->eof() && rowBuffer_.empty() && !found) {
        if (!eofSeen_) csvParser_.emitRejectionSummary();  // once, on EOF transition
        eofSeen_ = true;
    }
    return found;
}

bridge::GearSelector LiveTelemetryProvider::csvGearSelector() const {
    // No selector commanded (no column / blank) defaults to DRIVE so the twin
    // reaches RUNNING and the auto box can shift.
    if (!hasSample_ || currentSample_.gearSelector.empty()) {
        return bridge::GearSelector::DRIVE;
    }
    switch (std::toupper(static_cast<unsigned char>(currentSample_.gearSelector.front()))) {
        case 'P': return bridge::GearSelector::PARK;
        case 'R': return bridge::GearSelector::REVERSE;
        case 'N': return bridge::GearSelector::NEUTRAL;
        default:  return bridge::GearSelector::DRIVE;  // 'D' or unrecognised
    }
}

uint64_t LiveTelemetryProvider::streamTimestampUtcMs() const {
    // Monotonic non-zero ms: the twin treats timestampUtcMs == 0 as invalid and
    // times out to OFF, so never return zero.
    const auto ms = static_cast<uint64_t>(elapsedS_ * 1000.0);
    return ms > 0 ? ms : 1;
}

} // namespace input
