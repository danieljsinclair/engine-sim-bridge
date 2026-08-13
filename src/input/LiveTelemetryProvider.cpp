// LiveTelemetryProvider.cpp - Live telemetry input provider for engine-sim

#include "input/LiveTelemetryProvider.h"
#include "common/PresetExceptions.h"

#include <cctype>
#include <string_view>

namespace input {

LiveTelemetryProvider::LiveTelemetryProvider(const twin::IceVehicleProfile& profile)
    : ownedProfile_(profile)
    , profile_(ownedProfile_)
    , signalReceived_(false)
    , initialized_(false) {
}

LiveTelemetryProvider::LiveTelemetryProvider(std::istream& stream, bool autoStart, bool liveStream)
    : ownedProfile_(twin::IceVehicleProfile::zf8hp45())
    , profile_(ownedProfile_)
    , stream_(&stream)
    , liveStream_(liveStream) {
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
        elapsedS_ += dt;
        // Time slicing: jump elapsedS_ to startFromS_ so the twin's monotonic
        // timestamp starts from the right point. Mirrors ReplayTelemetryProvider.
        if (startFromS_ >= 0.0 && elapsedS_ < startFromS_) {
            elapsedS_ = startFromS_;
        }
        tryReadNextRow(elapsedS_);

        if (!initialized_.load() || !twinProvider_) {
            lastError_ = "Provider not initialized";
            return EngineInput{};
        }

        UpstreamSignal signal;
        if (hasSample_) {
            signal.throttleFraction = currentSample_.throttle;
            signal.speedKmh = currentSample_.roadSpeedKmh;
            signal.motorTorqueNm = currentSample_.motorTorqueNm;
            // The start/stop decision layer (VehicleStartController, read by
            // StartStopInputAdapter via getCurrentSignal()) needs the raw brake
            // enum and the resolved gear. The CSV path is the documented
            // vehicle-sim --stdout-csv pipe, so it must populate these here —
            // the JSON network path does the equivalent via submitSignal().
            signal.brakePedalState = currentSample_.brakePedalState;
            signal.gearSelector = csvGearSelector();
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
        twinProvider_->setGearSelector(static_cast<int>(csvGearSelector()));
        return twinProvider_->OnUpdateSimulation(dt);
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

    // Delegate to the twin for gearbox/clutch/throttle processing
    input = twinProvider_->OnUpdateSimulation(dt);

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
    // Parse the header once (the first non-blank line). Re-running a data row
    // through parseHeader resets the header and returns false — which (in an
    // older form) froze currentSample_ at row #1 for the whole run.
    if (headerParsed_) return true;
    std::string line;
    while (std::getline(*stream_, line)) {
        if (isBlankLine(line)) continue;
        if (csvParser_.parseHeader(line, lastError_)) {
            headerParsed_ = true;
            return true;
        }
        return false;  // first non-blank line was not a usable header
    }
    eofSeen_ = true;  // no header before EOF
    return false;
}

bool LiveTelemetryProvider::isBlankLine(std::string_view s) {
    const auto notSpace = [](unsigned char c) { return !std::isspace(c); };
    return std::find_if(s.begin(), s.end(), notSpace) == s.end();
}

bool LiveTelemetryProvider::isSampleBlank(const CsvSample& s) {
    // Both throttle=0 AND roadSpeedKmh at the dyno-off sentinel (-2) means no
    // CAN signals were decoded for this row. This is the signature of the blank
    // initial frames emitted while the bus wakes up. Such rows must be skipped
    // so hasSample_ stays false and the twin remains in OFF waiting for real data.
    return s.throttle == 0.0 && s.roadSpeedKmh == -2.0;
}

LiveTelemetryProvider::RowDisposition
LiveTelemetryProvider::classifyRow(const CsvSample& sample, double simElapsedS) {
    // Rows before --start-from are consumed-and-discarded (the stream has no seek).
    if (startFromS_ >= 0.0 && sample.timeS < startFromS_) return RowDisposition::Skip;
    // Anchor absolute epoch-style timeS to a relative offset on the first row.
    if (baselineTimeS_ < 0.0) baselineTimeS_ = sample.timeS;
    // Future row: stop scanning + surface the last in-window row. getline has
    // consumed this row, so it is not re-read next call — a minor ~1-row/call
    // skew on high-rate streams (negligible for the gearbox at ~442 rows/s).
    if (double recordingRelativeS = sample.timeS - baselineTimeS_;
        recordingRelativeS > simElapsedS) {
        return RowDisposition::Future;
    }
    return RowDisposition::Surface;
}

bool LiveTelemetryProvider::tryReadNextRow(double simElapsedS) {
    if (eofSeen_ || !stream_ || !ensureHeaderParsed()) return false;

    // LIVE mode (engine-sim-cli --live-telemetry stdin pipe): surface the LATEST
    // row currently available in the stream every frame. A live pipe delivers
    // rows in real time, so the freshest sample IS the current state — there is
    // no reason to hold an old row until a sim clock "catches up" to its recorded
    // timestamp. Timestamp pacing (tryReadNextRowPaced) is correct for a seekable
    // replay file, but on a live feed it forces the sim clock (elapsedS_, advanced
    // by dt per frame) to reach each row's time before it is shown — adding up to
    // ~0.5s+ latency (worst between sparse rows, e.g. t=0,2,3.5,4,6,8s). Echoing
    // the latest sample immediately makes the live response as instant as keyboard.
    if (liveStream_) return tryReadNextRowLive(simElapsedS);

    return tryReadNextRowPaced(simElapsedS);
}

// LIVE stdin path: surface the latest row available within a small time
// lookahead of the sim clock, with no strict timestamp pacing. Malformed rows,
// --start-from-prior rows, and blank rows (throttle=0 + road speed at the dyno-off
// sentinel) are skipped; everything else updates the current sample.
//
// The lookahead window (kLiveLookaheadS) is the latency/responsiveness trade-off:
//   - A live pipe delivers rows in real time, so the newest row is almost always
//     within the window -> surfaced next frame. Latency stays sub-window (as
//     instant as keyboard), which is what the a321e1f latency fix wanted.
//   - A file-redirected capture is fully buffered, so rows advance through the
//     window as the sim clock (simElapsedS) sweeps forward -> the twin walks the
//     speed ramp and the gearbox shifts. This is what drain-to-EOF broke: that
//     loop read the ENTIRE stream in one call and pinned currentSample_ to the
//     capture's FINAL (often near-standstill) row, freezing the gearbox in 1st.
// We stop reading at the first row beyond the window (or EOF) so the stream
// position advances correctly for both feed types.
bool LiveTelemetryProvider::tryReadNextRowLive(double simElapsedS) {
    // Same in-window scan as the paced path, but with a small lookahead horizon
    // (kLiveLookaheadS) instead of strict simElapsedS gating. This keeps a live
    // feed responsive (rows arriving near real time fall inside the horizon and are
    // surfaced next frame — latency stays sub-window, as instant as keyboard) while
    // still WALKING a file-redirected capture row-by-row as the sim clock sweeps
    // forward, so the twin sees the speed ramp and the gearbox shifts. The strict
    // paced path is correct for seekable replay files but added ~0.5s+ latency on a
    // live pipe (holding rows until the sim clock caught their timestamp); the
    // lookahead removes that wait. The ~1-row/call skew from breaking on the first
    // beyond-horizon row is the same minor effect the paced path already accepts.
    constexpr double kLiveLookaheadS = 0.25;  // latency budget
    const double timeDivisor = csvParser_.header().timeInMs ? 1000.0 : 1.0;
    std::string line;
    CsvSample lastInWindow{};
    bool foundInWindow = false;
    while (std::getline(*stream_, line)) {
        if (isBlankLine(line)) continue;
        CsvSample sample;
        if (std::string parseError; !csvParser_.parseRow(line, timeDivisor, sample, parseError)) {
            continue;  // malformed data row: skip
        }
        if (isSampleBlank(sample)) continue;
        if (startFromS_ >= 0.0 && sample.timeS < startFromS_) continue;
        // Anchor absolute epoch-style timeS to a relative offset on the first row,
        // then compare against the sim clock + lookahead horizon.
        if (baselineTimeS_ < 0.0) baselineTimeS_ = sample.timeS;
        if (const double recordingRelativeS = sample.timeS - baselineTimeS_;
            recordingRelativeS > simElapsedS + kLiveLookaheadS) break;  // future row
        lastInWindow = sample;
        foundInWindow = true;
    }
    // The LIVE path must NOT disconnect at EOF (see tryReadNextRowPaced note):
    // keep the last sample "connected" until --duration bounds the run so the
    // start/stop decision layer can complete its 0.5s crank delay.
    if (!liveStream_ && stream_->eof() && !foundInWindow) eofSeen_ = true;
    if (foundInWindow) {
        currentSample_ = lastInWindow;
        hasSample_ = true;
    }
    return foundInWindow;
}

// Timestamp-paced consumption (replay file path): surface the LAST row whose
// recording-time (relative to baselineTimeS_) is at or before simElapsedS, so
// 1 s of sim time == 1 s of recording time. Fixes the "stuck at 0–3 km/h" bug
// where a high-rate recording was consumed at one row per sim frame. Blank rows
// (no decoded signals) are skipped so they don't pollute the in-window sample.
bool LiveTelemetryProvider::tryReadNextRowPaced(double simElapsedS) {
    CsvSample lastInWindow{};
    bool foundInWindow = false;
    std::string line;
    while (std::getline(*stream_, line)) {
        if (isBlankLine(line)) continue;
        CsvSample sample;
        std::string parseError;
        if (double timeDivisor = csvParser_.header().timeInMs ? 1000.0 : 1.0;
            !csvParser_.parseRow(line, timeDivisor, sample, parseError)) {
            return false;  // malformed data row
        }
        // Skip blank rows (no decoded CAN signals) — they carry no telemetry
        // and would otherwise overwrite a valid in-window sample.
        if (isSampleBlank(sample)) continue;

        const RowDisposition disposition = classifyRow(sample, simElapsedS);
        if (disposition == RowDisposition::Skip) continue;
        if (disposition == RowDisposition::Future) break;
        lastInWindow = sample;
        foundInWindow = true;
    }
    // Mark EOF only on genuine exhaustion — a call that surfaced a row stays
    // "connected"; EOF is confirmed on a later call that surfaces nothing.
    // The LIVE path (engine-sim-cli --live-telemetry, a finite/buffered pipe such
    // as `vehicle-sim --stdout-csv | cli`) must NOT disconnect at EOF: the
    // start/stop decision layer needs wall-clock time AFTER the last row to
    // complete the 0.5s crank delay and pending transitions. Disconnecting here
    // would halt SimulationLoop (run() returns on !IsConnected) before ignition
    // ever fires. The --duration guard bounds the run instead, so we keep the
    // live feed "connected" past EOF and let the last sample ride until duration.
    if (!liveStream_ && stream_->eof() && !foundInWindow) eofSeen_ = true;
    if (foundInWindow) {
        currentSample_ = lastInWindow;
        hasSample_ = true;
    }
    return foundInWindow;  // false at clean EOF after all in-window rows consumed
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
