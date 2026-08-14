// ReplayTelemetryProvider.cpp
#include "input/ReplayTelemetryProvider.h"
#include "input/VirtualIceInputProvider.h"
#include "twin/IceVehicleProfile.h"
#include "io/UpstreamSignal.h"
#include "simulator/GearConventions.h"
#include "simulator/EngineSimTypes.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
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

// Two recorded samples are treated as the SAME speed level when their road
// speeds differ by less than this. The CAN quantization step is ~0.8 km/h, so
// 0.1 km/h cleanly separates real level changes from float jitter while never
// splitting a genuine level.
constexpr double kLevelStepKmh = 0.1;

// Continuous (de-quantized) road speed at time t. A recorded CSV road speed is a
// STAIRCASE: CAN-quantized (~0.8 km/h) with each level held ~0.2 s. Feeding the
// held value makes a hard-pinned/slip-locked RPM step in lockstep (the "rpm
// stepper"). This linearly interpolates between the START of the speed level
// containing t and the START of the NEXT speed level, by fractional time — so the
// feed ramps smoothly across the whole hold instead of step-holding. Dyno-off
// sentinels (speed < 0) are passed through unchanged. Pure feed interpolation.
double interpolateRoadSpeedKmh(const std::vector<CsvSample>& samples, double t) {
    if (samples.empty()) return -2.0;
    if (t <= samples.front().timeS) return samples.front().roadSpeedKmh;
    if (t >= samples.back().timeS) return samples.back().roadSpeedKmh;

    size_t i = 1;
    for (; i < samples.size(); ++i) {
        if (samples[i].timeS > t) break;
    }
    const size_t curIdx = i - 1;
    const double curSpeed = samples[curIdx].roadSpeedKmh;
    if (curSpeed < 0.0) return curSpeed;

    size_t startIdx = curIdx;
    while (startIdx > 0 &&
           std::abs(samples[startIdx - 1].roadSpeedKmh - curSpeed) <= kLevelStepKmh) {
        --startIdx;
    }
    size_t nextIdx = samples.size();
    for (size_t j = curIdx + 1; j < samples.size(); ++j) {
        if (std::abs(samples[j].roadSpeedKmh - curSpeed) > kLevelStepKmh) {
            nextIdx = j;
            break;
        }
    }
    if (nextIdx >= samples.size()) return curSpeed;

    const double startT = samples[startIdx].timeS;
    const double nextT = samples[nextIdx].timeS;
    if (nextT <= startT) return curSpeed;
    double frac = (t - startT) / (nextT - startT);
    if (frac < 0.0) frac = 0.0;
    else if (frac > 1.0) frac = 1.0;
    return curSpeed + frac * (samples[nextIdx].roadSpeedKmh - curSpeed);
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
        return true;
    }
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

EngineInput ReplayTelemetryProvider::OnUpdateSimulation(double dt) {
    EngineInput input;
    if (applyTimeSlicing(input, dt)) return input;

    const Sample& s = sampleAt(elapsedS_);
    currentTimestampS_ = s.timeS;
    buildBaseEngineInput(input, s);
    // De-quantized road speed: the recorded value is a CAN staircase (levels held
    // ~0.2 s). Feed the interpolated ramp so a pinned/slip-locked RPM does not
    // step in lockstep with the staircase. s still supplies throttle/selector/gear.
    const double smoothRoadSpeedKmh = interpolateRoadSpeedKmh(samples_, elapsedS_);

    if (autoGearbox_ && twinProvider_) {
        // Route the replay sample THROUGH the twin provider (the SAME twin the
        // live/Demo paths use) so --coupling-model / --wheel-coupling take effect
        // on replay. The twin owns the gearbox/clutch/coupling-model processing and
        // the cranking lifecycle. In PARK/NEUTRAL the twin disengages the clutch
        // and the engine free-revs; only in DRIVE does it couple + track road speed.
        input = driveThroughTwin(s, dt, smoothRoadSpeedKmh);
    } else {
        // Non-auto (e.g. rev-in-park): no gear forced, clutch disengaged, free-rev.
        handleNonAutoGearbox(input, s);
        // One-shot starter pulse on the first frame so the CrankingController
        // cranks. (The auto/twin path above lets the twin own the cranking
        // lifecycle — mirroring the live path — so the pulse is only needed here.)
        if (autoStart_ && !startFired_) {
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
    signal.isValid = true;
    // Monotonic non-zero timestamp (the twin treats 0 as invalid + times out).
    signal.timestampUtcMs = std::max<uint64_t>(1, static_cast<uint64_t>(elapsedS_ * 1000.0));
    // The CSV stalk (D/R/N/P) drives the twin's selector. A missing/blank selector
    // defaults to DRIVE so the bench-driving run engages and exercises the coupling
    // model (mirrors the live CSV path's default-selector contract).
    const bridge::GearSelector sel = s.gearSelector.empty()
        ? bridge::GearSelector::DRIVE : parseGearSelector(s.gearSelector);
    twinProvider_->setGearSelector(static_cast<int>(sel));
    twinProvider_->setUpstreamSignal(signal);
    // VirtualIceInputProvider maps TwinOutput -> EngineInput (throttle, gear,
    // clutchPressure, ignition, starter, gearSelector, gearAutoMode, road speed
    // pin, injected torque, diagnostics) — identical to the live path.
    EngineInput input = twinProvider_->OnUpdateSimulation(dt);

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
    const Sample& first = samples_.front();
    const bridge::GearSelector sel = first.gearSelector.empty()
        ? bridge::GearSelector::DRIVE : parseGearSelector(first.gearSelector);
    twinProvider_->setGearSelector(static_cast<int>(sel));

    input::UpstreamSignal signal;
    signal.throttleFraction = first.throttle;
    signal.speedKmh = first.roadSpeedKmh;
    signal.motorTorqueNm = first.motorTorqueNm;
    signal.isValid = true;
    signal.timestampUtcMs = 1;  // non-zero -> twin treats as valid telemetry
    twinProvider_->setUpstreamSignal(signal);

    // Past the 3s CRANK_FALLBACK_DURATION_S crank budget (CRANKING -> IDLE), then
    // a DRIVE selector promotes IDLE -> RUNNING. 100 frames @ 0.05s = 5s sim, with
    // margin over the 3s fallback so RUNNING is guaranteed before frame 1.
    constexpr double kPrimeDt = 0.05;
    constexpr int kPrimeFrames = 100;
    for (int i = 0; i < kPrimeFrames; ++i) {
        twinProvider_->OnUpdateSimulation(kPrimeDt);
    }
}

bool ReplayTelemetryProvider::applyTimeSlicing(EngineInput& input, double dt) {
    elapsedS_ += dt;

    // Time slicing: skip samples before startFromS.
    if (startFromS_ >= 0.0 && elapsedS_ < startFromS_) {
        elapsedS_ = startFromS_;
    }

    // Time slicing: stop at endAtS — emit an ignition-off frame and signal the
    // caller to return it immediately (no further processing this frame).
    if (endAtS_ >= 0.0 && elapsedS_ >= endAtS_) {
        if (session_) session_->stop();
        input.ignition = false;
        return true;
    }
    return false;
}

void ReplayTelemetryProvider::buildBaseEngineInput(EngineInput& input, const Sample& s) const {
    input.replayTimestampS = currentTimestampS_;
    input.throttle = s.throttle;
    input.ignition = ignitionOn_;
}

void ReplayTelemetryProvider::handleNonAutoGearbox(EngineInput& input, const Sample& s) const {
    input.roadSpeedKmh = s.roadSpeedKmh;
    input.gearAbsolute = s.gear;
    input.gearSelector = (s.gear >= 0) ? s.gear : 0;
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
