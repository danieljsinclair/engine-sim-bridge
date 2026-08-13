// ReplayTelemetryProvider.cpp
#include "input/ReplayTelemetryProvider.h"
#include "twin/AutomaticGearbox.h"
#include "twin/IceVehicleProfile.h"
#include "twin/SlipLockController.h"
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

} // namespace

ReplayTelemetryProvider::ReplayTelemetryProvider(std::string csvPath, bool autoStart,
                                                 bool autoGearbox)
    : csvPath_(std::move(csvPath)), autoStart_(autoStart), autoGearbox_(autoGearbox) {
    if (autoGearbox_) {
        gearboxProfile_ = twin::IceVehicleProfile::zf8hp45();  // owned; gearbox refs this
        gearbox_ = std::make_unique<twin::AutomaticGearbox>(gearboxProfile_);
        gearbox_->setGearSelector(bridge::GearSelector::DRIVE);
    }
}

ReplayTelemetryProvider::~ReplayTelemetryProvider() = default;

void ReplayTelemetryProvider::provideFeedback(const EngineSimStats& stats) {
    engineRpmFeedback_ = stats.currentRPM;
}

bool ReplayTelemetryProvider::Initialize() {
    if (!parseCsv()) return false;
    connected_ = !samples_.empty();
    if (!connected_) lastError_ = "No telemetry rows parsed from " + csvPath_;
    return connected_;
}

double ReplayTelemetryProvider::durationS() const {
    return samples_.empty() ? 0.0 : samples_.back().timeS;
}

void ReplayTelemetryProvider::setGearboxLogger(twin::IGearboxLogger* logger) {
    // Forwards to the owned AutomaticGearbox so the oracle (section D) can parse
    // per-frame gear/rpm/mph from the gearbox log during replay. No-op when the
    // auto-gearbox is disabled (gearbox_ null).
    if (gearbox_) gearbox_->setLogger(logger);
}

void ReplayTelemetryProvider::reconfigureProfile(const std::vector<double>& gearRatios,
                                                  double diffRatio, double tireRadiusM) {
    if (gearRatios.empty() || !autoGearbox_) return;
    gearboxProfile_.gearRatios = gearRatios;
    gearboxProfile_.diffRatio = diffRatio;
    gearboxProfile_.tireRadiusM = tireRadiusM;
    // BAND-TOP SHIFT MAP — identical to VirtualIceTwin::reconfigureProfile so the
    // replay path and the live (twin) path select the same gear at the same road
    // speed. The OLD map derived each upshift from `redline*(0.40+0.45*thr)`,
    // which at WOT pushed DA1->DA2 to ~33.8 mph (85% redline = 5525 rpm in DA1)
    // and left the box a gear or two LOW at every cruise speed vs the oracle
    // (the "DA1 at 25 mph" / over-rev symptom). The band-top map instead pins
    // each upshift to a ROAD SPEED (the top of the originating gear's band),
    // with only a ±5% throttle nudge, so the gear follows the oracle
    // (DA1->DA2 at ~15 mph ... DA6->DA7 at ~65 mph) independent of throttle.
    //   band tops (mph->km/h): 15,25,35,45,55,65 mph.
    static const std::vector<double> kUpshiftBandTopKmh = {24.14, 40.23, 56.33, 72.42, 88.51, 104.61};
    static constexpr double kDownshiftHysteresisKmh = 5.0;  // held band below each top
    // Narrow throttle sensitivity: light throttle upshifts a touch earlier, WOT
    // holds each gear a touch longer — but always centered on the band top so the
    // map owns steady-state (throttle never breaks convergence).
    auto bandScale = [](double thr) {
        const double t = std::clamp((thr - 0.05) / 0.95, 0.0, 1.0);
        return 0.95 + 0.10 * t;  // 0.95 (light) .. 1.05 (WOT)
    };

    const int numCols = static_cast<int>(gearRatios.size()) - 1;
    std::vector<double> upTops;
    upTops.reserve(numCols);
    for (int i = 0; i < numCols; ++i) {
        if (i < static_cast<int>(kUpshiftBandTopKmh.size())) {
            upTops.push_back(kUpshiftBandTopKmh[i]);
        } else {
            upTops.push_back(kUpshiftBandTopKmh.back() + 10.0 * (i - static_cast<int>(kUpshiftBandTopKmh.size()) + 1));
        }
    }

    gearboxProfile_.shiftTableThrottleLevels = {0.05,0.15,0.25,0.40,0.55,0.70,0.80,0.90,0.95,1.00};
    gearboxProfile_.shiftTable.clear();
    for (double thr : gearboxProfile_.shiftTableThrottleLevels) {
        const double s = bandScale(thr);
        std::vector<double> row;
        for (double top : upTops) {
            row.push_back(top * s);  // upshift N->N+1 at the top of N's band
        }
        gearboxProfile_.shiftTable.push_back(row);
    }
    gearboxProfile_.separateDownshiftTableEnabled = true;
    gearboxProfile_.downshiftTableThrottleLevels = gearboxProfile_.shiftTableThrottleLevels;
    gearboxProfile_.downshiftTable.clear();
    // Downshift G->G-1 fires at the BOTTOM of G's band (the top that entered G)
    // minus hysteresis: a symmetric dead band below the upshift, so a single
    // speed can never satisfy both an up- and a down-shift (no hunting).
    for (double thr : gearboxProfile_.downshiftTableThrottleLevels) {
        const double s = bandScale(thr);
        std::vector<double> row;
        for (double top : upTops) {
            row.push_back(std::max(0.0, top - kDownshiftHysteresisKmh) * s);
        }
        gearboxProfile_.downshiftTable.push_back(row);
    }
    gearboxProfile_.hysteresisFactor = 0.85;
    // Ground the idle floor in the V3's EMERGENT idle (~500 rpm, per the .mr —
    // the M156 has no idle scalar). The old hardcoded 750 was wrong for this
    // engine. Consumed declaratively by the anti-lug guard (idle + margin) and
    // by the SlipLock stall floor (clutch decouples below idle).
    gearboxProfile_.idleRpm = 500.0;
    // Reconstruct the gearbox with the matched profile
    gearbox_ = std::make_unique<twin::AutomaticGearbox>(gearboxProfile_);
    gearbox_->setGearSelector(bridge::GearSelector::DRIVE);
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

// Two recorded samples are treated as the SAME speed level when their road
// speeds differ by less than this. The CAN quantization step is ~0.8 km/h, so
// 0.1 km/h cleanly separates real level changes from float jitter while never
// splitting a genuine level.
constexpr double kLevelStepKmh = 0.1;

double ReplayTelemetryProvider::interpolatedRoadSpeedKmh(double t) const {
    if (samples_.empty()) return -2.0;
    if (t <= samples_.front().timeS) return samples_.front().roadSpeedKmh;
    if (t >= samples_.back().timeS) return samples_.back().roadSpeedKmh;

    // Index of the last sample at/below t (the current sample).
    size_t i = 1;
    for (; i < samples_.size(); ++i) {
        if (samples_[i].timeS > t) break;
    }
    const size_t curIdx = i - 1;
    const double curSpeed = samples_[curIdx].roadSpeedKmh;
    if (curSpeed < 0.0) return curSpeed;  // dyno-off sentinel: pass through

    // Level START: walk back to the first sample of the contiguous same-speed run.
    // This is the fixed anchor that makes the interpolation fraction sweep the
    // whole level (using the tracking sample curIdx would leave frac ~0 because it
    // tracks t).
    size_t startIdx = curIdx;
    while (startIdx > 0 &&
           std::abs(samples_[startIdx - 1].roadSpeedKmh - curSpeed) <= kLevelStepKmh) {
        --startIdx;
    }
    // NEXT level: first sample after curIdx whose speed differs (the ramp target).
    size_t nextIdx = samples_.size();
    for (size_t j = curIdx + 1; j < samples_.size(); ++j) {
        if (std::abs(samples_[j].roadSpeedKmh - curSpeed) > kLevelStepKmh) {
            nextIdx = j;
            break;
        }
    }
    if (nextIdx >= samples_.size()) return curSpeed;  // no next level: hold (steady)

    const double startT = samples_[startIdx].timeS;
    const double nextT = samples_[nextIdx].timeS;
    if (nextT <= startT) return curSpeed;
    double frac = (t - startT) / (nextT - startT);
    if (frac < 0.0) frac = 0.0;
    else if (frac > 1.0) frac = 1.0;
    return curSpeed + frac * (samples_[nextIdx].roadSpeedKmh - curSpeed);
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
    const double smoothRoadSpeedKmh = interpolatedRoadSpeedKmh(elapsedS_);

    if (autoGearbox_ && gearbox_) {
        // Follow the gear stalk. In PARK/NEUTRAL the clutch disengages + the
        // engine free-revs naturally (no dyno, no pinned RPM, natural idle
        // variation). Only in DRIVE does the gearbox decide gears + the dyno
        // tracks road speed.
        const bridge::GearSelector sel = s.gearSelector.empty()
            ? bridge::GearSelector::NEUTRAL : parseGearSelector(s.gearSelector);
        gearbox_->setGearSelector(sel);
        input.gearSelector = static_cast<int>(sel);
        input.roadSpeedKmh = smoothRoadSpeedKmh;
        if (sel == bridge::GearSelector::DRIVE) {
            handleAutoGearboxDrive(input, s, dt, smoothRoadSpeedKmh);
        } else {
            handleAutoGearboxNonDrive(input);
        }
    } else {
        // Non-auto (e.g. rev-in-park): no gear forced, clutch disengaged, free-rev.
        handleNonAutoGearbox(input, s);
    }

    // One-shot starter pulse on the first frame so the CrankingController cranks.
    if (autoStart_ && !startFired_) {
        input.starterButton = true;
        startFired_ = true;
    }

    processKeyboardInput(input);
    return input;
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

void ReplayTelemetryProvider::handleAutoGearboxDrive(EngineInput& input, const Sample& s,
                                                     double dt, double roadSpeedKmh) const {
    // roadSpeedKmh is the interpolated (de-quantized) feed; clamp to >=0 for the
    // gearbox's own speed input (a dyno-off sentinel would otherwise confuse it).
    const double speedForBox = (roadSpeedKmh >= 0.0) ? roadSpeedKmh : 0.0;
    gearbox_->update(dt, speedForBox, s.throttle, 0.0);
    input.gearAbsolute = gearbox_->getCurrentGear();
    input.gearAutoMode = true;

    // SlipLockController — pressure-modulated clutch launch controller
    // (dyno OFF). Drive the WHEELS to the (interpolated) road speed
    // (vehicleSpeedTargetKmh) and let the clutch couple them to the
    // engine via the torque-converter slip characteristic:
    //   - standstill (road-implied < idle):   pressure 0   (engine free to idle, no stall)
    //   - launch under throttle (high slip):  partial       (TC slip in power band)
    //   - road catches up (slip -> 0):        pressure -> 1 (locked, direct coupling)
    //   - decel (engine slower than road):    pressure 1    (locked, engine braking)
    // The stall floor (pressure == 0 whenever roadSpeedImpliedRpm < idleRpm)
    // is the lesson from the stall/redline circle: coupling below idle drags
    // the engine under idle and stalls it. See twin/SlipLockController.h.
    input.vehicleSpeedTargetKmh = roadSpeedKmh;
    input.engineRpmFloor = 0.0;  // dyno disabled downstream

    // Road-speed-implied engine RPM: the RPM the engine would be at if the
    // clutch were locked in the current gear at the current road speed.
    // engineRpm = wheelRadS * gearRatio * diffRatio,  wheelRadS = v / tireRadius.
    const int gear = gearbox_->getCurrentGear();
    double roadSpeedImpliedRpm = gearboxProfile_.idleRpm;  // fallback above the floor
    if (gear >= 1 && gear <= static_cast<int>(gearboxProfile_.gearRatios.size())) {
        const double speedMs = speedForBox / 3.6;
        const double wheelRadS = speedMs / gearboxProfile_.tireRadiusM;
        roadSpeedImpliedRpm = wheelRadS
                              * gearboxProfile_.gearRatios[gear - 1]
                              * gearboxProfile_.diffRatio
                              * 30.0 / 3.14159265358979;
    }

    // maxCreepPressure: clutch pressure at full throttle with zero road
    // speed. Mimics TC fluid coupling — 0.10 = 10% clutch at stall.
    // Tunable: lower = less creep (engine freer to rev), higher = more
    // creep (stronger launch feel, but risk of stall at high throttle).
    constexpr double kMaxCreepPressure = 0.10;
    const twin::SlipLockOutput slipLock = twin::computeSlipLockPressure(
        twin::SlipLockInput{
            engineRpmFeedback_,
            roadSpeedImpliedRpm,
            s.throttle,
            gearboxProfile_.idleRpm,
            gearboxProfile_.redlineRpm},
        kMaxCreepPressure);
    input.clutchPressure = slipLock.clutchPressure;
}

void ReplayTelemetryProvider::handleAutoGearboxNonDrive(EngineInput& input) const {
    // PARK/NEUTRAL/REVERSE: force neutral (0 = clutch out, dyno off, free-rev).
    // NOT -1 (which means "don't change" — the gear would stick at 1 from DRIVE).
    input.gearAbsolute = 0;
    input.gearAutoMode = false;
    // Make sure any prior vehicle-speed constraint is released.
    input.vehicleSpeedTargetKmh = -1.0;
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
