// ReplayTelemetryProvider.cpp
#include "input/ReplayTelemetryProvider.h"
#include "twin/AutomaticGearbox.h"
#include "twin/IceVehicleProfile.h"
#include "simulator/GearConventions.h"
#include "simulator/EngineSimTypes.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>

namespace input {
namespace {

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

std::vector<std::string> split(const std::string& line, char delim) {
    std::vector<std::string> out;
    std::string field;
    std::stringstream ss(line);
    while (std::getline(ss, field, delim)) out.push_back(field);
    return out;
}

bool parseDouble(const std::string& s, double& out) {
    const std::string t = trim(s);
    if (t.empty()) return false;
    try {
        size_t used = 0;
        out = std::stod(t, &used);
        return used != 0;
    } catch (...) {
        return false;
    }
}

bool parseInt(const std::string& s, int& out) {
    const std::string t = trim(s);
    if (t.empty()) return false;
    try {
        size_t used = 0;
        out = std::stoi(t, &used);
        return used != 0;
    } catch (...) {
        return false;
    }
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

void ReplayTelemetryProvider::reconfigureProfile(const std::vector<double>& gearRatios,
                                                  double diffRatio, double tireRadiusM) {
    if (gearRatios.empty() || !autoGearbox_) return;
    gearboxProfile_.gearRatios = gearRatios;
    gearboxProfile_.diffRatio = diffRatio;
    gearboxProfile_.tireRadiusM = tireRadiusM;
    // Auto-generate shift table from the ratios + redline. Shift speed for each
    // gear = the road speed at which the engine RPM reaches the shift-RPM band
    // (~40% redline at light throttle, ~85% at WOT).
    gearboxProfile_.shiftTableThrottleLevels = {0.05,0.15,0.25,0.40,0.55,0.70,0.80,0.90,0.95,1.00};
    gearboxProfile_.shiftTable.clear();
    for (double thr : gearboxProfile_.shiftTableThrottleLevels) {
        std::vector<double> row;
        double shiftRpm = gearboxProfile_.redlineRpm * (0.40 + 0.45 * thr);
        for (size_t i = 0; i + 1 < gearRatios.size(); ++i) {
            double speedMs = shiftRpm / 60.0 * 2.0 * 3.14159265358979 * tireRadiusM
                           / (gearRatios[i] * diffRatio);
            row.push_back(speedMs * 3.6);  // km/h
        }
        gearboxProfile_.shiftTable.push_back(row);
    }
    // Downshift table: ~70% of the upshift speed (hysteresis)
    gearboxProfile_.separateDownshiftTableEnabled = true;
    gearboxProfile_.downshiftTableThrottleLevels = gearboxProfile_.shiftTableThrottleLevels;
    gearboxProfile_.downshiftTable.clear();
    for (size_t t = 0; t < gearboxProfile_.shiftTable.size(); ++t) {
        std::vector<double> row;
        for (double upSpeed : gearboxProfile_.shiftTable[t]) {
            row.push_back(upSpeed * 0.70);
        }
        gearboxProfile_.downshiftTable.push_back(row);
    }
    gearboxProfile_.hysteresisFactor = 0.85;
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
    int colTime = -1, colThrottle = -1, colRoad = -1, colGear = -1, colClutch = -1, colGearSelector = -1;
    bool headerParsed = false;
    bool timeInMs = false;
    double firstTs = -1.0;

    while (std::getline(in, line)) {
        const std::string trimmed = trim(line);
        if (trimmed.empty()) continue;

        auto fields = split(trimmed, ',');
        if (!headerParsed) {
            for (size_t i = 0; i < fields.size(); ++i) {
                const std::string name = lower(trim(fields[i]));
                if (name == "timestamp_utc_ms" || name == "timestamp_ms" || name == "ts_ms") {
                    colTime = static_cast<int>(i);
                    timeInMs = true;
                } else if (name == "time_s" || name == "time" || name == "t" || name == "timecode") {
                    colTime = static_cast<int>(i);
                } else if (name == "throttle_pct" || name == "throttle" || name == "throttle_percent") {
                    colThrottle = static_cast<int>(i);
                } else if (name == "road_speed_kmh" || name == "road_speed" ||
                           name == "speed_kmh" || name == "speed") {
                    colRoad = static_cast<int>(i);
                } else if (name == "gear") {
                    colGear = static_cast<int>(i);
                } else if (name == "gear_selector" || name == "gearselector") {
                    colGearSelector = static_cast<int>(i);
                } else if (name == "clutch_pct" || name == "clutch") {
                    colClutch = static_cast<int>(i);
                }
            }
            headerParsed = true;
            if (colTime < 0) {
                lastError_ = "Telemetry CSV missing time column (time_s): " + csvPath_;
                return false;
            }
            // Detect raw CAN format (undecoded) — clear error instead of silent
            // failure (which would read 0 throttle/speed and the engine sits idle).
            bool hasCanId = false, hasDataHex = false;
            for (size_t i = 0; i < fields.size(); ++i) {
                const std::string name = lower(trim(fields[i]));
                if (name == "can_id") hasCanId = true;
                if (name == "data_hex") hasDataHex = true;
            }
            if (hasCanId && hasDataHex) {
                lastError_ = "This is a RAW CAN capture (can_id + data_hex columns). "
                             "Decode it first with: vehicle-sim --connect file:" +
                             csvPath_ + " --log-csv decoded.csv  "
                             "then replay the decoded file.";
                return false;
            }
            continue;
        }

        Sample s;
        double v = 0.0;
        if (colTime >= 0 && colTime < static_cast<int>(fields.size()) &&
            parseDouble(fields[colTime], v)) {
            if (firstTs < 0.0) firstTs = v;
            s.timeS = (v - firstTs) / (timeInMs ? 1000.0 : 1.0);
        } else {
            continue;
        }
        if (colThrottle >= 0 && colThrottle < static_cast<int>(fields.size()) &&
            parseDouble(fields[colThrottle], v)) {
            s.throttle = std::clamp(v / 100.0, 0.0, 1.0);
        }
        if (colRoad >= 0 && colRoad < static_cast<int>(fields.size()) &&
            parseDouble(fields[colRoad], v) && v >= 0.0) {
            s.roadSpeedKmh = v;
        }
        int gi = 0;
        if (colGear >= 0 && colGear < static_cast<int>(fields.size()) &&
            parseInt(fields[colGear], gi)) {
            s.gear = gi;
        }
        if (colGearSelector >= 0 && colGearSelector < static_cast<int>(fields.size())) {
            s.gearSelector = trim(fields[colGearSelector]);
        }
        if (colClutch >= 0 && colClutch < static_cast<int>(fields.size()) &&
            parseDouble(fields[colClutch], v) && v >= 0.0) {
            s.clutchPct = std::clamp(v / 100.0, 0.0, 1.0);
        }
        samples_.push_back(s);
    }

    if (samples_.empty()) {
        if (!headerParsed) lastError_ = "Empty telemetry CSV: " + csvPath_;
        return true;
    }
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
    return true;
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
    elapsedS_ += dt;
    EngineInput input;
    const Sample& s = sampleAt(elapsedS_);
    input.throttle = s.throttle;
    input.ignition = ignitionOn_;

    if (autoGearbox_ && gearbox_) {
        // Follow the gear stalk. In PARK/NEUTRAL the clutch disengages + the
        // engine free-revs naturally (no dyno, no pinned RPM, natural idle
        // variation). Only in DRIVE does the gearbox decide gears + the dyno
        // tracks road speed.
        bridge::GearSelector sel = s.gearSelector.empty()
            ? bridge::GearSelector::NEUTRAL : parseGearSelector(s.gearSelector);
        gearbox_->setGearSelector(sel);
        input.gearSelector = static_cast<int>(sel);
        input.roadSpeedKmh = s.roadSpeedKmh;
        if (sel == bridge::GearSelector::DRIVE) {
            double speedForBox = (s.roadSpeedKmh >= 0.0) ? s.roadSpeedKmh : 0.0;
            gearbox_->update(dt, speedForBox, s.throttle, 0.0);
            input.gearAbsolute = gearbox_->getCurrentGear();
            input.gearAutoMode = true;

            // Spike-A — pressure-modulated clutch launch controller (dyno OFF).
            // Drive the WHEELS to the CSV road speed (vehicleSpeedTargetKmh) and
            // let the clutch couple them to the engine. At launch (road slow,
            // high slip) keep pressure LOW so the engine revs freely instead of
            // being dragged; as road speed syncs, ramp pressure -> 1.0 (lockup).
            // Throttle biases the slip: at WOT launch we slip more (rev higher).
            input.vehicleSpeedTargetKmh = s.roadSpeedKmh;
            input.engineRpmFloor = 0.0;  // dyno disabled downstream
            // Speed-based clutch ramp (torque-converter style): low pressure at
            // standstill (clutch slips, engine can rev), ramps to full lockup as
            // road speed approaches ~15 km/h. Throttle reduces the floor (more
            // slip = higher revs at launch).
            constexpr double kSyncKmh = 15.0;
            const double speedFrac = std::clamp(speedForBox / kSyncKmh, 0.0, 1.0);
            const double minPressure = 0.15 * (1.0 - 0.6 * s.throttle);
            input.clutchPressure =
                std::clamp(minPressure + speedFrac * (1.0 - minPressure), 0.0, 1.0);
        } else {
            // PARK/NEUTRAL/REVERSE: force neutral (0 = clutch out, dyno off, free-rev).
            // NOT -1 (which means "don't change" — the gear would stick at 1 from DRIVE).
            input.gearAbsolute = 0;
            input.gearAutoMode = false;
            // Make sure any prior vehicle-speed constraint is released.
            input.vehicleSpeedTargetKmh = -1.0;
        }
    } else {
        // Non-auto (e.g. rev-in-park): no gear forced, clutch disengaged, free-rev.
        input.roadSpeedKmh = s.roadSpeedKmh;
        input.gearAbsolute = s.gear;
        input.gearSelector = (s.gear >= 0) ? s.gear : 0;
        input.clutchPressure = s.clutchPct;
        input.gearAutoMode = false;
    }

    // One-shot starter pulse on the first frame so the CrankingController cranks.
    if (autoStart_ && !startFired_) {
        input.starterButton = true;
        startFired_ = true;
    }

    // Q (quit) + P (preset cycle) during replay.
    if (keyboard_) {
        int key;
        while ((key = keyboard_->getKey()) > 0) {
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
    }

    return input;
}

} // namespace input
