// ReplayTelemetryProvider.cpp
#include "input/ReplayTelemetryProvider.h"
#include "twin/AutomaticGearbox.h"
#include "twin/IceVehicleProfile.h"
#include "simulator/GearConventions.h"

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

bool ReplayTelemetryProvider::Initialize() {
    if (!parseCsv()) return false;
    connected_ = !samples_.empty();
    if (!connected_) lastError_ = "No telemetry rows parsed from " + csvPath_;
    return connected_;
}

double ReplayTelemetryProvider::durationS() const {
    return samples_.empty() ? 0.0 : samples_.back().timeS;
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
    input.ignition = true;

    if (autoGearbox_ && gearbox_) {
        // Auto: the AutomaticGearbox decides gears from the CSV road speed, and
        // follows the gear_selector stalk (P/R/N/D). Coherent with the dyno-hold.
        if (!s.gearSelector.empty()) {
            gearbox_->setGearSelector(parseGearSelector(s.gearSelector));
        }
        double speedForBox = (s.roadSpeedKmh >= 0.0) ? s.roadSpeedKmh : 0.0;
        gearbox_->update(dt, speedForBox, s.throttle, 0.0);
        input.gearAbsolute = gearbox_->getCurrentGear();
        input.gearSelector = static_cast<int>(gearbox_->getGearSelector());
        input.gearAutoMode = (gearbox_->getGearSelector() == bridge::GearSelector::DRIVE);
        input.roadSpeedKmh = s.roadSpeedKmh;
        // Launch RPM floor (TC stopgap): rev to idle + throttle-scaled at standstill.
        input.engineRpmFloor = gearboxProfile_.idleRpm +
            s.throttle * (0.5 * gearboxProfile_.redlineRpm - gearboxProfile_.idleRpm);
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
            }
        }
    }

    return input;
}

} // namespace input
