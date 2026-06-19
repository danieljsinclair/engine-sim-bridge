// LiveTelemetryProvider.cpp
#include "input/LiveTelemetryProvider.h"
#include "simulator/EngineSimTypes.h"

#include <algorithm>
#include <cctype>
#include <iostream>

namespace input {

LiveTelemetryProvider::LiveTelemetryProvider(std::string /*streamId*/, bool autoStart)
    : autoStart_(autoStart), stream_(&std::cin) {
}

LiveTelemetryProvider::LiveTelemetryProvider(std::istream& stream, bool autoStart)
    : autoStart_(autoStart), stream_(&stream) {
}

LiveTelemetryProvider::~LiveTelemetryProvider() {
    Shutdown();
}

bool LiveTelemetryProvider::Initialize() {
    // stdin is always "open"; we just mark connected and wait for the first
    // header line to arrive in OnUpdateSimulation.
    connected_.store(true);
    headerParsed_ = false;
    hasSample_ = false;
    startFired_ = false;
    eofSeen_ = false;
    elapsedS_ = 0.0;
    lastError_.clear();
    return true;
}

void LiveTelemetryProvider::Shutdown() {
    connected_.store(false);
}

bool LiveTelemetryProvider::IsConnected() const {
    return connected_.load();
}

bool LiveTelemetryProvider::tryReadNextRow() {
    if (eofSeen_) return false;

    if (!stream_ || !*stream_) {
        eofSeen_ = true;
        connected_.store(false);
        return false;
    }

    std::string line;
    if (!std::getline(*stream_, line)) {
        eofSeen_ = true;
        connected_.store(false);
        return false;
    }

    // Skip empty lines.
    {
        const auto notSpace = [](unsigned char c) { return !std::isspace(c); };
        auto start = std::find_if(line.begin(), line.end(), notSpace);
        if (start == line.end()) return false;
    }

    if (!headerParsed_) {
        if (!csvParser_.parseHeader(line, lastError_)) {
            connected_.store(false);
            return false;
        }
        headerParsed_ = true;
        return false;  // header is not a data row
    }

    CsvSample sample;
    std::string parseError;
    double timeDivisor = csvParser_.header().timeInMs ? 1000.0 : 1.0;
    if (csvParser_.parseRow(line, timeDivisor, sample, parseError)) {
        currentSample_ = sample;
        hasSample_ = true;
        return true;
    }

    return false;
}

EngineInput LiveTelemetryProvider::OnUpdateSimulation(double dt) {
    elapsedS_ += dt;

    // Try to read a new row from stdin (non-blocking: only what's available).
    // We read at most one row per simulation step to avoid starving the loop.
    tryReadNextRow();

    EngineInput input;

    if (hasSample_) {
        input.throttle = currentSample_.throttle;
        input.roadSpeedKmh = currentSample_.roadSpeedKmh;
        input.gearAbsolute = currentSample_.gear;
        input.clutchPressure = currentSample_.clutchPct;
        input.gearSelector = 0;
        input.gearAutoMode = false;
    }

    // One-shot starter pulse on the first frame so the CrankingController cranks.
    if (autoStart_ && !startFired_) {
        input.starterButton = true;
        startFired_ = true;
    }

    return input;
}

void LiveTelemetryProvider::provideFeedback(const EngineSimStats& /*stats*/) {
    // Live provider does not currently use feedback.
}

} // namespace input
