// LiveTelemetryProvider.cpp - Live telemetry input provider for engine-sim

#include "input/LiveTelemetryProvider.h"

namespace input {

LiveTelemetryProvider::LiveTelemetryProvider(const twin::IceVehicleProfile& profile)
    : ownedProfile_(profile)
    , profile_(ownedProfile_)
    , signalReceived_(false)
    , initialized_(false) {
}

LiveTelemetryProvider::LiveTelemetryProvider(std::istream& stream, bool autoStart)
    : ownedProfile_(twin::IceVehicleProfile::zf8hp45())
    , profile_(ownedProfile_)
    , stream_(&stream)
    , autoStart_(autoStart) {
}

LiveTelemetryProvider::~LiveTelemetryProvider() {
    Shutdown();
}

bool LiveTelemetryProvider::Initialize() {
    if (initialized_.load()) {
        lastError_ = "Already initialized";
        return false;
    }

    // CSV stdin path: no twin provider needed
    if (stream_) {
        hasSample_ = false;
        startFired_ = false;
        eofSeen_ = false;
        elapsedS_ = 0.0;
        initialized_.store(true);
        return true;
    }

    try {
        twinProvider_ = std::make_unique<VirtualIceInputProvider>(profile_);
        if (!twinProvider_->Initialize()) {
            lastError_ = "Failed to initialize twin provider: " + twinProvider_->GetLastError();
            twinProvider_.reset();
            return false;
        }
        initialized_.store(true);
        return true;
    } catch (const std::exception& e) {
        lastError_ = std::string("Failed to create twin provider: ") + e.what();
        return false;
    }
}

void LiveTelemetryProvider::Shutdown() {
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
    // CSV stdin path
    if (stream_) {
        elapsedS_ += dt;
        tryReadNextRow();

        EngineInput input;
        if (hasSample_) {
            input.throttle = currentSample_.throttle;
            input.roadSpeedKmh = currentSample_.roadSpeedKmh;
            input.gearAbsolute = currentSample_.gear;
            input.clutchPressure = currentSample_.clutchPct;
        }
        if (autoStart_ && !startFired_) {
            input.starterButton = true;
            startFired_ = true;
        }
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
    currentSignal_.store(signal, std::memory_order_relaxed);
    signalReceived_.store(true, std::memory_order_relaxed);
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

void LiveTelemetryProvider::provideFeedback(const EngineSimStats& stats) {
    if (twinProvider_) {
        twinProvider_->provideFeedback(stats);
    }
}

UpstreamSignal LiveTelemetryProvider::getCurrentSignal() const {
    return currentSignal_.load(std::memory_order_relaxed);
}

// ============================================================================
// CSV stdin path
// ============================================================================

bool LiveTelemetryProvider::tryReadNextRow() {
    if (eofSeen_ || !stream_) return false;
    if (!stream_->good()) { eofSeen_ = true; return false; }

    std::string line;
    if (!std::getline(*stream_, line)) { eofSeen_ = true; return false; }

    // Skip empty/whitespace-only lines
    const auto notSpace = [](unsigned char c) { return !std::isspace(c); };
    if (std::find_if(line.begin(), line.end(), notSpace) == line.end()) return false;

    // First non-empty line is the header (parseHeader trims internally)
    if (!csvParser_.parseHeader(line, lastError_)) return false;

    // Now read the first data row
    while (std::getline(*stream_, line)) {
        if (std::find_if(line.begin(), line.end(), notSpace) == line.end()) continue;
        CsvSample sample;
        std::string parseError;
        double timeDivisor = csvParser_.header().timeInMs ? 1000.0 : 1.0;
        if (csvParser_.parseRow(line, timeDivisor, sample, parseError)) {
            currentSample_ = sample;
            hasSample_ = true;
            return true;
        }
        return false;  // malformed first data row
    }
    eofSeen_ = true;
    return false;
}

} // namespace input
