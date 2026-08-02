// LiveTelemetryProvider.cpp - Live telemetry input provider for engine-sim

#include "input/LiveTelemetryProvider.h"
#include "common/PresetExceptions.h"

#include <cctype>

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
    , stream_(&stream) {
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
        tryReadNextRow();

        if (!initialized_.load() || !twinProvider_) {
            lastError_ = "Provider not initialized";
            return EngineInput{};
        }

        UpstreamSignal signal;
        if (hasSample_) {
            signal.throttleFraction = currentSample_.throttle;
            signal.speedKmh = currentSample_.roadSpeedKmh;
            // The row is valid telemetry even when speed is blank (dyno off); a
            // non-zero timestamp keeps the twin's telemetry-timeout guard happy.
            signal.isValid = true;
            signal.timestampUtcMs = streamTimestampUtcMs();
        }

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

void LiveTelemetryProvider::provideFeedback(const EngineSimStats& stats) {
    if (twinProvider_) {
        twinProvider_->provideFeedback(stats);
    }
}

UpstreamSignal LiveTelemetryProvider::getCurrentSignal() const {
    return currentSignal_.load(std::memory_order_seq_cst);
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
        if (double timeDivisor = csvParser_.header().timeInMs ? 1000.0 : 1.0;
            csvParser_.parseRow(line, timeDivisor, sample, parseError)) {
            currentSample_ = sample;
            hasSample_ = true;
            return true;
        }
        return false;  // malformed first data row
    }
    eofSeen_ = true;
    return false;
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
