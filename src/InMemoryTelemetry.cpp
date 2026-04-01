// InMemoryTelemetry.cpp - Thread-safe in-memory telemetry storage
// Implements both ITelemetryWriter and ITelemetryReader interfaces

#include "ITelemetryProvider.h"
#include <cstring>

namespace telemetry {

// ============================================================================
// AtomicData constructor - Initialize all atomic members
// ============================================================================

InMemoryTelemetry::AtomicData::AtomicData()
    : currentRPM(0.0)
    , currentLoad(0.0)
    , exhaustFlow(0.0)
    , manifoldPressure(0.0)
    , activeChannels(0)
    , processingTimeMs(0.0)
    , underrunCount(0)
    , bufferHealthPct(0.0)
    , throttlePosition(0.0)
    , ignitionOn(false)
    , starterMotorEngaged(false)
    , timestamp(0.0)
{
}

// ============================================================================
// InMemoryTelemetry constructor
// ============================================================================

InMemoryTelemetry::InMemoryTelemetry()
    : data_()
{
}

// ============================================================================
// ITelemetryWriter implementation
// ============================================================================

void InMemoryTelemetry::write(const TelemetryData& data) {
    // Thread-safe atomic stores - each write is independent
    // Using memory_order_relaxed for performance - we don't need
    // synchronization between different fields, just atomicity per field
    data_.currentRPM.store(data.currentRPM, std::memory_order_relaxed);
    data_.currentLoad.store(data.currentLoad, std::memory_order_relaxed);
    data_.exhaustFlow.store(data.exhaustFlow, std::memory_order_relaxed);
    data_.manifoldPressure.store(data.manifoldPressure, std::memory_order_relaxed);
    data_.activeChannels.store(data.activeChannels, std::memory_order_relaxed);
    data_.processingTimeMs.store(data.processingTimeMs, std::memory_order_relaxed);
    data_.underrunCount.store(data.underrunCount, std::memory_order_relaxed);
    data_.bufferHealthPct.store(data.bufferHealthPct, std::memory_order_relaxed);
    data_.throttlePosition.store(data.throttlePosition, std::memory_order_relaxed);
    data_.ignitionOn.store(data.ignitionOn, std::memory_order_relaxed);
    data_.starterMotorEngaged.store(data.starterMotorEngaged, std::memory_order_relaxed);
    data_.timestamp.store(data.timestamp, std::memory_order_relaxed);
}

void InMemoryTelemetry::reset() {
    // Reset all counters to initial state
    TelemetryData zeroData = {};
    write(zeroData);
}

// ============================================================================
// ITelemetryReader implementation
// ============================================================================

TelemetryData InMemoryTelemetry::getSnapshot() const {
    TelemetryData snapshot;

    // Thread-safe atomic loads - create consistent snapshot
    // Using memory_order_relaxed for performance - individual fields
    // may be slightly inconsistent but that's acceptable for telemetry
    snapshot.currentRPM = data_.currentRPM.load(std::memory_order_relaxed);
    snapshot.currentLoad = data_.currentLoad.load(std::memory_order_relaxed);
    snapshot.exhaustFlow = data_.exhaustFlow.load(std::memory_order_relaxed);
    snapshot.manifoldPressure = data_.manifoldPressure.load(std::memory_order_relaxed);
    snapshot.activeChannels = data_.activeChannels.load(std::memory_order_relaxed);
    snapshot.processingTimeMs = data_.processingTimeMs.load(std::memory_order_relaxed);
    snapshot.underrunCount = data_.underrunCount.load(std::memory_order_relaxed);
    snapshot.bufferHealthPct = data_.bufferHealthPct.load(std::memory_order_relaxed);
    snapshot.throttlePosition = data_.throttlePosition.load(std::memory_order_relaxed);
    snapshot.ignitionOn = data_.ignitionOn.load(std::memory_order_relaxed);
    snapshot.starterMotorEngaged = data_.starterMotorEngaged.load(std::memory_order_relaxed);
    snapshot.timestamp = data_.timestamp.load(std::memory_order_relaxed);

    return snapshot;
}

} // namespace telemetry
