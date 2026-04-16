// InMemoryTelemetry.cpp - Thread-safe in-memory telemetry storage
// Implements both ITelemetryWriter and ITelemetryReader interfaces
// Per-concern atomic sub-structs mirror the ISP structs

#include "telemetry/ITelemetryProvider.h"

namespace telemetry {

// ============================================================================
// InMemoryTelemetry constructor
// ============================================================================

InMemoryTelemetry::InMemoryTelemetry()
    : engineState_()
    , framePerformance_()
    , audioDiagnostics_()
    , audioTiming_()
    , vehicleInputs_()
    , simulatorMetrics_()
{
}

// ============================================================================
// ITelemetryWriter implementation
// ============================================================================

void InMemoryTelemetry::reset() {
    engineState_.currentRPM.store(0.0, std::memory_order_relaxed);
    engineState_.currentLoad.store(0.0, std::memory_order_relaxed);
    engineState_.exhaustFlow.store(0.0, std::memory_order_relaxed);
    engineState_.manifoldPressure.store(0.0, std::memory_order_relaxed);
    engineState_.activeChannels.store(0, std::memory_order_relaxed);
    framePerformance_.processingTimeMs.store(0.0, std::memory_order_relaxed);
    audioDiagnostics_.underrunCount.store(0, std::memory_order_relaxed);
    audioDiagnostics_.bufferHealthPct.store(0.0, std::memory_order_relaxed);
    audioTiming_.renderMs.store(0.0, std::memory_order_relaxed);
    audioTiming_.headroomMs.store(0.0, std::memory_order_relaxed);
    audioTiming_.budgetPct.store(0.0, std::memory_order_relaxed);
    audioTiming_.framesRequested.store(0, std::memory_order_relaxed);
    audioTiming_.framesRendered.store(0, std::memory_order_relaxed);
    audioTiming_.callbackRateHz.store(0.0, std::memory_order_relaxed);
    audioTiming_.generatingRateFps.store(0.0, std::memory_order_relaxed);
    audioTiming_.trendPct.store(0.0, std::memory_order_relaxed);
    vehicleInputs_.throttlePosition.store(0.0, std::memory_order_relaxed);
    vehicleInputs_.ignitionOn.store(false, std::memory_order_relaxed);
    vehicleInputs_.starterMotorEngaged.store(false, std::memory_order_relaxed);
    simulatorMetrics_.timestamp.store(0.0, std::memory_order_relaxed);
}

// ============================================================================
// ISP per-component write methods
// ============================================================================

void InMemoryTelemetry::writeEngineState(const EngineStateTelemetry& state) {
    engineState_.currentRPM.store(state.currentRPM, std::memory_order_relaxed);
    engineState_.currentLoad.store(state.currentLoad, std::memory_order_relaxed);
    engineState_.exhaustFlow.store(state.exhaustFlow, std::memory_order_relaxed);
    engineState_.manifoldPressure.store(state.manifoldPressure, std::memory_order_relaxed);
    engineState_.activeChannels.store(state.activeChannels, std::memory_order_relaxed);
}

void InMemoryTelemetry::writeFramePerformance(const FramePerformanceTelemetry& perf) {
    framePerformance_.processingTimeMs.store(perf.processingTimeMs, std::memory_order_relaxed);
}

void InMemoryTelemetry::writeAudioDiagnostics(const AudioDiagnosticsTelemetry& diag) {
    audioDiagnostics_.underrunCount.store(diag.underrunCount, std::memory_order_relaxed);
    audioDiagnostics_.bufferHealthPct.store(diag.bufferHealthPct, std::memory_order_relaxed);
}

void InMemoryTelemetry::writeAudioTiming(const AudioTimingTelemetry& timing) {
    audioTiming_.renderMs.store(timing.renderMs, std::memory_order_relaxed);
    audioTiming_.headroomMs.store(timing.headroomMs, std::memory_order_relaxed);
    audioTiming_.budgetPct.store(timing.budgetPct, std::memory_order_relaxed);
    audioTiming_.framesRequested.store(timing.framesRequested, std::memory_order_relaxed);
    audioTiming_.framesRendered.store(timing.framesRendered, std::memory_order_relaxed);
    audioTiming_.callbackRateHz.store(timing.callbackRateHz, std::memory_order_relaxed);
    audioTiming_.generatingRateFps.store(timing.generatingRateFps, std::memory_order_relaxed);
    audioTiming_.trendPct.store(timing.trendPct, std::memory_order_relaxed);
}

void InMemoryTelemetry::writeVehicleInputs(const VehicleInputsTelemetry& inputs) {
    vehicleInputs_.throttlePosition.store(inputs.throttlePosition, std::memory_order_relaxed);
    vehicleInputs_.ignitionOn.store(inputs.ignitionOn, std::memory_order_relaxed);
    vehicleInputs_.starterMotorEngaged.store(inputs.starterMotorEngaged, std::memory_order_relaxed);
}

void InMemoryTelemetry::writeSimulatorMetrics(const SimulatorMetricsTelemetry& metrics) {
    simulatorMetrics_.timestamp.store(metrics.timestamp, std::memory_order_relaxed);
}

// ============================================================================
// ISP per-component read methods
// ============================================================================

EngineStateTelemetry InMemoryTelemetry::getEngineState() const {
    EngineStateTelemetry state;
    state.currentRPM = engineState_.currentRPM.load(std::memory_order_relaxed);
    state.currentLoad = engineState_.currentLoad.load(std::memory_order_relaxed);
    state.exhaustFlow = engineState_.exhaustFlow.load(std::memory_order_relaxed);
    state.manifoldPressure = engineState_.manifoldPressure.load(std::memory_order_relaxed);
    state.activeChannels = engineState_.activeChannels.load(std::memory_order_relaxed);
    return state;
}

FramePerformanceTelemetry InMemoryTelemetry::getFramePerformance() const {
    FramePerformanceTelemetry perf;
    perf.processingTimeMs = framePerformance_.processingTimeMs.load(std::memory_order_relaxed);
    return perf;
}

AudioDiagnosticsTelemetry InMemoryTelemetry::getAudioDiagnostics() const {
    AudioDiagnosticsTelemetry diag;
    diag.underrunCount = audioDiagnostics_.underrunCount.load(std::memory_order_relaxed);
    diag.bufferHealthPct = audioDiagnostics_.bufferHealthPct.load(std::memory_order_relaxed);
    return diag;
}

AudioTimingTelemetry InMemoryTelemetry::getAudioTiming() const {
    AudioTimingTelemetry timing;
    timing.renderMs = audioTiming_.renderMs.load(std::memory_order_relaxed);
    timing.headroomMs = audioTiming_.headroomMs.load(std::memory_order_relaxed);
    timing.budgetPct = audioTiming_.budgetPct.load(std::memory_order_relaxed);
    timing.framesRequested = audioTiming_.framesRequested.load(std::memory_order_relaxed);
    timing.framesRendered = audioTiming_.framesRendered.load(std::memory_order_relaxed);
    timing.callbackRateHz = audioTiming_.callbackRateHz.load(std::memory_order_relaxed);
    timing.generatingRateFps = audioTiming_.generatingRateFps.load(std::memory_order_relaxed);
    timing.trendPct = audioTiming_.trendPct.load(std::memory_order_relaxed);
    return timing;
}

VehicleInputsTelemetry InMemoryTelemetry::getVehicleInputs() const {
    VehicleInputsTelemetry inputs;
    inputs.throttlePosition = vehicleInputs_.throttlePosition.load(std::memory_order_relaxed);
    inputs.ignitionOn = vehicleInputs_.ignitionOn.load(std::memory_order_relaxed);
    inputs.starterMotorEngaged = vehicleInputs_.starterMotorEngaged.load(std::memory_order_relaxed);
    return inputs;
}

SimulatorMetricsTelemetry InMemoryTelemetry::getSimulatorMetrics() const {
    SimulatorMetricsTelemetry metrics;
    metrics.timestamp = simulatorMetrics_.timestamp.load(std::memory_order_relaxed);
    return metrics;
}

} // namespace telemetry
