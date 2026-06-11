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
    engineState_.currentRPM.store(0.0);
    engineState_.currentLoad.store(0.0);
    engineState_.exhaustFlow.store(0.0);
    engineState_.manifoldPressure.store(0.0);
    engineState_.activeChannels.store(0);
    engineState_.gear.store(0);
    engineState_.speedMph.store(0.0);
    engineState_.enginePhase.store(0);
    framePerformance_.processingTimeMs.store(0.0);
    audioDiagnostics_.underrunCount.store(0);
    audioDiagnostics_.bufferHealthPct.store(0.0);
    audioTiming_.renderMs.store(0.0);
    audioTiming_.headroomMs.store(0.0);
    audioTiming_.budgetPct.store(0.0);
    audioTiming_.framesRequested.store(0);
    audioTiming_.framesRendered.store(0);
    audioTiming_.callbackRateHz.store(0.0);
    audioTiming_.generatingRateFps.store(0.0);
    audioTiming_.trendPct.store(0.0);
    vehicleInputs_.throttlePosition.store(0.0);
    vehicleInputs_.ignitionOn.store(false);
    vehicleInputs_.starterMotorEngaged.store(false);
    simulatorMetrics_.timestamp.store(0.0);
}

// ============================================================================
// ISP per-component write methods
// ============================================================================

void InMemoryTelemetry::writeEngineState(const EngineStateTelemetry& state) {
    engineState_.currentRPM.store(state.currentRPM);
    engineState_.currentLoad.store(state.currentLoad);
    engineState_.exhaustFlow.store(state.exhaustFlow);
    engineState_.manifoldPressure.store(state.manifoldPressure);
    engineState_.activeChannels.store(state.activeChannels);
    engineState_.gear.store(state.gear);
    engineState_.speedMph.store(state.speedMph);
    engineState_.enginePhase.store(static_cast<int>(state.enginePhase));
}

void InMemoryTelemetry::writeFramePerformance(const FramePerformanceTelemetry& perf) {
    framePerformance_.processingTimeMs.store(perf.processingTimeMs);
}

void InMemoryTelemetry::writeAudioDiagnostics(const AudioDiagnosticsTelemetry& diag) {
    audioDiagnostics_.underrunCount.store(diag.underrunCount);
    audioDiagnostics_.bufferHealthPct.store(diag.bufferHealthPct);
}

void InMemoryTelemetry::writeAudioTiming(const AudioTimingTelemetry& timing) {
    audioTiming_.renderMs.store(timing.renderMs);
    audioTiming_.headroomMs.store(timing.headroomMs);
    audioTiming_.budgetPct.store(timing.budgetPct);
    audioTiming_.framesRequested.store(timing.framesRequested);
    audioTiming_.framesRendered.store(timing.framesRendered);
    audioTiming_.callbackRateHz.store(timing.callbackRateHz);
    audioTiming_.generatingRateFps.store(timing.generatingRateFps);
    audioTiming_.trendPct.store(timing.trendPct);
}

void InMemoryTelemetry::writeVehicleInputs(const VehicleInputsTelemetry& inputs) {
    vehicleInputs_.throttlePosition.store(inputs.throttlePosition);
    vehicleInputs_.ignitionOn.store(inputs.ignitionOn);
    vehicleInputs_.starterMotorEngaged.store(inputs.starterMotorEngaged);
}

void InMemoryTelemetry::writeSimulatorMetrics(const SimulatorMetricsTelemetry& metrics) {
    simulatorMetrics_.timestamp.store(metrics.timestamp);
}

// ============================================================================
// ISP per-component read methods
// ============================================================================

EngineStateTelemetry InMemoryTelemetry::getEngineState() const {
    EngineStateTelemetry state;
    state.currentRPM = engineState_.currentRPM.load();
    state.currentLoad = engineState_.currentLoad.load();
    state.exhaustFlow = engineState_.exhaustFlow.load();
    state.manifoldPressure = engineState_.manifoldPressure.load();
    state.activeChannels = engineState_.activeChannels.load();
    state.gear = engineState_.gear.load();
    state.speedMph = engineState_.speedMph.load();
    state.enginePhase = static_cast<EnginePhase>(engineState_.enginePhase.load());
    return state;
}

FramePerformanceTelemetry InMemoryTelemetry::getFramePerformance() const {
    FramePerformanceTelemetry perf;
    perf.processingTimeMs = framePerformance_.processingTimeMs.load();
    return perf;
}

AudioDiagnosticsTelemetry InMemoryTelemetry::getAudioDiagnostics() const {
    AudioDiagnosticsTelemetry diag;
    diag.underrunCount = audioDiagnostics_.underrunCount.load();
    diag.bufferHealthPct = audioDiagnostics_.bufferHealthPct.load();
    return diag;
}

AudioTimingTelemetry InMemoryTelemetry::getAudioTiming() const {
    AudioTimingTelemetry timing;
    timing.renderMs = audioTiming_.renderMs.load();
    timing.headroomMs = audioTiming_.headroomMs.load();
    timing.budgetPct = audioTiming_.budgetPct.load();
    timing.framesRequested = audioTiming_.framesRequested.load();
    timing.framesRendered = audioTiming_.framesRendered.load();
    timing.callbackRateHz = audioTiming_.callbackRateHz.load();
    timing.generatingRateFps = audioTiming_.generatingRateFps.load();
    timing.trendPct = audioTiming_.trendPct.load();
    return timing;
}

VehicleInputsTelemetry InMemoryTelemetry::getVehicleInputs() const {
    VehicleInputsTelemetry inputs;
    inputs.throttlePosition = vehicleInputs_.throttlePosition.load();
    inputs.ignitionOn = vehicleInputs_.ignitionOn.load();
    inputs.starterMotorEngaged = vehicleInputs_.starterMotorEngaged.load();
    return inputs;
}

SimulatorMetricsTelemetry InMemoryTelemetry::getSimulatorMetrics() const {
    SimulatorMetricsTelemetry metrics;
    metrics.timestamp = simulatorMetrics_.timestamp.load();
    return metrics;
}

} // namespace telemetry
