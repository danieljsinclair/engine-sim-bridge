// NullTelemetryWriter.h - No-op telemetry writer for when no writer is injected
#ifndef NULL_TELEMETRY_WRITER_H
#define NULL_TELEMETRY_WRITER_H

#include "telemetry/ITelemetryProvider.h"

class NullTelemetryWriter : public telemetry::ITelemetryWriter {
public:
    void writeEngineState(const telemetry::EngineStateTelemetry&) override {}
    void writeFramePerformance(const telemetry::FramePerformanceTelemetry&) override {}
    void writeAudioDiagnostics(const telemetry::AudioDiagnosticsTelemetry&) override {}
    void writeAudioTiming(const telemetry::AudioTimingTelemetry&) override {}
    void writeVehicleInputs(const telemetry::VehicleInputsTelemetry&) override {}
    void writeSimulatorMetrics(const telemetry::SimulatorMetricsTelemetry&) override {}
    void reset() override {}
    const char* getName() const override { return "NullTelemetryWriter"; }
};

#endif // NULL_TELEMETRY_WRITER_H
