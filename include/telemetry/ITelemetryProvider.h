// ITelemetryProvider.h - Telemetry interfaces for dependency injection
// Reader/Writer split for ISP compliance
// Bridge writes via ITelemetryWriter, presentation reads via ITelemetryReader

#ifndef I_TELEMETRY_PROVIDER_H
#define I_TELEMETRY_PROVIDER_H

#include "common/RelaxedAtomic.h"
#include "simulation/EnginePhase.h"

namespace telemetry {

// ============================================================================
// ISP Component Structs - Per-concern telemetry data
// Components push only their own data (Interface Segregation Principle)
// ============================================================================

struct EngineStateTelemetry {
    double currentRPM = 0.0;
    double currentLoad = 0.0;        // 0.0 - 1.0
    double exhaustFlow = 0.0;        // m^3/s
    double manifoldPressure = 0.0;   // Pa
    int32_t activeChannels = 0;
    int32_t gear = 0;                // -1 = Park, 0 = Neutral, 1+ = Forward
    double speedMph = 0.0;           // Vehicle speed in MPH
    EnginePhase enginePhase = EnginePhase::Stopped;
};

struct FramePerformanceTelemetry {
    double processingTimeMs = 0.0;   // Last frame processing time
};

struct AudioDiagnosticsTelemetry {
    int32_t underrunCount = 0;
    double bufferHealthPct = 0.0;    // 0-100 (buffer fullness)
};

struct VehicleInputsTelemetry {
    double throttlePosition = 0.0;   // 0.0 - 1.0
    bool ignitionOn = false;
    bool starterMotorEngaged = false;
};

struct SimulatorMetricsTelemetry {
    double timestamp = 0.0;          // Seconds since start
};

struct AudioTimingTelemetry {
    double renderMs = 0.0;           // Last render time in ms
    double headroomMs = 0.0;         // Buffer headroom in ms
    double budgetPct = 0.0;          // Render budget percentage used
    int32_t framesRequested = 0;     // Frames requested per callback
    int32_t framesRendered = 0;      // Frames actually rendered
    double callbackRateHz = 0.0;     // Callback throughput in Hz
    double generatingRateFps = 0.0;  // Frame generation rate
    double trendPct = 0.0;           // Throughput trend percentage
};

// ============================================================================
// ITelemetryWriter - Bridge writes telemetry here
// Used by: Bridge (runSimulation, Update)
// ============================================================================

class ITelemetryWriter {
public:
    virtual ~ITelemetryWriter() = default;

    // ISP per-component write methods
    virtual void writeEngineState(const EngineStateTelemetry& state) = 0;
    virtual void writeFramePerformance(const FramePerformanceTelemetry& perf) = 0;
    virtual void writeAudioDiagnostics(const AudioDiagnosticsTelemetry& diag) = 0;
    virtual void writeAudioTiming(const AudioTimingTelemetry& timing) = 0;
    virtual void writeVehicleInputs(const VehicleInputsTelemetry& inputs) = 0;
    virtual void writeSimulatorMetrics(const SimulatorMetricsTelemetry& metrics) = 0;

    virtual void reset() = 0;
    virtual const char* getName() const = 0;
};

// ============================================================================
// ITelemetryReader - Presentation reads telemetry from here
// Used by: IPresentation implementations (Console, TUI, GUI)
// ============================================================================

class ITelemetryReader {
public:
    virtual ~ITelemetryReader() = default;

    // ISP per-component read methods
    virtual EngineStateTelemetry getEngineState() const = 0;
    virtual FramePerformanceTelemetry getFramePerformance() const = 0;
    virtual AudioDiagnosticsTelemetry getAudioDiagnostics() const = 0;
    virtual AudioTimingTelemetry getAudioTiming() const = 0;
    virtual VehicleInputsTelemetry getVehicleInputs() const = 0;
    virtual SimulatorMetricsTelemetry getSimulatorMetrics() const = 0;

    virtual const char* getName() const = 0;
};

// ============================================================================
// InMemoryTelemetry - Default implementation (Writer + Reader)
// Zero-copy, thread-safe, suitable for real-time display
// ============================================================================

class InMemoryTelemetry : public ITelemetryWriter, public ITelemetryReader {
public:
    InMemoryTelemetry();
    ~InMemoryTelemetry() override = default;

    // ITelemetryWriter implementation
    void reset() override;
    const char* getName() const override { return "InMemoryTelemetry"; }

    // ISP per-component write methods (ITelemetryWriter overrides)
    void writeEngineState(const EngineStateTelemetry& state) override;
    void writeFramePerformance(const FramePerformanceTelemetry& perf) override;
    void writeAudioDiagnostics(const AudioDiagnosticsTelemetry& diag) override;
    void writeAudioTiming(const AudioTimingTelemetry& timing) override;
    void writeVehicleInputs(const VehicleInputsTelemetry& inputs) override;
    void writeSimulatorMetrics(const SimulatorMetricsTelemetry& metrics) override;

    // ISP per-component read methods (ITelemetryReader overrides)
    EngineStateTelemetry getEngineState() const override;
    FramePerformanceTelemetry getFramePerformance() const override;
    AudioDiagnosticsTelemetry getAudioDiagnostics() const override;
    AudioTimingTelemetry getAudioTiming() const override;
    VehicleInputsTelemetry getVehicleInputs() const override;
    SimulatorMetricsTelemetry getSimulatorMetrics() const override;

private:
    // Per-concern atomic sub-structs mirror the ISP structs.
    // RelaxedDouble/Int/Bool use memory_order_relaxed internally
    // (implementation in .cpp) to avoid unnecessary barriers on ARM.
    struct AtomicEngineState {
        RelaxedDouble currentRPM{0.0};
        RelaxedDouble currentLoad{0.0};
        RelaxedDouble exhaustFlow{0.0};
        RelaxedDouble manifoldPressure{0.0};
        RelaxedInt activeChannels{0};
        RelaxedInt gear{0};
        RelaxedDouble speedMph{0.0};
        RelaxedInt enginePhase{0};
    };

    struct AtomicFramePerformance {
        RelaxedDouble processingTimeMs{0.0};
    };

    struct AtomicAudioDiagnostics {
        RelaxedInt underrunCount{0};
        RelaxedDouble bufferHealthPct{0.0};
    };

    struct AtomicAudioTiming {
        RelaxedDouble renderMs{0.0};
        RelaxedDouble headroomMs{0.0};
        RelaxedDouble budgetPct{0.0};
        RelaxedInt framesRequested{0};
        RelaxedInt framesRendered{0};
        RelaxedDouble callbackRateHz{0.0};
        RelaxedDouble generatingRateFps{0.0};
        RelaxedDouble trendPct{0.0};
    };

    struct AtomicVehicleInputs {
        RelaxedDouble throttlePosition{0.0};
        RelaxedBool ignitionOn{false};
        RelaxedBool starterMotorEngaged{false};
    };

    struct AtomicSimulatorMetrics {
        RelaxedDouble timestamp{0.0};
    };

    AtomicEngineState engineState_;
    AtomicFramePerformance framePerformance_;
    AtomicAudioDiagnostics audioDiagnostics_;
    AtomicAudioTiming audioTiming_;
    AtomicVehicleInputs vehicleInputs_;
    AtomicSimulatorMetrics simulatorMetrics_;
};

} // namespace telemetry

#endif // I_TELEMETRY_PROVIDER_H
