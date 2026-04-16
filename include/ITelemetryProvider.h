// ITelemetryProvider.h - Telemetry interfaces for dependency injection
// Reader/Writer split for ISP compliance
// Bridge writes via ITelemetryWriter, presentation reads via ITelemetryReader

#ifndef I_TELEMETRY_PROVIDER_H
#define I_TELEMETRY_PROVIDER_H

#include <atomic>
#include <cstdint>

namespace telemetry {

// ============================================================================
// TelemetryData - Structured engine telemetry (non-atomic snapshot)
// ============================================================================

struct TelemetryData {
    // Engine state
    double currentRPM;
    double currentLoad;           // 0.0 - 1.0
    double exhaustFlow;           // m^3/s
    double manifoldPressure;      // Pa
    int32_t activeChannels;

    // Performance metrics
    double processingTimeMs;      // Last frame processing time

    // Audio diagnostics
    int32_t underrunCount;
    double bufferHealthPct;       // 0-100 (buffer fullness)

    // Control inputs (echo back for display)
    double throttlePosition;      // 0.0 - 1.0
    bool ignitionOn;
    bool starterMotorEngaged;

    // Timestamp
    double timestamp;             // Seconds since start
};

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

// ============================================================================
// ITelemetryWriter - Bridge writes telemetry here
// Used by: Bridge (runSimulation, Update)
// ============================================================================

class ITelemetryWriter {
public:
    virtual ~ITelemetryWriter() = default;

    /**
     * Write telemetry data from bridge.
     * Called from bridge's simulation thread.
     *
     * @param data Telemetry data to write
     *
     * Thread Safety: Implementation must be thread-safe.
     * Bridge calls this from simulation thread, storage handles synchronization.
     */
    virtual void write(const TelemetryData& data) = 0;

    /**
     * Reset telemetry counters (e.g., on simulation restart).
     */
    virtual void reset() = 0;

    /**
     * Get writer name for diagnostics.
     */
    virtual const char* getName() const = 0;
};

// ============================================================================
// ITelemetryReader - Presentation reads telemetry from here
// Used by: IPresentation implementations (Console, TUI, GUI)
// ============================================================================

class ITelemetryReader {
public:
    virtual ~ITelemetryReader() = default;

    /**
     * Get current telemetry snapshot.
     * Called from presentation layer (main thread).
     *
     * @return Copy of current telemetry data
     *
     * Thread Safety: Implementation must be thread-safe.
     * Returns snapshot to avoid race conditions during reads.
     */
    virtual TelemetryData getSnapshot() const = 0;

    /**
     * Get reader name for diagnostics.
     */
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
    void write(const TelemetryData& data) override;
    void reset() override;
    const char* getName() const override { return "InMemoryTelemetry"; }

    // ITelemetryReader implementation
    TelemetryData getSnapshot() const override;

    // ISP per-component write methods
    void writeEngineState(const EngineStateTelemetry& state);
    void writeFramePerformance(const FramePerformanceTelemetry& perf);
    void writeAudioDiagnostics(const AudioDiagnosticsTelemetry& diag);
    void writeVehicleInputs(const VehicleInputsTelemetry& inputs);
    void writeSimulatorMetrics(const SimulatorMetricsTelemetry& metrics);

    // ISP per-component read methods
    EngineStateTelemetry getEngineState() const;
    FramePerformanceTelemetry getFramePerformance() const;
    AudioDiagnosticsTelemetry getAudioDiagnostics() const;
    VehicleInputsTelemetry getVehicleInputs() const;
    SimulatorMetricsTelemetry getSimulatorMetrics() const;

private:
    // Atomic storage for thread-safe write/read
    struct AtomicData {
        std::atomic<double> currentRPM;
        std::atomic<double> currentLoad;
        std::atomic<double> exhaustFlow;
        std::atomic<double> manifoldPressure;
        std::atomic<int32_t> activeChannels;
        std::atomic<double> processingTimeMs;
        std::atomic<int32_t> underrunCount;
        std::atomic<double> bufferHealthPct;
        std::atomic<double> throttlePosition;
        std::atomic<bool> ignitionOn;
        std::atomic<bool> starterMotorEngaged;
        std::atomic<double> timestamp;

        AtomicData();  // Initializes atomics
    };

    AtomicData data_;
};

} // namespace telemetry

#endif // I_TELEMETRY_PROVIDER_H
