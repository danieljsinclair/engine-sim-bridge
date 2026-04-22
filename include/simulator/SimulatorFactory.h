// SimulatorFactory.h - Composition root for creating simulator instances
// Creates Simulator subclass, wires mode-specific details, wraps in BridgeSimulator.
// OCP: Factory is the only place that knows about SineSimulator vs PistonEngineSimulator.

#ifndef SIMULATOR_FACTORY_H
#define SIMULATOR_FACTORY_H

#include "simulator/ISimulator.h"
#include "simulator/engine_sim_bridge.h"
#include <memory>
#include <string>
#include <optional>

class ILogging;

namespace telemetry { class ITelemetryWriter; }

// ============================================================================
// SimulatorType - Enum for factory creation
// ============================================================================

enum class SimulatorType {
    SineWave,      // SineSimulator (test mode)
    PistonEngine   // PistonEngineSimulator (real physics)
};

// ============================================================================
// SimulatorConfig - Configuration for factory creation
// ============================================================================

struct SimulatorConfig {
    SimulatorType type = SimulatorType::PistonEngine;
    std::string scriptPath;
    std::string assetBasePath;
    int sampleRate = 0;  // Resolved from EngineSimDefaults if unset
    std::optional<int> simulationFrequency;        // Override SIMULATION_FREQUENCY if set
    std::optional<double> synthLatency;            // Override TARGET_SYNTH_LATENCY if set
};

// ============================================================================
// SimulatorFactory - Creates fully-wired ISimulator instances
// ============================================================================

class SimulatorFactory {
public:
    /**
     * Creates a fully-wired ISimulator instance.
     *
     * For SineWave: creates SineSimulator (self-contained), wraps in BridgeSimulator.
     * For PistonEngine: creates PistonEngineSimulator, compiles script (if provided),
     *   loads impulse responses, wraps in BridgeSimulator.
     *
     * @param config Simulator type and configuration
     * @param logger Optional logging interface (can be nullptr)
     * @param telemetryWriter Optional telemetry writer (can be nullptr)
     * @return Unique pointer to ready-to-use ISimulator
     */
    static std::unique_ptr<ISimulator> create(
        const SimulatorConfig& config,
        ILogging* logger = nullptr,
        telemetry::ITelemetryWriter* telemetryWriter = nullptr);

    static SimulatorType getDefaultType();

private:
    SimulatorFactory() = delete;
    ~SimulatorFactory() = delete;
};

#endif // SIMULATOR_FACTORY_H
