// SimulatorFactory.h - Composition root for creating simulator instances
// Creates Simulator subclass, wires mode-specific details, wraps in BridgeSimulator.
// OCP: Factory is the only place that knows about SineSimulator vs PistonEngineSimulator.

#ifndef SIMULATOR_FACTORY_H
#define SIMULATOR_FACTORY_H

#include "simulator/ISimulator.h"
#include "simulator/EngineSimTypes.h"
#include <memory>
#include <string>

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
     * The factory extracts simulationFrequency, fluidSimulationSteps, and synthLatency
     * from the ISimulatorConfig to initialize the Simulator subclass directly.
     * The ISimulatorConfig is then passed to BridgeSimulator::create() for runtime use.
     *
     * @param type What kind of simulator to create
     * @param scriptPath Path to engine script (for PistonEngine type)
     * @param assetBasePath Base path for resolving asset files (impulse responses)
     * @param config ISimulator configuration (used for both Simulator subclass init and BridgeSimulator)
     * @param logger Optional logging interface (can be nullptr)
     * @param telemetryWriter Optional telemetry writer (can be nullptr)
     * @return Unique pointer to ready-to-use ISimulator
     */
    static std::unique_ptr<ISimulator> create(
        SimulatorType type,
        const std::string& scriptPath,
        const std::string& assetBasePath,
        const ISimulatorConfig& config,
        ILogging* logger = nullptr,
        telemetry::ITelemetryWriter* telemetryWriter = nullptr);

    static SimulatorType getDefaultType();

private:
    SimulatorFactory() = delete;
    ~SimulatorFactory() = delete;
};

#endif // SIMULATOR_FACTORY_H
