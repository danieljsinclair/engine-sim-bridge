// SimulatorFactory.h - Composition root for creating simulator instances
// Creates Simulator subclass, wires mode-specific details, wraps in BridgeSimulator.
// OCP: Factory is the only place that knows about SineSimulator vs PistonEngineSimulator.

#ifndef SIMULATOR_FACTORY_H
#define SIMULATOR_FACTORY_H

#include "simulator/ISimulator.h"
#include "simulator/EngineSimTypes.h"
#include <memory>
#include <string>
#include <vector>

class ILogging;

namespace telemetry { class ITelemetryWriter; }

struct SimulationConfig;  // forward -- full definition in simulation/SimulationLoop.h

// ============================================================================
// SimulatorType - Enum for factory creation
// ============================================================================

enum class SimulatorType {
    SineWave,      // SineSimulator (test mode)
    PistonEngine   // PistonEngineSimulator (.mr script or .json preset)
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
     * @param telemetryWriter Optional telemetry writer reserved for future factory-level telemetry wiring
     * @return Unique pointer to ready-to-use ISimulator
     */
    static std::unique_ptr<ISimulator> create(
        SimulatorType type,
        const std::string& scriptPath,
        const std::string& assetBasePath,
        const ISimulatorConfig& config,
        ILogging* logger = nullptr,
        telemetry::ITelemetryWriter* /*telemetryWriter*/ = nullptr);

    static SimulatorType getDefaultType();

    /**
     * Discover .json preset files in the same directory as the given preset path.
     * Returns sorted list of full paths to .json files.
     */
    static std::vector<std::string> discoverPresets(const std::string& currentPresetPath);

    /**
     * Configure dyno load torque on an existing simulator.
     * Callers can compose create() + configureLoadTorque() instead of createAndConfigure().
     */
    static bool configureLoadTorque(ISimulator* simulator, double loadFraction, ILogging* logger = nullptr);

    /**
     * Create and configure simulator with optional dyno load torque.
     * Combines create() + configureLoadTorque() for factory convenience.
     */
    static std::unique_ptr<ISimulator> createAndConfigure(
        const SimulationConfig& config,
        const std::string& scriptPath,
        const std::string& assetBasePath,
        ILogging* logger,
        telemetry::ITelemetryWriter* telemetryWriter);

    /**
     * Discover preset paths and find current index.
     * Returns preset info (short name + full path) and the index of currentPresetPath.
     */
    struct PresetNames {
        std::string shortName;  // display name (e.g., "V8", "Inline-4")
        std::string fullPath;   // fully qualified path
    };

    struct PresetDiscoveryResult {
        std::vector<PresetNames> presets;
        size_t currentIndex = 0;
    };
    static PresetDiscoveryResult discoverPresetPaths(const std::string& currentPresetPath);

private:
    SimulatorFactory() = delete;
    ~SimulatorFactory() = delete;
};

#endif // SIMULATOR_FACTORY_H
