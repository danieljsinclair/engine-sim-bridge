// SimulatorFactory.cpp - Composition root for creating simulator instances
// Creates Simulator subclass, wires mode-specific details, wraps in BridgeSimulator.
// OCP: BridgeSimulator doesn't know what Simulator it has; factory is the only place that does.

#include "simulator/SimulatorFactory.h"
#include "simulator/BridgeSimulator.h"
#include "simulator/SineSimulator.h"
#include "simulator/ScriptCompileHelpers.h"
#include "simulator/ScriptExecutionHelpers.h"
#include "simulator/ScriptLoadHelpers.h"
#include "simulator/PresetEngineFactory.h"
#include "simulator/SimulatorInitHelpers.h"
#include "simulator/EngineSimTypes.h"
#include "common/ILogging.h"
#include "telemetry/ITelemetryProvider.h"
#include "piston_engine_simulator.h"

#include "engine-sim/include/simulator.h"

#include <memory>
#include <stdexcept>
#include <algorithm>
#include <filesystem>

static void initSimulator(Simulator* sim, const ISimulatorConfig& config);
static bool endsWith(const std::string& str, const std::string& suffix);

namespace {

struct LoadedSimulation {
    Engine* engine = nullptr;
    Vehicle* vehicle = nullptr;
    Transmission* transmission = nullptr;
    std::string resolvedAssetPath;
    int initialGear = -1;
};

std::unique_ptr<Simulator> createSineWaveSimulator(const ISimulatorConfig& config) {
    auto sineSim = std::make_unique<SineSimulator>();
    initSimulator(sineSim.get(), config);
    sineSim->loadSimulation(nullptr, nullptr, nullptr);
    return sineSim;
}

void applyLoadedEngineSettings(Simulator* simulator, Engine* engine) {
    simulator->setSimulationFrequency(engine->getSimulationFrequency());

    Synthesizer::AudioParameters audioParams = simulator->synthesizer().getAudioParameters();
    audioParams.inputSampleNoise = static_cast<float>(engine->getInitialJitter());
    audioParams.airNoise = static_cast<float>(engine->getInitialNoise());
    audioParams.dF_F_mix = static_cast<float>(engine->getInitialHighFrequencyGain());
    simulator->synthesizer().setAudioParameters(audioParams);
}

std::unique_ptr<Simulator> buildPistonEngineSimulator(
    LoadedSimulation loaded,
    const ISimulatorConfig& config,
    ILogging* logger)
{
    auto pistonSim = std::make_unique<PistonEngineSimulator>();
    initSimulator(pistonSim.get(), config);
    pistonSim->loadSimulation(loaded.engine, loaded.vehicle, loaded.transmission);
    applyLoadedEngineSettings(pistonSim.get(), loaded.engine);

    if (loaded.transmission && loaded.initialGear >= 0) {
        loaded.transmission->changeGear(loaded.initialGear);
    }

    if (!ScriptLoadHelpers::loadImpulseResponses(pistonSim.get(), loaded.engine, loaded.resolvedAssetPath, logger)) {
        pistonSim->destroy();
        throw std::runtime_error("Failed to load impulse responses (asset base: " + loaded.resolvedAssetPath + ")");
    }

    return pistonSim;
}

LoadedSimulation loadPresetSimulation(const std::string& scriptPath, const std::string& assetBasePath) {
    PresetLoadResult result = PresetEngineFactory::loadFromFile(scriptPath);
    if (!result.success()) {
        throw std::runtime_error("Failed to load preset: " + scriptPath + " — " + result.error);
    }

    LoadedSimulation loaded;
    loaded.engine = result.engine;
    loaded.vehicle = result.vehicle;
    loaded.transmission = result.transmission;
    loaded.initialGear = result.initialGear;
    loaded.resolvedAssetPath = ScriptLoadHelpers::resolveAssetBasePath(
        ScriptLoadHelpers::normalizeScriptPath(scriptPath), assetBasePath);
    return loaded;
}

LoadedSimulation loadScriptSimulation(const std::string& scriptPath, const std::string& assetBasePath) {
    std::string normalizedPath = ScriptLoadHelpers::normalizeScriptPath(scriptPath);
    std::string resolvedAssetPath = ScriptLoadHelpers::resolveAssetBasePath(normalizedPath, assetBasePath);

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
    const auto simDir = script_compile_helpers::findEngineSimRoot(normalizedPath, resolvedAssetPath);
    const es_script::Compiler::Output output = script_execution_helpers::compileScript(normalizedPath, simDir);

    if (!output.engine) {
        throw std::runtime_error("Script did not create an engine: " + normalizedPath);
    }

    LoadedSimulation loaded;
    loaded.engine = output.engine;
    loaded.vehicle = output.vehicle ? output.vehicle : ScriptLoadHelpers::createDefaultVehicle();
    loaded.transmission = output.transmission ? output.transmission : ScriptLoadHelpers::createDefaultTransmission();
    loaded.resolvedAssetPath = resolvedAssetPath;
    return loaded;
#else
    throw std::runtime_error("Script loading not available (Piranha support disabled)");
#endif
}

std::unique_ptr<Simulator> createPistonEngineSimulator(
    const std::string& scriptPath,
    const std::string& assetBasePath,
    const ISimulatorConfig& config,
    ILogging* logger)
{
    if (scriptPath.empty()) {
        auto pistonSim = std::make_unique<PistonEngineSimulator>();
        initSimulator(pistonSim.get(), config);
        return pistonSim;
    }

    LoadedSimulation loaded = endsWith(scriptPath, ".json")
        ? loadPresetSimulation(scriptPath, assetBasePath)
        : loadScriptSimulation(scriptPath, assetBasePath);

    return buildPistonEngineSimulator(std::move(loaded), config, logger);
}

std::unique_ptr<ISimulator> wrapBridgeSimulator(
    SimulatorType type,
    const std::string& scriptPath,
    std::unique_ptr<Simulator> simulator)
{
    auto bridgeSim = std::make_unique<BridgeSimulator>(std::move(simulator));
    if (type == SimulatorType::PistonEngine && !scriptPath.empty()) {
        bridgeSim->setNameFromScript(scriptPath);
    }
    return bridgeSim;
}

}  // namespace

// ============================================================================
// Shared Simulator init — common to all Simulator subclasses
// ============================================================================

static void initSimulator(Simulator* sim, const ISimulatorConfig& config) {
    Simulator::Parameters simParams;
    simParams.systemType = Simulator::SystemType::NsvOptimized;
    sim->initialize(simParams);

    sim->setSimulationFrequency(config.simulationFrequency);
    sim->setFluidSimulationSteps(config.fluidSimulationSteps);
    sim->setTargetSynthesizerLatency(config.targetSynthesizerLatency);
}

static bool endsWith(const std::string& str, const std::string& suffix) {
    if (suffix.size() > str.size()) return false;
    return std::equal(suffix.rbegin(), suffix.rend(), str.rbegin(),
        [](char a, char b) { return tolower(a) == tolower(b); });
}

// ============================================================================
// SimulatorFactory Implementation
// ============================================================================

std::unique_ptr<ISimulator> SimulatorFactory::create(
    SimulatorType type,
    const std::string& scriptPath,
    const std::string& assetBasePath,
    const ISimulatorConfig& config,
    ILogging* logger,
    telemetry::ITelemetryWriter* telemetryWriter)
{
    switch (type) {
        case SimulatorType::SineWave:
            return wrapBridgeSimulator(type, scriptPath, createSineWaveSimulator(config));

        case SimulatorType::PistonEngine:
            return wrapBridgeSimulator(type, scriptPath,
                createPistonEngineSimulator(scriptPath, assetBasePath, config, logger));

        default:
            throw std::runtime_error("Unknown simulator type in SimulatorFactory::create()");
    }
}

SimulatorType SimulatorFactory::getDefaultType() {
    return SimulatorType::PistonEngine;
}

std::vector<std::string> SimulatorFactory::discoverPresets(const std::string& currentPresetPath) {
    std::vector<std::string> presets;
    std::filesystem::path presetDir = std::filesystem::path(currentPresetPath).parent_path();

    if (!std::filesystem::exists(presetDir)) {
        return presets;
    }

    for (const auto& entry : std::filesystem::directory_iterator(presetDir)) {
        if (entry.is_regular_file() && entry.path().extension() == ".json") {
            presets.push_back(entry.path().string());
        }
    }

    std::sort(presets.begin(), presets.end());
    return presets;
}

// ============================================================================
// configureLoadTorque - Configure dyno in load torque mode
// hold=false + rotationSpeed=0 = brake-only (velocity-dependent damping).
// m_maxTorque is the load knob — the engine must work against this torque.
// Returns true if dyno was configured, false otherwise.
// ============================================================================

namespace {
    bool configureLoadTorque(ISimulator* simulator, double loadFraction, ILogging* logger) {
        if (loadFraction <= 0) return false;  // OK: no load to configure

        auto* bridgeSim = dynamic_cast<BridgeSimulator*>(simulator);
        const bool configured = bridgeSim->configureDynoLoad(loadFraction);

        if (configured && logger) {
            logger->info(LogMask::BRIDGE, "Load: %d%% (%d ft*lbs max)",
                         static_cast<int>(loadFraction * 100),
                         static_cast<int>(loadFraction * EngineSimDefaults::DYNO_MAX_TORQUE_FT_LBS));
        }
        return configured;
    }
}

// ============================================================================
// createAndConfigure - Create simulator and optionally configure load torque
// ============================================================================

std::unique_ptr<ISimulator> SimulatorFactory::createAndConfigure(
    SimulatorType type,
    const std::string& scriptPath,
    const std::string& assetBasePath,
    const ISimulatorConfig& engineConfig,
    double targetLoad,
    ILogging* logger,
    telemetry::ITelemetryWriter* telemetryWriter)
{
    auto sim = create(type, scriptPath, assetBasePath, engineConfig, logger, telemetryWriter);
    configureLoadTorque(sim.get(), targetLoad, logger);
    return sim;
}

// ============================================================================
// discoverPresetPaths - Discover presets and find current index
// ============================================================================

SimulatorFactory::PresetDiscoveryResult SimulatorFactory::discoverPresetPaths(const std::string& currentPresetPath) {
    PresetDiscoveryResult result;
    result.paths = discoverPresets(currentPresetPath);

    if (result.paths.size() > 1) {
        // Find current preset index in the sorted list
        for (size_t i = 0; i < result.paths.size(); ++i) {
            if (result.paths[i] == currentPresetPath) {
                result.currentIndex = i;
                break;
            }
        }
    }

    return result;
}
