// SimulatorFactory.cpp - Composition root for creating simulator instances
// Creates Simulator subclass, wires mode-specific details, wraps in BridgeSimulator.
// OCP: BridgeSimulator doesn't know what Simulator it has; factory is the only place that does.

#include "simulator/SimulatorFactory.h"
#include "simulation/SimulationLoop.h"
#include "simulator/BridgeSimulator.h"
#include "simulator/SineSimulator.h"
#include "simulator/SineEngine.h"
#include "simulator/SineVehicle.h"
#include "simulator/SineTransmission.h"
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

#include "common/PresetExceptions.h"

#include <memory>
#include <algorithm>
#include <filesystem>
#include <string_view>

static void initSimulator(Simulator* sim, const ISimulatorConfig& config);
static bool endsWith(std::string_view str, std::string_view suffix);

namespace {

struct LoadedSimulation {
    Engine* engine = nullptr;
    Vehicle* vehicle = nullptr;
    Transmission* transmission = nullptr;
    std::string resolvedAssetPath;
    int initialGear = -1;
};

// Metadata returned by each create function — the single wrap path consumes this.
struct SimulatorInit {
    std::unique_ptr<Simulator> simulator;
    std::string name;
    EnginePhase initialPhase = EnginePhase::Stopped;
};

SimulatorInit createSineWaveSimulator(const ISimulatorConfig& config) {
    // SineEngine has no built-in frequency — must provide one explicitly
    ISimulatorConfig sineConfig = config;
    if (sineConfig.simulationFrequency <= 0) {
        sineConfig.simulationFrequency = EngineSimDefaults::SIMULATION_FREQUENCY;
    }
    auto sineSim = std::make_unique<SineSimulator>();
    initSimulator(sineSim.get(), sineConfig);
    sineSim->loadSimulation(nullptr, nullptr, nullptr);
    return {std::move(sineSim), "SineWave", EnginePhase::Running};
}

void applyLoadedEngineSettings(Simulator* simulator, const Engine* engine) {
    simulator->setSimulationFrequency(static_cast<int>(engine->getSimulationFrequency()));

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
        throw SimulatorException("Failed to load impulse responses (asset base: " + loaded.resolvedAssetPath + ")");
    }

    return pistonSim;
}

LoadedSimulation loadPresetSimulation(const std::string& scriptPath, const std::string& assetBasePath) {
    PresetLoadResult result = PresetEngineFactory::loadFromFile(scriptPath);
    if (!result.success()) {
        throw SimulatorException("Failed to load preset: " + scriptPath + " — " + result.error);
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
    std::filesystem::path normalizedPath = ScriptLoadHelpers::normalizeScriptPath(scriptPath);
    std::string resolvedAssetPath = ScriptLoadHelpers::resolveAssetBasePath(normalizedPath.string(), assetBasePath);

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
    const auto simDir = script_compile_helpers::findEngineSimRoot(normalizedPath, resolvedAssetPath);
    const es_script::Compiler::Output output = script_execution_helpers::compileScript(normalizedPath, simDir);

    if (!output.engine) {
        throw SimulatorException("Script did not create an engine: " + normalizedPath.string());
    }

    LoadedSimulation loaded;
    loaded.engine = output.engine;
    loaded.vehicle = output.vehicle ? output.vehicle : ScriptLoadHelpers::createDefaultVehicle();
    loaded.transmission = output.transmission ? output.transmission : ScriptLoadHelpers::createDefaultTransmission();
    loaded.resolvedAssetPath = resolvedAssetPath;
    return loaded;
#else
    throw SimulatorException("Script loading not available (Piranha support disabled)");
#endif
}

SimulatorInit createPistonEngineSimulator(
    const std::string& scriptPath,
    const std::string& assetBasePath,
    const ISimulatorConfig& config,
    ILogging* logger)
{
    std::string name;
    std::unique_ptr<Simulator> sim;

    if (scriptPath.empty()) {
        sim = std::make_unique<PistonEngineSimulator>();
        initSimulator(sim.get(), config);
        name = "PistonEngine";
    } else {
        LoadedSimulation loaded = endsWith(scriptPath, ".json")
            ? loadPresetSimulation(scriptPath, assetBasePath)
            : loadScriptSimulation(scriptPath, assetBasePath);
        sim = buildPistonEngineSimulator(std::move(loaded), config, logger);
        name = std::filesystem::path(scriptPath).stem().string();
    }

    return {std::move(sim), std::move(name), EnginePhase::Stopped};
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

static bool endsWith(std::string_view str, std::string_view suffix) {
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
    telemetry::ITelemetryWriter* /*telemetryWriter*/)
{
    SimulatorInit simInit;
    switch (type) {
        case SimulatorType::SineWave:
            simInit = createSineWaveSimulator(config);
            break;

        case SimulatorType::PistonEngine:
            simInit = createPistonEngineSimulator(scriptPath, assetBasePath, config, logger);
            break;

        default:
            throw SimulatorException("Unknown simulator type in SimulatorFactory::create()");
    }

    auto bridgeSim = std::make_unique<BridgeSimulator>(std::move(simInit.simulator), simInit.name);

    // Populate engineConfig_ + size the audio buffer so the simulator reports a
    // correct simulationFrequency and has a usable audio buffer before the
    // session layer calls create() (which re-runs initAudioConfig — idempotent
    // — and adds the brake constraint exactly once). initAudioConfig only is
    // called here; full create() is non-idempotent (addConstraint on brake).
    bridgeSim->initAudioConfig(config);

    return bridgeSim;
}

SimulatorType SimulatorFactory::getDefaultType() {
    return SimulatorType::PistonEngine;
}

std::vector<std::string> SimulatorFactory::discoverPresets(const std::string& presetPath) {
    std::vector<std::string> presets;
    auto presetDir = std::filesystem::path(presetPath);

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

bool SimulatorFactory::configureLoadTorque(ISimulator* simulator, double loadFraction, ILogging* logger) {
    if (loadFraction <= 0) return false;  // OK: no load to configure

    auto* bridgeSim = dynamic_cast<BridgeSimulator*>(simulator);
    const bool configured = bridgeSim->configureDynoLoad(loadFraction);

    if (configured && logger) {
        logger->info(LogMask::BRIDGE, __ilog_format("Load: %d%% (%d ft*lbs max)",
                     static_cast<int>(loadFraction * 100),
                     static_cast<int>(loadFraction * EngineSimDefaults::DYNO_MAX_TORQUE_FT_LBS)));
    }
    return configured;
}

// ============================================================================
// createAndConfigure - Create simulator and optionally configure load torque
// ============================================================================

std::unique_ptr<ISimulator> SimulatorFactory::createAndConfigure(
    const SimulationConfig& config,
    const std::string& scriptPath,
    const std::string& assetBasePath,
    ILogging* logger,
    telemetry::ITelemetryWriter* telemetryWriter)
{
    auto sim = create(config.simulatorType, scriptPath, assetBasePath, config.engineConfig, logger, telemetryWriter);
    configureLoadTorque(sim.get(), config.targetLoad, logger);
    return sim;
}

// ============================================================================
// discoverPresetPaths - Discover presets (scan a directory) and find the index
// of a specific current preset file among the results.
// ============================================================================

SimulatorFactory::PresetDiscoveryResult SimulatorFactory::discoverPresetPaths(const std::string& dirToScan, std::string_view currentFullPath) {
    PresetDiscoveryResult result;
    auto rawPaths = discoverPresets(dirToScan);

    for (auto& path : rawPaths) {
        PresetNames info;
        info.fullPath = std::move(path);

        // Extract short name from filename stem (e.g., "v8.json" -> "v8")
        std::filesystem::path p(info.fullPath);
        info.shortName = p.stem().string();

        result.presets.push_back(std::move(info));
    }

    // Resolve the current preset's position in the sorted list. An empty
    // currentFullPath (no current selection) matches nothing and leaves
    // currentIndex at its default. Only meaningful when cycling is possible.
    if (!currentFullPath.empty() && result.presets.size() > 1) {
        for (size_t i = 0; i < result.presets.size(); ++i) {
            if (result.presets[i].fullPath == currentFullPath) {
                result.currentIndex = i;
                break;
            }
        }
    }

    return result;
}
