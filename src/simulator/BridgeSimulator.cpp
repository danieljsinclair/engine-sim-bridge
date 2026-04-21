// BridgeSimulator.cpp - Production ISimulator wrapping PistonEngineSimulator
// Uses composition to call PistonEngineSimulator directly (no C API).

#include "simulator/BridgeSimulator.h"
#include "simulator/ScriptLoadHelpers.h"
#include "common/ILogging.h"

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
#include "../scripting/include/compiler.h"
#endif

BridgeSimulator::BridgeSimulator() = default;

BridgeSimulator::~BridgeSimulator() {
    destroy();
}

bool BridgeSimulator::create(const EngineSimConfig& config, ILogging* logger, telemetry::ITelemetryWriter* telemetryWriter) {
    initDependencies(logger, telemetryWriter);

    // Apply sensible defaults
    EngineSimConfig effective = config;
    if (effective.sampleRate <= 0) effective.sampleRate = 48000;
    if (effective.inputBufferSize <= 0) effective.inputBufferSize = 1024;
    if (effective.audioBufferSize <= 0) effective.audioBufferSize = 96000;
    if (effective.simulationFrequency <= 0) effective.simulationFrequency = 10000;
    if (effective.fluidSimulationSteps <= 0) effective.fluidSimulationSteps = 8;
    if (effective.targetSynthesizerLatency <= 0.0) effective.targetSynthesizerLatency = 0.05;

    initAudioConfig(effective);

    // Create and initialize PistonEngineSimulator directly
    m_simulator = std::make_unique<PistonEngineSimulator>();
    Simulator::Parameters simParams;
    simParams.systemType = Simulator::SystemType::NsvOptimized;
    m_simulator->initialize(simParams);
    m_simulator->setSimulationFrequency(effective.simulationFrequency);
    m_simulator->setFluidSimulationSteps(effective.fluidSimulationSteps);
    m_simulator->setTargetSynthesizerLatency(effective.targetSynthesizerLatency);

    m_created = true;
    return true;
}

bool BridgeSimulator::loadScript(const std::string& path, const std::string& assetBase) {
    if (!m_simulator || !m_created) return false;

    // No script path — nothing to load
    if (path.empty()) {
        name_ = "Unnamed";
        return true;
    }

    // Already loaded — skip recompilation
    if (m_simulator->getEngine() != nullptr) {
        setNameFromScript(path);
        return true;
    }

    setNameFromScript(path);

    // Compile script → engine/vehicle/transmission
    Engine* engine = nullptr;
    Vehicle* vehicle = nullptr;
    Transmission* transmission = nullptr;

    if (!compileScript(path, &engine, &vehicle, &transmission)) {
        return false;  // m_lastError set by compileScript
    }

    // Wire into simulator
    m_simulator->loadSimulation(engine, vehicle, transmission);

    // Load impulse responses (asset-based audio samples)
    std::string scriptPath = ScriptLoadHelpers::normalizeScriptPath(path);
    std::string resolvedAssetPath = ScriptLoadHelpers::resolveAssetBasePath(scriptPath, assetBase);
    if (!ScriptLoadHelpers::loadImpulseResponses(m_simulator.get(), engine, resolvedAssetPath, logger_)) {
        m_lastError = "Failed to load impulse responses";
        logger_->error(LogMask::BRIDGE, "%s (asset base: %s)", m_lastError.c_str(), resolvedAssetPath.c_str());
        return false;
    }

    logger_->info(LogMask::BRIDGE, "BridgeSimulator: Script loaded successfully");
    return true;
}

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
bool BridgeSimulator::compileScript(const std::string& scriptPath,
                                     Engine** outEngine, Vehicle** outVehicle, Transmission** outTransmission) {
    std::string normalizedPath = ScriptLoadHelpers::normalizeScriptPath(scriptPath);
    if (normalizedPath.empty()) {
        m_lastError = "Script path normalization failed";
        return false;
    }

    logger_->info(LogMask::BRIDGE, "BridgeSimulator: Compiling script: %s", normalizedPath.c_str());

    // Lazily create compiler on first script load
    if (!m_compiler) {
        m_compiler = new es_script::Compiler();
        m_compiler->initialize();
    }

    if (!m_compiler->compile(normalizedPath.c_str())) {
        m_lastError = "Failed to compile script: " + normalizedPath;
        logger_->error(LogMask::BRIDGE, "%s", m_lastError.c_str());
        return false;
    }

    es_script::Compiler::Output output = m_compiler->execute();

    *outEngine = output.engine;
    *outVehicle = output.vehicle ? output.vehicle : ScriptLoadHelpers::createDefaultVehicle();
    *outTransmission = output.transmission ? output.transmission : ScriptLoadHelpers::createDefaultTransmission();

    if (!*outEngine) {
        m_lastError = "Script did not create an engine";
        logger_->error(LogMask::BRIDGE, "%s", m_lastError.c_str());
        return false;
    }

    return true;
}
#else
bool BridgeSimulator::compileScript(const std::string&,
                                     Engine**, Vehicle**, Transmission**) {
    m_lastError = "Script loading not available (Piranha support disabled)";
    logger_->error(LogMask::BRIDGE, "%s", m_lastError.c_str());
    return false;
}
#endif

void BridgeSimulator::destroy() {
    if (m_simulator) {
        m_simulator->endAudioRenderingThread();
        m_simulator->destroy();
        m_simulator = nullptr;
    }

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
    if (m_compiler) {
        m_compiler->destroy();
        delete m_compiler;
        m_compiler = nullptr;
    }
#endif

    m_created = false;
}

std::string BridgeSimulator::getLastError() const {
    return m_lastError;
}
