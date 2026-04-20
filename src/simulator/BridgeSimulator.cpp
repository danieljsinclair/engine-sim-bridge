// BridgeSimulator.cpp - Production ISimulator wrapping PistonEngineSimulator
// Uses composition to call PistonEngineSimulator directly (no C API).

#include "simulator/BridgeSimulator.h"
#include "simulator/ScriptLoadHelpers.h"
#include "common/ILogging.h"

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
#include "../scripting/include/compiler.h"
#endif

#include <cmath>

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

    // Store for renderOnDemand() simulation stepping
    sampleRate_ = effective.sampleRate;
    simulationFrequency_ = effective.simulationFrequency;

    // Create and initialize PistonEngineSimulator directly
    m_simulator = std::make_unique<PistonEngineSimulator>();
    Simulator::Parameters simParams;
    simParams.systemType = Simulator::SystemType::NsvOptimized;
    m_simulator->initialize(simParams);
    m_simulator->setSimulationFrequency(effective.simulationFrequency);
    m_simulator->setFluidSimulationSteps(effective.fluidSimulationSteps);
    m_simulator->setTargetSynthesizerLatency(effective.targetSynthesizerLatency);

    // Allocate audio conversion buffer (RAII in base class)
    ensureAudioConversionBufferSize(4096 * 2);

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
    // Create compiler for script loading (used by loadScript)
    m_compiler = new es_script::Compiler();
    m_compiler->initialize();
#endif

    m_created = true;
    return true;
}

bool BridgeSimulator::loadScript(const std::string& path, const std::string& assetBase) {
    if (!m_simulator || !m_created) return false;

    // If no script path, just set the name and return success (sine mode or empty config)
    if (path.empty()) {
        name_ = "Unnamed";
        return true;
    }

    // If simulator already has an engine loaded (e.g., from a prior loadSimulation call),
    // skip script compilation. This mirrors the C API's polymorphic check.
    if (m_simulator->getEngine() != nullptr) {
        m_engine = m_simulator->getEngine();
        m_vehicle = m_simulator->getVehicle();
        m_transmission = m_simulator->getTransmission();
        setNameFromScript(path);
        return true;
    }

    setNameFromScript(path);

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
    // Normalize script path to absolute
    std::string scriptPathStr = ScriptLoadHelpers::normalizeScriptPath(path);
    if (scriptPathStr.empty()) {
        m_lastError = "Script path normalization failed";
        return false;
    }

    logger_->info(LogMask::BRIDGE, "BridgeSimulator: Compiling script: %s", scriptPathStr.c_str());

    // Compile the Piranha script
    if (!m_compiler->compile(scriptPathStr.c_str())) {
        m_lastError = "Failed to compile script: " + scriptPathStr;
        logger_->error(LogMask::BRIDGE, "%s", m_lastError.c_str());
        return false;
    }

    // Execute the compiled script
    es_script::Compiler::Output output = m_compiler->execute();

    // Create defaults for missing components
    m_vehicle = output.vehicle ? output.vehicle : ScriptLoadHelpers::createDefaultVehicle();
    m_transmission = output.transmission ? output.transmission : ScriptLoadHelpers::createDefaultTransmission();
    m_engine = output.engine;

    if (!m_engine) {
        m_lastError = "Script did not create an engine";
        logger_->error(LogMask::BRIDGE, "%s", m_lastError.c_str());
        return false;
    }

    // Load the simulation into PistonEngineSimulator
    m_simulator->loadSimulation(m_engine, m_vehicle, m_transmission);

    // Resolve asset path and load impulse responses
    std::string resolvedAssetPath = ScriptLoadHelpers::resolveAssetBasePath(scriptPathStr, assetBase);
    if (!ScriptLoadHelpers::loadImpulseResponses(m_simulator.get(), m_engine, resolvedAssetPath, logger_)) {
        m_lastError = "Failed to load impulse responses";
        logger_->error(LogMask::BRIDGE, "%s (asset base: %s)", m_lastError.c_str(), resolvedAssetPath.c_str());
        return false;
    }

    logger_->info(LogMask::BRIDGE, "BridgeSimulator: Script loaded successfully");
    return true;
#else
    (void)assetBase;
    m_lastError = "Script loading not available (Piranha support disabled)";
    logger_->error(LogMask::BRIDGE, "%s", m_lastError.c_str());
    return false;
#endif
}

void BridgeSimulator::destroy() {
    if (m_simulator) {
        m_simulator->endAudioRenderingThread();
        m_simulator->destroy();
        m_simulator = nullptr;
    }
    // Engine/vehicle/transmission are cleaned up by Simulator::destroy() above
    m_engine = nullptr;
    m_vehicle = nullptr;
    m_transmission = nullptr;

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
    if (m_compiler) {
        m_compiler->destroy();
        delete m_compiler;
        m_compiler = nullptr;
    }
#endif

    // Audio conversion buffer is RAII-managed by base class, no cleanup needed
    m_created = false;
}

std::string BridgeSimulator::getLastError() const {
    return m_lastError;
}

void BridgeSimulator::update(double deltaTime) {
    if (!m_simulator || !m_created) return;

    // ceil=true: slight over-production prevents Threaded underruns, ensures at least
    // 1 step for tiny dt (e.g. SyncPull retry calls update(1/sampleRate)).
    advanceFixedSteps(m_simulator.get(), simulationFrequency_, deltaTime, true);

    // Push telemetry (writer is never null after initDependencies)
    if (m_simulator->getEngine()) {
        EngineSimStats stats = getStats();
        pushTelemetry(stats);
    }
}

EngineSimStats BridgeSimulator::getStats() const {
    EngineSimStats stats = {};
    if (m_simulator && m_simulator->getEngine()) {
        auto* engine = m_simulator->getEngine();
        stats.currentRPM = engine->getSpeed() * 60.0 / (2.0 * M_PI);
        stats.currentLoad = 0.0;  // Not directly available
        stats.exhaustFlow = m_simulator->getTotalExhaustFlow();
        stats.processingTimeMs = m_simulator->getAverageProcessingTime() * 1000.0;
    }
    return stats;
}

void BridgeSimulator::setThrottle(double position) {
    if (!m_simulator || !m_created) return;
    if (m_simulator->getEngine()) {
        m_simulator->getEngine()->setSpeedControl(position);
    }
}

void BridgeSimulator::setIgnition(bool on) {
    if (!m_simulator || !m_created) return;
    if (m_simulator->getEngine()) {
        m_simulator->getEngine()->getIgnitionModule()->m_enabled = on;
    }
}

void BridgeSimulator::setStarterMotor(bool on) {
    if (!m_simulator || !m_created) return;
    m_simulator->m_starterMotor.m_enabled = on;
}

bool BridgeSimulator::renderOnDemand(float* buffer, int32_t frames, int32_t* written) {
    if (!m_simulator || !m_created || !buffer || frames <= 0) {
        if (written) *written = 0;
        return false;
    }

    // Ensure conversion buffer is large enough
    size_t requiredSize = frames * 2;
    int16_t* conversionBuffer = ensureAudioConversionBufferSize(requiredSize);

    // Run simulation for this audio frame.
    // ceil=false: SyncPull retry loop handles any deficit from truncation.
    const double dt = static_cast<double>(frames) / sampleRate_;
    advanceFixedSteps(m_simulator.get(), simulationFrequency_, dt, false);

    // Render and read audio
    m_simulator->synthesizer().renderAudioOnDemand();

    int samplesRead = m_simulator->readAudioOutput(frames, conversionBuffer);
    EngineSimAudio::convertInt16ToStereoFloat(conversionBuffer, buffer, samplesRead);

    if (samplesRead < frames) {
        EngineSimAudio::fillSilence(buffer + samplesRead * 2, frames - samplesRead);
    }

    if (written) *written = samplesRead;
    return true;
}

bool BridgeSimulator::readAudioBuffer(float* buffer, int32_t frames, int32_t* read) {
    if (!m_simulator || !m_created || !buffer || frames <= 0) {
        if (read) *read = 0;
        return false;
    }

    // Ensure conversion buffer is large enough
    size_t requiredSize = frames * 2;
    int16_t* conversionBuffer = ensureAudioConversionBufferSize(requiredSize);

    int samplesRead = m_simulator->readAudioOutput(frames, conversionBuffer);
    EngineSimAudio::convertInt16ToStereoFloat(conversionBuffer, buffer, samplesRead);

    if (samplesRead < frames) {
        EngineSimAudio::fillSilence(buffer + samplesRead * 2, frames - samplesRead);
    }

    if (read) *read = samplesRead;
    return true;
}

bool BridgeSimulator::start() {
    if (!m_simulator || !m_created) return false;

    drainSynthesizerBuffer(m_simulator.get());
    m_simulator->startAudioRenderingThread();
    return true;
}

void BridgeSimulator::stop() {
    if (!m_simulator || !m_created) return;
    m_simulator->endAudioRenderingThread();
}
