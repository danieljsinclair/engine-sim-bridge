// BridgeSimulator.cpp - Universal ISimulator implementation
// Composes an injected Simulator subclass.
// All audio pipeline, telemetry, and control methods are identical
// regardless of which Simulator subclass is injected (OCP).

#include "simulator/BridgeSimulator.h"

BridgeSimulator::BridgeSimulator(std::unique_ptr<Simulator> simulator)
    : m_simulator(std::move(simulator))
{
    if (m_simulator) {
        name_ = "Simulator";
    }
}

BridgeSimulator::~BridgeSimulator() {
    destroy();
}

// ============================================================================
// ISimulator Lifecycle
// ============================================================================

bool BridgeSimulator::create(const EngineSimConfig& config, ILogging* logger, telemetry::ITelemetryWriter* telemetryWriter) {
    initDependencies(logger, telemetryWriter);
    initAudioConfig(config);
    m_created = true;
    return true;
}

void BridgeSimulator::destroy() {
    if (m_simulator) {
        m_simulator->endAudioRenderingThread();
        m_simulator->destroy();
        m_simulator = nullptr;
    }
    m_created = false;
}

std::string BridgeSimulator::getLastError() const {
    return m_lastError;
}

// ============================================================================
// ISimulator Audio Pipeline
// ============================================================================

void BridgeSimulator::update(double deltaTime) {
    if (!m_created || !m_simulator) return;
    advanceFixedSteps(m_simulator.get(), simulationFrequency_, deltaTime, true);
    pushTelemetry(getStats());
}

bool BridgeSimulator::renderOnDemand(float* buffer, int32_t frames, int32_t* written) {
    if (!m_created || !m_simulator || !buffer || frames <= 0) {
        if (written) *written = 0;
        return false;
    }

    const double dt = static_cast<double>(frames) / sampleRate_;
    advanceFixedSteps(m_simulator.get(), simulationFrequency_, dt, false);
    m_simulator->synthesizer().renderAudioOnDemand();

    int16_t* conversionBuffer = ensureAudioConversionBufferSize(frames);
    int samplesRead = m_simulator->readAudioOutput(frames, conversionBuffer);
    EngineSimAudio::convertInt16ToStereoFloat(conversionBuffer, buffer, samplesRead);

    if (samplesRead < frames) {
        EngineSimAudio::fillSilence(buffer + samplesRead * 2, frames - samplesRead);
    }
    if (written) *written = samplesRead;
    return true;
}

bool BridgeSimulator::readAudioBuffer(float* buffer, int32_t frames, int32_t* read) {
    if (!m_created || !m_simulator || !buffer || frames <= 0) {
        if (read) *read = 0;
        return false;
    }

    int16_t* conversionBuffer = ensureAudioConversionBufferSize(frames);
    int samplesRead = m_simulator->readAudioOutput(frames, conversionBuffer);
    EngineSimAudio::convertInt16ToStereoFloat(conversionBuffer, buffer, samplesRead);

    if (samplesRead < frames) {
        EngineSimAudio::fillSilence(buffer + samplesRead * 2, frames - samplesRead);
    }
    if (read) *read = samplesRead;
    return true;
}

bool BridgeSimulator::start() {
    if (!m_created || !m_simulator) return false;
    drainSynthesizerBuffer(m_simulator.get());
    m_simulator->startAudioRenderingThread();
    return true;
}

void BridgeSimulator::stop() {
    if (!m_created || !m_simulator) return;
    m_simulator->endAudioRenderingThread();
}

// ============================================================================
// ISimulator Telemetry & Control
// ============================================================================

EngineSimStats BridgeSimulator::getStats() const {
    EngineSimStats stats = {};
    if (m_simulator && m_simulator->getEngine()) {
        stats.currentRPM = m_simulator->getEngine()->getSpeed() * 60.0 / (2.0 * M_PI);
        stats.exhaustFlow = m_simulator->getTotalExhaustFlow();
        stats.processingTimeMs = m_simulator->getAverageProcessingTime() * 1000.0;
    }
    return stats;
}

void BridgeSimulator::setThrottle(double position) {
    if (!m_created || !m_simulator) return;
    if (position < 0.0) position = 0.0;
    if (position > 1.0) position = 1.0;
    if (m_simulator->getEngine()) {
        m_simulator->getEngine()->setSpeedControl(position);
    }
}

void BridgeSimulator::setIgnition(bool on) {
    if (!m_created || !m_simulator) return;
    if (m_simulator->getEngine()) {
        m_simulator->getEngine()->getIgnitionModule()->m_enabled = on;
    }
}

void BridgeSimulator::setStarterMotor(bool on) {
    if (!m_created || !m_simulator) return;
    m_simulator->m_starterMotor.m_enabled = on;
}

// ============================================================================
// Private Helpers
// ============================================================================

void BridgeSimulator::initDependencies(ILogging* logger, telemetry::ITelemetryWriter* telemetryWriter) {
    if (logger) {
        logger_ = logger;
    } else {
        defaultLogger_ = std::make_unique<ConsoleLogger>();
        logger_ = defaultLogger_.get();
    }

    if (telemetryWriter) {
        telemetryWriter_ = telemetryWriter;
    } else {
        defaultTelemetryWriter_ = std::make_unique<NullTelemetryWriter>();
        telemetryWriter_ = defaultTelemetryWriter_.get();
    }
}

void BridgeSimulator::initAudioConfig(const EngineSimConfig& config) {
    sampleRate_ = (config.sampleRate > 0) ? config.sampleRate : EngineSimDefaults::SAMPLE_RATE;
    simulationFrequency_ = (config.simulationFrequency > 0) ? config.simulationFrequency : EngineSimDefaults::SIMULATION_FREQUENCY;
    ensureAudioConversionBufferSize(EngineSimDefaults::MAX_AUDIO_CHUNK_FRAMES * EngineSimDefaults::AUDIO_CHANNELS_STEREO);
}

void BridgeSimulator::pushTelemetry(const EngineSimStats& stats) {
    telemetry::EngineStateTelemetry engine;
    engine.currentRPM = stats.currentRPM;
    engine.currentLoad = stats.currentLoad;
    engine.exhaustFlow = stats.exhaustFlow;
    engine.manifoldPressure = stats.manifoldPressure;
    engine.activeChannels = stats.activeChannels;
    telemetryWriter_->writeEngineState(engine);

    telemetry::FramePerformanceTelemetry perf;
    perf.processingTimeMs = stats.processingTimeMs;
    telemetryWriter_->writeFramePerformance(perf);
}

void BridgeSimulator::setNameFromScript(const std::string& scriptPath) {
    if (scriptPath.empty()) {
        name_ = "Unnamed";
        return;
    }
    auto lastSlash = scriptPath.find_last_of("/\\");
    std::string filename = (lastSlash != std::string::npos)
        ? scriptPath.substr(lastSlash + 1)
        : scriptPath;
    auto lastDot = filename.find_last_of('.');
    name_ = (lastDot != std::string::npos)
        ? filename.substr(0, lastDot)
        : filename;
}

void BridgeSimulator::advanceFixedSteps(Simulator* sim, int simulationFrequency, double dt, bool ceil) {
    sim->startFrame(dt);
    const int simSteps = ceil
        ? static_cast<int>(std::ceil(simulationFrequency * dt))
        : static_cast<int>(simulationFrequency * dt);
    for (int i = 0; i < simSteps; ++i) {
        sim->simulateStep();
    }
    sim->endFrame();
}

void BridgeSimulator::drainSynthesizerBuffer(Simulator* sim) {
    constexpr int chunkSize = EngineSimDefaults::MAX_AUDIO_CHUNK_FRAMES;
    int16_t drainBuffer[chunkSize];
    while (sim->readAudioOutput(chunkSize, drainBuffer) > 0) {
        // Drain all pre-fill
    }
}

int16_t* BridgeSimulator::ensureAudioConversionBufferSize(size_t requiredSize) {
    if (requiredSize > m_audioConversionBuffer.size()) {
        m_audioConversionBuffer.resize(requiredSize);
    }
    return m_audioConversionBuffer.data();
}
