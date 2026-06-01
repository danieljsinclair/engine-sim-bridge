// BridgeSimulator.cpp - Universal ISimulator implementation
// Composes an injected Simulator subclass.
// All audio pipeline, telemetry, and control methods are identical
// regardless of which Simulator subclass is injected (OCP).

#include "simulator/BridgeSimulator.h"

#include <vector>
#include <cstring>

BridgeSimulator::BridgeSimulator(std::unique_ptr<Simulator> simulator, const std::string& name)
    : m_simulator(std::move(simulator))
    , name_(name)
{}

BridgeSimulator::~BridgeSimulator() {
    destroy();
}

// ============================================================================
// ISimulator Lifecycle
// ============================================================================

bool BridgeSimulator::create(const ISimulatorConfig& config, ILogging* logger, telemetry::ITelemetryWriter* telemetryWriter) {
    initDependencies(logger, telemetryWriter);
    initAudioConfig(config);
    return true;
}

void BridgeSimulator::destroy() {
    if (m_simulator) {
        m_simulator->endAudioRenderingThread();
        m_simulator->destroy();
        m_simulator = nullptr;
    }
}

std::string BridgeSimulator::getLastError() const {
    return m_lastError;
}

// ============================================================================
// ISimulator Audio Pipeline
// ============================================================================

void BridgeSimulator::update(double deltaTime) {
    advanceFixedSteps(m_simulator.get(), engineConfig_.simulationFrequency, deltaTime, true);
    pushTelemetry(getStats());
}

bool BridgeSimulator::renderOnDemand(float* buffer, int32_t frames, int32_t* written) {
    if (!buffer) throw std::runtime_error("BridgeSimulator::renderOnDemand() called with null buffer");
    if (frames <= 0) throw std::runtime_error("BridgeSimulator::renderOnDemand() called with invalid frame count");

    const double dt = static_cast<double>(frames) / engineConfig_.sampleRate;
    advanceFixedSteps(m_simulator.get(), engineConfig_.simulationFrequency, dt, false);
    m_simulator->synthesizer().renderAudioOnDemand();

    int16_t* conversionBuffer = ensureAudioConversionBufferSize(frames);
    int samplesRead = m_simulator->readAudioOutput(frames, conversionBuffer);
    EngineSimAudio::convertInt16ToStereoFloat(conversionBuffer, samplesRead, buffer, engineConfig_.volume, engineConfig_.convolutionLevel);

    if (samplesRead < frames) {
        EngineSimAudio::fillSilence(buffer + samplesRead * 2, frames - samplesRead);
    }
    if (written) *written = samplesRead;
    return true;
}

bool BridgeSimulator::readAudioBuffer(float* buffer, int32_t frames, int32_t* read) {
    if (!buffer) throw std::runtime_error("BridgeSimulator::readAudioBuffer() called with null buffer");
    if (frames <= 0) throw std::runtime_error("BridgeSimulator::readAudioBuffer() called with invalid frame count");

    int16_t* conversionBuffer = ensureAudioConversionBufferSize(frames);
    int samplesRead = m_simulator->readAudioOutput(frames, conversionBuffer);
    EngineSimAudio::convertInt16ToStereoFloat(conversionBuffer, samplesRead, buffer, engineConfig_.volume, engineConfig_.convolutionLevel);

    if (samplesRead < frames) {
        EngineSimAudio::fillSilence(buffer + samplesRead * 2, frames - samplesRead);
    }
    if (read) *read = samplesRead;
    return true;
}

bool BridgeSimulator::start() {
    drainSynthesizerBuffer(m_simulator.get());
    m_simulator->startAudioRenderingThread();
    return true;
}

void BridgeSimulator::stop() {
    m_simulator->endAudioRenderingThread();
}

// ============================================================================
// ISimulator Telemetry & Control
// ============================================================================

EngineSimStats BridgeSimulator::getStats() const {
    EngineSimStats stats = {};

    stats.currentRPM = m_simulator->getEngine()->getSpeed() * 60.0 / (2.0 * M_PI);
    stats.exhaustFlow = m_simulator->getTotalExhaustFlow();
    stats.processingTimeMs = m_simulator->getAverageProcessingTime() * 1000.0;

    if (m_simulator->m_dyno.m_enabled) {
        stats.dynoTorque = m_simulator->getFilteredDynoTorque();
        stats.dynoTargetRPM = std::abs(m_simulator->m_dyno.m_rotationSpeed) * 30.0 / M_PI;
    }

    if (m_simulator->getTransmission()) {
        stats.gear = m_simulator->getTransmission()->getGear();
    }

    if (m_simulator->getVehicle()) {
        stats.speedMph = m_simulator->getVehicle()->getSpeed();
    }

    return stats;
}

void BridgeSimulator::setThrottle(double position) {
    if (position < 0.0) position = 0.0;
    if (position > 1.0) position = 1.0;
    if (m_simulator->getEngine()) {
        m_simulator->getEngine()->setSpeedControl(position);
    }
}

void BridgeSimulator::setIgnition(bool on) {
    if (m_simulator->getEngine()) {
        m_simulator->getEngine()->getIgnitionModule()->m_enabled = on;
    }
}

void BridgeSimulator::setStarterMotor(bool on) {
    m_simulator->m_starterMotor.m_enabled = on;
}

void BridgeSimulator::setEnginePhase(EnginePhase phase) {
    enginePhase_ = phase;
}

// ============================================================================
// Drivetrain state transfer for engine hot-swap
// ============================================================================

bool BridgeSimulator::changeGear(int gearDelta) {
    if (gearDelta == 0) return false;  // OK: no change requested

    auto* trans = m_simulator->getTransmission();
    if (!trans) return false;

    int currentGear = trans->getGear();
    int newGear = currentGear + gearDelta;
    int gearCount = trans->getGearCount();

    if (newGear < -1 || newGear >= gearCount) return false;

    trans->changeGear(newGear);
    trans->setClutchPressure(newGear > 0 ? 1.0 : 0.0);
    return true;
}

void BridgeSimulator::setDynoTorqueScale(double scale) {
    if (scale < 0.0) throw std::runtime_error("Dyno scale must be non-negative");
    if (!m_simulator->m_dyno.m_enabled) return;  // OK: no-op when dyno disabled
    m_simulator->m_dyno.m_maxTorque = units::torque(EngineSimDefaults::DYNO_MAX_TORQUE_FT_LBS, units::ft_lb) * scale;
}

bool BridgeSimulator::configureDynoLoad(double loadFraction) {
    if (loadFraction <= 0) return false;

    m_simulator->m_dyno.m_enabled = true;
    m_simulator->m_dyno.m_hold = false;       // Brake-only: resists but doesn't drive
    const double radPerRpm = 3.14159265358979323846 / 30.0;
    m_simulator->m_dyno.m_rotationSpeed = EngineSimDefaults::DYNO_IDLE_RPM * radPerRpm;
    m_simulator->m_dyno.m_maxTorque = units::torque(EngineSimDefaults::DYNO_MAX_TORQUE_FT_LBS, units::ft_lb) * loadFraction;
    return true;
}

BridgeSimulator::DrivetrainSnapshot BridgeSimulator::captureDrivetrainState() const {
    DrivetrainSnapshot snapshot;

    auto* body = m_simulator->getVehicleMassBody();
    if (body) {
        snapshot.vehicleMassVtheta = body->v_theta;
        snapshot.vehicleMassI = body->I;
        snapshot.vehicleMassM = body->m;
    }

    auto* trans = m_simulator->getTransmission();
    if (trans) {
        snapshot.gear = trans->getGear();
    }

    snapshot.enginePhase = enginePhase_;

    return snapshot;
}

void BridgeSimulator::restoreDrivetrainState(const DrivetrainSnapshot& snapshot) {
    auto* body = m_simulator->getVehicleMassBody();
    if (body) {
        body->v_theta = snapshot.vehicleMassVtheta;
        body->I = snapshot.vehicleMassI;
        body->m = snapshot.vehicleMassM;
    }

    // Restore gear and engage clutch so drivetrain spins the new engine
    auto* trans = m_simulator->getTransmission();
    if (trans && snapshot.gear >= 0) {
        trans->changeGear(snapshot.gear);
        trans->setClutchPressure(snapshot.gear > 0 ? 1.0 : 0.0);
    }

    enginePhase_ = snapshot.enginePhase;
}

std::vector<uint8_t> BridgeSimulator::saveState() const {
    DrivetrainSnapshot snapshot = captureDrivetrainState();

    std::vector<uint8_t> data(sizeof(DrivetrainSnapshot));
    std::memcpy(data.data(), &snapshot, sizeof(DrivetrainSnapshot));
    return data;
}

void BridgeSimulator::restoreState(const std::vector<uint8_t>& data) {
    if (data.size() < sizeof(DrivetrainSnapshot)) throw std::runtime_error("Invalid snapshot data size");

    DrivetrainSnapshot snapshot;
    std::memcpy(&snapshot, data.data(), sizeof(DrivetrainSnapshot));
    restoreDrivetrainState(snapshot);
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

void BridgeSimulator::initAudioConfig(const ISimulatorConfig& config) {
    engineConfig_ = config;
    if (m_simulator) {
        if (config.simulationFrequency > 0) {
            m_simulator->setSimulationFrequency(config.simulationFrequency);
        } else {
            engineConfig_.simulationFrequency = m_simulator->getSimulationFrequency();
        }
    }
    ensureAudioConversionBufferSize(engineConfig_.maxChunkFrames);
}

void BridgeSimulator::pushTelemetry(const EngineSimStats& stats) {
    telemetry::EngineStateTelemetry engine;
    engine.currentRPM = stats.currentRPM;
    engine.currentLoad = stats.currentLoad;
    engine.exhaustFlow = stats.exhaustFlow;
    engine.manifoldPressure = stats.manifoldPressure;
    engine.activeChannels = stats.activeChannels;
    engine.gear = stats.gear;
    engine.speedMph = stats.speedMph;
    engine.enginePhase = enginePhase_;
    telemetryWriter_->writeEngineState(engine);

    telemetry::FramePerformanceTelemetry perf;
    perf.processingTimeMs = stats.processingTimeMs;
    telemetryWriter_->writeFramePerformance(perf);
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
    std::vector<int16_t> drainBuffer(engineConfig_.maxChunkFrames);
    while (sim->readAudioOutput(engineConfig_.maxChunkFrames, drainBuffer.data()) > 0) {
        // Drain all pre-fill
    }
}

int16_t* BridgeSimulator::ensureAudioConversionBufferSize(size_t requiredSize) {
    if (requiredSize > m_audioConversionBuffer.size()) {
        m_audioConversionBuffer.resize(requiredSize);
    }
    return m_audioConversionBuffer.data();
}
