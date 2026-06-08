// BridgeSimulator.cpp - Universal ISimulator implementation
// Composes an injected Simulator subclass.
// All audio pipeline, telemetry, and control methods are identical
// regardless of which Simulator subclass is injected (OCP).

#include "simulator/BridgeSimulator.h"
#include "simulator/GearConventions.h"

#include <vector>
#include <cstring>
#include <common/Verification.h>
#include "simulator/BridgeSimulator.h"

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

    // Wire brake constraint into the physics system
    if (m_simulator->getVehicle() != nullptr) {
        m_brakeConstraint.initialize(
            m_simulator->getVehicleMassBody(),
            m_simulator->getVehicle());
        m_simulator->getSystem()->addConstraint(&m_brakeConstraint);
    }

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

bool BridgeSimulator::readAudioBuffer(float* buffer, int32_t framesToRead, int32_t* read) {
    ASSERT(buffer, "BridgeSimulator::readAudioBuffer() called with null buffer");
    ASSERT(read, "BridgeSimulator::readAudioBuffer() called with null *read pointer");
    *read = 0;
    if (framesToRead > 0) {
        int16_t* conversionBuffer = ensureAudioConversionBufferSize(framesToRead);
        int samplesRead = m_simulator->readAudioOutput(framesToRead, conversionBuffer);
        EngineSimAudio::convertInt16ToStereoFloat(conversionBuffer, samplesRead, buffer, engineConfig_.volume, engineConfig_.convolutionLevel);
        *read = samplesRead;
        return true;
    }

    return false;
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

void BridgeSimulator::getEngineStats(EngineSimStats& stats) const {
    ASSERT(m_simulator, "BridgeSimulator::getStats() called but m_simulator is null");
    if (m_simulator->getEngine()) {
        stats.currentRPM = m_simulator->getEngine()->getSpeed() * EngineSimDefaults::RAD_PER_SEC_TO_RPM;
        stats.exhaustFlow = m_simulator->getTotalExhaustFlow();
        stats.processingTimeMs = m_simulator->getAverageProcessingTime() * 1000.0;
    }
}

void BridgeSimulator::getDynoStats(EngineSimStats& stats) const {
    ASSERT(m_simulator, "BridgeSimulator::getDynoStats() called but m_simulator is null");
    if (m_simulator->m_dyno.m_enabled) {
        stats.dynoTorque = m_simulator->getFilteredDynoTorque();
        stats.dynoTargetRPM = std::abs(m_simulator->m_dyno.m_rotationSpeed) * EngineSimDefaults::RAD_PER_SEC_TO_RPM;
        // Engine torque from dyno measurement (internal units = Nm)
        stats.engineTorqueNm = m_simulator->getFilteredDynoTorque();
    }
}

void BridgeSimulator::getVehicleStats(EngineSimStats& stats) const {
    ASSERT(m_simulator, "BridgeSimulator::getStats() called but m_simulator is null");
    if (m_simulator->getVehicle()) {
        // Vehicle::getSpeed() returns m/s
        stats.vehicleSpeedKmh = m_simulator->getVehicle()->getSpeed() * EngineSimDefaults::MS_TO_KMH;
    }
}

void BridgeSimulator::getTransmissionStats(EngineSimStats& stats) const {
    ASSERT(m_simulator, "BridgeSimulator::getStats() called but m_simulator is null");
    if (m_simulator->getTransmission()) {
        // Translate engine-sim convention (EngineSimGear) to bridge convention (BridgeGear)
        int rawGear = m_simulator->getTransmission()->getGear();
        stats.gear = static_cast<int>(bridge::toBridge(rawGear));

        // Real torque from clutch constraint (populated by solver after each step)
        if (!m_simulator->m_dyno.m_enabled) {
            const auto& clutch = m_simulator->getTransmission()->getClutchConstraint();
            // ClutchConstraint J = [-1, +1] on angular velocity of body 0 (crankshaft),
            // body 1 (drivetrain virtual mass). The solver stores F_t[i][k] = J * lambda/dt.
            // Engine crankshaft spins CW (v_theta < 0). During acceleration, lambda < 0
            // so F_t[0][0] > 0 (clutch reaction opposing engine) and F_t[0][1] < 0
            // (clutch driving the drivetrain in its negative/CW direction).
            // For user display: engine producing power = positive, engine braking = negative.
            stats.engineTorqueNm = clutch.F_t[0][0];
            stats.drivetrainTorqueNm = -clutch.F_t[0][1];

            // Wheel-side torque: clutch torque * gear_ratio * diff_ratio.
            // A friction clutch transmits equal-and-opposite torque (Newton's 3rd law),
            // so both sides show the same magnitude. Gear multiplication gives different
            // values -- engine side = raw clutch torque, wheel side = multiplied torque.
            const double gearRatio = m_simulator->getTransmission()->getGearRatio();
            if (m_simulator->getVehicle() && gearRatio > 0.0) {
                const double diffRatio = m_simulator->getVehicle()->getDiffRatio();
                stats.drivetrainTorqueNm = -clutch.F_t[0][1] * gearRatio * diffRatio;
            }
        }
    }
}

EngineSimStats BridgeSimulator::getStats() const {
    EngineSimStats stats = {};

    getEngineStats(stats);
    getDynoStats(stats);
    getVehicleStats(stats);
    getTransmissionStats(stats);

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

int BridgeSimulator::setGear(int gear) {
    int newGear = 0x7FFFFFFF;
    if (m_simulator->getTransmission()) {
        // Translate bridge convention (BridgeGear) to engine-sim convention (EngineSimGear)
        int engineSimGear = static_cast<int>(bridge::toEngineSim(static_cast<bridge::BridgeGear>(gear)));
        m_simulator->getTransmission()->changeGear(engineSimGear);
        newGear = engineSimGear;
    }
    return newGear;
}

int BridgeSimulator::getGear() const {
    if (m_simulator->getTransmission()) {
        int engineSimGear = m_simulator->getTransmission()->getGear();
        return static_cast<int>(bridge::toBridge(engineSimGear));

    }
    return 0;  // Default to neutral if no transmission
}

// ============================================================================
// Drivetrain state transfer for engine hot-swap
// ============================================================================

bool BridgeSimulator::changeGear(int gearDelta) {
    if (gearDelta == 0) return false;  // OK: no change requested

    auto* trans = m_simulator->getTransmission();
    if (!trans) {
        logger_->warning(LogMask::BRIDGE, "Cannot change gear: no transmission in simulator");
        return false;
    }

    int currentGear = trans->getGear();
    int newGear = currentGear + gearDelta;
    int gearCount = trans->getGearCount();

    newGear = std::max(newGear, -1);  // Clamp at -1 (neutral)
    newGear = std::min(newGear, gearCount - 1);  // Clamp at max gear

    trans->changeGear(newGear);
    trans->setClutchPressure(newGear > 0 ? 1.0 : 0.0);
    return true;
}

void BridgeSimulator::setClutchPressure(double pressure) {
    if (m_simulator->getTransmission()) {
        m_simulator->getTransmission()->setClutchPressure(pressure);
    }
}

void BridgeSimulator::setBrakePressure(double pressure) {
    m_brakeConstraint.setBrakeLevel(pressure);
}

double BridgeSimulator::getEngineRpm() const {
    if (m_simulator->getEngine()) {
        return m_simulator->getEngine()->getSpeed() * EngineSimDefaults::RAD_PER_SEC_TO_RPM;
    }
    return 0.0;
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

void BridgeSimulator::setEnginePhase(EnginePhase phase) {
    enginePhase_ = phase;
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

    // Phase is NOT restored — it's operational state, not drivetrain physics.
    // The new engine starts at Stopped. The caller (transferDrivetrainState)
    // engages the starter, which transitions to Cranking. The engine catches
    // via starter motor + transferred drivetrain momentum.
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
    engine.speedMph = stats.speedMph();
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
