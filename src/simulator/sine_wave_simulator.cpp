#include "simulator/sine_wave_simulator.h"
#include "simulator/SineEngine.h"
#include "simulator/SineVehicle.h"
#include "simulator/SineTransmission.h"
#include <cmath>

SineWaveSimulator::SineWaveSimulator(ILogging* logger)
    : Simulator()
    , m_phase(0.0)
    , created_(false)
    , lastError_()
{
    (void)logger;  // Logger is now injected via create() -> initDependencies()
    name_ = "SineWave";
}

SineWaveSimulator::~SineWaveSimulator() {
    destroy();
}

// ============================================================================
// ISimulator Interface Implementation
// ============================================================================

bool SineWaveSimulator::create(const EngineSimConfig& config, ILogging* logger, telemetry::ITelemetryWriter* telemetryWriter) {
    initDependencies(logger, telemetryWriter);

    initAudioConfig(config);

    Simulator::Parameters simParams;
    simParams.systemType = Simulator::SystemType::NsvOptimized;
    initialize(simParams);
    setSimulationFrequency(simulationFrequency_);

    // Create dummy objects — loadSimulation() stores them in Simulator::m_engine etc.
    Engine* engine       = new SineEngine();
    Vehicle* vehicle     = new SineVehicle();
    Transmission* tranny = new SineTransmission();

    loadSimulation(engine, vehicle, tranny);

    created_ = true;
    return true;
}

bool SineWaveSimulator::loadScript(const std::string& path, const std::string& assetBase) {
    (void)path;
    (void)assetBase;
    return true;
}

void SineWaveSimulator::destroy() {
    endAudioRenderingThread();

    if (m_system != nullptr) {
        m_system->reset();
        delete m_system;
        m_system = nullptr;
    }

    // Clean up owned objects — Simulator stores but doesn't own them
    if (m_exhaustFlowStagingBuffer) { delete[] m_exhaustFlowStagingBuffer; m_exhaustFlowStagingBuffer = nullptr; }
    if (m_engine)        { m_engine->destroy(); delete m_engine;        m_engine = nullptr; }
    if (m_vehicle)       { delete m_vehicle;                            m_vehicle = nullptr; }
    if (m_transmission)  { delete m_transmission;                       m_transmission = nullptr; }

    Simulator::destroy();
    created_ = false;
}

std::string SineWaveSimulator::getLastError() const {
    return lastError_;
}

// ============================================================================
// Simulator Protected Overrides
// ============================================================================

void SineWaveSimulator::initialize(const Parameters &params) {
    Simulator::initialize(params);
}

void SineWaveSimulator::loadSimulation(Engine* engine, Vehicle* vehicle, Transmission* transmission) {
    Simulator::loadSimulation(engine, vehicle, transmission);

    // Initialize synthesizer with clean parameters for sine wave mode
    initializeSynthesizer();
    static const int16_t kUnitImpulse[1] = { INT16_MAX };
    synthesizer().initializeImpulseResponse(kUnitImpulse, 1, 1.0f, 0);

    Synthesizer::AudioParameters p;
    p.airNoise         = 0.0f;
    p.inputSampleNoise = 0.0f;
    p.dF_F_mix         = 0.0f;
    synthesizer().setAudioParameters(p);

    // Physics setup
    m_vehicleMass.reset();
    m_vehicleMass.m = 1.0;
    m_vehicleMass.I = 1.0;
    m_system->addRigidBody(&m_vehicleMass);

    transmission->addToSystem(m_system, &m_vehicleMass, vehicle, engine);
    vehicle->addToSystem(m_system, &m_vehicleMass);

    m_dyno.connectCrankshaft(engine->getOutputCrankshaft());
    m_system->addConstraint(&m_dyno);

    m_starterMotor.connectCrankshaft(engine->getOutputCrankshaft());
    m_starterMotor.m_maxTorque = engine->getStarterTorque();
    m_starterMotor.m_rotationSpeed = -engine->getStarterSpeed();
    m_system->addConstraint(&m_starterMotor);

    // Allocate staging buffer — mirrors PistonEngineSimulator::placeAndInitialize()
    m_exhaustFlowStagingBuffer = new double[engine->getExhaustSystemCount()];
}

void SineWaveSimulator::simulateStep_() {
    // Advance sine wave phase — the "physics" of a sine wave oscillator.
    // RPM is derived from throttle position via SineEngine::getRpm() with no lag.
    // Mirrors PistonEngineSimulator::simulateStep_() which advances combustion
    // and fluid dynamics. Computes the output sample here; writeToSynthesizer()
    // just writes it to the staging buffer.
    const double rpm = m_engine ? m_engine->getRpm() : 800.0;
    const double frequency = rpm / 6.0;
    const double phaseIncrement = TWO_PI / (synthesizer().getInputSampleRate() / frequency);

    m_phase += phaseIncrement;
    if (m_phase >= TWO_PI) {
        m_phase -= TWO_PI;
    }

    m_sineValue = std::sin(m_phase) * 28000.0;
}

void SineWaveSimulator::writeToSynthesizer() {
    // Zero → populate → write. Same pipeline as PistonEngineSimulator.
    const int exhaustSystemCount = m_engine->getExhaustSystemCount();
    for (int i = 0; i < exhaustSystemCount; ++i) {
        m_exhaustFlowStagingBuffer[i] = 0;
    }

    m_exhaustFlowStagingBuffer[0] = m_sineValue;

    synthesizer().writeInput(m_exhaustFlowStagingBuffer);
}
