#include "simulator/sine_wave_simulator.h"
#include "throttle.h"
#include "units.h"
#include <cmath>

// Private helper classes - minimal self-initializing assets for sine wave mode
namespace {
    class SineEngine : public Engine {
    public:
        SineEngine() {
            Throttle* throttle = new Throttle();
            Parameters p = {};
            p.name              = "Sine Wave Test Engine";
            p.cylinderBanks     = 0;
            p.cylinderCount     = 0;
            p.crankshaftCount   = 1;
            // Keep one input channel alive for synthesizer pipeline.
            p.exhaustSystemCount = 1;
            p.intakeCount       = 0;
            p.throttle          = throttle;
            initialize(p);

            // Initialize the single crankshaft with no rod journals
            Crankshaft::Parameters cp = {};
            cp.mass              = units::mass(10, units::kg);
            cp.flywheelMass      = units::mass(5, units::kg);
            cp.momentOfInertia   = 0.1;
            cp.crankThrow        = units::distance(50, units::mm);
            cp.rodJournals       = 0;
            getCrankshaft(0)->initialize(cp);
        }

        virtual double getRpm() const override {
            // Engine::getSpeedControl() is non-const in the base API despite being a pure read.
            return 800.0 + const_cast<SineEngine*>(this)->getSpeedControl() * 5200.0;
        }

        virtual double getSpeed() const override {
            // Base contract: return rad/s so units::toRpm() in the bridge gives correct RPM.
            // 1 RPM = 2π/60 rad/s = 0.104719755 rad/s
            return getRpm() * 0.104719755;
        }
    };

    class SineVehicle : public Vehicle {
    public:
        SineVehicle() {
            Parameters p = {};
            p.mass       = units::mass(1000, units::kg);
            p.diffRatio  = 1.0;
            p.tireRadius = units::distance(0.3, units::m);
            initialize(p);
        }
    };

    class SineTransmission : public Transmission {
    public:
        SineTransmission() {
            static const double gearRatios[] = {1.0};
            Parameters p = {};
            p.GearCount         = 1;
            p.GearRatios        = gearRatios;
            p.MaxClutchTorque   = units::torque(1000, units::Nm);
            initialize(p);
        }
    };
}

SineWaveSimulator::SineWaveSimulator(ILogging* logger)
    : Simulator()
    , m_dummyEngine(nullptr)
    , m_dummyVehicle(nullptr)
    , m_dummyTransmission(nullptr)
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

    // Store for renderOnDemand() simulation stepping
    sampleRate_ = (config.sampleRate > 0) ? config.sampleRate : 48000;
    simulationFrequency_ = (config.simulationFrequency > 0) ? config.simulationFrequency : 10000;

    // Allocate audio conversion buffer (RAII in base class)
    ensureAudioConversionBufferSize(4096 * 2);

    Simulator::Parameters simParams;
    simParams.systemType = Simulator::SystemType::NsvOptimized;
    initialize(simParams);
    setSimulationFrequency(simulationFrequency_);

    m_dummyEngine        = new SineEngine();
    m_dummyVehicle       = new SineVehicle();
    m_dummyTransmission  = new SineTransmission();

    // loadSimulation() handles all physics wiring + synthesizer setup
    loadSimulation(m_dummyEngine, m_dummyVehicle, m_dummyTransmission);

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

    if (m_dummyEngine)       { m_dummyEngine->destroy(); delete m_dummyEngine;       m_dummyEngine       = nullptr; }
    if (m_dummyVehicle)      { delete m_dummyVehicle;                                m_dummyVehicle      = nullptr; }
    if (m_dummyTransmission) { delete m_dummyTransmission;                           m_dummyTransmission = nullptr; }

    Simulator::destroy();
    created_ = false;

    // Audio conversion buffer is RAII-managed by base class, no cleanup needed
}

std::string SineWaveSimulator::getLastError() const {
    return lastError_;
}

void SineWaveSimulator::update(double deltaTime) {
    if (!created_) return;

    // ceil=true: slight over-production prevents Threaded underruns, ensures at least
    // 1 step for tiny dt (e.g. SyncPull retry calls update(1/sampleRate)).
    advanceFixedSteps(this, simulationFrequency_, deltaTime, true);

    EngineSimStats stats = getStats();
    pushTelemetry(stats);
}

EngineSimStats SineWaveSimulator::getStats() const {
    EngineSimStats stats = {};

    if (m_dummyEngine && created_) {
        stats.currentRPM = m_dummyEngine->getRpm();
        stats.currentLoad = 0.0;
        stats.exhaustFlow = getTotalExhaustFlow();
        stats.manifoldPressure = 0.0;
        stats.activeChannels = 1;
        stats.processingTimeMs = getAverageProcessingTime() * 1000.0;
    }

    return stats;
}

void SineWaveSimulator::setThrottle(double position) {
    if (!m_dummyEngine) return;

    if (position < 0.0) position = 0.0;
    if (position > 1.0) position = 1.0;

    m_dummyEngine->setSpeedControl(position);
}

void SineWaveSimulator::setIgnition(bool on) {
    (void)on;
}

void SineWaveSimulator::setStarterMotor(bool on) {
    (void)on;
}

bool SineWaveSimulator::renderOnDemand(float* buffer, int32_t frames, int32_t* written) {
    if (!created_ || !buffer || frames <= 0) {
        if (written) *written = 0;
        return false;
    }

    // Run simulation to feed synthesizer with sine samples.
    // ceil=false: SyncPull retry loop handles any deficit from truncation.
    const double dt = static_cast<double>(frames) / sampleRate_;
    advanceFixedSteps(this, simulationFrequency_, dt, false);

    // Render and read audio into separate int16 buffer (avoids float/int16 aliasing)
    synthesizer().renderAudioOnDemand();

    // Ensure conversion buffer is large enough
    int16_t* conversionBuffer = ensureAudioConversionBufferSize(frames);
    int samplesRead = readAudioOutput(frames, conversionBuffer);

    // Convert mono int16 to stereo float using separate buffers (no aliasing)
    EngineSimAudio::convertInt16ToStereoFloat(conversionBuffer, buffer, samplesRead);

    if (written) *written = samplesRead;

    return true;
}

bool SineWaveSimulator::readAudioBuffer(float* buffer, int32_t frames, int32_t* read) {
    if (!created_ || !buffer || frames <= 0) {
        if (read) *read = 0;
        return false;
    }

    // Ensure conversion buffer is large enough
    int16_t* conversionBuffer = ensureAudioConversionBufferSize(frames);
    int samplesRead = readAudioOutput(frames, conversionBuffer);

    // Convert mono int16 to stereo float using separate buffers (no aliasing)
    EngineSimAudio::convertInt16ToStereoFloat(conversionBuffer, buffer, samplesRead);

    if (read) *read = samplesRead;

    return true;
}

bool SineWaveSimulator::start() {
    if (!created_) return false;

    drainSynthesizerBuffer(this);
    startAudioRenderingThread();
    return true;
}

void SineWaveSimulator::stop() {
    if (!created_) return;

    endAudioRenderingThread();
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

    m_dummyTransmission->addToSystem(m_system, &m_vehicleMass, m_dummyVehicle, m_dummyEngine);
    m_dummyVehicle->addToSystem(m_system, &m_vehicleMass);

    m_dyno.connectCrankshaft(engine->getOutputCrankshaft());
    m_system->addConstraint(&m_dyno);

    m_starterMotor.connectCrankshaft(engine->getOutputCrankshaft());
    m_starterMotor.m_maxTorque = engine->getStarterTorque();
    m_starterMotor.m_rotationSpeed = -engine->getStarterSpeed();
    m_system->addConstraint(&m_starterMotor);
}

void SineWaveSimulator::simulateStep_() {
    // SineEngine computes RPM on demand from getSpeedControl() in getRpm().
    // No physics to step; writeToSynthesizer() reads engine state directly.
}

void SineWaveSimulator::writeToSynthesizer() {
    const double rpm = m_dummyEngine->getRpm();
    const double frequency = rpm / 6.0;
    const double phaseIncrement = TWO_PI / (synthesizer().getInputSampleRate() / frequency);

    double sample = std::sin(m_phase) * 28000.0;
    synthesizer().writeInput(&sample);

    m_phase += phaseIncrement;
    if (m_phase >= TWO_PI) {
        m_phase -= TWO_PI;
    }
}
