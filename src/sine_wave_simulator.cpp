#include "sine_wave_simulator.h"
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
    , defaultLogger_(logger ? nullptr : new ConsoleLogger())
    , logger_(logger ? logger : defaultLogger_.get())
{
}

SineWaveSimulator::~SineWaveSimulator() {
    destroy();
}

void SineWaveSimulator::initialize(const Parameters &params) {
    // Initialize base simulator (sets up synthesizer, physics system, etc.)
    Simulator::initialize(params);

    // Create minimal dummy objects - required by base Simulator class
    m_dummyEngine        = new SineEngine();
    m_dummyVehicle       = new SineVehicle();
    m_dummyTransmission  = new SineTransmission();

    // Load simulation (sets up physics constraints)
    loadSimulation(m_dummyEngine, m_dummyVehicle, m_dummyTransmission);

    // Logger is always available (default or injected)
    logger_->debug(LogMask::PHYSICS, "SineWaveSimulator initialized (1 dummy crankshaft, 0 cylinders)");
}

void SineWaveSimulator::loadSimulation(Engine* engine, Vehicle* vehicle, Transmission* transmission) {
    // Call base to set m_engine, m_vehicle, m_transmission pointers
    Simulator::loadSimulation(engine, vehicle, transmission);

    // Initialize synthesizer after engine is attached so channel count is valid.
    initializeSynthesizer();
    static const int16_t kUnitImpulse[1] = { INT16_MAX };
    synthesizer().initializeImpulseResponse(kUnitImpulse, 1, 1.0f, 0);

    // Override defaults for a clean sine: the real-engine defaults apply air noise
    // (random amplitude modulation) and jitter, which are meaningless for a sine wave.
    Synthesizer::AudioParameters p;
    p.airNoise         = 0.0f;  // no random noise overlay
    p.inputSampleNoise = 0.0f;  // no jitter
    p.dF_F_mix         = 0.0f;  // skip derivative path, straight signal passthrough
    synthesizer().setAudioParameters(p);
    
    // Setup physics system connections (required by Simulator::simulateStep())
    m_vehicleMass.reset();
    m_vehicleMass.m = 1.0;
    m_vehicleMass.I = 1.0;
    m_system->addRigidBody(&m_vehicleMass);
    
    transmission->addToSystem(m_system, &m_vehicleMass, vehicle, engine);
    vehicle->addToSystem(m_system, &m_vehicleMass);
    
    // Connect dyno and starter motor to crankshaft (required by Simulator::simulateStep())
    m_dyno.connectCrankshaft(engine->getOutputCrankshaft());
    m_system->addConstraint(&m_dyno);
    
    m_starterMotor.connectCrankshaft(engine->getOutputCrankshaft());
    m_starterMotor.m_maxTorque = engine->getStarterTorque();
    m_starterMotor.m_rotationSpeed = -engine->getStarterSpeed();
    m_system->addConstraint(&m_starterMotor);
}

void SineWaveSimulator::destroy() {
    // Stop audio/render state before releasing owned objects.
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
}

void SineWaveSimulator::simulateStep_() {
    // SineEngine computes RPM on demand from getSpeedControl() in getRpm().
    // No physics to step; writeToSynthesizer() reads engine state directly.
}

void SineWaveSimulator::writeToSynthesizer() {
    const double rpm = m_dummyEngine->getRpm();
    const double frequency = rpm / 6.0;
    const double phaseIncrement = TWO_PI / (synthesizer().getInputSampleRate() / frequency);

    // Scale to INT16 range so the synthesizer leveler has signal to work with.
    // The leveler target is 30000 with max gain 1.9 — normalized [-1,1] values
    // would be rounded to zero before leveling could take effect.
    double sample = std::sin(m_phase) * 28000.0;
    synthesizer().writeInput(&sample);
    
    // Update phase accumulator
    m_phase += phaseIncrement;
    if (m_phase >= TWO_PI) {
        m_phase -= TWO_PI;
    }
}
