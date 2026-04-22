// SineSimulator.cpp - Pure Simulator subclass for sine wave test output
// Lifecycle mirrors PistonEngineSimulator: factory calls loadSimulation() after shared init.
// Overrides simulateStep_() and writeToSynthesizer() for sine-specific behaviour.

#include "simulator/SineSimulator.h"
#include "simulator/SineEngine.h"
#include "simulator/SineVehicle.h"
#include "simulator/SineTransmission.h"
#include "simulator/SimulatorInitHelpers.h"
#include "simulator/engine_sim_bridge.h"
#include "synthesizer.h"
#include <cmath>

SineSimulator::~SineSimulator() {
    destroy();
}

void SineSimulator::loadSimulation(Engine* engine, Vehicle* vehicle, Transmission* transmission) {
    // Ignore passed-in pointers (nullptr from factory) — create sine-specific objects instead.
    // This mirrors PistonEngineSimulator which receives real objects from the script compiler.
    Engine* sineEngine = new SineEngine();
    Vehicle* sineVehicle = new SineVehicle();
    Transmission* sineTranny = new SineTransmission();

    // Store pointers in base class — same as PistonEngineSimulator line 34
    Simulator::loadSimulation(sineEngine, sineVehicle, sineTranny);

    // Initialize synthesizer — base loadSimulation() doesn't call it.
    // PistonEngineSimulator gets it via placeAndInitialize(); we call directly.
    initializeSynthesizer();

    // Unit impulse response required to prevent null deref:
    // ConvolutionFilter::f() dereferences m_shiftRegister which is nullptr without this init,
    // even though convolution=0.0f below disables convolution processing.
    static const int16_t kUnitImpulse[1] = { INT16_MAX };
    synthesizer().initializeImpulseResponse(kUnitImpulse, 1, 1.0f, 0);

    // Disable all noise and convolution — override defaults (airNoise=1.0,
    // inputSampleNoise=0.5, dF_F_mix=0.01, convolution=1.0f) that add unwanted
    // hiss/roar to a pure sine test signal. Convolution is disabled for performance
    // but initializeImpulseResponse above is still needed to prevent null pointer.
    Synthesizer::AudioParameters p;
    p.airNoise         = 0.0f;
    p.inputSampleNoise = 0.0f;
    p.dF_F_mix         = 0.0f;
    p.convolution      = 0.0f;
    synthesizer().setAudioParameters(p);

    // Shared physics wiring — DRY via SimulatorInitHelpers
    SimulatorInitHelpers::wirePhysics(
        sineEngine, sineTranny, sineVehicle,
        m_vehicleMass, m_system,
        m_dyno, m_starterMotor,
        m_exhaustFlowStagingBuffer,
        sineEngine->getExhaustSystemCount());
}

void SineSimulator::destroy() {
    // Idempotent — safe to call multiple times (destructor + explicit destroy)
    // Note: endAudioRenderingThread() NOT called here — BridgeSimulator::destroy()
    // handles audio thread lifecycle before calling the inner simulator's destroy().

    // Shared system/staging cleanup — DRY via SimulatorInitHelpers
    SimulatorInitHelpers::cleanupPhysics(m_system, m_exhaustFlowStagingBuffer);

    // SineSimulator owns engine/vehicle/transmission (created via new in loadSimulation())
    if (m_engine)        { m_engine->destroy(); delete m_engine;        m_engine = nullptr; }
    if (m_vehicle)       { delete m_vehicle;                            m_vehicle = nullptr; }
    if (m_transmission)  { delete m_transmission;                       m_transmission = nullptr; }

    Simulator::destroy();
}

void SineSimulator::simulateStep_() {
    const double rpm = m_engine ? m_engine->getRpm() : 800.0;
    const double frequency = rpm / 6.0;
    const double phaseIncrement = TWO_PI / (synthesizer().getInputSampleRate() / frequency);

    m_phase += phaseIncrement;
    if (m_phase >= TWO_PI) {
        m_phase -= TWO_PI;
    }

    m_sineValue = std::sin(m_phase) * 28000.0;
}

void SineSimulator::writeToSynthesizer() {
    const int exhaustSystemCount = m_engine->getExhaustSystemCount();
    for (int i = 0; i < exhaustSystemCount; ++i) {
        m_exhaustFlowStagingBuffer[i] = 0;
    }

    m_exhaustFlowStagingBuffer[0] = m_sineValue;
    synthesizer().writeInput(m_exhaustFlowStagingBuffer);
}
