// SimulatorInitHelpers.cpp - Shared physics wiring/cleanup for Simulator subclasses
// DRY extraction: common init/destroy logic shared across SineSimulator and PistonEngineSimulator.
// Ownership of engine/vehicle/transmission is subclass-specific; these helpers handle
// only the COMMON physics wiring and system cleanup.

#include "simulator/SimulatorInitHelpers.h"

#include "simulator.h"
#include "engine.h"
#include "synthesizer.h"

#include <array>
#include <memory>
// simulator.h transitively includes: engine.h, transmission.h, vehicle.h,
// dynamometer.h, starter_motor.h, scs.h (RigidBody, RigidBodySystem, etc.)

namespace SimulatorInitHelpers {

void wirePhysics(const PhysicsWiringParams& p)
{
    // reset() clears position/velocity but also zeros mass and inertia (m=0, I=0).
    // m=0 causes division-by-zero in the physics solver's energy calculation.
    // Values are arbitrary for sine (no real physics) but must be nonzero.
    // 1.0 matches PistonEngine and is the simplest valid value.
    p.vehicleMass->reset();
    p.vehicleMass->m = 1.0;
    p.vehicleMass->I = 1.0;
    p.system->addRigidBody(p.vehicleMass);

    // Wire transmission and vehicle into physics system
    p.transmission->addToSystem(p.system, p.vehicleMass, p.vehicle, p.engine);
    p.vehicle->addToSystem(p.system, p.vehicleMass);

    // Dynamometer constraint
    p.dyno->connectCrankshaft(p.engine->getOutputCrankshaft());
    p.system->addConstraint(p.dyno);

    // Starter motor constraint
    p.starterMotor->connectCrankshaft(p.engine->getOutputCrankshaft());
    p.starterMotor->m_maxTorque = p.engine->getStarterTorque();
    p.starterMotor->m_rotationSpeed = -p.engine->getStarterSpeed();
    p.system->addConstraint(p.starterMotor);

    // Exhaust flow staging buffer — mirrors PistonEngineSimulator::placeAndInitialize()
    *p.outStagingBuffer = std::make_unique<double[]>(p.stagingCount).release();
}

void cleanupPhysics(
    atg_scs::RigidBodySystem*& system,
    double*& stagingBuffer)
{
    if (system != nullptr) {
        system->reset();
        std::unique_ptr<atg_scs::RigidBodySystem> systemDeleter(system);
        system = nullptr;
    }

    if (stagingBuffer != nullptr) {
        std::unique_ptr<double[]> bufferDeleter(stagingBuffer);
        stagingBuffer = nullptr;
    }
}

void initializeConvolutionFilters(Simulator* simulator) {
    if (!simulator) return;

    const Engine* engine = simulator->getEngine();
    if (!engine) return;

    const int exhaustCount = engine->getExhaustSystemCount();
    if (exhaustCount == 0) return;

    // Unit impulse: [INT16_MAX] represents a single sample at full scale
    // This initializes the ConvolutionFilter's m_shiftRegister to prevent null deref
    static const std::array<int16_t, 1> kUnitImpulse = { INT16_MAX };

    for (int i = 0; i < exhaustCount; ++i) {
        // Only initialize if not already initialized (avoid double-init)
        // We can't easily check m_shiftRegister here, but unit impulse is safe to re-init
        simulator->synthesizer().initializeImpulseResponse(
            kUnitImpulse.data(),
            1,
            1.0f,
            i
        );
    }

    // Disable convolution to save CPU (unit impulse makes it identity transform)
    // Presets have no real IR data, so convolution is unnecessary
    Synthesizer::AudioParameters params = simulator->synthesizer().getAudioParameters();
    params.convolution = 0.0f;
    simulator->synthesizer().setAudioParameters(params);
}

} // namespace SimulatorInitHelpers
