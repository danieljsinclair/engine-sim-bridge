// SimulatorInitHelpers.cpp - Shared physics wiring/cleanup for Simulator subclasses
// DRY extraction: common init/destroy logic shared across SineSimulator and PistonEngineSimulator.
// Ownership of engine/vehicle/transmission is subclass-specific; these helpers handle
// only the COMMON physics wiring and system cleanup.

#include "simulator/SimulatorInitHelpers.h"

#include "simulator.h"
// simulator.h transitively includes: engine.h, transmission.h, vehicle.h,
// dynamometer.h, starter_motor.h, scs.h (RigidBody, RigidBodySystem, etc.)

namespace SimulatorInitHelpers {

void wirePhysics(
    Engine* engine,
    Transmission* transmission,
    Vehicle* vehicle,
    atg_scs::RigidBody& vehicleMass,
    atg_scs::RigidBodySystem* system,
    Dynamometer& dyno,
    StarterMotor& starterMotor,
    double*& outStagingBuffer,
    int stagingCount)
{
    // reset() clears position/velocity but also zeros mass and inertia (m=0, I=0).
    // m=0 causes division-by-zero in the physics solver's energy calculation.
    // Values are arbitrary for sine (no real physics) but must be nonzero.
    // 1.0 matches PistonEngine and is the simplest valid value.
    vehicleMass.reset();
    vehicleMass.m = 1.0;
    vehicleMass.I = 1.0;
    system->addRigidBody(&vehicleMass);

    // Wire transmission and vehicle into physics system
    transmission->addToSystem(system, &vehicleMass, vehicle, engine);
    vehicle->addToSystem(system, &vehicleMass);

    // Dynamometer constraint
    dyno.connectCrankshaft(engine->getOutputCrankshaft());
    system->addConstraint(&dyno);

    // Starter motor constraint
    starterMotor.connectCrankshaft(engine->getOutputCrankshaft());
    starterMotor.m_maxTorque = engine->getStarterTorque();
    starterMotor.m_rotationSpeed = -engine->getStarterSpeed();
    system->addConstraint(&starterMotor);

    // Exhaust flow staging buffer — mirrors PistonEngineSimulator::placeAndInitialize()
    outStagingBuffer = new double[stagingCount];
}

void cleanupPhysics(
    atg_scs::RigidBodySystem*& system,
    double*& stagingBuffer)
{
    if (system != nullptr) {
        system->reset();
        delete system;
        system = nullptr;
    }

    if (stagingBuffer != nullptr) {
        delete[] stagingBuffer;
        stagingBuffer = nullptr;
    }
}

} // namespace SimulatorInitHelpers
