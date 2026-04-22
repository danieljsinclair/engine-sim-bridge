// SimulatorInitHelpers.h - Shared physics wiring/cleanup for Simulator subclasses
// DRY extraction of common init/destroy logic used by both SineSimulator
// and PistonEngineSimulator. Ownership of engine/vehicle/transmission remains
// subclass-specific; these helpers handle the COMMON physics wiring only.

#ifndef ENGINE_SIM_BRIDGE_SIMULATOR_INIT_HELPERS_H
#define ENGINE_SIM_BRIDGE_SIMULATOR_INIT_HELPERS_H

// Forward declarations (avoid pulling full engine-sim headers here)
class Engine;
class Transmission;
class Vehicle;
class Dynamometer;
class StarterMotor;

namespace atg_scs {
    class RigidBodySystem;
    class RigidBody;
}

namespace SimulatorInitHelpers {

/// Shared physics wiring: vehicleMass, transmission, vehicle, dyno, starter motor, staging buffer.
/// Called after loadSimulation() — wires the common constraint graph that all Simulator
/// subclasses need regardless of engine type.
///
/// @param engine           The loaded Engine (for crankshaft/starter torque/speed)
/// @param transmission     The loaded Transmission
/// @param vehicle          The loaded Vehicle
/// @param vehicleMass      The vehicle rigid body (m_vehicleMass)
/// @param system           The physics system (m_system)
/// @param dyno             The dynamometer constraint (m_dyno)
/// @param starterMotor     The starter motor constraint (m_starterMotor)
/// @param outStagingBuffer Receives the newly allocated staging buffer (caller manages lifetime)
/// @param stagingCount     Number of exhaust systems (staging buffer element count)
void wirePhysics(
    Engine* engine,
    Transmission* transmission,
    Vehicle* vehicle,
    atg_scs::RigidBody& vehicleMass,
    atg_scs::RigidBodySystem* system,
    Dynamometer& dyno,
    StarterMotor& starterMotor,
    double*& outStagingBuffer,
    int stagingCount);

/// Shared cleanup: system reset/delete, staging buffer delete.
/// Does NOT delete engine/vehicle/transmission — ownership is subclass-specific.
/// The caller is responsible for its own ownership cleanup and calling Simulator::destroy().
///
/// @param system           Ref to m_system pointer (will be nulled)
/// @param stagingBuffer    Ref to staging buffer pointer (will be deleted and nulled)
void cleanupPhysics(
    atg_scs::RigidBodySystem*& system,
    double*& stagingBuffer);

} // namespace SimulatorInitHelpers

#endif // ENGINE_SIM_BRIDGE_SIMULATOR_INIT_HELPERS_H
