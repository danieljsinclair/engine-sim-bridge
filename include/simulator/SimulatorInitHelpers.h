// SimulatorInitHelpers.h - Shared physics wiring/cleanup for Simulator subclasses
// DRY extraction of common init/destroy logic used by both SineSimulator
// and PistonEngineSimulator. Ownership of engine/vehicle/transmission remains
// subclass-specific; these helpers handle the COMMON physics wiring only.

#ifndef ENGINE_SIM_BRIDGE_SIMULATOR_INIT_HELPERS_H
#define ENGINE_SIM_BRIDGE_SIMULATOR_INIT_HELPERS_H

#include <cstddef>

// Forward declarations (avoid pulling full engine-sim headers here)
class Engine;
class Transmission;
class Vehicle;
class Dynamometer;
class StarterMotor;
class Simulator;

namespace atg_scs {
    class RigidBodySystem;
    struct RigidBody;
}

namespace SimulatorInitHelpers {

/// Physics wiring parameters — groups the common constraint graph wiring arguments.
struct PhysicsWiringParams {
    Engine* engine = nullptr;
    Transmission* transmission = nullptr;
    Vehicle* vehicle = nullptr;
    atg_scs::RigidBody* vehicleMass = nullptr;
    atg_scs::RigidBodySystem* system = nullptr;
    Dynamometer* dyno = nullptr;
    StarterMotor* starterMotor = nullptr;
    double** outStagingBuffer = nullptr;
    int stagingCount = 0;
};

/// Shared physics wiring: vehicleMass, transmission, vehicle, dyno, starter motor, staging buffer.
/// Called after loadSimulation() — wires the common constraint graph that all Simulator
/// subclasses need regardless of engine type.
void wirePhysics(const PhysicsWiringParams& params);

/// Shared cleanup: system reset/delete, staging buffer delete.
/// Does NOT delete engine/vehicle/transmission — ownership is subclass-specific.
/// The caller is responsible for its own ownership cleanup and calling Simulator::destroy().
///
/// @param system           Ref to m_system pointer (will be nulled)
/// @param stagingBuffer    Ref to staging buffer pointer (will be deleted and nulled)
void cleanupPhysics(
    atg_scs::RigidBodySystem*& system,
    double*& stagingBuffer);

/// Initialize convolution filters with unit impulses to prevent null pointer crashes.
/// Required when loading presets (no WAV impulse response files) because ConvolutionFilter::f()
/// dereferences m_shiftRegister which is nullptr without this initialization.
/// Even with convolution disabled, the unit impulse prevents segfault if re-enabled later.
///
/// @param simulator        The simulator to initialize (must have loaded engine/synthesizer)
void initializeConvolutionFilters(Simulator* simulator);

} // namespace SimulatorInitHelpers

#endif // ENGINE_SIM_BRIDGE_SIMULATOR_INIT_HELPERS_H
