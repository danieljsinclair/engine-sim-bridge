#ifndef I_START_CONTROLLER_H
#define I_START_CONTROLLER_H

// IStartController.h - composable start/ignition primitives interface.
//
// The SINGLE seam every frontend calls through (owner spec 2026-09-02). The
// concrete implementation is VehicleStartController; this interface lets input
// providers (keyboard, demo) and the CLI --start flag call the shared state
// machine without depending on the concrete class. Three composable primitives:
//
//   requestIgnition(bool on) - direct ignition control (CLI/iOS separate
//                              ignition: elongate cranking, crank without
//                              ignition). The McLaren mod's ignition-after-
//                              starter sequencing.
//   requestStarter()          - engage the starter motor WITHOUT firing
//                              ignition (starter-then-ignition sequencing).
//   requestCombinedStart()    - combined start (CSV path by design, --start
//                              flag): fire starter+ignition together.
//
// VehicleStartController (input/VehicleStartController.h) is the canonical
// implementation. SimulationLoop owns the concrete instance and exposes it to
// input providers via this interface in SessionDependencies.

namespace input {

class IStartController {
public:
    virtual ~IStartController() = default;
    virtual void requestIgnition(bool on) = 0;
    virtual void requestStarter() = 0;
    virtual void requestCombinedStart() = 0;
};

} // namespace input

#endif // I_START_CONTROLLER_H
