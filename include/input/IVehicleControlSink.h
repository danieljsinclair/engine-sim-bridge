#ifndef I_VEHICLE_CONTROL_SINK_H
#define I_VEHICLE_CONTROL_SINK_H

// IVehicleControlSink.h - the seam through which SimulationLoop's vehicle
// start/stop decision layer (VehicleStartController) commands ignition on
// input providers that own a VirtualIceTwin.
//
// Why it exists: the twin gates ALL of its processing (throttle smoothing,
// gearbox, cranking lifecycle) on its ignitionOn_ flag, and that flag now
// defaults OFF so a twin with no ignition command can never self-start (gap 2:
// UpLeckHill ran the engine from t=0.44s, before any driver input). On live
// paths the CONTROLLER owns every start, so its ignition level must reach the
// twin each frame — through this seam, discovered by SimulationLoop via
// dynamic_cast on the injected IInputProvider.
//
// Providers without a twin (keyboard, replay) do not implement it and are
// untouched. Providers that own their own ignition (demo keyboard, manual
// twin) likewise do not implement it — their authority is deliberately kept.
//
// SRP: the loop depends only on this interface, never on a concrete provider.

namespace input {

class IVehicleControlSink {
public:
    virtual ~IVehicleControlSink() = default;

    // Ignition LEVEL (not a pulse): true while the vehicle start/stop decision
    // says the engine should run. Implementation must be callable every frame
    // from the simulation thread.
    virtual void setIgnition(bool on) = 0;
};

} // namespace input

#endif // I_VEHICLE_CONTROL_SINK_H
