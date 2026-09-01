#ifndef I_ENGINE_ACTUATOR_H
#define I_ENGINE_ACTUATOR_H

// IEngineActuator.h - minimal seam so the vehicle-driven start/stop decision
// layer can drive the engine without depending on the full ISimulator /
// ICombustionEngine surface. BridgeSimulator already implements both methods
// and adapts to this interface at the composition root.
//
// SRP: VehicleStartController depends only on this interface, never on a
// concrete simulator or on the manual twin path.

namespace input {

class IEngineActuator {
public:
    virtual ~IEngineActuator() = default;

    // Turn the combustion ignition (spark/fuel) on/off.
    virtual void setIgnition(bool on) = 0;

    // Engage/disengage the starter motor.
    virtual void setStarterMotor(bool on) = 0;
};

} // namespace input

#endif // I_ENGINE_ACTUATOR_H
