#ifndef IVEHICLE_TWIN_H
#define IVEHICLE_TWIN_H

#include <twin/TwinOutput.h>

namespace twin {

enum class TwinState {
    OFF,
    CRANKING,
    IDLE,
    RUNNING,
    SHIFTING
};

// RPM feedback from the simulator — the twin uses this to make lifecycle decisions
struct TwinFeedback {
    double engineRpm = 0.0;
    double vehicleSpeedKmh = 0.0;
    bool isValid = false;
};

// Unified interface for all vehicle twin implementations.
// ManualTwin (keyboard), VirtualIceTwin (physics demo), future OBDTwin (live vehicle)
class IVehicleTwin {
public:
    virtual ~IVehicleTwin() = default;
    virtual TwinOutput update(double dt, const TwinFeedback& feedback) = 0;
    virtual TwinState getState() const = 0;
};

} // namespace twin
#endif
