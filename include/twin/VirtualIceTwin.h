#ifndef VIRTUAL_ICE_TWIN_H
#define VIRTUAL_ICE_TWIN_H

#include <twin/IceVehicleProfile.h>
#include <twin/AutomaticGearbox.h>
#include <twin/ThrottleSmoother.h>
#include <twin/TwinOutput.h>
#include <io/UpstreamSignal.h>

namespace twin {

enum class TwinState {
    OFF,
    CRANKING,
    IDLE,
    RUNNING,
    SHIFTING
};

class VirtualIceTwin {
public:
    explicit VirtualIceTwin(const IceVehicleProfile& profile);

    TwinOutput update(double dt, const input::UpstreamSignal& signal);

    TwinState getState() const { return state_; }

    int getCurrentGear() const { return gearbox_.getCurrentGear(); }

    double getSmoothedThrottle() const { return throttleSmoother_.getCurrentValue(); }

private:
    const IceVehicleProfile& profile_;
    AutomaticGearbox gearbox_;
    ThrottleSmoother throttleSmoother_;

    TwinState state_ = TwinState::OFF;
    double timeWithoutValidTelemetryS_ = 0.0;
    double shiftTimerS_ = 0.0;
    double clutchPressure_ = 1.0;

    void updateShiftExecution(double dt);
};

}

#endif
