#ifndef VIRTUAL_ICE_TWIN_H
#define VIRTUAL_ICE_TWIN_H

#include <twin/IVehicleTwin.h>
#include <twin/IceVehicleProfile.h>
#include <twin/AutomaticGearbox.h>
#include <twin/ThrottleSmoother.h>
#include <io/UpstreamSignal.h>
#include <simulator/GearConventions.h>

namespace twin {

class VirtualIceTwin {
public:
    explicit VirtualIceTwin(const IceVehicleProfile& profile);

    TwinOutput update(double dt, const input::UpstreamSignal& signal);

    TwinState getState() const { return state_; }

    void setEngineRpmFeedback(double rpm) { engineRpmFeedback_ = rpm; }
    void setVehicleSpeedFeedback(double kmh) { vehicleSpeedFeedbackKmh_ = kmh; }

    void setGearSelector(bridge::GearSelector s) { selector_ = s; }
    bridge::GearSelector getGearSelector() const { return selector_; }

    void setIgnition(bool on) { ignitionOn_ = on; }
    bool getIgnition() const { return ignitionOn_; }

    int getCurrentGear() const { return gearbox_.getCurrentGear(); }

    double getSmoothedThrottle() const { return throttleSmoother_.getCurrentValue(); }

private:
    const IceVehicleProfile& profile_;
    AutomaticGearbox gearbox_;
    ThrottleSmoother throttleSmoother_;

    TwinState state_ = TwinState::OFF;
    double timeWithoutValidTelemetryS_ = 0.0;
    double shiftTimerS_ = 0.0;
    double crankingTimerS_ = 0.0;
    double engineRpmFeedback_ = 0.0;
    double vehicleSpeedFeedbackKmh_ = 0.0;
    double clutchPressure_ = 1.0;
    bridge::GearSelector selector_ = bridge::GearSelector::NEUTRAL;
    bool ignitionOn_ = true;

    void updateShiftExecution(double dt);
};

}

#endif
