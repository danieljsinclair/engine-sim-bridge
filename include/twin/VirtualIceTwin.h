#ifndef VIRTUAL_ICE_TWIN_H
#define VIRTUAL_ICE_TWIN_H

#include <twin/IVehicleTwin.h>
#include <twin/IceVehicleProfile.h>
#include <twin/AutomaticGearbox.h>
#include <twin/ThrottleSmoother.h>
#include <twin/IGearboxLogger.h>
#include <io/UpstreamSignal.h>
#include <simulator/GearConventions.h>
#include <memory>
#include <vector>

namespace twin {

class VirtualIceTwin {
public:
    explicit VirtualIceTwin(IceVehicleProfile profile);

    // Reconfigure the gearbox to match the actual engine preset's ratios.
    // Auto-generates a shift table from the ratios + redline (same logic as
    // ReplayTelemetryProvider::reconfigureProfile). Call after the simulator
    // loads so the gearbox matches whatever engine preset is active.
    void reconfigureProfile(const std::vector<double>& gearRatios,
                             double diffRatio, double tireRadiusM);

    TwinOutput update(double dt, const input::UpstreamSignal& signal);

    TwinState getState() const { return state_; }

    void setEngineRpmFeedback(double rpm) { engineRpmFeedback_ = rpm; }
    void setVehicleSpeedFeedback(double kmh) { vehicleSpeedFeedbackKmh_ = kmh; }
    // Drivetrain torque feedback (Nm) feeds the torque-driven shift logic.
    void setDrivetrainTorqueFeedback(double nm) { drivetrainTorqueNm_ = nm; }
    void setGearboxLogger(IGearboxLogger* logger);

    void setGearSelector(bridge::GearSelector s) { selector_ = s; }
    bridge::GearSelector getGearSelector() const { return selector_; }

    void setIgnition(bool on) { ignitionOn_ = on; }
    bool getIgnition() const { return ignitionOn_; }

    int getCurrentGear() const { return gearbox_->getCurrentGear(); }

    double getSmoothedThrottle() const { return throttleSmoother_.getCurrentValue(); }

private:
    IceVehicleProfile profile_;  // owned (was const ref — caused dangling + no reconfigure)
    std::unique_ptr<AutomaticGearbox> gearbox_;
    ThrottleSmoother throttleSmoother_;

    TwinState state_ = TwinState::OFF;
    double timeWithoutValidTelemetryS_ = 0.0;
    double shiftTimerS_ = 0.0;
    double crankingTimerS_ = 0.0;
    double engineRpmFeedback_ = 0.0;
    double vehicleSpeedFeedbackKmh_ = 0.0;
    double drivetrainTorqueNm_ = 0.0;
    double clutchPressure_ = 1.0;
    bridge::GearSelector selector_ = bridge::GearSelector::NEUTRAL;
    bool ignitionOn_ = true;

    void updateShiftExecution(double dt);
};

}

#endif
