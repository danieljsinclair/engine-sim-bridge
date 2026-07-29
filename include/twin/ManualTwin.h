#ifndef MANUAL_TWIN_H
#define MANUAL_TWIN_H

#include <twin/IVehicleTwin.h>
#include <simulator/GearConventions.h>

namespace twin {

class ManualTwin : public IVehicleTwin {
public:
    ManualTwin() = default;

    TwinOutput update(double dt, const TwinFeedback& feedback) override;
    TwinState getState() const override;

    void setThrottle(double throttle);
    int setGear(int gear);
    void requestGearUp();
    void requestGearDown();
    void setIgnition(bool on);
    void setStarterMotor(bool on);

    int getCurrentGear() const { return currentGear_; }

private:
    TwinState state_ = TwinState::OFF;
    double timeWithoutValidFeedbackS_ = 0.0;

    double inputThrottle_ = 0.0;
    int inputGear_ = static_cast<int>(bridge::BridgeGear::NEUTRAL);
    int currentGear_ = static_cast<int>(bridge::BridgeGear::NEUTRAL);
    bool gearUpRequested_ = false;
    bool gearDownRequested_ = false;
    bool ignitionRequested_ = false;
    bool starterRequested_ = false;

    static constexpr double IDLE_RPM_THRESHOLD = 400.0;
    static constexpr double INVALID_FEEDBACK_TIMEOUT_S = 5.0;
};

} // namespace twin
#endif
