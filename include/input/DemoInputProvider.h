#ifndef DEMO_INPUT_PROVIDER_H
#define DEMO_INPUT_PROVIDER_H

#include "io/IInputProvider.h"
#include "input/IDemoControls.h"
#include "twin/IGearboxLogger.h"
#include "simulator/EngineSimTypes.h"
#include "io/UpstreamSignal.h"
#include "input/IThrottleSource.h"
#include "input/GearSelectorInput.h"
#include "input/IgnitionInput.h"
#include "input/BrakeInput.h"
#include "input/DemoVehiclePhysics.h"
#include "input/VirtualIceInputProvider.h"
#include "twin/IceVehicleProfile.h"

#include <memory>
#include <string>

namespace input {

class DemoThrottleSource;

class DemoInputProvider : public IInputProvider, public IDemoControls {
public:
    DemoInputProvider(
        std::unique_ptr<IThrottleSource> throttleSource,
        std::unique_ptr<GearSelectorInput> gearSelector,
        std::unique_ptr<IgnitionInput> ignition,
        const twin::IceVehicleProfile& profile,
        const DemoVehiclePhysicsConfig& physicsConfig = {}
    );

    ~DemoInputProvider() override;

    bool Initialize() override;
    void Shutdown() override;
    bool IsConnected() const override;
    EngineInput OnUpdateSimulation(double dt) override;
    std::string GetProviderName() const override;
    std::string GetLastError() const override;

    double getDemoRoadSpeedKmh() const;
    int getDemoGear() const;

    void provideFeedback(const EngineSimStats& stats) override;

    // IDemoControls — called by consumer (CLI/OBD/iOS) to control the demo
    void setThrottle(double level) override;
    void shiftUp() override;
    void shiftDown() override;
    int getGearSelectorState() const override;
    void setIgnition(bool on) override;
    void toggleIgnition() override;
    bool isIgnitionOn() const override;
    void requestExit() override;
    void setBrake(double level) override;

    // Gearbox diagnostic logging
    void setGearboxLogger(twin::IGearboxLogger* logger);

private:
    const twin::IceVehicleProfile profile_;
    std::unique_ptr<IThrottleSource> throttleSource_;
    std::unique_ptr<GearSelectorInput> gearSelector_;
    std::unique_ptr<IgnitionInput> ignition_;
    VirtualIceInputProvider twinProvider_;
    DemoVehiclePhysics physics_;
    BrakeInput brakeInput_;
    bool initialized_ = false;
    std::string lastError_;
    double roadSpeedKmh_ = 0.0;
    int currentGear_ = 0;
    int lastForwardedSelector_ = 0;
    class DemoThrottleSource* demoThrottle_;  // non-owning, set from throttleSource_.get()
};

}

#endif
