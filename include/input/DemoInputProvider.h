#ifndef DEMO_INPUT_PROVIDER_H
#define DEMO_INPUT_PROVIDER_H

#include "io/IInputProvider.h"
#include "simulator/EngineSimTypes.h"
#include "io/UpstreamSignal.h"
#include "input/IThrottleSource.h"
#include "input/KeyboardDemoThrottleSource.h"
#include "input/DemoVehiclePhysics.h"
#include "input/VirtualIceInputProvider.h"
#include "twin/IceVehicleProfile.h"

#include <memory>
#include <string>

namespace input {

class DemoInputProvider : public IInputProvider {
public:
    DemoInputProvider(
        std::unique_ptr<IThrottleSource> throttleSource,
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

private:
    twin::IceVehicleProfile profile_;
    std::unique_ptr<IThrottleSource> throttleSource_;
    VirtualIceInputProvider twinProvider_;
    DemoVehiclePhysics physics_;
    bool initialized_ = false;
    std::string lastError_;
    double roadSpeedKmh_ = 0.0;
    int currentGear_ = 0;
    int lastForwardedSelector_ = 0;
};

}

#endif
