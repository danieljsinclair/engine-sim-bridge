#ifndef MANUAL_TWIN_PROVIDER_H
#define MANUAL_TWIN_PROVIDER_H

#include "io/IInputProvider.h"
#include "input/IThrottleSource.h"
#include "twin/ManualTwin.h"

#include <string>

class ISimulator;

namespace input {

class ManualTwinProvider : public IInputProvider {
public:
    ManualTwinProvider(std::unique_ptr<IThrottleSource> throttleSource, ISimulator& simulator);
    ~ManualTwinProvider() override;

    bool Initialize() override;
    void Shutdown() override;
    bool IsConnected() const override;
    EngineInput OnUpdateSimulation(double dt) override;
    std::string GetProviderName() const override;
    std::string GetLastError() const override;

    void setGearUpRequested(bool requested);
    void setGearDownRequested(bool requested);
    void setIgnitionRequested(bool requested);
    void setStarterRequested(bool requested);

private:
    void doShutdown();
    std::unique_ptr<IThrottleSource> throttleSource_;
    ISimulator& simulator_;
    twin::ManualTwin twin_;
    bool initialized_ = false;
    std::string lastError_;
    bool gearUpRequested_ = false;
    bool gearDownRequested_ = false;
    bool ignitionRequested_ = false;
    bool starterRequested_ = false;
};

} // namespace input
#endif
