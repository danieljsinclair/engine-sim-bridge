#ifndef VIRTUAL_ICE_INPUT_PROVIDER_H
#define VIRTUAL_ICE_INPUT_PROVIDER_H

#include "io/IInputProvider.h"
#include "io/UpstreamSignal.h"
#include "twin/IceVehicleProfile.h"
#include "twin/VirtualIceTwin.h"

#include <memory>
#include <string>

namespace input {

class VirtualIceInputProvider : public IInputProvider {
public:
    explicit VirtualIceInputProvider(const twin::IceVehicleProfile& profile);
    ~VirtualIceInputProvider() override;

    // IInputProvider lifecycle
    bool Initialize() override;
    void Shutdown() override;
    bool IsConnected() const override;

    // IInputProvider input queries
    EngineInput OnUpdateSimulation(double dt) override;
    std::string GetProviderName() const override;
    std::string GetLastError() const override;

    // Set the upstream signal source (for testing or manual injection)
    void setUpstreamSignal(const UpstreamSignal& signal);

private:
    const twin::IceVehicleProfile& profile_;
    std::unique_ptr<twin::VirtualIceTwin> twin_;
    std::string lastError_;
    bool isInitialized_;

    UpstreamSignal currentSignal_;
};

}

#endif
