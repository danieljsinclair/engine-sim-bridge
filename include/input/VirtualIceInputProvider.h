#ifndef VIRTUAL_ICE_INPUT_PROVIDER_H
#define VIRTUAL_ICE_INPUT_PROVIDER_H

#include "io/IInputProvider.h"
#include "simulator/EngineSimTypes.h"
#include "io/UpstreamSignal.h"
#include "twin/IceVehicleProfile.h"
#include "twin/VirtualIceTwin.h"
#include "twin/IGearboxLogger.h"

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

    // Forward gear selector changes to the twin
    void setGearSelector(int selector);

    // Forward ignition state to the twin
    void setIgnition(bool on);

    // Forward simulator RPM feedback to the twin for cranking transition
    void provideFeedback(const EngineSimStats& stats) override;

    // Enable gearbox diagnostic logging
    void setGearboxLogger(twin::IGearboxLogger* logger);

private:
    void doShutdown();
    const twin::IceVehicleProfile& profile_;
    std::unique_ptr<twin::VirtualIceTwin> twin_;
    std::string lastError_;
    bool isInitialized_;
    twin::IGearboxLogger* pendingLogger_ = nullptr;

    UpstreamSignal currentSignal_;
};

}

#endif
