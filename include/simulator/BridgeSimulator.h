// BridgeSimulator.h - Production implementation of ISimulator
// Uses composition to wrap PistonEngineSimulator directly (no C API).
// DIP: Consumers depend on ISimulator, not on raw bridge types.
// Phase B: Refactored to use composition instead of inheritance

#ifndef BRIDGE_SIMULATOR_H
#define BRIDGE_SIMULATOR_H

#include "simulator/SimulatorBase.h"
#include "simulator/engine_sim_bridge.h"
#include "piston_engine_simulator.h"

// Forward declarations for script loading (only used when Piranha is enabled)
#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
namespace es_script { class Compiler; }
#endif

class Engine;
class Vehicle;
class Transmission;

#include <memory>
#include <string>

class BridgeSimulator : public SimulatorBase {
public:
    BridgeSimulator();
    ~BridgeSimulator() override;

    // ISimulator lifecycle
    bool create(const EngineSimConfig& config, ILogging* logger, telemetry::ITelemetryWriter* telemetryWriter) override;
    bool loadScript(const std::string& path, const std::string& assetBase) override;
    void destroy() override;
    std::string getLastError() const override;

    // ISimulator simulation
    void update(double deltaTime) override;
    EngineSimStats getStats() const override;

    // ISimulator control inputs
    void setThrottle(double position) override;
    void setIgnition(bool on) override;
    void setStarterMotor(bool on) override;

    // ISimulator audio production
    bool renderOnDemand(float* buffer, int32_t frames, int32_t* written) override;
    bool readAudioBuffer(float* buffer, int32_t frames, int32_t* read) override;
    bool start() override;
    void stop() override;

private:
    std::unique_ptr<PistonEngineSimulator> m_simulator;
    std::string m_lastError;
    bool m_created = false;

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
    es_script::Compiler* m_compiler = nullptr;
#endif
    // Engine objects created by script loading - cleaned up via Simulator::destroy()
    Engine* m_engine = nullptr;
    Vehicle* m_vehicle = nullptr;
    Transmission* m_transmission = nullptr;
};

#endif // BRIDGE_SIMULATOR_H
