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

private:
    // SimulatorBase pure virtuals
    Simulator* getSimulator() override { return m_simulator.get(); }
    const Simulator* getSimulator() const override { return m_simulator.get(); }
    bool isReady() const override { return m_created && m_simulator != nullptr; }

    // Compile a Piranha script and return engine/vehicle/transmission.
    // Returns nullptr for engine on failure (sets m_lastError).
    bool compileScript(const std::string& scriptPath,
                       Engine** outEngine, Vehicle** outVehicle, Transmission** outTransmission);

    std::unique_ptr<PistonEngineSimulator> m_simulator;
    std::string m_lastError;
    bool m_created = false;

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
    es_script::Compiler* m_compiler = nullptr;
#endif
};

#endif // BRIDGE_SIMULATOR_H
