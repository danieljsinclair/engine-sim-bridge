// SimulatorFactory.cpp - Composition root for creating simulator instances
// Creates Simulator subclass, wires mode-specific details, wraps in BridgeSimulator.
// OCP: BridgeSimulator doesn't know what Simulator it has; factory is the only place that does.

#include "simulator/SimulatorFactory.h"
#include "simulator/BridgeSimulator.h"
#include "simulator/SineSimulator.h"
#include "simulator/ScriptLoadHelpers.h"
#include "simulator/engine_sim_bridge.h"
#include "common/ILogging.h"
#include "telemetry/ITelemetryProvider.h"
#include "piston_engine_simulator.h"

#include <memory>
#include <stdexcept>

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
#include "../scripting/include/compiler.h"
#endif

// ============================================================================
// Shared Simulator init — common to all Simulator subclasses
// ============================================================================

static void initSimulator(Simulator* sim, const SimulatorConfig& config) {
    Simulator::Parameters simParams;
    simParams.systemType = Simulator::SystemType::NsvOptimized;
    sim->initialize(simParams);
    sim->setSimulationFrequency(config.simulationFrequency.value_or(EngineSimDefaults::SIMULATION_FREQUENCY));
    sim->setFluidSimulationSteps(EngineSimDefaults::FLUID_SIMULATION_STEPS);
    sim->setTargetSynthesizerLatency(config.synthLatency.value_or(EngineSimDefaults::TARGET_SYNTH_LATENCY));
}

// ============================================================================
// SimulatorFactory Implementation
// ============================================================================

std::unique_ptr<ISimulator> SimulatorFactory::create(
    const SimulatorConfig& config,
    ILogging* logger,
    telemetry::ITelemetryWriter* telemetryWriter)
{
    std::unique_ptr<Simulator> sim;

    switch (config.type) {
        case SimulatorType::SineWave: {
            auto sineSim = std::make_unique<SineSimulator>();
            initSimulator(sineSim.get(), config);
            sineSim->loadSimulation(nullptr, nullptr, nullptr);
            sim = std::move(sineSim);
            break;
        }

        case SimulatorType::PistonEngine: {
            auto pistonSim = std::make_unique<PistonEngineSimulator>();
            initSimulator(pistonSim.get(), config);

            // Compile and load script if path provided
            if (!config.scriptPath.empty()) {
                std::string normalizedPath = ScriptLoadHelpers::normalizeScriptPath(config.scriptPath);
                std::string resolvedAssetPath = ScriptLoadHelpers::resolveAssetBasePath(normalizedPath, config.assetBasePath);

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
                // Lazily create compiler
                auto compiler = std::make_unique<es_script::Compiler>();
                compiler->initialize();

                if (!compiler->compile(normalizedPath.c_str())) {
                    throw std::runtime_error("Failed to compile script: " + normalizedPath);
                }

                es_script::Compiler::Output output = compiler->execute();
                Engine* engine = output.engine;
                Vehicle* vehicle = output.vehicle ? output.vehicle : ScriptLoadHelpers::createDefaultVehicle();
                Transmission* transmission = output.transmission ? output.transmission : ScriptLoadHelpers::createDefaultTransmission();

                if (!engine) {
                    throw std::runtime_error("Script did not create an engine: " + normalizedPath);
                }

                pistonSim->loadSimulation(engine, vehicle, transmission);

                // Load impulse responses
                if (!ScriptLoadHelpers::loadImpulseResponses(pistonSim.get(), engine, resolvedAssetPath, logger)) {
                    throw std::runtime_error("Failed to load impulse responses (asset base: " + resolvedAssetPath + ")");
                }

                compiler->destroy();
#else
                throw std::runtime_error("Script loading not available (Piranha support disabled)");
#endif
            }

            sim = std::move(pistonSim);
            break;
        }

        default:
            throw std::runtime_error("Unknown simulator type in SimulatorFactory::create()");
    }

    auto bridgeSim = std::make_unique<BridgeSimulator>(std::move(sim));

    // Set display name from script path for PistonEngine mode
    if (config.type == SimulatorType::PistonEngine && !config.scriptPath.empty()) {
        bridgeSim->setNameFromScript(config.scriptPath);
    }

    return bridgeSim;
}

SimulatorType SimulatorFactory::getDefaultType() {
    return SimulatorType::PistonEngine;
}
