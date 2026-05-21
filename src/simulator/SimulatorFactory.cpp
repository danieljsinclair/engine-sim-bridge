// SimulatorFactory.cpp - Composition root for creating simulator instances
// Creates Simulator subclass, wires mode-specific details, wraps in BridgeSimulator.
// OCP: BridgeSimulator doesn't know what Simulator it has; factory is the only place that does.

#include "simulator/SimulatorFactory.h"
#include "simulator/BridgeSimulator.h"
#include "simulator/SineSimulator.h"
#include "simulator/ScriptLoadHelpers.h"
#include "simulator/PresetEngineFactory.h"
#include "simulator/SimulatorInitHelpers.h"
#include "simulator/EngineSimTypes.h"
#include "common/ILogging.h"
#include "telemetry/ITelemetryProvider.h"
#include "piston_engine_simulator.h"

#include <memory>
#include <stdexcept>
#include <algorithm>

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
#include "../scripting/include/compiler.h"
#endif

// ============================================================================
// Shared Simulator init — common to all Simulator subclasses
// ============================================================================

static void initSimulator(Simulator* sim, const ISimulatorConfig& config) {
    Simulator::Parameters simParams;
    simParams.systemType = Simulator::SystemType::NsvOptimized;
    sim->initialize(simParams);

    sim->setSimulationFrequency(config.simulationFrequency);
    sim->setFluidSimulationSteps(config.fluidSimulationSteps);
    sim->setTargetSynthesizerLatency(config.targetSynthesizerLatency);
}

static bool endsWith(const std::string& str, const std::string& suffix) {
    if (suffix.size() > str.size()) return false;
    return std::equal(suffix.rbegin(), suffix.rend(), str.rbegin(),
        [](char a, char b) { return tolower(a) == tolower(b); });
}

// ============================================================================
// SimulatorFactory Implementation
// ============================================================================

std::unique_ptr<ISimulator> SimulatorFactory::create(
    SimulatorType type,
    const std::string& scriptPath,
    const std::string& assetBasePath,
    const ISimulatorConfig& config,
    ILogging* logger,
    telemetry::ITelemetryWriter* telemetryWriter)
{
    std::unique_ptr<Simulator> sim;

    switch (type) {
        case SimulatorType::SineWave: {
            auto sineSim = std::make_unique<SineSimulator>();
            initSimulator(sineSim.get(), config);
            sineSim->loadSimulation(nullptr, nullptr, nullptr);
            sim = std::move(sineSim);
            break;
        }

        case SimulatorType::PistonEngine: {
            if (!scriptPath.empty()) {
                // Route by file extension: .json → preset loader, .mr → Piranha compiler
                if (endsWith(scriptPath, ".json")) {
                    // JSON preset — no Piranha dependency
                    PresetLoadResult result = PresetEngineFactory::loadFromFile(scriptPath);
                    if (!result.success()) {
                        throw std::runtime_error("Failed to load preset: " + scriptPath + " — " + result.error);
                    }

                    auto pistonSim = std::make_unique<PistonEngineSimulator>();
                    initSimulator(pistonSim.get(), config);
                    pistonSim->loadSimulation(result.engine, result.vehicle, result.transmission);

                    // Apply transmission state from preset JSON.
                    // Must happen after loadSimulation() because changeGear()
                    // accesses m_vehicle->getMass(), which is set by addToSystem()
                    // inside loadSimulation().
                    if (result.transmission && result.initialGear >= 0) {
                        result.transmission->changeGear(result.initialGear);
                    }

                    // Load impulse responses from WAV files referenced in the JSON
                    if (!ScriptLoadHelpers::loadImpulseResponses(
                            pistonSim.get(), result.engine,
                            ScriptLoadHelpers::resolveAssetBasePath(
                                ScriptLoadHelpers::normalizeScriptPath(scriptPath), assetBasePath),
                            logger)) {
                        pistonSim->destroy();
                        throw std::runtime_error("Failed to load impulse responses for preset: " + scriptPath);
                    }

                    sim = std::move(pistonSim);
                } else {
                    // .mr script — Piranha compilation (desktop only)
                    std::string normalizedPath = ScriptLoadHelpers::normalizeScriptPath(scriptPath);
                    std::string resolvedAssetPath = ScriptLoadHelpers::resolveAssetBasePath(normalizedPath, assetBasePath);

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
                    auto compiler = std::make_unique<es_script::Compiler>();
                    compiler->initialize();

                    auto cleanup = [&compiler]() { compiler->destroy(); };

                    try {
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

                        auto pistonSim = std::make_unique<PistonEngineSimulator>();
                        initSimulator(pistonSim.get(), config);
                        pistonSim->loadSimulation(engine, vehicle, transmission);

                        if (!ScriptLoadHelpers::loadImpulseResponses(pistonSim.get(), engine, resolvedAssetPath, logger)) {
                            pistonSim->destroy();
                            throw std::runtime_error("Failed to load impulse responses (asset base: " + resolvedAssetPath + ")");
                        }

                        sim = std::move(pistonSim);
                    } catch (...) {
                        cleanup();
                        throw;
                    }
                    cleanup();
#else
                    throw std::runtime_error("Script loading not available (Piranha support disabled)");
#endif
                }
            } else {
                auto pistonSim = std::make_unique<PistonEngineSimulator>();
                initSimulator(pistonSim.get(), config);
                sim = std::move(pistonSim);
            }
            break;
        }

        default:
            throw std::runtime_error("Unknown simulator type in SimulatorFactory::create()");
    }

    auto bridgeSim = std::make_unique<BridgeSimulator>(std::move(sim));

    if (type == SimulatorType::PistonEngine && !scriptPath.empty()) {
        bridgeSim->setNameFromScript(scriptPath);
    }

    return bridgeSim;
}

SimulatorType SimulatorFactory::getDefaultType() {
    return SimulatorType::PistonEngine;
}
