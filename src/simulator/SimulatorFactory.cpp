// SimulatorFactory.cpp - Factory implementation for creating simulator instances
// Implements Factory pattern for DI and OCP compliance
// Phase 2: Core logic to create SineWaveSimulator and BridgeSimulator

#include "simulator/SimulatorFactory.h"
#include "simulator/sine_wave_simulator.h"
#include "simulator/BridgeSimulator.h"
#include "common/ILogging.h"
#include <memory>
#include <stdexcept>

// ============================================================================
// SimulatorFactory Implementation
// ============================================================================

std::unique_ptr<ISimulator> SimulatorFactory::create(
    SimulatorType type,
    ILogging* logger)
{
    switch (type) {
        case SimulatorType::SineWave:
            // SineWaveSimulator implements ISimulator directly
            // It accepts an optional logger parameter in its constructor
            return std::make_unique<SineWaveSimulator>(logger);

        case SimulatorType::PistonEngine:
            // Create BridgeSimulator that wraps PistonEngineSimulator with composition
            return std::make_unique<BridgeSimulator>();

        default:
            // Should never happen with enum class, but defensive programming
            throw std::runtime_error("Unknown simulator type in SimulatorFactory::create()");
    }
}

SimulatorType SimulatorFactory::getDefaultType() {
    // Default to PistonEngine for production use
    return SimulatorType::PistonEngine;
}
