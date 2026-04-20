// SimulatorFactory.h - Factory for creating simulator instances
// Implements Factory pattern for DI and OCP compliance
// Phase 1: Creates appropriate ISimulator* based on SimulatorType enum

#ifndef SIMULATOR_FACTORY_H
#define SIMULATOR_FACTORY_H

#include "simulator/ISimulator.h"
#include <memory>

class ILogging;

// ============================================================================
// SimulatorType - Enum for factory creation
// ============================================================================

enum class SimulatorType {
    SineWave,    // Simple sine wave test simulator
    PistonEngine  // Full physics engine simulator
};

// ============================================================================
// SimulatorFactory - Factory for creating simulator instances
// ============================================================================

class SimulatorFactory {
public:
    /**
     * Creates a simulator instance of the specified type.
     *
     * @param type The type of simulator to create
     * @param logger Optional logging interface (can be nullptr)
     * @return Unique pointer to ISimulator instance
     *
     * Factory is responsible for lifetime management of the created simulator.
     * Caller should use std::move() to transfer ownership.
     */
    static std::unique_ptr<ISimulator> create(
        SimulatorType type,
        ILogging* logger = nullptr);

    /**
     * Gets the default simulator type for the current platform.
     * Currently defaults to PistonEngine for production use.
     *
     * @return Default simulator type
     */
    static SimulatorType getDefaultType();

private:
    // Factory pattern - private constructor to prevent instantiation
    SimulatorFactory() = delete;
    ~SimulatorFactory() = delete;
};

#endif // SIMULATOR_FACTORY_H
