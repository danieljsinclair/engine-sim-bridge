// SineWaveSimulator.h - Bridge-provided test simulator for sine wave output
#ifndef ENGINE_SIM_BRIDGE_SINE_WAVE_SIMULATOR_H
#define ENGINE_SIM_BRIDGE_SINE_WAVE_SIMULATOR_H

#include "simulator.h"
#include "ILogging.h"
#include "engine.h"
#include "vehicle.h"
#include "transmission.h"
#include "throttle.h"
#include <cmath>
#include <memory>

/**
 * SineWaveSimulator - Test simulator with dummy engine for consistent output.
 *
 * Derives from Simulator (same as PistonEngineSimulator) and implements the
 * same DRY interface. Creates dummy Engine/Vehicle/Transmission objects
 * and uses the base class loadSimulation() method.
 *
 * Key differences from PistonEngineSimulator:
 * - No physics simulation (crankshaft, pistons, valves, etc.)
 * - Throttle directly controls RPM with no lag
 * - Predictable sine wave audio output for testing
 *
 * From the bridge's perspective, this is identical to PistonEngineSimulator:
 * - call loadSimulation()
 * - call engine->setSpeedControl()
 * - call startFrame()/simulateStep()/endFrame()
 *
 * This proves buffer management, threading, and control flow work correctly
 * without requiring complex physics simulation.
 */
class SineWaveSimulator : public Simulator {
public:
    // Constructor: inject logger (uses ConsoleLogger if null)
    explicit SineWaveSimulator(ILogging* logger = nullptr);
    virtual ~SineWaveSimulator();

    /**
     * Initialize with dummy engine/vehicle/transmission.
     * Creates minimal objects and calls parent loadSimulation().
     */
    virtual void initialize(const Parameters &params) override;

    /**
     * Override loadSimulation to connect physics constraints (dyno, starter).
     */
    virtual void loadSimulation(Engine* engine, Vehicle* vehicle, Transmission* transmission) override;

    /**
     * Clean up dummy objects.
     */
    virtual void destroy() override;

protected:
    /**
     * Override physics simulation - just update RPM from throttle.
     */
    virtual void simulateStep_() override;

    /**
     * Override synthesizer writing - generate simple sine wave.
     */
    virtual void writeToSynthesizer() override;

private:
    Engine* m_dummyEngine;
    Vehicle* m_dummyVehicle;
    Transmission* m_dummyTransmission;
    atg_scs::RigidBody m_vehicleMass;  // Required by Vehicle/Transmission addToSystem()

    double m_phase;       // Sine wave phase accumulator

    // Logging: owns ConsoleLogger by default, or uses injected logger
    std::unique_ptr<ConsoleLogger> defaultLogger_;
    ILogging* logger_;    // Non-null, points to defaultLogger_ or injected logger

    static constexpr double TWO_PI = 2.0 * M_PI;
};

#endif // ENGINE_SIM_BRIDGE_SINE_WAVE_SIMULATOR_H
