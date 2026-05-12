// SineSimulator.h - Pure Simulator subclass for sine wave test output
// No ISimulator, no diamond inheritance. Lifecycle mirrors PistonEngineSimulator:
// factory calls loadSimulation() after shared init, same virtual path for both types.
// Created by SimulatorFactory for sine mode, wrapped in BridgeSimulator.

#ifndef ENGINE_SIM_BRIDGE_SINE_SIMULATOR_H
#define ENGINE_SIM_BRIDGE_SINE_SIMULATOR_H

#include "simulator.h"
#include "types.h"

class SineSimulator : public Simulator {
public:
    SineSimulator() = default;
    ~SineSimulator() override;

    void destroy() override;

    /** Overrides Simulator::loadSimulation() — mirrors PistonEngineSimulator lifecycle.
     *  Factory calls this after initSimulator(). Creates sine-specific engine/vehicle/transmission,
     *  wires physics, configures synthesizer for pure sine output. */
    void loadSimulation(Engine* engine, Vehicle* vehicle, Transmission* transmission) override;

protected:
    void simulateStep_() override;
    void writeToSynthesizer() override;

private:
    real_t m_phase = 0.0f;
    real_t m_sineValue = 0.0f;
    real_t* m_exhaustFlowStagingBuffer = nullptr;

    static constexpr real_t TWO_PI = real_t(2.0) * constants::pi;
};

#endif // ENGINE_SIM_BRIDGE_SINE_SIMULATOR_H
