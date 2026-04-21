// SineWaveSimulator.h - Bridge-provided test simulator for sine wave output
#ifndef ENGINE_SIM_BRIDGE_SINE_WAVE_SIMULATOR_H
#define ENGINE_SIM_BRIDGE_SINE_WAVE_SIMULATOR_H

#include "simulator.h"
#include "simulator/SimulatorBase.h"
#include <string>

/**
 * SineWaveSimulator - Test simulator with dummy engine for consistent output.
 *
 * Derives from Simulator AND SimulatorBase.
 * Creates dummy Engine/Vehicle/Transmission objects and uses base class methods.
 *
 * Key differences from PistonEngineSimulator:
 * - No physics simulation (crankshaft, pistons, valves, etc.)
 * - Throttle directly controls RPM with no lag
 * - Predictable sine wave audio output for testing
 */
class SineWaveSimulator : public Simulator, public SimulatorBase {
public:
    explicit SineWaveSimulator(ILogging* logger = nullptr);
    virtual ~SineWaveSimulator();

    // ISimulator interface methods
    bool create(const EngineSimConfig& config, ILogging* logger, telemetry::ITelemetryWriter* telemetryWriter) override;
    bool loadScript(const std::string& path, const std::string& assetBase) override;
    void destroy() override;
    std::string getLastError() const override;

    // Simulator overrides (required by Simulator base class)
    virtual void initialize(const Parameters &params) override;
    virtual void loadSimulation(Engine* engine, Vehicle* vehicle, Transmission* transmission) override;

protected:
    virtual void simulateStep_() override;
    virtual void writeToSynthesizer() override;

    // SimulatorBase pure virtuals
    Simulator* getSimulator() override { return this; }
    const Simulator* getSimulator() const override { return this; }
    bool isReady() const override { return created_; }

private:
    double m_phase;       // Sine wave phase accumulator
    double m_sineValue;   // Current sine sample — populated by simulateStep_()
    double* m_exhaustFlowStagingBuffer = nullptr;  // Mirrors PistonEngineSimulator staging buffer

    // ISimulator state
    bool created_;
    std::string lastError_;

    static constexpr double TWO_PI = 2.0 * M_PI;
};

#endif // ENGINE_SIM_BRIDGE_SINE_WAVE_SIMULATOR_H
