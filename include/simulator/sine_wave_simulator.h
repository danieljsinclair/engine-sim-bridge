// SineWaveSimulator.h - Bridge-provided test simulator for sine wave output
#ifndef ENGINE_SIM_BRIDGE_SINE_WAVE_SIMULATOR_H
#define ENGINE_SIM_BRIDGE_SINE_WAVE_SIMULATOR_H

#include "simulator.h"
#include "simulator/SimulatorBase.h"
#include "engine.h"
#include "vehicle.h"
#include "transmission.h"
#include "throttle.h"
#include <cmath>
#include <memory>
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
    void update(double deltaTime) override;
    EngineSimStats getStats() const override;
    void setThrottle(double position) override;
    void setIgnition(bool on) override;
    void setStarterMotor(bool on) override;
    bool renderOnDemand(float* buffer, int32_t frames, int32_t* written) override;
    bool readAudioBuffer(float* buffer, int32_t frames, int32_t* read) override;
    bool start() override;
    void stop() override;

    // Simulator overrides (required by Simulator base class)
    virtual void initialize(const Parameters &params) override;
    virtual void loadSimulation(Engine* engine, Vehicle* vehicle, Transmission* transmission) override;

protected:
    virtual void simulateStep_() override;
    virtual void writeToSynthesizer() override;

private:
    Engine* m_dummyEngine;
    Vehicle* m_dummyVehicle;
    Transmission* m_dummyTransmission;
    atg_scs::RigidBody m_vehicleMass;  // Required by Vehicle/Transmission addToSystem()

    double m_phase;       // Sine wave phase accumulator

    // Stored from EngineSimConfig for renderOnDemand simulation stepping
    int sampleRate_ = 48000;
    int simulationFrequency_ = 10000;

    // ISimulator state
    bool created_;
    std::string lastError_;

    static constexpr double TWO_PI = 2.0 * M_PI;
};

#endif // ENGINE_SIM_BRIDGE_SINE_WAVE_SIMULATOR_H
