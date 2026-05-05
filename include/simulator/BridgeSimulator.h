// BridgeSimulator.h - Universal ISimulator implementation
// Composes an injected Simulator subclass (PistonEngineSimulator or SineSimulator).
// OCP: BridgeSimulator doesn't know or care which Simulator it has.
// Factory is the composition root that wires mode-specific details.

#ifndef BRIDGE_SIMULATOR_H
#define BRIDGE_SIMULATOR_H

#include "simulator/ISimulator.h"
#include "simulator/EngineSimTypes.h"
#include "common/ILogging.h"
#include "telemetry/ITelemetryProvider.h"
#include "telemetry/NullTelemetryWriter.h"
#include "engine-sim/include/simulator.h"

#include <memory>
#include <string>
#include <vector>

class BridgeSimulator : public ISimulator {
public:
    // Constructor takes an already-initialized Simulator subclass.
    // The factory is responsible for creating and wiring the Simulator.
    explicit BridgeSimulator(std::unique_ptr<Simulator> simulator);
    ~BridgeSimulator() override;

    // ISimulator lifecycle
    bool create(const ISimulatorConfig& config, ILogging* logger, telemetry::ITelemetryWriter* telemetryWriter) override;
    void destroy() override;
    std::string getLastError() const override;
    const char* getName() const override { return name_.c_str(); }

    // ISimulator audio pipeline
    void update(double deltaTime) override;
    bool renderOnDemand(float* buffer, int32_t frames, int32_t* written) override;
    bool readAudioBuffer(float* buffer, int32_t frames, int32_t* read) override;
    bool start() override;
    void stop() override;

    // ISimulator telemetry & control
    EngineSimStats getStats() const override;
    void setThrottle(double position) override;
    void setIgnition(bool on) override;
    void setStarterMotor(bool on) override;

    Simulator* getInternalSimulator() { return m_simulator.get(); }
    const Simulator* getInternalSimulator() const { return m_simulator.get(); }

    // Set display name from script path (called by factory for PistonEngine mode)
    void setNameFromScript(const std::string& scriptPath);

private:
    void initDependencies(ILogging* logger, telemetry::ITelemetryWriter* telemetryWriter);
    void initAudioConfig(const ISimulatorConfig& config);
    void pushTelemetry(const EngineSimStats& stats);

    static void advanceFixedSteps(Simulator* sim, int simulationFrequency, double dt, bool ceil);
    void drainSynthesizerBuffer(Simulator* sim);

    int16_t* ensureAudioConversionBufferSize(size_t requiredSize);

    std::unique_ptr<Simulator> m_simulator;
    std::string m_lastError;
    std::string name_;
    bool m_created = false;

    // Dependencies (never null after create())
    ILogging* logger_ = nullptr;
    telemetry::ITelemetryWriter* telemetryWriter_ = nullptr;
    std::unique_ptr<ConsoleLogger> defaultLogger_;
    std::unique_ptr<NullTelemetryWriter> defaultTelemetryWriter_;

    // Audio config
    std::vector<int16_t> m_audioConversionBuffer;
    ISimulatorConfig engineConfig_;
};

#endif // BRIDGE_SIMULATOR_H
