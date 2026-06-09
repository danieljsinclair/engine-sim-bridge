// BridgeSimulator.h - Universal ISimulator implementation
// Composes an injected Simulator subclass (PistonEngineSimulator or SineSimulator).
// OCP: BridgeSimulator doesn't know or care which Simulator it has.
// Factory is the composition root that wires mode-specific details.

#ifndef BRIDGE_SIMULATOR_H
#define BRIDGE_SIMULATOR_H

#include "simulator/ISimulator.h"
#include "simulator/ICombustionEngine.h"
#include "simulator/EngineSimTypes.h"
#include "simulation/EnginePhase.h"
#include "common/ILogging.h"
#include "telemetry/ITelemetryProvider.h"
#include "telemetry/NullTelemetryWriter.h"
#include "engine-sim/include/simulator.h"
#include "simulator/BrakeConstraint.h"

#include <memory>
#include <string>
#include <vector>

class BridgeSimulator : public ICombustionEngine {
public:
    // Constructor takes an already-initialized Simulator subclass.
    // The factory is responsible for creating and wiring the Simulator.
    explicit BridgeSimulator(std::unique_ptr<Simulator> simulator, const std::string& name = "Simulator");
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
    int getSimulationFrequency() const override { return engineConfig_.simulationFrequency; }
    EngineSimStats getStats() const override;

    // TODO: circle-back — remove getInternalSimulator(); tests should inject dependencies instead
    Simulator* getInternalSimulator() { return m_simulator.get(); }
    const Simulator* getInternalSimulator() const { return m_simulator.get(); }

    void setThrottle(double position) override;
    void setIgnition(bool on) override;
    void setStarterMotor(bool on) override;
    int setGear(int gear) override;
    int getGear() const override;
    void setClutchPressure(double pressure) override;
    void setBrakePressure(double pressure) override;
    double getEngineRpm() const override;
    EnginePhase getEnginePhase() const override { return enginePhase_; }
    void applyTransition(const TransitionDecision& decision) override;

    // ISimulator state capture/restore for hot-swap
    std::vector<uint8_t> saveState() const override;
    void restoreState(const std::vector<uint8_t>& data) override;

    // Drivetrain state transfer for engine hot-swap (preserves road speed)
    struct DrivetrainSnapshot {
        double vehicleMassVtheta = 0.0;
        double vehicleMassI = 0.0;
        double vehicleMassM = 0.0;
        EnginePhase enginePhase = EnginePhase::Stopped;
        int gear = 0;
    };
    DrivetrainSnapshot captureDrivetrainState() const;
    void restoreDrivetrainState(const DrivetrainSnapshot& snapshot);
    bool changeGear(int gearDelta);
    void setDynoTorqueScale(double scale);

    // Configure dyno in load torque mode (brake-only).
    // loadFraction: 0.0-1.0 fraction of DYNO_MAX_TORQUE_FT_LBS.
    // Returns true if configured, false if loadFraction <= 0.
    bool configureDynoLoad(double loadFraction);

    // Set display name directly
    void setName(const std::string& name) { name_ = name; }

private:
    void initDependencies(ILogging* logger, telemetry::ITelemetryWriter* telemetryWriter);
    void initAudioConfig(const ISimulatorConfig& config);
    void pushTelemetry(const EngineSimStats& stats);

    // Single source of truth for engine phase — written via applyTransition()
    EnginePhase enginePhase_ = EnginePhase::Stopped;

    static void advanceFixedSteps(Simulator* sim, int simulationFrequency, double dt, bool ceil);
    void drainSynthesizerBuffer(Simulator* sim);

    int16_t* ensureAudioConversionBufferSize(size_t requiredSize);

    std::unique_ptr<Simulator> m_simulator;
    BrakeConstraint m_brakeConstraint;
    std::string m_lastError;
    std::string name_;

    // metrics
    void getEngineStats(EngineSimStats& stats) const;
    void getDynoStats(EngineSimStats& stats) const;
    void getVehicleStats(EngineSimStats& stats) const;
    void getTransmissionStats(EngineSimStats& stats) const;

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
