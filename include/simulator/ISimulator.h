#pragma once
#include <string>
#include <cstdint>
#include <vector>
#include "simulator/EngineSimTypes.h"
#include "simulation/EnginePhase.h"

class ILogging;

namespace telemetry { class ITelemetryWriter; }

class ISimulator {
public:
    virtual ~ISimulator() = default;
    virtual bool create(const ISimulatorConfig& config, ILogging* logger, telemetry::ITelemetryWriter* telemetryWriter) = 0;
    virtual void destroy() = 0;
    virtual std::string getLastError() const = 0;
    virtual const char* getName() const = 0;
    virtual void update(double deltaTime) = 0;
    virtual EngineSimStats getStats() const = 0;
    virtual void setThrottle(double position) = 0;
    virtual bool renderOnDemand(float* buffer, int32_t frames, int32_t* written) = 0;

    // Twin-specific control methods
    virtual int setGear(int /* gear */) { return 0;}
    virtual int getGear() const { return 0; }
    virtual void setClutchPressure(double /* pressure */) {}
    virtual double getEngineRpm() const { return 0.0; }
    virtual bool readAudioBuffer(float* buffer, int32_t frames, int32_t* read) = 0;
    virtual bool start() = 0;
    virtual void stop() = 0;
    virtual int getSimulationFrequency() const = 0;

    // Read-only phase — display/telemetry. Writing via ICombustionEngine::setEnginePhase().
    virtual EnginePhase getEnginePhase() const { return EnginePhase::Stopped; }

    // Drivetrain state capture/restore for hot-swap
    virtual std::vector<uint8_t> saveState() const { return {}; }
    virtual void restoreState(const std::vector<uint8_t>&) {}

    static const char* getVersion();
};
