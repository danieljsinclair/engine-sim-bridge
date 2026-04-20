#pragma once
#include <string>
#include <cstdint>
#include "simulator/engine_sim_bridge.h"

class ILogging;

namespace telemetry { class ITelemetryWriter; }

class ISimulator {
public:
    virtual ~ISimulator() = default;
    virtual bool create(const EngineSimConfig& config, ILogging* logger, telemetry::ITelemetryWriter* telemetryWriter) = 0;
    virtual bool loadScript(const std::string& path, const std::string& assetBase) = 0;
    virtual void destroy() = 0;
    virtual std::string getLastError() const = 0;
    virtual const char* getName() const = 0;
    virtual void update(double deltaTime) = 0;
    virtual EngineSimStats getStats() const = 0;
    virtual void setThrottle(double position) = 0;
    virtual void setIgnition(bool on) = 0;
    virtual void setStarterMotor(bool on) = 0;
    virtual bool renderOnDemand(float* buffer, int32_t frames, int32_t* written) = 0;
    virtual bool readAudioBuffer(float* buffer, int32_t frames, int32_t* read) = 0;
    virtual bool start() = 0;
    virtual void stop() = 0;
    static const char* getVersion();
};
