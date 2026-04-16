#pragma once

#include <string>
#include <cstdint>

struct EngineSimStats;
struct EngineSimConfig;
class ILogging;

class ISimulator {
public:
    virtual ~ISimulator() = default;

    // Lifecycle
    virtual bool create(const EngineSimConfig& config) = 0;
    virtual bool loadScript(const std::string& path, const std::string& assetBase) = 0;
    virtual bool setLogging(ILogging* logger) = 0;
    virtual void destroy() = 0;
    virtual std::string getLastError() const = 0;

    // Simulation
    virtual void update(double deltaTime) = 0;
    virtual EngineSimStats getStats() const = 0;

    // Control inputs
    virtual void setThrottle(double position) = 0;
    virtual void setIgnition(bool on) = 0;
    virtual void setStarterMotor(bool on) = 0;

    // Audio frame production
    virtual bool renderOnDemand(float* buffer, int32_t frames, int32_t* written) = 0;
    virtual bool readAudioBuffer(float* buffer, int32_t frames, int32_t* read) = 0;
    virtual bool startAudioThread() = 0;

    // Version (static -- not instance-dependent)
    static const char* getVersion();
};
