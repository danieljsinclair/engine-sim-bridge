#ifdef USE_MOCK_ENGINE_SIM

#include "../include/mock_engine_sim_internal.h"

// Mock implementation wrapper - redirects to mock functions
extern "C" {

EngineSimResult EngineSimCreate(
    const EngineSimConfig* config,
    EngineSimHandle* outHandle)
{
    return MockEngineSimCreateInternal(config, (MockEngineSimHandle*)outHandle);
}

EngineSimResult EngineSimLoadScript(
    EngineSimHandle handle,
    const char* scriptPath,
    const char* assetBasePath)
{
    return MockEngineSimLoadScriptInternal((MockEngineSimHandle)handle, scriptPath, assetBasePath);
}

EngineSimResult EngineSimStartAudioThread(
    EngineSimHandle handle)
{
    return MockEngineSimStartAudioThreadInternal((MockEngineSimHandle)handle);
}

EngineSimResult EngineSimDestroy(
    EngineSimHandle handle)
{
    return MockEngineSimDestroyInternal((MockEngineSimHandle)handle);
}

EngineSimResult EngineSimSetThrottle(
    EngineSimHandle handle,
    double position)
{
    return MockEngineSimSetThrottleInternal((MockEngineSimHandle)handle, position);
}

EngineSimResult EngineSimUpdate(
    EngineSimHandle handle,
    double deltaTime)
{
    return MockEngineSimUpdateInternal((MockEngineSimHandle)handle, deltaTime);
}

EngineSimResult EngineSimRender(
    EngineSimHandle handle,
    float* buffer,
    int32_t frames,
    int32_t* outSamplesWritten)
{
    return MockEngineSimRenderInternal((MockEngineSimHandle)handle, buffer, frames, outSamplesWritten);
}

EngineSimResult EngineSimReadAudioBuffer(
    EngineSimHandle handle,
    float* buffer,
    int32_t frames,
    int32_t* outSamplesRead)
{
    return MockEngineSimReadAudioBufferInternal((MockEngineSimHandle)handle, buffer, frames, outSamplesRead);
}

EngineSimResult EngineSimGetStats(
    EngineSimHandle handle,
    EngineSimStats* outStats)
{
    return MockEngineSimGetStatsInternal((MockEngineSimHandle)handle, outStats);
}

const char* EngineSimGetLastError(
    EngineSimHandle handle)
{
    return MockEngineSimGetLastErrorInternal((MockEngineSimHandle)handle);
}

const char* EngineSimGetVersion(void)
{
    return MockEngineSimGetVersionInternal();
}

EngineSimResult EngineSimValidateConfig(
    const EngineSimConfig* config)
{
    return MockEngineSimValidateConfigInternal(config);
}

EngineSimResult EngineSimSetSpeedControl(
    EngineSimHandle handle,
    double position)
{
    return MockEngineSimSetSpeedControlInternal((MockEngineSimHandle)handle, position);
}

EngineSimResult EngineSimSetStarterMotor(
    EngineSimHandle handle,
    int enabled)
{
    return MockEngineSimSetStarterMotorInternal((MockEngineSimHandle)handle, enabled);
}

EngineSimResult EngineSimSetIgnition(
    EngineSimHandle handle,
    int enabled)
{
    return MockEngineSimSetIgnitionInternal((MockEngineSimHandle)handle, enabled);
}

EngineSimResult EngineSimShiftGear(
    EngineSimHandle handle,
    int gear)
{
    return MockEngineSimShiftGearInternal((MockEngineSimHandle)handle, gear);
}

EngineSimResult EngineSimSetClutch(
    EngineSimHandle handle,
    double pressure)
{
    return MockEngineSimSetClutchInternal((MockEngineSimHandle)handle, pressure);
}

EngineSimResult EngineSimSetDyno(
    EngineSimHandle handle,
    int enabled)
{
    return MockEngineSimSetDynoInternal((MockEngineSimHandle)handle, enabled);
}

EngineSimResult EngineSimSetDynoHold(
    EngineSimHandle handle,
    int enabled,
    double speed)
{
    return MockEngineSimSetDynoHoldInternal((MockEngineSimHandle)handle, enabled, speed);
}

EngineSimResult EngineSimLoadImpulseResponse(
    EngineSimHandle handle,
    int exhaustIndex,
    const int16_t* impulseData,
    int sampleCount,
    float volume)
{
    return MockEngineSimLoadImpulseResponseInternal((MockEngineSimHandle)handle, exhaustIndex, impulseData, sampleCount, volume);
}

} // extern "C"

#endif // USE_MOCK_ENGINE_SIM