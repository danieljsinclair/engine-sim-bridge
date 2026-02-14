#ifndef MOCK_ENGINE_SIM_INTERNAL_H
#define MOCK_ENGINE_SIM_INTERNAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "mock_engine_sim.h"

// Internal mock implementation - types are now from engine_sim_bridge.h
typedef EngineSimHandle MockEngineSimHandle;

// Internal function prototypes (these are the actual implementations)
EngineSimResult MockEngineSimCreateInternal(
    const EngineSimConfig* config,
    MockEngineSimHandle* outHandle);

EngineSimResult MockEngineSimLoadScriptInternal(
    MockEngineSimHandle handle,
    const char* scriptPath,
    const char* assetBasePath);

EngineSimResult MockEngineSimStartAudioThreadInternal(
    MockEngineSimHandle handle);

EngineSimResult MockEngineSimDestroyInternal(
    MockEngineSimHandle handle);

EngineSimResult MockEngineSimSetThrottleInternal(
    MockEngineSimHandle handle,
    double position);

EngineSimResult MockEngineSimUpdateInternal(
    MockEngineSimHandle handle,
    double deltaTime);

EngineSimResult MockEngineSimRenderInternal(
    MockEngineSimHandle handle,
    float* buffer,
    int32_t frames,
    int32_t* outSamplesWritten);

EngineSimResult MockEngineSimReadAudioBufferInternal(
    MockEngineSimHandle handle,
    float* buffer,
    int32_t frames,
    int32_t* outSamplesRead);

EngineSimResult MockEngineSimGetStatsInternal(
    MockEngineSimHandle handle,
    EngineSimStats* outStats);

const char* MockEngineSimGetLastErrorInternal(
    MockEngineSimHandle handle);

const char* MockEngineSimGetVersionInternal(void);

EngineSimResult MockEngineSimValidateConfigInternal(
    const EngineSimConfig* config);

EngineSimResult MockEngineSimSetSpeedControlInternal(
    MockEngineSimHandle handle,
    double position);

EngineSimResult MockEngineSimSetStarterMotorInternal(
    MockEngineSimHandle handle,
    int enabled);

EngineSimResult MockEngineSimSetIgnitionInternal(
    MockEngineSimHandle handle,
    int enabled);

EngineSimResult MockEngineSimShiftGearInternal(
    MockEngineSimHandle handle,
    int gear);

EngineSimResult MockEngineSimSetClutchInternal(
    MockEngineSimHandle handle,
    double pressure);

EngineSimResult MockEngineSimSetDynoInternal(
    MockEngineSimHandle handle,
    int enabled);

EngineSimResult MockEngineSimSetDynoHoldInternal(
    MockEngineSimHandle handle,
    int enabled,
    double speed);

EngineSimResult MockEngineSimLoadImpulseResponseInternal(
    MockEngineSimHandle handle,
    int exhaustIndex,
    const int16_t* impulseData,
    int sampleCount,
    float volume);

#ifdef __cplusplus
}
#endif

#endif /* MOCK_ENGINE_SIM_INTERNAL_H */