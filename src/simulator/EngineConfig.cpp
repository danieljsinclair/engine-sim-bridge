// EngineConfig.cpp - Engine configuration and script loading
// Phase F: Moved to engine-sim-bridge for reusability

#include "simulator/EngineConfig.h"

EngineConfig::EngineConfig() {}
EngineConfig::~EngineConfig() {}

EngineSimConfig EngineConfig::createDefault(int sampleRate, int simulationFrequency) {
    EngineSimConfig config = {};
    config.sampleRate = sampleRate > 0 ? sampleRate : EngineSimDefaults::SAMPLE_RATE;
    config.inputBufferSize = EngineSimDefaults::INPUT_BUFFER_SIZE;
    config.audioBufferSize = EngineSimDefaults::AUDIO_BUFFER_SIZE;
    config.simulationFrequency = simulationFrequency > 0 ? simulationFrequency : EngineSimDefaults::SIMULATION_FREQUENCY;
    config.fluidSimulationSteps = EngineSimDefaults::FLUID_SIMULATION_STEPS;
    config.targetSynthesizerLatency = EngineSimDefaults::TARGET_SYNTH_LATENCY;
    config.volume = 1.0f;
    config.convolutionLevel = 0.5f;
    config.airNoise = 1.0f;
    return config;
}

EngineSimHandle EngineConfig::createAndLoad(
    const EngineSimConfig& config,
    const std::string& configPath,
    const std::string& assetBasePath,
    EngineSimAPI& api,
    std::string& error)
{
    EngineSimHandle handle = nullptr;
    EngineSimResult result = api.Create(&config, nullptr, &handle);
    if (result != ESIM_SUCCESS || !handle) {
        error = "Failed to create simulator";
        return nullptr;
    }

    // Load script separately
    if (!configPath.empty()) {
        result = api.LoadScript(handle, configPath.c_str(), assetBasePath.c_str());
        if (result != ESIM_SUCCESS) {
            error = "Failed to load script";
            api.Destroy(handle);
            return nullptr;
        }
    }

    return handle;
}
