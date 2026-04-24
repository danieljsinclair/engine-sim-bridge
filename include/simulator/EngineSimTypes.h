#ifndef ATG_ENGINE_SIM_BRIDGE_H
#define ATG_ENGINE_SIM_BRIDGE_H

#include <stdint.h>
#include <stddef.h>
#include <cstring>

class ILogging;
class Simulator;

// ============================================================================
// DEFAULTS — single source of truth for simulation parameters
// ============================================================================
//
// Constant relationships:
//
// SIMULATION_FREQUENCY (10000 Hz) — physics step rate
//   |-- Synthesizer inputSampleRate = SIMULATION_FREQUENCY
//   |   |-- Upsampling ratio = SAMPLE_RATE / SIMULATION_FREQUENCY = 44100/10000 = 4.41x
//   |   |-- Each simulateStep() advances of synth input write pointer by this ratio
//   |-- Steps per 60Hz tick = SIMULATION_FREQUENCY / UPDATE_RATE_HZ = 10000/60 ~ 167
//   |   |-- Dynamically adjusted +/-10% by TARGET_SYNTH_LATENCY feedback loop
//   |-- Physics timestep = 1/SIMULATION_FREQUENCY = 0.1ms
//       |-- Overridable via --sim-freq CLI flag
//
// SAMPLE_RATE (44100 Hz) — audio output rate, matches upstream engine-sim
//   |-- Synthesizer audioSampleRate
//   |-- CoreAudio hardware device rate
//   |-- AUDIO_BUFFER_SIZE = SAMPLE_RATE * 2 seconds = 88200
//   |-- FRAMES_PER_UPDATE = SAMPLE_RATE / 60 = 735

namespace EngineSimDefaults {
    // Primary constants
    constexpr int32_t SAMPLE_RATE            = 44100;   // Hz — matches upstream engine-sim (simulator.cpp:226)
    constexpr int32_t SIMULATION_FREQUENCY   = 10000;   // Hz — physics step rate
    constexpr int32_t FLUID_SIMULATION_STEPS = 8;       // Substeps per physics step
    constexpr double  TARGET_SYNTH_LATENCY   = 0.02;    // seconds — M4 Pro best latency tested (0.01 too fast for threaded mode) - can be overridden with --synth-latency CLI flag

    // Derived constants — relationships are explicit
    constexpr int32_t UPDATE_RATE_HZ             = 60;                              // Main loop tick rate (from original 60 FPS GUI)
    constexpr double  BUFFER_DURATION_SECONDS    = 2.0;                             // Ring buffer safety margin for threaded mode
    constexpr int32_t AUDIO_BUFFER_SIZE          = SAMPLE_RATE * static_cast<int32_t>(BUFFER_DURATION_SECONDS);  // 88200
    constexpr int32_t INPUT_BUFFER_SIZE          = 1024;                            // Per-channel input ring buffer
    constexpr int32_t FRAMES_PER_UPDATE          = SAMPLE_RATE / UPDATE_RATE_HZ;   // 735 frames per 60Hz tick
    constexpr double  UPDATE_INTERVAL            = 1.0 / UPDATE_RATE_HZ;           // 16.67ms

    // Audio channel constants
    constexpr int32_t AUDIO_CHANNELS_MONO        = 1;   // Synthesizer output is mono (summed exhaust)
    constexpr int32_t AUDIO_CHANNELS_STEREO      = AUDIO_CHANNELS_MONO * 2;  // Hardware output is stereo (L+R duplicate)

    // Audio I/O constants
    constexpr int32_t MAX_AUDIO_CHUNK_FRAMES     = 4096; // Max frames per single read/drain operation
}

// ISimulatorConfig — Configuration for ISimulator implementations
// Inline initializers from EngineSimDefaults (single source of truth)
//
// Field ownership:
// - sampleRate: CROSS-CUTTING value shared across ISimulator, IAudioBuffer, and IAudioHardwareProvider
//   - ISimulator: computes dt = frames / sampleRate for renderOnDemand()
//   - IAudioBuffer: receives sampleRate as parameter to initialize()
//   - IAudioHardwareProvider: receives sampleRate via AudioStreamFormat for hardware configuration
//   - Canonical source is ISimulatorConfig.sampleRate (set from EngineSimDefaults::SAMPLE_RATE)
// - simulationFrequency, fluidSimulationSteps, synthLatency: ISimulator-only (factory sets on Simulator subclass)
// - maxChunkFrames, volume, convolutionLevel: ISimulator-only (runtime use by BridgeSimulator)
//
// Note: volume and convolutionLevel are runtime-tunable defaults, not constants
struct ISimulatorConfig {
    int32_t sampleRate = EngineSimDefaults::SAMPLE_RATE;
    int32_t simulationFrequency = EngineSimDefaults::SIMULATION_FREQUENCY;
    int32_t fluidSimulationSteps = EngineSimDefaults::FLUID_SIMULATION_STEPS;
    int32_t maxChunkFrames = EngineSimDefaults::MAX_AUDIO_CHUNK_FRAMES;
    double targetSynthesizerLatency = EngineSimDefaults::TARGET_SYNTH_LATENCY;
    float volume = 0.5f;           // Runtime-tunable default
    float convolutionLevel = 0.5f; // Runtime-tunable default
};

// Runtime statistics
struct EngineSimStats {
    double currentRPM = 0.0;
    double currentLoad = 0.0;
    double exhaustFlow = 0.0;
    double manifoldPressure = 0.0;
    int32_t activeChannels = 0;
    double processingTimeMs = 0.0;
};

namespace EngineSimAudio {
    constexpr int STEREO = 2;

// Converts mono int16 samples to stereo float32 (interleaved) - balanced channels
inline void convertInt16ToStereoFloat(
        const int16_t* input,
        int32_t frameCount,
        float* output,
        float volume,
        float convolutionLevel) {
    constexpr float scale = 1.0f / 32768.0f;  // int16_t range [-32768, 32767] normalized to [-1.0, 1.0]
    for (int32_t i = 0; i < frameCount; ++i) {
        const float sample = static_cast<float>(input[i]) * scale;
        // Interleaved stereo layout: [L, R, L, R, ...]
        output[i * STEREO] = sample * volume;           // Left channel
        output[i * STEREO + 1] = sample * convolutionLevel;  // Right channel
    }
}

// Converts mono int16 samples to stereo float32 with clipping protection
inline void convertInt16ToStereoFloatClipped(const int16_t* input, float* output, int32_t frameCount) {
    constexpr float scale = 1.0f / 32768.0f;  // int16_t range [-32768, 32767] normalized to [-1.0, 1.0]
    for (int32_t i = 0; i < frameCount; ++i) {
        float sample = static_cast<float>(input[i]) * scale;
        // Clamp to valid float audio range [-1.0, 1.0]
        if (sample > 1.0f) sample = 1.0f;
        if (sample < -1.0f) sample = -1.0f;
        // Interleaved stereo layout: [L, R, L, R, ...]
        output[i * STEREO] = sample;      // Left channel
        output[i * STEREO + 1] = sample;  // Right channel
    }
}

// Fills a stereo float buffer with silence (zeros)
inline void fillSilence(float* buffer, int32_t frames) {
    // STEREO channels per frame, sizeof(float) bytes per sample
    std::memset(buffer, 0, frames * STEREO * sizeof(float));
}

} // namespace EngineSimAudio

inline const char* EngineSimGetVersion() {
    return "1.0.0";
}

#endif // ATG_ENGINE_SIM_BRIDGE_H
