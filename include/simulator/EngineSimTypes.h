#ifndef ATG_ENGINE_SIM_BRIDGE_H
#define ATG_ENGINE_SIM_BRIDGE_H

#include <stdint.h>
#include <stddef.h>

#include "audio/EngineSimAudio.h"
#include "audio/SpeakerProtection.h"

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

    // Simulation defaults
    constexpr double  DEFAULT_DURATION_SECONDS   = 3.0;  // Default non-interactive simulation duration
    constexpr float   DEFAULT_HARDWARE_VOLUME    = 1.0f; // Default hardware output volume (0.0 to 1.0)
    constexpr int32_t DEFAULT_PREFILL_MS         = 50;   // Default pre-fill buffer duration in ms for sync-pull mode
}

// ISimulatorConfig — Configuration for ISimulator implementations
// Inline initializers from EngineSimDefaults (single source of truth)
struct ISimulatorConfig {
    int32_t sampleRate = EngineSimDefaults::SAMPLE_RATE;
    int32_t simulationFrequency = EngineSimDefaults::SIMULATION_FREQUENCY;
    int32_t fluidSimulationSteps = EngineSimDefaults::FLUID_SIMULATION_STEPS;
    int32_t maxChunkFrames = EngineSimDefaults::MAX_AUDIO_CHUNK_FRAMES;
    double targetSynthesizerLatency = EngineSimDefaults::TARGET_SYNTH_LATENCY;
    float volume = 0.5f;           // Runtime-tunable default
    float convolutionLevel = 0.5f; // Runtime-tunable default
    bool speakerProtection = true;           // On by default
    float speakerProtectionDrive = 1.0f;     // Tanh drive (1.0 = transparent below 0.7)
    bool breachRecovery = true;              // On by default
    bool trendHold = true;                   // On by default
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

inline const char* EngineSimGetVersion() {
    return "1.0.0";
}

#endif // ATG_ENGINE_SIM_BRIDGE_H
