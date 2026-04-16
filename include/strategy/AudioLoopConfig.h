// AudioLoopConfig.h - Audio loop and engine simulation constants
// Moved from CLI config: these are audio/simulation concerns, not CLI-specific.
// Shared by bridge (SimulationLoop, strategies) and CLI (argument parsing).

#ifndef AUDIO_LOOP_CONFIG_H
#define AUDIO_LOOP_CONFIG_H

// ============================================================================
// Audio Loop Configuration
// ============================================================================

struct AudioLoopConfig {
    static constexpr int SAMPLE_RATE = 44100;
    static constexpr int CHANNELS = 2;
    static constexpr double UPDATE_INTERVAL = 1.0 / 60.0;  // 60Hz
    static constexpr int FRAMES_PER_UPDATE = SAMPLE_RATE / 60;  // 735 frames
    static constexpr int WARMUP_ITERATIONS = 3;  // Minimal warmup
    static constexpr int PRE_FILL_ITERATIONS = 40;  // 0.67s initial buffer (matches working commit)
    static constexpr int RE_PRE_FILL_ITERATIONS = 0;  // No re-pre-fill (matches working commit)
};

// ============================================================================
// Engine Simulation Constants
// ============================================================================

struct EngineConstants {
    static constexpr int DEFAULT_SIMULATION_FREQUENCY = 10000;
    static constexpr int MIN_SIMULATION_FREQUENCY = 1000;
    static constexpr int MAX_SIMULATION_FREQUENCY = 100000;
};

#endif // AUDIO_LOOP_CONFIG_H
