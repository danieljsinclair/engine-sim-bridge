// AudioLoopConfig.h - Audio loop and engine simulation constants
// Moved from CLI config: these are audio/simulation concerns, not CLI-specific.
// Shared by bridge (SimulationLoop, strategies) and CLI (argument parsing).

#ifndef AUDIO_LOOP_CONFIG_H
#define AUDIO_LOOP_CONFIG_H

#include "simulator/engine_sim_bridge.h"

// ============================================================================
// Audio Loop Configuration
// ============================================================================

struct AudioLoopConfig {
    static constexpr int SAMPLE_RATE           = EngineSimDefaults::SAMPLE_RATE;
    static constexpr int CHANNELS              = EngineSimDefaults::AUDIO_CHANNELS_STEREO;
    static constexpr double UPDATE_INTERVAL    = EngineSimDefaults::UPDATE_INTERVAL;
    static constexpr int FRAMES_PER_UPDATE     = EngineSimDefaults::FRAMES_PER_UPDATE;
    static constexpr int WARMUP_ITERATIONS     = 3;
    static constexpr int PRE_FILL_ITERATIONS   = 40;   // ~0.67s initial buffer
    static constexpr int RE_PRE_FILL_ITERATIONS = 0;
};


#endif // AUDIO_LOOP_CONFIG_H
