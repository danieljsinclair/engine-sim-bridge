// AudioLoopConfig.h - Audio loop and engine simulation constants
// Moved from CLI config: these are audio/simulation concerns, not CLI-specific.
// Shared by bridge (SimulationLoop, strategies) and CLI (argument parsing).

#ifndef AUDIO_LOOP_CONFIG_H
#define AUDIO_LOOP_CONFIG_H

#include "simulator/EngineSimTypes.h"

// ============================================================================
// Audio Loop Configuration
// ============================================================================

struct AudioLoopConfig {
    // Loop-specific constants (not in EngineSimDefaults)
    static constexpr int WARMUP_ITERATIONS     = 3;
    static constexpr int PRE_FILL_ITERATIONS   = 40;   // ~0.67s initial buffer
    static constexpr int RE_PRE_FILL_ITERATIONS = 0;
};


#endif // AUDIO_LOOP_CONFIG_H
