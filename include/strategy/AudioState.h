// AudioState.h - Core audio playback state
// SRP: Single responsibility - manages only playback state
// OCP: New states can be added without modifying existing code
// DIP: High-level modules depend on this abstraction
// Phase F: Moved to engine-sim-bridge submodule

#ifndef AUDIO_STATE_H
#define AUDIO_STATE_H

#include <atomic>

// AudioState is a passive data holder — its owner (strategy) sets sampleRate
// during initialize(). No dependency on EngineSimDefaults.

struct AudioState {
    AudioState()
        : isPlaying(false)
        , sampleRate(0)
    {}

    std::atomic<bool> isPlaying;
    int sampleRate;

    void reset() {
        isPlaying.store(false);
        sampleRate = 0;
    }
};

#endif // AUDIO_STATE_H
