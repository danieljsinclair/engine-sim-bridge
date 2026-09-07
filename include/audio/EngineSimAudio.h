// EngineSimAudio.h - Audio format conversion and buffer utilities
// SRP: Audio sample format conversion only
// Phase F: Extracted from EngineSimTypes.h for SRP compliance

#ifndef ENGINE_SIM_AUDIO_H
#define ENGINE_SIM_AUDIO_H

#include <cstdint>
#include <cstring>

namespace EngineSimAudio {
    constexpr int STEREO = 2;

// Converts mono int16 samples to stereo float32 (interleaved) - balanced channels
inline void convertInt16ToStereoFloat(
        const int16_t* input,
        int32_t frameCount,
        float* output,
        float volume,
        float convolutionLevel) {
    constexpr float scale = 1.0f / 32768.0f;
    for (int32_t i = 0; i < frameCount; ++i) {
        const float sample = static_cast<float>(input[i]) * scale;
        output[i * STEREO] = sample * volume;
        output[i * STEREO + 1] = sample * convolutionLevel;
    }
}

// Converts mono int16 samples to stereo float32 with clipping protection
inline void convertInt16ToStereoFloatClipped(const int16_t* input, float* output, int32_t frameCount) {
    constexpr float scale = 1.0f / 32768.0f;
    for (int32_t i = 0; i < frameCount; ++i) {
        float sample = static_cast<float>(input[i]) * scale;
        if (sample > 1.0f) sample = 1.0f;
        if (sample < -1.0f) sample = -1.0f;
        output[i * STEREO] = sample;
        output[i * STEREO + 1] = sample;
    }
}

// Fills a stereo float buffer with silence (zeros)
inline void fillSilence(float* buffer, int32_t frames) {
    std::memset(buffer, 0, frames * STEREO * sizeof(float));
}

} // namespace EngineSimAudio

#endif // ENGINE_SIM_AUDIO_H
