#ifndef ATG_ENGINE_SIM_WAV_LOADER_H
#define ATG_ENGINE_SIM_WAV_LOADER_H

// DR_WAV_IMPLEMENTATION must be defined in exactly ONE .cpp file before including this header.
// This is the standard pattern for stb-style single-header libraries.
// Example (in one .cpp file):
//   #define DR_WAV_IMPLEMENTATION
//   #include "common/wav_loader.h"
#include <dr_wav.h>

#include <vector>
#include <inttypes.h>
#include <string>

/**
 * WAV file loader for engine-sim bridge layer.
 * Uses dr_wav library (https://github.com/mackron/dr_libs).
 *
 * Responsibility: Load WAV files and return audio samples.
 * Single Responsibility Principle - only handles WAV file I/O.
 */
class WavLoader {
public:
    struct Result {
        std::vector<int16_t> samples;
        int sampleRate = 0;
        bool valid = false;

        size_t getSampleCount() const { return samples.size(); }
        const int16_t* getData() const { return samples.data(); }
    };

    /**
     * Load a WAV file from disk.
     *
     * @param filepath Path to WAV file (absolute or relative)
     * @return Result struct containing samples and metadata. Check result.valid.
     */
    static Result load(const std::string& filepath) {
        Result result;
        result.valid = false;

        drwav wav;
        if (!drwav_init_file(&wav, filepath.c_str(), nullptr)) {
            return result;
        }

        result.sampleRate = wav.sampleRate;
        result.samples.resize(wav.totalPCMFrameCount * wav.channels);

        size_t framesRead = drwav_read_pcm_frames_s16(
            &wav,
            wav.totalPCMFrameCount,
            result.samples.data()
        );

        result.valid = (framesRead == wav.totalPCMFrameCount);
        drwav_uninit(&wav);

        return result;
    }

    /**
     * Check if a file appears to be a valid WAV file by header inspection.
     */
    static bool isValidWavFile(const std::string& filepath) {
        drwav wav;
        bool valid = drwav_init_file(&wav, filepath.c_str(), nullptr);
        if (valid) {
            drwav_uninit(&wav);
        }
        return valid;
    }
};

#endif /* ATG_ENGINE_SIM_WAV_LOADER_H */
