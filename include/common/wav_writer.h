// wav_writer.h - WAV file writer for engine-sim bridge layer.
//
// Writes interleaved float32 audio (the native render-buffer format) to a
// 16-bit PCM WAV file. Uses the same dr_wav library as WavLoader
// (engine-sim/dependencies/dr_libs/dr_wav.h) — the library supports both
// reading and writing. DR_WAV_IMPLEMENTATION is already defined in
// ScriptLoadHelpers.cpp / wav_loader.cpp, so this header just links against
// the existing implementation.
//
// Lifecycle: open(path, sampleRate, channels) -> writeFrames() x N -> destructor
// finalizes the file (or call close() explicitly). The writer is NOT thread-
// safe by itself; the caller (the audio callback) serializes access.

#ifndef ATG_ENGINE_SIM_WAV_WRITER_H
#define ATG_ENGINE_SIM_WAV_WRITER_H

#include <dr_wav.h>

#include <algorithm>
#include <cassert>
#include <climits>
#include <cstdint>
#include <string>
#include <vector>

class WavWriter {
public:
    WavWriter() = default;

    // Open a WAV file for writing. Returns true on success.
    // sampleRate: e.g. 44100. channels: 1 (mono) or 2 (stereo).
    bool open(const std::string& filepath, uint32_t sampleRate, uint32_t channels) {
        close();

        drwav_data_format format;
        format.container = drwav_container_riff;  // standard WAV (not RF64)
        format.format = DR_WAVE_FORMAT_PCM;       // 16-bit PCM
        format.channels = channels;
        format.sampleRate = sampleRate;
        format.bitsPerSample = 16;

        bool ok = drwav_init_file_write(&m_wav, filepath.c_str(), &format, nullptr);
        m_open = ok;
        return ok;
    }

    // Write interleaved float32 frames (the render-buffer native format).
    // Converts float [-1, 1] -> int16 in a scratch buffer, then hands the
    // int16 PCM to drwav_write_pcm_frames (which writes raw bytes — it does
    // NOT convert float->int; with bitsPerSample=16 it expects int16 input).
    // Returns the number of frames actually written.
    uint64_t writeFrames(const float* samples, uint64_t frameCount) {
        if (!m_open || frameCount == 0) return 0;
        const uint64_t total = frameCount * m_wav.channels;
        if (total > static_cast<uint64_t>(UINT_MAX)) return 0;

        // Lazily size the scratch buffer to the per-callback frame count.
        if (m_int16Scratch.size() < total) {
            m_int16Scratch.resize(static_cast<size_t>(total));
        }

        for (uint64_t i = 0; i < total; ++i) {
            float s = samples[i];
            // Clamp to [-1, 1] before scaling to avoid int16 overflow.
            s = (s < -1.0f) ? -1.0f : (s > 1.0f) ? 1.0f : s;
            m_int16Scratch[i] = static_cast<int16_t>(s * 32767.0f);
        }

        return drwav_write_pcm_frames(&m_wav, frameCount, m_int16Scratch.data());
    }

    bool isOpen() const { return m_open; }

    void close() {
        if (m_open) {
            drwav_uninit(&m_wav);
            m_open = false;
        }
    }

    ~WavWriter() { close(); }

    // Non-copyable (owns an open file handle).
    WavWriter(const WavWriter&) = delete;
    WavWriter& operator=(const WavWriter&) = delete;

private:
    drwav m_wav{};
    bool m_open = false;
    std::vector<int16_t> m_int16Scratch;
};

#endif /* ATG_ENGINE_SIM_WAV_WRITER_H */
