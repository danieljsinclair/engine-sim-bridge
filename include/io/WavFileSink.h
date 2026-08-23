// WavFileSink.h - IAudioSink that writes rendered frames to a 16-bit PCM WAV file.
//
// Streams to disk as frames arrive rather than accumulating the whole run in
// memory, so an open-ended interactive capture costs a bounded staging buffer
// instead of growing without limit. The RIFF/WAVE header is written up front
// with placeholder sizes and patched in finalize(), once the true frame count
// is known — the standard streaming-WAV approach.
//
// Format: RIFF/WAVE, PCM (format tag 1), 16-bit signed little-endian,
// interleaved, at the sample rate and channel count given to the constructor.
// Float samples are clamped to [-1, 1] before conversion, so an over-driven
// render clips rather than wrapping around into noise.
//
// Threading: writeFrames() runs on the audio render thread, finalize() on the
// owning thread after playback has stopped. A mutex guards the file handle so
// the two cannot interleave, and writeFrames() after finalize() is ignored.

#ifndef WAV_FILE_SINK_H
#define WAV_FILE_SINK_H

#include "io/IAudioSink.h"

#include <cstdint>
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>

namespace io {

class WavFileSink : public IAudioSink {
public:
    /// Open path for writing and emit a placeholder header.
    /// isOpen() reports whether the file could be created; construction never throws.
    WavFileSink(const std::string& path, int sampleRate, int channelCount);
    ~WavFileSink() override;

    WavFileSink(const WavFileSink&) = delete;
    WavFileSink& operator=(const WavFileSink&) = delete;

    void writeFrames(const float* interleaved, int frameCount, int channelCount) override;
    bool finalize() override;

    /// True when the output file was created and the header written.
    bool isOpen() const;

    /// Frames accepted so far (excludes frames dropped because the file is closed).
    uint64_t framesWritten() const;

    /// Path this sink writes to.
    const std::string& path() const { return path_; }

private:
    // Emit the 44-byte canonical header with the sizes it currently knows.
    // Called once at construction (zero sizes) and again from finalize().
    bool writeHeader(uint64_t frameCount);

    std::string path_;
    int sampleRate_;
    int channelCount_;

    mutable std::mutex mutex_;
    std::FILE* file_ = nullptr;
    uint64_t frameCount_ = 0;
    bool finalized_ = false;
    bool finalizeResult_ = false;

    // Staging buffer for float -> int16 conversion, reused across callbacks so
    // the render thread performs no allocation in the steady state.
    std::vector<int16_t> staging_;
};

} // namespace io

#endif // WAV_FILE_SINK_H
