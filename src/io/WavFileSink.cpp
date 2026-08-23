// WavFileSink.cpp - 16-bit PCM WAV implementation of IAudioSink.
// See include/io/WavFileSink.h for the format and threading contract.

#include "io/WavFileSink.h"

#include <algorithm>
#include <cmath>

namespace io {

namespace {

constexpr int BITS_PER_SAMPLE = 16;
constexpr int BYTES_PER_SAMPLE = BITS_PER_SAMPLE / 8;
constexpr uint16_t WAVE_FORMAT_PCM = 1;
constexpr uint32_t HEADER_BYTES = 44;          // canonical RIFF/WAVE PCM header
constexpr uint32_t FMT_CHUNK_BYTES = 16;       // PCM fmt chunk payload size
constexpr uint32_t RIFF_PREFIX_BYTES = 8;      // "RIFF" + size field
constexpr float SAMPLE_CLIP_LIMIT = 1.0f;
constexpr float INT16_SCALE = 32767.0f;

// Little-endian scalar writers. The WAV container is defined little-endian, so
// emit byte-by-byte rather than memcpy-ing host integers — correct on any
// endianness without a byte-swap branch.
void putU32(std::FILE* file, uint32_t value) {
    const unsigned char bytes[4] = {
        static_cast<unsigned char>(value & 0xFFu),
        static_cast<unsigned char>((value >> 8) & 0xFFu),
        static_cast<unsigned char>((value >> 16) & 0xFFu),
        static_cast<unsigned char>((value >> 24) & 0xFFu)
    };
    std::fwrite(bytes, 1, sizeof(bytes), file);
}

void putU16(std::FILE* file, uint16_t value) {
    const unsigned char bytes[2] = {
        static_cast<unsigned char>(value & 0xFFu),
        static_cast<unsigned char>((value >> 8) & 0xFFu)
    };
    std::fwrite(bytes, 1, sizeof(bytes), file);
}

void putTag(std::FILE* file, const char (&tag)[5]) {
    std::fwrite(tag, 1, 4, file);
}

// Convert one float sample to clamped 16-bit PCM. NaN maps to silence: a NaN
// that reached the output would otherwise become an arbitrary loud sample.
int16_t toInt16(float sample) {
    if (std::isnan(sample)) {
        return 0;
    }
    const float clamped = std::clamp(sample, -SAMPLE_CLIP_LIMIT, SAMPLE_CLIP_LIMIT);
    return static_cast<int16_t>(std::lround(clamped * INT16_SCALE));
}

} // anonymous namespace

WavFileSink::WavFileSink(const std::string& path, int sampleRate, int channelCount)
    : path_(path)
    , sampleRate_(sampleRate)
    , channelCount_(channelCount)
{
    // Boundary input: the path comes from a CLI flag and the sample rate/channel
    // count from config, so a bad value is a user/config error, not a programmer
    // one — report via isOpen() rather than asserting.
    const bool formatValid = sampleRate_ > 0 && channelCount_ > 0;
    if (formatValid) {
        file_ = std::fopen(path_.c_str(), "wb");
    }

    if (file_ && !writeHeader(0)) {
        std::fclose(file_);
        file_ = nullptr;
    }
}

WavFileSink::~WavFileSink() {
    // Finalize on destruction so a capture is still valid if the owner is torn
    // down by an exception path that never reached an explicit finalize().
    finalize();
}

bool WavFileSink::isOpen() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return file_ != nullptr;
}

uint64_t WavFileSink::framesWritten() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return frameCount_;
}

bool WavFileSink::writeHeader(uint64_t frameCount) {
    const uint32_t frames = static_cast<uint32_t>(frameCount);
    const uint32_t dataBytes = frames * static_cast<uint32_t>(channelCount_) * BYTES_PER_SAMPLE;
    const uint32_t byteRate = static_cast<uint32_t>(sampleRate_) * static_cast<uint32_t>(channelCount_) * BYTES_PER_SAMPLE;
    const uint16_t blockAlign = static_cast<uint16_t>(channelCount_ * BYTES_PER_SAMPLE);

    if (std::fseek(file_, 0, SEEK_SET) != 0) {
        return false;
    }

    putTag(file_, "RIFF");
    putU32(file_, HEADER_BYTES - RIFF_PREFIX_BYTES + dataBytes);
    putTag(file_, "WAVE");

    putTag(file_, "fmt ");
    putU32(file_, FMT_CHUNK_BYTES);
    putU16(file_, WAVE_FORMAT_PCM);
    putU16(file_, static_cast<uint16_t>(channelCount_));
    putU32(file_, static_cast<uint32_t>(sampleRate_));
    putU32(file_, byteRate);
    putU16(file_, blockAlign);
    putU16(file_, BITS_PER_SAMPLE);

    putTag(file_, "data");
    putU32(file_, dataBytes);

    return std::ferror(file_) == 0;
}

void WavFileSink::writeFrames(const float* interleaved, int frameCount, int channelCount) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Silently ignore nothing-to-do and post-finalize calls: the render thread
    // cannot handle an error here, and a late callback after teardown is normal.
    const bool writable = file_ != nullptr
                       && !finalized_
                       && interleaved != nullptr
                       && frameCount > 0
                       && channelCount == channelCount_;
    if (writable) {
        const size_t sampleCount = static_cast<size_t>(frameCount) * static_cast<size_t>(channelCount);
        staging_.resize(sampleCount);
        for (size_t i = 0; i < sampleCount; ++i) {
            staging_[i] = toInt16(interleaved[i]);
        }

        const size_t written = std::fwrite(staging_.data(), sizeof(int16_t), sampleCount, file_);
        frameCount_ += written / static_cast<size_t>(channelCount);
    }
}

bool WavFileSink::finalize() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (finalized_) {
        return finalizeResult_;
    }
    finalized_ = true;

    if (file_ == nullptr) {
        finalizeResult_ = false;
        return finalizeResult_;
    }

    // Patch the header with the real sizes, then close. A zero-frame run still
    // leaves a structurally valid (empty) WAV rather than a truncated file.
    const bool headerOk = writeHeader(frameCount_);
    const bool closeOk = std::fclose(file_) == 0;
    file_ = nullptr;

    finalizeResult_ = headerOk && closeOk;
    return finalizeResult_;
}

} // namespace io
