#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <algorithm>

enum class SampleFormat : uint8_t {
    Float32,
    Int16,
    Int24,
    Int32
};

struct AudioBufferView {
    void* samples;
    int frameCount;
    int channelCount;
    SampleFormat format;
    bool isInterleaved;

    AudioBufferView()
        : samples(nullptr)
        , frameCount(0)
        , channelCount(0)
        , format(SampleFormat::Float32)
        , isInterleaved(true) {}

    AudioBufferView(float* data, int frames, int channels)
        : samples(data)
        , frameCount(frames)
        , channelCount(channels)
        , format(SampleFormat::Float32)
        , isInterleaved(true) {}

    AudioBufferView(void* data, int frames, int channels,
                    SampleFormat fmt, bool interleaved)
        : samples(data)
        , frameCount(frames)
        , channelCount(channels)
        , format(fmt)
        , isInterleaved(interleaved) {}

    float* asFloat() const {
        return (format == SampleFormat::Float32)
            ? static_cast<float*>(samples)
            : nullptr;
    }

    int16_t* asInt16() const {
        return (format == SampleFormat::Int16)
            ? static_cast<int16_t*>(samples)
            : nullptr;
    }

    size_t totalInterleavedSamples() const {
        return static_cast<size_t>(frameCount) * channelCount;
    }

    size_t bytesPerSample() const {
        switch (format) {
            case SampleFormat::Float32: return sizeof(float);
            case SampleFormat::Int16:   return sizeof(int16_t);
            case SampleFormat::Int24:   return 4;
            case SampleFormat::Int32:   return sizeof(int32_t);
        }
        return 0;
    }

    size_t sizeInBytes() const {
        return totalInterleavedSamples() * bytesPerSample();
    }
};
