// AudioTypes.h - Platform-agnostic audio buffer types
// Decouples the bridge library from CoreAudio-specific types (AudioBufferList*)
// DIP: High-level modules depend on this abstraction, not platform-specific types
// Phase G: Created for iOS/macOS platform decoupling

#ifndef AUDIO_TYPES_H
#define AUDIO_TYPES_H

/**
 * AudioBufferDescriptor - Platform-agnostic audio buffer description
 *
 * Describes a contiguous interleaved audio buffer without any
 * platform-specific types. Used by IAudioBuffer::render() and
 * IAudioHardwareProvider::AudioCallback.
 *
 * Consumers (e.g. CoreAudioHardwareProvider, AVAudioEngineHardwareProvider)
 * create AudioBufferDescriptor from their platform-specific buffer types
 * before passing to the strategy layer.
 */
struct AudioBufferDescriptor {
    float* buffer;          // Pointer to interleaved float sample data
    int frameCount;         // Number of audio frames available in the buffer
    int channelCount;       // Number of audio channels (typically 2 for stereo)

    AudioBufferDescriptor()
        : buffer(nullptr), frameCount(0), channelCount(0) {}

    AudioBufferDescriptor(float* buf, int frames, int channels)
        : buffer(buf), frameCount(frames), channelCount(channels) {}
};

#endif // AUDIO_TYPES_H
