// NullAudioHardwareProvider.h - Headless no-op audio hardware provider.
//
// Implements IAudioHardwareProvider without any real audio backend, so tests can
// build a seed/headless session (including the Q2 hot-swap path) without CoreAudio.
// Every method is a no-op; getHardwareState() reports not-initialized / not-playing.

#ifndef NULL_AUDIO_HARDWARE_PROVIDER_H
#define NULL_AUDIO_HARDWARE_PROVIDER_H

#include "hardware/IAudioHardwareProvider.h"

class NullAudioHardwareProvider : public IAudioHardwareProvider {
public:
    bool initialize(const AudioStreamFormat&) override { return true; }
    void cleanup() override {}

    bool startPlayback() override { return true; }
    void stopPlayback() override {}

    void setVolume(double) override {}
    double getVolume() const override { return 0.0; }

    bool registerAudioCallback(const AudioCallback&) override { return true; }

    AudioHardwareState getHardwareState() const override { return AudioHardwareState{}; }
    void resetDiagnostics() override {}
};

#endif // NULL_AUDIO_HARDWARE_PROVIDER_H
