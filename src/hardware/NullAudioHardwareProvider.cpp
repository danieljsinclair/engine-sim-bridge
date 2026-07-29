// NullAudioHardwareProvider.cpp - Headless no-op audio hardware provider.
// See include/hardware/NullAudioHardwareProvider.h.

#include "hardware/NullAudioHardwareProvider.h"

// All methods are no-ops; the .cpp exists so the provider is linkable into the
// bridge library as a concrete translation unit (no inline definition needed).

bool NullAudioHardwareProvider::initialize(const AudioStreamFormat&) {
    return true;
}

bool NullAudioHardwareProvider::startPlayback() {
    return true;
}

double NullAudioHardwareProvider::getVolume() const {
    return 0.0;
}

bool NullAudioHardwareProvider::registerAudioCallback(const AudioCallback&) {
    return true;
}

AudioHardwareState NullAudioHardwareProvider::getHardwareState() const {
    return AudioHardwareState{};
}
