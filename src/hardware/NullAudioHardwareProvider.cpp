// NullAudioHardwareProvider.cpp - Headless no-op audio hardware provider.
// See include/hardware/NullAudioHardwareProvider.h.

#include "hardware/NullAudioHardwareProvider.h"

// All methods are no-ops; the .cpp exists so the provider is linkable into the
// bridge library as a concrete translation unit (no inline definition needed).
