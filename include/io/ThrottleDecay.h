// ThrottleDecay.h - Shared throttle decay utility
// Used by all input providers (KeyboardInputProvider, IOSInputProvider, etc.)
// DRY: single source of truth for the decay formula

#ifndef THROTTLE_DECAY_H
#define THROTTLE_DECAY_H

#include <algorithm>

namespace input {
namespace ThrottleDecay {

// Decay throttle toward baseline when above it.
// factor=0.5 gives rapid decay (~50% per tick), matching CLI behavior.
inline double decay(double current, double baseline, double factor = 0.5) {
    return current > baseline ? std::max(baseline, current * factor) : current;
}

}} // namespace input::ThrottleDecay

#endif // THROTTLE_DECAY_H
