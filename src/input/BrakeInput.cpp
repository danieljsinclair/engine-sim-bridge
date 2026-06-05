#include "input/BrakeInput.h"
#include <algorithm>

namespace input {

BrakeInput::BrakeInput(int holdFrames)
    : holdFrames_(holdFrames) {
}

void BrakeInput::setLevel(double level) {
    lastLevel_ = std::clamp(level, 0.0, 1.0);
    framesSinceSet_ = 0;
}

double BrakeInput::pollLevel() {
    if (framesSinceSet_ < holdFrames_) {
        ++framesSinceSet_;
        return lastLevel_;
    }
    lastLevel_ = 0.0;
    return 0.0;
}

} // namespace input
