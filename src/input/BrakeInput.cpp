#include "input/BrakeInput.h"
#include <algorithm>

namespace input {

void BrakeInput::setLevel(double level) {
    level_ = std::clamp(level, 0.0, 1.0);
}

double BrakeInput::pollLevel() const {
    return level_;
}

} // namespace input
