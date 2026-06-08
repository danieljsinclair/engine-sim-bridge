#include "input/IgnitionInput.h"

namespace input {

IgnitionInput::IgnitionInput()
    : ignitionOn_(true) {
}

void IgnitionInput::setOn(bool on) {
    ignitionOn_ = on;
}

bool IgnitionInput::isOn() const {
    return ignitionOn_;
}

void IgnitionInput::toggle() {
    ignitionOn_ = !ignitionOn_;
}

} // namespace input
