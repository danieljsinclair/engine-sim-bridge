#include "input/GearSelectorInput.h"

namespace input {

GearSelectorInput::GearSelectorInput()
    : gearSelector_(static_cast<int>(bridge::GearSelector::NEUTRAL)) {
}

int GearSelectorInput::prndlIndex() const {
    for (int i = 0; i < PRNDL_COUNT; ++i) {
        if (PRNDL_ORDER[i] == gearSelector_) return i;
    }
    return 2; // Default to NEUTRAL
}

void GearSelectorInput::shiftUp() {
    int idx = prndlIndex();
    if (idx < PRNDL_COUNT - 1) {
        gearSelector_ = PRNDL_ORDER[idx + 1];
    }
}

void GearSelectorInput::shiftDown() {
    int idx = prndlIndex();
    if (idx > 0) {
        gearSelector_ = PRNDL_ORDER[idx - 1];
    }
}

int GearSelectorInput::getState() const {
    return gearSelector_;
}

} // namespace input
