#ifndef GEAR_SELECTOR_INPUT_H
#define GEAR_SELECTOR_INPUT_H

#include "simulator/GearConventions.h"

namespace input {

// Manages gear selector state (P/R/N/D). Pure state object — no input source knowledge.
// Consumer calls shiftUp()/shiftDown() to change state.
class GearSelectorInput {
public:
    GearSelectorInput();

    void shiftUp();      // P→R→N→D (clamped at D)
    void shiftDown();    // D→N→R→P (clamped at P)
    int getState() const;

private:
    static constexpr int PRNDL_ORDER[] = {
        static_cast<int>(bridge::GearSelector::PARK),
        static_cast<int>(bridge::GearSelector::REVERSE),
        static_cast<int>(bridge::GearSelector::NEUTRAL),
        static_cast<int>(bridge::GearSelector::DRIVE)
    };
    static constexpr int PRNDL_COUNT = 4;

    int gearSelector_;

    int prndlIndex() const;
};

} // namespace input

#endif
