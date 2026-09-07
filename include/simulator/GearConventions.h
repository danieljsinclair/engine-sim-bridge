#ifndef GEAR_CONVENTIONS_H
#define GEAR_CONVENTIONS_H

#include <string>

namespace bridge {

// Gear selector positions (PRND + manual 1-8)
enum class GearSelector : int {
    PARK    = -2,
    REVERSE = -1,
    NEUTRAL =  0,
    DRIVE   = 99,
    // 1-8 = manual/semi-auto gear selection (same values as BridgeGear)
};

// Bridge convention: 0=neutral, 1=1st, 2=2nd...
enum class BridgeGear : int {
    NEUTRAL = 0,
    FIRST   = 1,
    SECOND  = 2,
    THIRD   = 3,
    FOURTH  = 4,
    FIFTH   = 5,
    SIXTH   = 6,
    SEVENTH = 7,
    EIGHTH  = 8
};

// Engine-sim convention: -1=neutral, 0=1st, 1=2nd...
enum class EngineSimGear : int {
    NEUTRAL = -1,
    FIRST   = 0,
    SECOND  = 1,
    THIRD   = 2,
    FOURTH  = 3,
    FIFTH   = 4,
    SIXTH   = 5,
    SEVENTH = 6,
    EIGHTH  = 7
};

// Convert bridge gear to engine-sim gear
inline EngineSimGear toEngineSim(BridgeGear g) {
    return static_cast<EngineSimGear>(static_cast<int>(g) - 1);
}

// Convert engine-sim gear to bridge gear
inline BridgeGear toBridge(int rawEngineSimGear) {
    if (rawEngineSimGear == -1) return BridgeGear::NEUTRAL;
    return static_cast<BridgeGear>(rawEngineSimGear + 1);
}

// ---------------------------------------------------------------------------
// Gear-naming conventions (F4 consolidation): the single-character glyphs of
// the console gear readout "[selector][mode][gear]". Moved verbatim from the
// CLI's ConsolePresentation so the mapping has one home; the CLI keeps thin
// call-throughs at its presentation seam. Every cell of these tables —
// including the '?' invalid branches — is pinned by characterization nets.
// Selector encoding: PARK=-2, REVERSE=-1, NEUTRAL=0, manual digits 1-8,
// DRIVE=99.

// Field 1: gear-selector glyph. P/R/N/D; manual gear-selection digits 1-8
// render as their digit (DRIVE=99 never collides); any other value '?'.
inline char gearSelectorChar(int selector) {
    switch (static_cast<GearSelector>(selector)) {
        case GearSelector::PARK:    return 'P';
        case GearSelector::REVERSE: return 'R';
        case GearSelector::NEUTRAL: return 'N';
        case GearSelector::DRIVE:   return 'D';
        default:
            // Manual gear-selection positions share the BridgeGear numbering
            // (FIRST=1 .. EIGHTH=8). DRIVE is 99, so these never collide.
            // All of 1-8 render as their digit; previously '1' fell through to '?'.
            if (selector >= 1 && selector <= 8) {
                return static_cast<char>('0' + selector);
            }
            return '?';
    }
}

// Field 3 under AUTO: what the transmission is actually doing (P/R/N/1-8).
// PARK/REVERSE come from the selector (the physics has no reverse/park
// gear); NEUTRAL/DRIVE/forward reflect the physical gear number.
inline char gearChar(int selector, int physicalGear) {
    switch (static_cast<GearSelector>(selector)) {
        case GearSelector::PARK:    return 'P';   // transmission parked/locked
        case GearSelector::REVERSE: return 'R';
        default: break;                            // NEUTRAL/DRIVE/manual -> physical gear
    }
    if (physicalGear == 0) return 'N';
    if (physicalGear >= 1 && physicalGear <= 8) return static_cast<char>('0' + physicalGear);
    return '?';
}

// Field 3 under MANUAL: an engaged gear (1-8) mirrors its digit; P/R/N are
// engaged transmission states (not gears) and mirror too; DRIVE in manual
// means NO gear is selected yet — '-' rather than an echoed 'D' that read
// like a gear ("DMD" was widely misread as gear "D").
inline char manualGearChar(int selector) {
    switch (static_cast<GearSelector>(selector)) {
        case GearSelector::PARK:    return 'P';
        case GearSelector::REVERSE: return 'R';
        case GearSelector::NEUTRAL: return 'N';
        case GearSelector::DRIVE:   return '-';   // no gear selected in manual
        default: break;                           // FIRST=1 .. EIGHTH=8 render as digits
    }
    if (selector >= 1 && selector <= 8) {
        return static_cast<char>('0' + selector);
    }
    return '?';
}

// "[selector][mode][gear]" composite (no framing). Manual shows what the
// driver has engaged (manualGearChar); auto derives it from the physical
// gear via gearChar.
inline std::string gearTriple(int selector, bool autoMode, int physicalGear) {
    const char field1 = gearSelectorChar(selector);
    const char field2 = autoMode ? 'A' : 'M';
    const char field3 = autoMode ? gearChar(selector, physicalGear) : manualGearChar(selector);
    return std::string(1, field1) + field2 + field3;
}

} // namespace bridge
#endif
