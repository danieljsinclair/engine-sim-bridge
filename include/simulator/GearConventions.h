#ifndef GEAR_CONVENTIONS_H
#define GEAR_CONVENTIONS_H

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

} // namespace bridge
#endif
