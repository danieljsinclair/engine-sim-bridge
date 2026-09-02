// CsvGearCoercion.h — shared reverse-gear coercion for the CSV telemetry paths.
//
// An 'R' stalk command is only honoured while the vehicle is GENUINELY
// reversing (road speed clearly negative, below the reverse-active threshold).
// A standstill 'R' (or a contradictory forward 'R' — the car actually moving
// forward) must NOT select REVERSE gear: at a standstill there is nothing to
// reverse into, and a forward 'R' row is a corrupted/contradictory signal.
// Such rows map to PARK (true standstill) or NEUTRAL (creeping forward), never
// REVERSE.
//
// This replaces the iter2 standstill-only coercion (|speed| < 1 km/h), which
// leaked RAR frames: em-dinner.csv carries ~5k 'R' rows at near-standstill
// negative speeds (-2..-1 km/h) that the 1 km/h window failed to catch. Only a
// clearly-reversing speed (< kReverseActiveSpeedKmh) keeps REVERSE.
//
// Shared by BOTH the live (LiveTelemetryProvider) and replay
// (ReplayTelemetryProvider) paths so the two transports apply identical
// coercion. Extracted from LiveTelemetryProvider.cpp:227-235 (the live-only
// original); the replay path previously forwarded the raw selector, so a replay
// of the same capture could select REVERSE where the live path coerced to
// PARK/NEUTRAL.

#ifndef CSV_GEAR_COERCION_H
#define CSV_GEAR_COERCION_H

#include "simulator/GearConventions.h"

namespace input {

// A recorded 'R' row only keeps REVERSE when the vehicle is genuinely
// reversing (clearly-negative road speed). At or above this threshold the
// selector is coerced to PARK (standstill/sentinel) or NEUTRAL (creeping
// forward). Negative so that small negative creep (a true backing maneuver)
// keeps REVERSE.
constexpr double kReverseActiveSpeedKmh = -3.5;

// Apply reverse coercion to a parsed CSV gear selector, given the sample's
// road speed. Returns the (possibly coerced) selector. REVERSE is preserved
// only when roadSpeedKmh < kReverseActiveSpeedKmh; otherwise a standstill 'R'
// becomes PARK and a forward 'R' becomes NEUTRAL.
inline bridge::GearSelector coerceCsvReverseGear(bridge::GearSelector sel,
                                                  double roadSpeedKmh) {
    if (sel == bridge::GearSelector::REVERSE &&
        roadSpeedKmh >= kReverseActiveSpeedKmh) {
        // Not genuinely reversing: a forward 'R' (speed > 0) is a contradictory
        // signal -> NEUTRAL; standstill / sentinel 'R' -> PARK. Never REVERSE.
        sel = (roadSpeedKmh > 0.0) ? bridge::GearSelector::NEUTRAL
                                   : bridge::GearSelector::PARK;
    }
    return sel;
}

}  // namespace input

#endif  // CSV_GEAR_COERCION_H
