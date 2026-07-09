# Gearbox Data Model Redesign

**Date:** 2026-05-19
**Purpose:** Define data-driven gearbox configuration for VirtualICE twin automatic transmission
**Design Goal:** All gearbox characteristics are captured as data values, not imperative code.

---

## Executive Summary

The current `IceVehicleProfile` struct captures basic gearbox parameters but has several hardcoded behaviors in `AutomaticGearbox.cpp` that should be data-driven. This redesign introduces new fields to support:

1. Redline forced upshift
2. Kickdown hold behavior
3. Throttle-decreasing upshift suppression
4. Gear-dependent hysteresis
5. Coast-down downshift scheduling
6. Throttle-dependent shift timing
7. Separate upshift/downshift timing
8. Named RUNNING→IDLE transition thresholds

The design maintains backward compatibility through sensible defaults and keeps the data model flat and readable.

---

## Current State Analysis

### Existing IceVehicleProfile Fields

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `gearRatios` | `vector<double>` | {4.714, 3.143, ...} | Gear ratios for 1-8 gears |
| `diffRatio` | `double` | 3.15 | Final drive ratio |
| `tireRadiusM` | `double` | 0.32 | Tire radius in meters |
| `vehicleMassKg` | `double` | 1800.0 | Vehicle mass in kg |
| `shiftTable` | `vector<vector<double>>` | 5x7 table | Upshift speeds [throttle][gear] |
| `hysteresisFactor` | `double` | 0.85 | Downshift speed multiplier |
| `kickdownThrottleThreshold` | `double` | 0.95 | Throttle % for kickdown |
| `kickdownDelta` | `double` | 0.4 | Throttle delta for kickdown |
| `kickdownWindowMs` | `double` | 100.0 | Window for throttle delta |
| `shiftDisengageMs` | `double` | 50.0 | Clutch disengage time |
| `shiftPauseMs` | `double` | 200.0 | Shift pause duration |
| `shiftReengageMs` | `double` | 100.0 | Clutch reengage time |
| `throttleSmoothingTauMs` | `double` | 50.0 | Throttle smoothing time constant |
| `minShiftIntervalS` | `double` | 3.0 | Minimum time between shifts |
| `redlineRpm` | `double` | 6500.0 | Engine redline RPM |
| `idleRpm` | `double` | 750.0 | Engine idle RPM |
| `throttleIdleThreshold` | `double` | 0.05 | IDLE→RUNNING transition |
| `idleThrottle` | `double` | 0.0 | No throttle at idle |
| `standstillThresholdKmh` | `double` | 1.0 | Standstill speed threshold |

### Hardcoded Behaviors in AutomaticGearbox.cpp

1. **Line 54:** `speedKmh < profile_.standstillThresholdKmh` — blocks all shifts at standstill
2. **Line 218:** `rpm <= profile_.redlineRpm * 0.9` — safe gear uses 90% of redline (magic number)
3. **Line 197-200:** Kickdown only checks throttle threshold and delta — no hold behavior
4. **Line 37-45:** Throttle delta tracking for kickdown — no upshift suppression
5. **Line 105:** All gears use same `hysteresisFactor` — no gear-dependent values
6. **Line 84-99:** Upshifts use same timing for all throttle levels
7. **Line 103-118:** Downshifts use same timing as upshifts
8. **Line 151:** Hardcoded throttle levels `{0.1, 0.25, 0.5, 0.75, 1.0}` — not configurable
9. **Line 198:** `throttleFraction >= profile_.kickdownThrottleThreshold` — no release threshold
10. **Line 203:** `throttleDeltaHistory_ >= profile_.kickdownDelta` — no cooldown or hysteresis

---

## New Data Model Design

### New Fields by Gap

#### Gap 1: Redline Forced Upshift

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `redlineForcedUpshiftEnabled` | `bool` | false | Force upshift at redline regardless of throttle |
| `redlineUpshiftRpm` | `double` | `redlineRpm` | RPM threshold for forced upshift (defaults to redlineRpm) |
| `redlineUpshiftTolerance` | `double` | 100.0 | RPM tolerance (prevents hunting near redline) |

**Rationale:** Some transmissions force an upshift at redline to prevent engine damage, even under WOT. The tolerance prevents immediate downshift after upshift.

#### Gap 2: Kickdown Hold Behavior

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `kickdownHoldEnabled` | `bool` | true | Hold gear after kickdown until condition met |
| `kickdownHoldThrottleReleaseThreshold` | `double` | 0.80 | Throttle must drop below this to release hold |
| `kickdownHoldToRedlineEnabled` | `bool` | true | Hold gear until redline when kickdown active |
| `kickdownCooldownMs` | `double` | 500.0 | Minimum time before another kickdown can trigger |

**Rationale:** Real TCUs hold the gear after kickdown until the driver lifts or redline is reached. The cooldown prevents rapid kickdown cycling.

#### Gap 3: Throttle-Decreasing Upshift Suppression

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `upshiftSuppressionEnabled` | `bool` | true | Suppress upshifts when throttle decreasing |
| `upshiftSuppressionThrottleDeltaThreshold` | `double` | -0.05 | Throttle delta per frame to trigger suppression |
| `upshiftSuppressionWindowMs` | `double` | 200.0 | Window over which to measure throttle delta |

**Rationale:** Don't upshift into deceleration — wait until throttle stabilizes or increases.

#### Gap 4: Gear-Dependent Hysteresis

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `hysteresisPerGearEnabled` | `bool` | false | Use per-gear hysteresis values |
| `hysteresisFactors` | `vector<double>` | {0.85, 0.85, ...} | Per-gear hysteresis (index = gear-1) |

**Rationale:** Lower gears need more hysteresis (larger ratio steps), higher gears need less. When disabled, falls back to `hysteresisFactor`.

#### Gap 5: Coast-Down Downshift Schedule

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `coastDownShiftsEnabled` | `bool` | true | Enable separate coast-down downshift logic |
| `coastDownShiftTable` | `vector<vector<double>>` | {} | Optional coast-down speeds [throttle][gear] |
| `coastDownSpeedMultiplier` | `double` | 1.15 | Multiplier for powered downshifts (earlier coast-down shifts) |

**Rationale:** Coast-down downshifts happen earlier than powered downshifts. Can be configured via separate table or simple multiplier.

#### Gap 6: Throttle-Dependent Shift Timing

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `throttleDependentTimingEnabled` | `bool` | true | Vary shift timing by throttle |
| `timingThrottleLevels` | `vector<double>` | {0.2, 0.5, 0.8, 1.0} | Throttle levels for timing interpolation |
| `timingDisengageMs` | `vector<double>` | {50.0, 40.0, 30.0, 25.0} | Disengage times per throttle |
| `timingPauseMs` | `vector<double>` | {250.0, 200.0, 150.0, 120.0} | Pause times per throttle |
| `timingReengageMs` | `vector<double>` | {120.0, 100.0, 80.0, 60.0} | Reengage times per throttle |

**Rationale:** Harder throttle = faster shifts. ZF 8HP shifts are ~200ms normally, ~100ms in Sport mode.

#### Gap 7: Upshift vs Downshift Timing

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `separateDownshiftTimingEnabled` | `bool` | true | Use different timing for downshifts |
| `downshiftDisengageMs` | `vector<double>` | {40.0, 35.0, 25.0, 20.0} | Disengage times for downshifts |
| `downshiftPauseMs` | `vector<double>` | {180.0, 150.0, 120.0, 100.0} | Pause times for downshifts |
| `downshiftReengageMs` | `vector<double>` | {100.0, 80.0, 60.0, 50.0} | Reengage times for downshifts |

**Rationale:** Downshifts are often quicker and may have different clutch profiles (rev-matching).

#### Gap 8: RUNNING→IDLE Transition Thresholds

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `throttleRunningToIdleThreshold` | `double` | 0.02 | Throttle below this = RUNNING→IDLE |
| `throttleIdleToRunningThreshold` | `double` | 0.05 | Throttle above this = IDLE→RUNNING |

**Rationale:** Hysteresis prevents rapid IDLE↔RUNNING transitions at low throttle. Currently uses `throttleIdleThreshold` only.

### Additional Configuration Fields

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `shiftTableThrottleLevels` | `vector<double>` | {0.1, 0.25, 0.5, 0.75, 1.0} | Throttle levels for shift table interpolation |
| `safeDownshiftRedlineFactor` | `double` | 0.9 | Fraction of redline for safe gear calculation (was hardcoded) |

---

## Complete Updated IceVehicleProfile Struct

```cpp
struct IceVehicleProfile {
    // === Existing fields ===
    std::vector<double> gearRatios;
    double diffRatio = 3.15;
    double tireRadiusM = 0.32;
    double vehicleMassKg = 1800.0;
    std::vector<std::vector<double>> shiftTable;
    double hysteresisFactor = 0.85;
    double kickdownThrottleThreshold = 0.95;
    double kickdownDelta = 0.4;
    double kickdownWindowMs = 100.0;
    double shiftDisengageMs = 50.0;
    double shiftPauseMs = 200.0;
    double shiftReengageMs = 100.0;
    double throttleSmoothingTauMs = 50.0;
    double minShiftIntervalS = 3.0;
    double redlineRpm = 6500.0;
    double idleRpm = 750.0;
    double throttleIdleThreshold = 0.05;     // IDLE → RUNNING transition (5%)
    double idleThrottle = 0.0;               // No throttle injection
    double standstillThresholdKmh = 1.0;     // Below this speed = standstill

    // === New fields: Redline forced upshift ===
    bool redlineForcedUpshiftEnabled = false;
    double redlineUpshiftRpm = 6500.0;       // Defaults to redlineRpm
    double redlineUpshiftTolerance = 100.0;

    // === New fields: Kickdown hold behavior ===
    bool kickdownHoldEnabled = true;
    double kickdownHoldThrottleReleaseThreshold = 0.80;
    bool kickdownHoldToRedlineEnabled = true;
    double kickdownCooldownMs = 500.0;

    // === New fields: Throttle-decreasing upshift suppression ===
    bool upshiftSuppressionEnabled = true;
    double upshiftSuppressionThrottleDeltaThreshold = -0.05;
    double upshiftSuppressionWindowMs = 200.0;

    // === New fields: Gear-dependent hysteresis ===
    bool hysteresisPerGearEnabled = false;
    std::vector<double> hysteresisFactors;  // Per-gear hysteresis (index = gear-1)

    // === New fields: Coast-down downshift schedule ===
    bool coastDownShiftsEnabled = true;
    std::vector<std::vector<double>> coastDownShiftTable;  // Optional separate table
    double coastDownSpeedMultiplier = 1.15;  // Earlier downshifts when coasting

    // === New fields: Throttle-dependent shift timing ===
    bool throttleDependentTimingEnabled = true;
    std::vector<double> timingThrottleLevels;      // {0.2, 0.5, 0.8, 1.0}
    std::vector<double> timingDisengageMs;         // Per throttle level
    std::vector<double> timingPauseMs;             // Per throttle level
    std::vector<double> timingReengageMs;          // Per throttle level

    // === New fields: Upshift vs downshift timing ===
    bool separateDownshiftTimingEnabled = true;
    std::vector<double> downshiftDisengageMs;      // Per throttle level
    std::vector<double> downshiftPauseMs;          // Per throttle level
    std::vector<double> downshiftReengageMs;       // Per throttle level

    // === New fields: RUNNING→IDLE transition thresholds ===
    double throttleRunningToIdleThreshold = 0.02;  // RUNNING → IDLE
    double throttleIdleToRunningThreshold = 0.05;  // IDLE → RUNNING

    // === New fields: Shift table configuration ===
    std::vector<double> shiftTableThrottleLevels;  // {0.1, 0.25, 0.5, 0.75, 1.0}
    double safeDownshiftRedlineFactor = 0.9;      // Safe gear calculation (was 0.9 hardcoded)

    IceVehicleProfile() = default;

    // ... constructor would need to be updated with new fields ...

    static IceVehicleProfile zf8hp45();
};
```

---

## ZF 8HP45 Factory Configuration

### Research-Based Values (Where Available)

The following values are based on the ZF shift scheduling and execution research documents. Values marked **TODO** require web researcher verification.

```cpp
static IceVehicleProfile zf8hp45() {
    IceVehicleProfile p;

    // === Existing ZF 8HP45 values ===
    p.gearRatios = {4.714, 3.143, 2.106, 1.667, 1.285, 1.000, 0.839, 0.667};
    p.diffRatio = 3.15;
    p.tireRadiusM = 0.32;
    p.vehicleMassKg = 1800.0;
    p.shiftTable = {
        {20.0, 35.0, 50.0, 65.0, 80.0, 95.0, 110.0},  // 10% throttle
        {30.0, 50.0, 70.0, 90.0, 110.0, 130.0, 155.0}, // 25% throttle
        {40.0, 65.0, 90.0, 115.0, 140.0, 170.0, 200.0},// 50% throttle
        {55.0, 85.0, 115.0, 145.0, 180.0, 215.0, 255.0},// 75% throttle
        {70.0, 105.0, 140.0, 180.0, 220.0, 265.0, 315.0}// 100% throttle
    };
    p.hysteresisFactor = 0.85;
    p.kickdownThrottleThreshold = 0.95;
    p.kickdownDelta = 0.4;
    p.kickdownWindowMs = 100.0;
    p.shiftDisengageMs = 50.0;
    p.shiftPauseMs = 200.0;
    p.shiftReengageMs = 100.0;
    p.throttleSmoothingTauMs = 50.0;
    p.minShiftIntervalS = 3.0;
    p.redlineRpm = 6500.0;
    p.idleRpm = 750.0;
    p.throttleIdleThreshold = 0.05;
    p.idleThrottle = 0.0;
    p.standstillThresholdKmh = 1.0;

    // === Redline forced upshift ===
    p.redlineForcedUpshiftEnabled = true;        // TODO: Verify ZF behavior
    p.redlineUpshiftRpm = p.redlineRpm;          // Use same as redline
    p.redlineUpshiftTolerance = 100.0;           // TODO: Verify tolerance

    // === Kickdown hold behavior ===
    p.kickdownHoldEnabled = true;                 // ZF holds gear after kickdown
    p.kickdownHoldThrottleReleaseThreshold = 0.80; // TODO: Verify threshold
    p.kickdownHoldToRedlineEnabled = true;       // ZF holds to redline
    p.kickdownCooldownMs = 500.0;                // TODO: Verify cooldown

    // === Throttle-decreasing upshift suppression ===
    p.upshiftSuppressionEnabled = true;          // ZF suppresses upshifts on decel
    p.upshiftSuppressionThrottleDeltaThreshold = -0.05; // TODO: Verify delta
    p.upshiftSuppressionWindowMs = 200.0;        // TODO: Verify window

    // === Gear-dependent hysteresis ===
    p.hysteresisPerGearEnabled = false;          // Start with simple hysteresis
    // When enabled, would be populated with gear-specific values
    // p.hysteresisFactors = {0.82, 0.84, 0.85, 0.86, 0.87, 0.88, 0.89, 0.90};

    // === Coast-down downshift schedule ===
    p.coastDownShiftsEnabled = true;             // ZF has coast-down logic
    // p.coastDownShiftTable = {};               // TODO: Get coast-down shift table
    p.coastDownSpeedMultiplier = 1.15;           // Earlier downshifts when coasting (TODO: Verify)

    // === Throttle-dependent shift timing ===
    p.throttleDependentTimingEnabled = true;
    p.timingThrottleLevels = {0.2, 0.5, 0.8, 1.0};  // Light, medium, heavy, WOT
    p.timingDisengageMs = {50.0, 40.0, 30.0, 25.0};   // Faster disengage at high throttle (TODO: Verify)
    p.timingPauseMs = {250.0, 200.0, 150.0, 120.0};   // Faster shifts at high throttle (TODO: Verify)
    p.timingReengageMs = {120.0, 100.0, 80.0, 60.0};  // Faster reengage at high throttle (TODO: Verify)

    // === Upshift vs downshift timing ===
    p.separateDownshiftTimingEnabled = true;
    p.downshiftDisengageMs = {40.0, 35.0, 25.0, 20.0};  // Quicker disengage (TODO: Verify)
    p.downshiftPauseMs = {180.0, 150.0, 120.0, 100.0};  // Quicker shifts (TODO: Verify)
    p.downshiftReengageMs = {100.0, 80.0, 60.0, 50.0}; // Quicker reengage (TODO: Verify)

    // === RUNNING→IDLE transition thresholds ===
    p.throttleRunningToIdleThreshold = 0.02;     // TODO: Verify hysteresis
    p.throttleIdleToRunningThreshold = 0.05;     // Matches existing throttleIdleThreshold

    // === Shift table configuration ===
    p.shiftTableThrottleLevels = {0.1, 0.25, 0.5, 0.75, 1.0}; // Matches shiftTable structure
    p.safeDownshiftRedlineFactor = 0.9;         // Was hardcoded, now explicit

    return p;
}
```

---

## Values Requiring Web Research

The following values need verification from ZF 8HP documentation or real vehicle data:

### High Priority

1. **`redlineForcedUpshiftEnabled`** — Does ZF 8HP force upshift at redline under WOT?
2. **`redlineUpshiftTolerance`** — What's the RPM gap before allowing another shift?
3. **`kickdownHoldThrottleReleaseThreshold`** — At what throttle % does kickdown hold release?
4. **`kickdownCooldownMs`** — Minimum time between kickdown triggers?

### Medium Priority

5. **`upshiftSuppressionThrottleDeltaThreshold`** — What delta/frame triggers suppression?
6. **`upshiftSuppressionWindowMs`** — Time window for measuring throttle delta?
7. **`coastDownShiftTable`** — Actual coast-down shift speeds for ZF 8HP45
8. **`coastDownSpeedMultiplier`** — Alternative: multiplier for powered downshifts?

### Lower Priority

9. **Shift timing values** — `timingDisengageMs`, `timingPauseMs`, `timingReengageMs` per throttle
10. **Downshift timing values** — `downshiftDisengageMs`, `downshiftPauseMs`, `downshiftReengageMs`
11. **`throttleRunningToIdleThreshold`** — RUNNING→IDLE hysteresis value

---

## Implementation Notes

### Backward Compatibility

All new fields have sensible defaults that maintain existing behavior:

- **Disabled features** (redline forced upshift, per-gear hysteresis, coast-down table) default to `false` or empty
- **Enabled features** (kickdown hold, upshift suppression, throttle timing) use conservative defaults
- Existing tests pass without modification
- New features can be enabled incrementally

### Constructor Update

The constructor will need to be expanded to include all new fields. Consider:

1. **Option A:** Use builder pattern for complex initialization
2. **Option B:** Use named parameters (C++20 designated initializers)
3. **Option C:** Keep default constructor + setters for new fields

### Data Validation

The gearbox algorithm should validate:

- `hysteresisFactors.size() == gearRatios.size()` when `hysteresisPerGearEnabled == true`
- All vector lengths match (e.g., `timingThrottleLevels.size() == timingDisengageMs.size()`)
- `coastDownShiftTable` dimensions match `shiftTable` when provided
- Thresholds are in valid ranges (0-1 for fractions, positive for times)

### Algorithm Changes (Out of Scope)

The following algorithm changes are NOT part of this design (deferred to implementation):

1. Redline check in `update()` loop
2. Kickdown hold state machine
3. Throttle delta tracking for upshift suppression
4. Hysteresis lookup by gear
5. Coast-down table lookup
6. Timing interpolation by throttle
7. Direction-specific timing selection

---

## Acceptance Criteria

The data model is complete when:

1. [ ] All 8 gaps have corresponding fields in `IceVehicleProfile`
2. [ ] Fields have sensible defaults maintaining existing behavior
3. [ ] `zf8hp45()` factory is populated with researched values where available
4. [ ] TODO comments mark values requiring web research
5. [ ] Data model remains flat and readable (no complex nesting)
6. [ ] No imperative logic in the profile (pure data)
7. [ ] Field names are self-documenting
8. [ ] Units are clear in field names (ms, kmh, rpm, etc.)

---

## Future Extensions

The data model can be extended for:

1. **Driver modes** — Mode-specific threshold offsets (Sport, Economy)
2. **Grade detection** — Uphill/downhill shift modifiers
3. **Torque-aware shifting** — Load-dependent threshold adjustment
4. **Adaptive learning** — Driver behavior pattern adaptation
5. **Transmission temp** — Cold/warm shift timing adjustments

---

## References

- `~/vscode/escli.refac7/engine-sim-bridge/docs/architecture/zf-shift-scheduling-research.md`
- `~/vscode/escli.refac7/engine-sim-bridge/docs/architecture/shift-execution-research.md`
- `~/vscode/escli.refac7/engine-sim-bridge/include/twin/IceVehicleProfile.h`
- `~/vscode/escli.refac7/engine-sim-bridge/src/twin/AutomaticGearbox.cpp`