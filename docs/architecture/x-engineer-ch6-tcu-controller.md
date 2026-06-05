# x-engineer.org Chapter 6: TCU Controller (Transmission Control Unit)

**Source**: https://x-engineer.org/modeling-simulation-vehicle-automatic-transmission/6/
**Retrieved**: 2026-05-20
**Context**: Academic tutorial on modeling automatic transmissions in Scilab/Xcos for a 7-speed automatic transmission (BMW-like).

---

## 1. Overview

Shifting gears in an automatic transmission follows a **shift schedule** designed according to criteria such as fuel economy, dynamic performance, or comfort. Modern automatic transmissions are **adaptive**, meaning the shift scheduling is modified according to driving speed and driving style (economy vs. sport).

A TCU shift scheduler consists of:
1. **Static shift lines (maps)** -- define the right gear for a given speed and throttle position
2. **Dynamic corrections** -- modify the static gear calculation based on the rate of accelerator pedal change and vehicle state (acceleration vs. deceleration)

### Top-Level Architecture

The TCU has two major subsystems:

```
External Inputs: VehV_kph, AccrPedlPosn_prc
         |
         v
    +------------------+         +--------------------+
    |   Shift Lines    |-------->|  Shift Scheduler   |
    |                  |         |                    |
    | Inputs:          |         | Inputs:            |
    |  - VehV_kph      |         |  - VehV_kph       |
    |  - AccrPedlPosn  |         |  - UpShiftLim     |
    |  - GearNr_z (fb) |         |  - DownShiftLim   |
    |                  |         |  - AccrPedlPosn    |
    | Outputs:         |         |                    |
    |  - UpShiftLim    |         | Outputs:           |
    |  - DownShiftLim  |         |  - GearNr_z        |
    +------------------+         |  - GearNrRaw_z     |
                                 +--------------------+
         ^                               |
         |         GearNr_z (feedback)   |
         +-------------------------------+
```

The feedback loop (`GearNr_z` back into Shift Lines) is critical: the shift limits depend on the *current gear*, so the system must know what gear it is in to look up the correct speed thresholds.

---

## 2. Shift Lines (Maps)

For a given vehicle speed and accelerator pedal (throttle) position, there is exactly **one gear** engaged. The shift lines define which gear the transmission should be in for a particular speed/throttle combination.

For a 7-forward-gear transmission:
- **6 upshift lines** (1->2, 2->3, 3->4, 4->5, 5->6, 6->7)
- **6 downshift lines** (7->6, 6->5, 5->4, 4->3, 3->2, 2->1)

### Key Concept: Hysteresis

- The **upshift line** is the **maximum vehicle speed** for a given throttle position in a given gear. Crossing it triggers an upshift.
- The **downshift line** is the **minimum vehicle speed** for a given throttle position in a given gear. Crossing it triggers a downshift.
- **Upshift lines lie above downshift lines** at the same throttle position, creating a **hysteresis band**. This prevents gear hunting (rapid oscillation between gears).

### Shift Lines Block Diagram

The shift lines use **2-D lookup tables** with bilinear interpolation:

```
Inputs:
  AccrPedlPosn_prc  --> [Upshift 2D Lookup]  --> VehUpShiftLim_kph
  GearNr_z          --> [Downshift 2D Lookup]--> VehDownShiftLim_kph

Parameters:
  GbxGearNr_z_X                           : Gear number array (axis) [1..7]
  AccrPedlPosnTrnsUpShift_prc_Y           : Upshift throttle axis [%]
  AccrPedlPosnTrnsDownShift_prc_Y         : Downshift throttle axis [%]
  VehVUpShiftLim_kph_Z                    : Upshift speed map [kph]
  VehVDownShiftLim_kph_Z                  : Downshift speed map [kph]
```

The current gear selects a **column** in the map, the throttle position selects a **row** (with interpolation), and the result is the speed threshold.

---

## 3. Complete Shift Data Tables

### 3.1 Upshift Lines [kph]

Each column is the upshift speed threshold for the transition *into* that gear (i.e., the speed above which you shift into the next higher gear). Gear 7 values are capped at 350 kph (top speed).

| Throttle [%] | Gear 1->2 | Gear 2->3 | Gear 3->4 | Gear 4->5 | Gear 5->6 | Gear 6->7 | Gear 7 |
|---|---|---|---|---|---|---|---|
| 0     | 12  | 21  | 32  | 45  | 64  | 84  | 350 |
| 21    | 12  | 21  | 32  | 45  | 64  | 86  | 350 |
| 21.01 | 12  | 21  | 32  | 46  | 64  | 86  | 350 |
| 21.02 | 12  | 21  | 32  | 47  | 65  | 86  | 350 |
| 23    | 12  | 21  | 34  | 49  | 68  | 86  | 350 |
| 25    | 12  | 21  | 36  | 52  | 72  | 91  | 350 |
| 28    | 12  | 22  | 39  | 57  | 78  | 99  | 350 |
| 33    | 14  | 25  | 44  | 63  | 87  | 109 | 350 |
| 34    | 14  | 25  | 45  | 66  | 90  | 114 | 350 |
| 34.01 | 15  | 26  | 45  | 67  | 92  | 115 | 350 |
| 38    | 16  | 30  | 52  | 76  | 104 | 131 | 350 |
| 72    | 44  | 75  | 118 | 165 | 227 | 281 | 350 |
| 74    | 45  | 77  | 120 | 169 | 232 | 287 | 350 |
| 76    | 47  | 80  | 123 | 174 | 238 | 294 | 350 |
| 82    | 52  | 86  | 132 | 186 | 255 | 315 | 350 |
| 84    | 54  | 88  | 134 | 190 | 260 | 320 | 350 |
| 85    | 55  | 89  | 135 | 192 | 263 | 320 | 350 |
| 87    | 56  | 91  | 135 | 192 | 263 | 320 | 350 |
| 89    | 58  | 91  | 135 | 192 | 263 | 320 | 350 |
| 100   | 58  | 91  | 135 | 192 | 263 | 320 | 350 |

**Observations from the upshift data:**
- At very low throttle (0-21%), upshift points are very early (12 kph for 1->2)
- At WOT (100%), the 1->2 upshift is at 58 kph, 6->7 at 320 kph
- The data has very fine granularity near throttle transitions (21.00, 21.01, 21.02, 34.00, 34.01) suggesting these are break points where the map changes behavior
- At high throttle (87-100%), all values plateau -- the engine reaches its maximum power band

### 3.2 Downshift Lines [kph]

| Throttle [%] | Gear 2->1 | Gear 3->2 | Gear 4->3 | Gear 5->4 | Gear 6->5 | Gear 7->6 | Gear 7 |
|---|---|---|---|---|---|---|---|
| 0     | 0   | 10  | 16  | 24  | 37  | 50  | 68  |
| 20    | 0   | 10  | 16  | 24  | 37  | 50  | 68  |
| 21    | 0   | 10  | 16  | 24  | 37  | 50  | 68  |
| 23    | 0   | 10  | 16  | 24  | 38  | 51  | 69  |
| 25    | 0   | 10  | 16  | 25  | 39  | 52  | 70  |
| 32    | 0   | 11  | 17  | 27  | 42  | 57  | 73  |
| 35    | 0   | 11  | 18  | 28  | 43  | 59  | 75  |
| 35.01 | 0   | 11  | 18  | 29  | 44  | 59  | 77  |
| 37    | 0   | 11  | 19  | 29  | 45  | 62  | 81  |
| 38    | 0   | 11  | 19  | 30  | 46  | 64  | 83  |
| 54    | 0   | 11  | 24  | 42  | 68  | 91  | 125 |
| 56    | 0   | 11  | 25  | 44  | 70  | 95  | 129 |
| 74    | 0   | 13  | 30  | 58  | 91  | 131 | 188 |
| 78    | 0   | 13  | 31  | 60  | 95  | 138 | 202 |
| 80    | 0   | 13  | 32  | 62  | 97  | 144 | 211 |
| 81    | 0   | 13  | 32  | 64  | 100 | 147 | 215 |
| 86    | 0   | 13  | 35  | 73  | 113 | 161 | 215 |
| 94    | 0   | 13  | 35  | 73  | 113 | 161 | 215 |
| 94.01 | 0   | 17  | 45  | 82  | 126 | 192 | 270 |
| 100   | 0   | 17  | 45  | 82  | 126 | 192 | 270 |

**Observations from the downshift data:**
- Gear 2->1 downshift is always at 0 kph (first gear is held to standstill)
- At WOT (100%), the 2->1 downshift is at 0 kph but the 7->6 downshift is at 270 kph (vs 320 kph upshift = 50 kph hysteresis band)
- There is a dramatic jump at 94.01% throttle (performance/kickdown mode)
- At 0% throttle, all downshifts happen very early (maximum fuel economy / engine braking)

### 3.3 Hysteresis Examples (selected from the data)

Comparing upshift vs downshift at the same throttle positions:

| Throttle | Transition | Upshift [kph] | Downshift [kph] | Hysteresis Gap [kph] |
|---|---|---|---|---|
| 0%   | 1->2 / 2->1 | 12  | 0   | 12  |
| 0%   | 2->3 / 3->2 | 21  | 10  | 11  |
| 0%   | 6->7 / 7->6 | 84  | 50  | 34  |
| 50%* | 1->2 / 2->1 | ~30 | 0   | ~30 |
| 100% | 1->2 / 2->1 | 58  | 0   | 58  |
| 100% | 6->7 / 7->6 | 320 | 270 | 50  |

*interpolated

**Key insight**: The hysteresis band increases with higher throttle and higher gears. At WOT, the 1->2 shift has a 58 kph hysteresis band, while at light throttle it is only 12 kph. This is a fundamental design principle: under hard acceleration, you want the gear held longer to avoid hunting.

---

## 4. Dynamic Corrections

### 4.1 Shift Scheduler Architecture

The shift scheduler applies three dynamic corrections to the raw gear decision from the shift maps:

```
                    +---------------------------+
                    |  Minimum Time in Gear     |
                    |  Correction               |
                    |                           |
 GearNr_z -------->| Inputs: GearNr_z,         |---> GearUpShiftDly_B
 GearNrRaw_z ----->|         GearNrRaw_z       |---> GearDownShiftDly_B
                    | Params: TrnsUpGearMinT=2s |
                    |         TrnsDownGearMinT=1s|
                    +---------------------------+

                    +---------------------------+
                    |  Engine Braking           |
                    |  Correction               |
                    |                           |
 VehV_kph -------->| Inputs: VehV_kph,         |---> GearShiftStopEngBrk_B
 AccrPedlPosn --->>|         AccrPedlPosn_prc  |
                    | Params: MaxPedlEngBrk=1%  |
                    |         MinVehVEngBrk=10  |
                    +---------------------------+

                    +---------------------------+
                    |  Tip In-Out               |
                    |  Correction               |
                    |                           |
 AccrPedlPosn --->>| Inputs: AccrPedlPosn_prc  |---> GearShiftStopTipOut_B
                    | Params: MinGrdt=-10%/s    |---> GearShiftStopTipIn_B
                    |         MaxGrdt=+10%/s    |
                    +---------------------------+

All correction flags feed into:

                    +---------------------------+
                    |  Gear Calculation         |
                    |                           |
 VehV_kph -------->|                           |---> GearNr_z
 UpShiftLim ------>|                           |
 DownShiftLim --->>|                           |---> GearNrRaw_z
 All correction -->|  flags                    |
 GearNr_z (fb) --->|                           |
                    +---------------------------+
```

### 4.2 Minimum Time in Gear Correction

**Purpose**: Prevent successive upshifts or downshifts by enforcing a minimum dwell time in each gear. Since this simulation model has instant gear changes (no actuation system model), the timer is essential to prevent unrealistic shifting.

**Algorithm**:
1. Compare `GearNr_z` (current confirmed gear) vs `GearNrRaw_z` (raw gear from maps)
2. If `GearNrRaw_z > GearNr_z` (upshift detected), start the upshift counter
3. If `GearNrRaw_z < GearNr_z` (downshift detected), start the downshift counter
4. Each counter counts down from the configured minimum time
5. While the counter is active, `GearUpShiftDly_B` or `GearDownShiftDly_B` is set, **blocking** further shifts in that direction
6. Cross-reset: An upshift resets the downshift timer, and vice versa

**Parameters**:

| Parameter | Value | Unit | Description |
|---|---|---|---|
| TrnsUpGearMinT_s_C | **2** | s | Minimum time in gear after an upshift |
| TrnsDownGearMinT_s_C | **1** | s | Minimum time in gear after a downshift |

**Important**: Upshifts require 2 seconds (longer dwell), downshifts only 1 second. This asymmetry reflects real transmission behavior -- upshifts are more disruptive and should be less frequent.

### 4.3 Engine Braking Correction

**Purpose**: Prevent unwanted upshifts when the driver fully releases the accelerator pedal. Without this correction, releasing the throttle at high speed would cause the transmission to shift to the highest gear (e.g., 3rd to 7th at 100 kph), which is undesirable because the driver loses engine braking capability.

**Algorithm**:
```
IF AccrPedlPosn_prc < AccrPedlPosnMaxEngBrk_prc_C   (pedal near zero)
AND VehV_kph >= VehVMaxEngBrk_kph_C                  (vehicle still moving)
THEN GearShiftStopEngBrk_B = TRUE                     (block all shifts)
ELSE GearShiftStopEngBrk_B = FALSE                    (allow shifts)
```

**Logic**: Two comparators feed an AND gate:
- `AccrPedlPosn_prc < 1%` -- driver has released the pedal
- `VehV_kph >= 10 kph` -- vehicle is still moving at meaningful speed

When **both** conditions are true, shifts are blocked, keeping the current gear for engine braking.

**Parameters**:

| Parameter | Value | Unit | Description |
|---|---|---|---|
| AccrPedlPosnMaxEngBrk_prc_C | **1** | % | Maximum pedal position for engine braking mode |
| VehVMaxEngBrk_kph_C | **10** | kph | Minimum vehicle speed for engine braking correction |

### 4.4 Tip In-Out Correction

**Purpose**: Prevent gear shifts when the driver suddenly presses (tip-in) or releases (tip-out) the accelerator pedal. These rapid movements indicate transient driver intent and shifting during them causes poor drivability.

**Algorithm**:
1. Calculate the **gradient** (rate of change) of the accelerator pedal position:
   ```
   pedalGradient = d(AccrPedlPosn_prc) / dt
   ```
2. Compare against two calibrated thresholds:
   - **Tip-out** (sudden release): `pedalGradient < -10 %/s` => set `GearShiftStopTipOut_B`
   - **Tip-in** (sudden press): `pedalGradient > +10 %/s` => set `GearShiftStopTipIn_B`
3. When either flag is active, the corresponding shifts are inhibited

**Note from the tutorial**: The tip-in correction is simplified. A production system would consider engine power reserve -- if the engine lacks sufficient power reserve to provide meaningful acceleration, a tip-in should *allow* a downshift. The tip-out correction, however, is correct as-is: a sudden release means the driver wants to slow down, so holding gear for engine braking is appropriate.

**Parameters**:

| Parameter | Value | Unit | Description |
|---|---|---|---|
| AccrPedlMinGrdt_prcps_C | **-10** | %/s | Minimum pedal gradient (tip-out threshold) |
| AccrPedlMaxGrdt_prcps_C | **+10** | %/s | Maximum pedal gradient (tip-in threshold) |

**Note**: The tutorial states these values are small for visualization purposes. Real production values would be **5-10x higher** (i.e., -50 to -100 %/s and +50 to +100 %/s).

---

## 5. Gear Calculation Algorithm

The gear calculation is the final decision block that integrates shift lines with all dynamic corrections.

### Complete Algorithm (Pseudocode)

```
INPUTS:
  VehV_kph            -- current vehicle speed
  VehUpShiftLim_kph   -- upshift speed limit (from 2D shift map lookup)
  VehDownShiftLim_kph -- downshift speed limit (from 2D shift map lookup)
  GearUpShiftDly_B    -- upshift blocked by min-time-in-gear (TRUE=blocked)
  GearDownShiftDly_B  -- downshift blocked by min-time-in-gear (TRUE=blocked)
  GearShiftStopEngBrk_B -- shifts blocked by engine braking (TRUE=blocked)
  GearShiftStopTipOut_B -- shifts blocked by tip-out (TRUE=blocked)
  GearShiftStopTipIn_B  -- shifts blocked by tip-in (TRUE=blocked)

STATE:
  GearNr_z (current gear, with 1-sample-time delay for algebraic loop avoidance)

OUTPUTS:
  GearNr_z     -- confirmed current gear
  GearNrRaw_z  -- raw gear (before correction application, for min-time detection)

ALGORITHM:

// UP SHIFT LOGIC
upshiftCondition = (VehV_kph > VehUpShiftLim_kph)
upshiftAllowed = NOT(GearUpShiftDly_B)
               AND NOT(GearShiftStopEngBrk_B)
               AND NOT(GearShiftStopTipOut_B)
               AND NOT(GearShiftStopTipIn_B)

IF upshiftCondition AND upshiftAllowed THEN
    GearNrRaw_z = GearNr_z + 1
END IF

// DOWN SHIFT LOGIC
downshiftCondition = (VehV_kph < VehDownShiftLim_kph)
downshiftAllowed = NOT(GearDownShiftDly_B)

IF downshiftCondition AND downshiftAllowed THEN
    GearNrRaw_z = GearNr_z - 1
END IF

// CONFIRM GEAR (with sample delay)
GearNr_z = delay(GearNrRaw_z, 1 sample)
```

### Key Design Decisions

1. **Sequential only**: The algorithm only shifts one gear at a time (no skip-shifts like 6->4). The tutorial notes that real transmissions can skip gears during heavy braking.

2. **Correction flags are all active-high blockers**: When any correction flag is TRUE, it blocks the corresponding shift direction. The NOT gates in the block diagram invert these to create "allowed" signals.

3. **Upshifts have 4 safety gates**: speed condition + NOT(upshift delay) + NOT(engine braking) + NOT(tip-out) + NOT(tip-in). All five must be true for an upshift.

4. **Downshifts have only 2 safety gates**: speed condition + NOT(downshift delay). Engine braking and tip corrections do NOT block downshifts.

5. **Algebraic loop avoidance**: The current gear has a one-sample-time delay. This prevents the feedback loop (gear -> shift map -> speed limit -> gear calculation -> gear) from creating a circular dependency.

---

## 6. Comparison with Our AutomaticGearbox Implementation

### 6.1 Feature Mapping

| Feature | x-engineer TCU | Our AutomaticGearbox | Assessment |
|---|---|---|---|
| **Shift map structure** | Separate upshift/downshift 2D tables (throttle x gear -> speed) | Single upshift-only table (throttle x fromGear -> speed) with hysteresis multiplier | We compute downshift as `upshiftSpeed * hysteresisFactor`. They use genuinely separate maps with different data. Their approach is more flexible and realistic. |
| **Interpolation** | 2D bilinear (Scilab/Xcos `interpolate2d`) | 1D linear along throttle axis per gear pair | Functionally similar for our use case. |
| **Number of gears** | 7 forward gears, 6 shift lines | 8 forward gears (ZF 8HP45), 7 shift lines | We support one more gear. |
| **Hysteresis** | Separate upshift and downshift maps with different data at every throttle point | Single hysteresis factor (0.85) applied to upshift table to derive downshift points | Their approach is richer. Our factor-based approach approximates this but cannot capture the throttle-dependent hysteresis band width visible in their data. |
| **Min time in gear** | Separate upshift (2s) and downshift (1s) timers with cross-reset | Single `minShiftIntervalS` (3s) applied uniformly | Our approach is simpler but less nuanced. Their asymmetric timing (2s up / 1s down) reflects real physics better. Their cross-reset logic (upshift resets downshift timer) is also more sophisticated. |
| **Engine braking** | Explicit correction: blocks all shifts when pedal < 1% AND speed >= 10 kph | `coastDownShiftsEnabled` with `coastDownThrottleThreshold` (5%) and `coastDownSpeedMultiplier` (1.15) | Different philosophy. Theirs blocks shifts entirely during engine braking. Ours *allows* coast-down shifts but at higher speed thresholds (1.15x multiplier). |
| **Tip-in/out** | Gradient-based detection with thresholds (-10/+10 %/s), blocks shifts | `kickdownDelta` (0.4 in 100ms window) for tip-in only; `upshiftSuppression` on throttle decrease (-0.05 delta) | Our kickdown is related to their tip-in but uses absolute delta instead of gradient. Our upshift suppression covers the tip-out case but with simpler logic (delta threshold, not gradient). |
| **Kickdown** | Not a separate mechanism (tip-in correction is simplified) | Explicit kickdown system with throttle threshold (0.95), delta detection (0.4/100ms), hold logic, cooldown (500ms), redline hold | Our kickdown is more sophisticated than the tutorial's simplified tip-in. We hold gear through kickdown, they block shifts. |
| **Gear skipping** | Sequential only (1->2->3, not 1->3) | Sequential for normal shifts; `findSafeGear()` can skip during kickdown | Our kickdown can skip gears (e.g., 4->2). The tutorial explicitly notes this is a simplification. |
| **Redline protection** | Not explicitly shown (shift maps implicitly prevent it) | Explicit `redlineForcedUpshiftEnabled` at 6175 RPM with tolerance band and post-upshift hold-off | Our explicit redline protection adds a safety net the tutorial model lacks (their maps are designed to never exceed redline). |
| **Throttle smoothing** | Not shown (raw pedal used) | EMA smoothing with configurable tau (50ms) | Our addition for noise rejection. |

### 6.2 Structural Differences

**Their approach: Correction flags block the gear calculation**
- The gear calculation always runs (comparing speed to limits)
- Correction flags are boolean inhibitors that gate the result
- Multiple corrections can be active simultaneously
- Flags are independent and ANDed together

**Our approach: Integrated logic with state management**
- Shift decisions are computed inline with corrections woven into the control flow
- Kickdown, redline, coast-down, and upshift suppression are separate code paths
- We use a `while` loop for multi-gear shifts; they do single-gear incremental shifts

### 6.3 Parameter Comparison

| Parameter | x-engineer | Our ZF 8HP45 profile | Notes |
|---|---|---|---|
| Min upshift interval | 2.0 s | 3.0 s (uniform) | We are more conservative |
| Min downshift interval | 1.0 s | 3.0 s (uniform) | We are much more conservative on downshifts |
| Engine braking pedal threshold | 1% | 5% (coastDownThrottle) | Ours is less aggressive |
| Engine braking min speed | 10 kph | N/A (standstill at 1 kph) | Different mechanism |
| Tip-in gradient threshold | +10 %/s (demo; real ~+50-100) | kickdownDelta=0.4/100ms = ~4 %/s | Ours is more sensitive |
| Tip-out gradient threshold | -10 %/s (demo; real ~-50-100) | upshiftSuppression delta = -0.05 | Not directly comparable (delta vs gradient) |
| Throttle smoothing | Not applied | tau=50ms EMA | Our addition |
| Redline protection | Implicit in maps | Explicit 6175 RPM | Our safety net |

### 6.4 Potential Improvements from This Research

1. **Separate upshift/downshift maps**: Our single-table-with-hysteresis-factor approach works but cannot capture the asymmetric, throttle-dependent hysteresis bands visible in the x-engineer data. Consider migrating to separate maps.

2. **Asymmetric minimum shift intervals**: Their 2s up / 1s down split is physically motivated. Our uniform 3s is unnecessarily conservative for downshifts.

3. **Gradient-based tip detection**: Computing `dThrottle/dt` as a true gradient rather than using a delta-window would be more robust and align with industry practice.

4. **Correction flag architecture**: Restructuring to compute the raw gear from maps first, then apply correction flags as inhibitors, would be cleaner and more testable. Our current approach mixes decision and correction logic.

5. **Algebraic loop awareness**: Their explicit 1-sample delay on the gear output is a pattern we should be aware of if we ever run the gearbox model in a tight simulation loop with the shift map lookup.

6. **Engine braking as shift inhibitor**: Their approach (block ALL shifts during engine braking) vs our approach (allow coast-down shifts at higher speed) reflects different tuning philosophies. Consider making this configurable.

---

## 7. All Extracted Parameter Values (Consolidated)

### Calibration Constants

| Parameter Name | Value | Unit | Context |
|---|---|---|---|
| TrnsUpGearMinT_s_C | 2 | s | Min time in gear after upshift |
| TrnsDownGearMinT_s_C | 1 | s | Min time in gear after downshift |
| AccrPedlPosnMaxEngBrk_prc_C | 1 | % | Max pedal for engine braking |
| VehVMaxEngBrk_kph_C | 10 | kph | Min speed for engine braking |
| AccrPedlMinGrdt_prcps_C | -10 | %/s | Tip-out gradient threshold |
| AccrPedlMaxGrdt_prcps_C | +10 | %/s | Tip-in gradient threshold |

### Shift Map Dimensions

- **Upshift map**: 20 throttle breakpoints x 7 gear columns = 140 data points
- **Downshift map**: 20 throttle breakpoints x 7 gear columns = 140 data points
- **Total calibration data**: 280 data points

### Throttle Breakpoints (shared for both maps)

Upshift throttle axis: 0, 21, 21.01, 21.02, 23, 25, 28, 33, 34, 34.01, 38, 72, 74, 76, 82, 84, 85, 87, 89, 100

Downshift throttle axis: 0, 20, 21, 23, 25, 32, 35, 35.01, 37, 38, 54, 56, 74, 78, 80, 81, 86, 94, 94.01, 100

The fine granularity at certain throttle positions (21.00, 21.01, 21.02 and 34.00, 34.01 for upshifts; 35.00, 35.01 and 94.00, 94.01 for downshifts) indicates calibration zones where shift behavior changes significantly over a narrow throttle range.
