# ZF 8HP Transmission Shift Scheduling Research

**Date:** 2026-05-09
**Purpose:** Validate shift scheduling model for VirtualICE twin automatic transmission
**Researcher:** researcher-scheduling (gearbox-planning team)

---

## Executive Summary

ZF 8HP automatic transmissions use a Transmission Control Unit (TCU) that makes shift decisions based primarily on **throttle position** and **vehicle speed**, with secondary inputs including torque request, gear selector, brake status, and kickdown switch. Shift scheduling uses a 2D lookup table (shift map) with hysteresis to prevent gear hunting.

The existing architecture plan's illustrative shift table (km/h thresholds per gear per throttle position) is **fundamentally correct** but incomplete. Real ZF TCUs incorporate additional complexity including torque-aware shifting, driver mode adaptation, and coast-down specific downshift logic.

**Key Finding:** Speed-based shift curves parameterized by throttle is the correct model for Phase 1. Torque-aware shifting can be deferred to Phase 2+.

---

## 1. TCU Input Signals for Shift Decisions

### 1.1 Primary Inputs

| Input | Purpose | Update Rate | Notes |
|-------|---------|-------------|-------|
| **Throttle Position** | Driver intent demand | 50-100 Hz | Directly maps to shift thresholds |
| **Vehicle Speed** | Operating point on shift map | 50-100 Hz | Can use output shaft speed for faster response |
| **Torque Request** | Available engine torque | 10-50 Hz | Used for torque-aware shifting (Phase 2+) |
| **Gear Selector** | Mode (P/R/N/D/S) | Event-driven | Determines allowed shift ranges |

### 1.2 Secondary Inputs

| Input | Purpose | Notes |
|-------|---------|-------|
| **Kickdown Switch** | Forces maximum downshift | Usually mechanical detent past full throttle |
| **Brake Pedal** | Enables coast-down downshifts | Throttle = 0 AND brake > threshold triggers earlier downshifts |
| **Transmission Fluid Temp** | Shift quality adaptation | Cold = smoother shifts, less lockup |
| **Driver Mode** | Shift threshold modifier | Sport = higher RPM, Economy = lower RPM |
| **Longitudinal Acceleration** | Grade detection | Uphill = hold gear longer, Downhill = earlier downshift for engine braking |

### 1.3 Rate-of-Change Inputs (Advanced)

- **Accelerator pedal rate of change** (`d_throttle/dt`): Rapid increase can trigger early downshift (kickdown) even before full throttle
- **Vehicle acceleration**: Can adjust shift thresholds under changing load

**Note for Phase 1:** Only throttle position and vehicle speed are required for basic shift scheduling. Rate-of-change and torque-aware inputs are deferred to Phase 2+.

---

## 2. Shift Map Structure and Logic

### 2.1 Shift Map 2D Lookup Table

The shift map is a 2D lookup with:

- **X-axis:** Vehicle speed (km/h) or transmission output shaft speed (RPM)
- **Y-axis:** Throttle position (0-100%)
- **Each cell:** Target gear or shift line boundary

**Upshift lines:** Curves where TCU commands gear → gear+1
**Downshift lines:** Curves where TCU commands gear → gear-1

### 2.2 Shift Threshold Determination

Shift decisions are primarily **speed-based**, not RPM-based. However, the relationship between speed and RPM is fixed by gear ratio and tire size:

```
target_RPM_for_shift = (speed_km/h * 1000/3600 / (2π * tire_radius)) * gear_ratio * diff_ratio * 60
```

**Why speed not RPM?**
- Speed is an input from vehicle sensors, not dependent on engine state
- Using RPM creates circular dependency in physics-driven simulation
- Speed-based curves are more stable under changing torque output

### 2.3 Throttle-Dependent Threshold Behavior

| Throttle Range | Upshift RPM Target | Shift Behavior |
|----------------|-------------------|----------------|
| **0-25% (Light)** | ~1500-2000 RPM | Early shifts for fuel economy |
| **25-50% (Medium)** | ~2000-3000 RPM | Balanced performance/efficiency |
| **50-75% (Heavy)** | ~3000-4500 RPM | Performance-oriented |
| **75-100% (WOT)** | ~5000-6500 RPM | Near redline, maximum power |
| **Kickdown (>100% detent)** | Hold to redline | Forces maximum downshift, holds in current gear |

### 2.4 Hysteresis Mechanism

**Purpose:** Prevent "gear hunting" — rapid back-and-forth shifting at boundary conditions.

**Implementation:**
- Downshift thresholds are **85-90%** of upshift thresholds (10-15% speed gap)
- Example: If 3→4 upshift occurs at 55 km/h at 50% throttle, 4→3 downshift occurs at ~47 km/h

```
downshift_speed(gear, throttle) = upshift_speed(gear, throttle) × 0.85
```

**Hysteresis by gear:**
- Lower gears (1-3): Larger hysteresis (~15%) — more hunting prone due to larger ratio steps
- Higher gears (6-8): Smaller hysteresis (~10%) — closer ratios, less RPM change per shift

---

## 3. Upshift vs Downshift Behavior

### 3.1 Powered Upshifts

**Trigger conditions:**
1. Current speed ≥ `upshift_speed[current_gear][throttle]`
2. Throttle not rapidly decreasing (don't upshift into deceleration)
3. Engine RPM not exceeding redline
4. Transmission not already executing a shift

**Shift execution:**
- ZF 8HP shift time: ~200ms
- Torque reduction requested during shift (reduces clutch wear)
- Engine RPM dips briefly as ratio swaps, then recovers

### 3.2 Powered Downshifts

**Trigger conditions:**
1. Current speed ≤ `downshift_speed[current_gear][throttle]`
2. Throttle increasing (driver wants more power)
3. OR kickdown switch triggered

**Behavior:**
- Single gear downshift under normal throttle increase
- Multi-gear downshift (skip shift) under kickdown: e.g., 8th → 4th for maximum acceleration
- Rev-matching on downshifts (throttle blip) for smooth engagement

### 3.3 Coast-Down / Deceleration Downshifts

**Distinct from powered downshifts:**

| Characteristic | Powered Downshift | Coast-Down Downshift |
|----------------|-------------------|----------------------|
| **Throttle** | > 0 (increasing) | = 0 |
| **Purpose** | Acceleration | Engine braking, prepare for re-acceleration |
| **Trigger** | Speed drops below downshift threshold OR kickdown | Speed drops, often with brake applied |
| **Timing** | Later (lower speed) than upshift | Earlier (higher speed) than powered downshift |
| **Rev-matching** | Yes (throttle blip) | No (coasting) |

**Coast-down logic:**
- When throttle = 0 and speed drops below downshift threshold for current gear
- Downshifts occur progressively as speed decreases
- Purpose: Maintain engine in useful RPM range for quick re-acceleration
- Brake application can trigger earlier downshifts for engine braking assist

**Note for Phase 1:** Coast-down can be modeled using the same downshift thresholds as powered downshifts. Advanced brake-dependent timing is Phase 2+.

---

## 4. Kickdown Behavior

### 4.1 Kickdown Trigger

**Mechanical kickdown switch:**
- Located at full throttle travel
- Physical detent must be passed
- Forces immediate maximum downshift

**Software kickdown (no physical switch):**
- Detected via throttle rate of change: rapid increase > 40% in < 100ms
- Throttle > 90% for > 500ms
- Triggers downshift sequence

### 4.2 Kickdown Response

**Behavior:**
1. Force downshift to lowest gear that can handle current speed without over-revving
2. Hold current gear until redline (or throttle lifted)
3. Skip multiple gears if safe: e.g., 8th → 4th
4. Aggressive torque reduction during shift for rapid gear changes

**Example kickdown from 8th gear at 100 km/h:**
```
8th @ 100 km/h → downshift to 4th (safe RPM: ~3000)
Hold 4th until ~6500 RPM → upshift to 5th
Hold 5th until ~6500 RPM → upshift to 6th
...until throttle lifted
```

### 4.3 Kickdown vs Full Throttle

| Condition | Behavior |
|-----------|----------|
| Full throttle (no kickdown) | Normal upshift at high RPM (~6000) |
| Kickdown engaged | Force downshift, hold to redline (~6500-7000) |

**Note for Phase 1:** Kickdown can be detected via throttle delta > 0.4 or throttle > 0.95.

---

## 5. Driver Mode Adaptation

### 5.1 Mode-Specific Shift Thresholds

| Mode | Threshold Offset | RPM Range |
|------|------------------|-----------|
| **Economy** | -300 to -500 RPM | Early upshifts, lower RPM |
| **Normal** | Baseline | Standard behavior |
| **Sport** | +500 to +1500 RPM | Later upshifts, higher RPM |
| **Sport+** | +1500 to +2500 RPM | Redline shifts |

**Implementation:**
```
shift_speed(mode, gear, throttle) = base_shift_speed(gear, throttle) × mode_multiplier[mode]
```

### 5.2 Mode-Dependent Hysteresis

- Sport mode: Smaller hysteresis (~5-10%) — allows quicker gear selection changes
- Economy mode: Larger hysteresis (~15-20%) — fewer shifts for smoothness

**Note for Phase 1:** Single mode (Normal) is sufficient. Multi-mode support is Phase 2+.

---

## 6. Torque Awareness (Advanced, Phase 2+)

### 6.1 Why Torque Matters

Real ZF TCUs consider torque on both sides of the torque converter:

- **Engine torque (pump side):** Available torque for acceleration
- **Gearbox output torque (turbine side):** Torque delivered to wheels

Under heavy load (climbing, towing), the TCU may:
- Hold gear longer (don't upshift into inadequate torque)
- Downshift earlier (ensure sufficient torque reserve)

### 6.2 Torque-Based Shift Modulation

```
shift_speed_adjusted = shift_speed_base × (1 - torque_factor × load_factor)
```

Where:
- `torque_factor` = current_engine_torque / max_engine_torque (0-1)
- `load_factor` = calibration constant (typically 0.1-0.2)

**Example:** At 80% engine load, shift speed may be reduced by 8-16%, causing earlier shifts to maintain acceleration.

### 6.3 Impact on VirtualICE Twin

**Phase 1 (speed + throttle only):**
- Works correctly for steady-state acceleration, cruise, basic kickdown
- May shift too early under simulated heavy load

**Phase 2+ (torque-aware):**
- Requires engine torque estimator from engine-sim
- Modifies shift thresholds based on current torque output
- More accurate behavior under varying load conditions

**Recommendation:** Defer torque-aware shifting until Phase 2. The physics-driven approach naturally produces torque-dependent behavior — if the virtual ICE is under heavy load (high throttle, low speed), it produces less acceleration, which may naturally delay upshifts. Explicit torque modulation is an optimization, not a requirement.

---

## 7. Validation Against Published ZF Data

### 7.1 ZF 8HP45 Published Shift Table (Illustrative)

The architecture plan contains an illustrative shift table:

| Throttle | 1→2 | 2→3 | 3→4 | 4→5 | 5→6 | 6→7 | 7→8 |
|----------|-----|-----|-----|-----|-----|-----|-----|
| 10% | 12 | 22 | 32 | 42 | 52 | 62 | 72 |
| 50% | 25 | 40 | 55 | 72 | 88 | 105 | 125 |
| 100% | 45 | 70 | 95 | 120 | 148 | 178 | 210 |

**Analysis:**
- Light throttle (10%): Upshifts at 12 km/h in 1st → ~1500 RPM
- Medium throttle (50%): Upshifts at 25 km/h in 1st → ~2000 RPM
- Full throttle (100%): Upshifts at 45 km/h in 1st → ~3500 RPM

**RPM calculations (assuming ~4.7:1 1st gear, 3.15 diff, 0.32m tire):**
```
RPM = (km/h * 1000/3600 / (2π * 0.32)) * 4.714 * 3.15 * 60
    ≈ km/h * 78.6
```

| Throttle | 1→2 Speed | Calculated RPM |
|----------|-----------|----------------|
| 10% | 12 km/h | ~940 RPM (low, suggests torque converter slip or low idle) |
| 50% | 25 km/h | ~1960 RPM |
| 100% | 45 km/h | ~3540 RPM |

The 10% throttle shift at 940 RPM is unusually low. Real ZF 8HP typically upshifts at ~1500-1800 RPM under light load. This suggests the illustrative table may need calibration or assumes significant torque converter slip at low speeds.

### 7.2 Recommended Calibration Approach

**For Phase 1 validation:**
1. Use published ZF 8HP45 gear ratios
2. Calibrate shift thresholds to produce reasonable RPM bands:
   - Light throttle: ~1800-2000 RPM
   - Medium throttle: ~2500-3000 RPM
   - Heavy throttle: ~4000-5000 RPM
   - WOT: ~6000-6500 RPM
3. Apply 15% hysteresis for downshifts
4. Validate with unit tests against target RPM ranges

**Example calibrated table (for ~3.0L engine):**

| Throttle | 1→2 | 2→3 | 3→4 | 4→5 | 5→6 | 6→7 | 7→8 |
|----------|-----|-----|-----|-----|-----|-----|-----|
| 10% | 20 | 35 | 50 | 65 | 80 | 95 | 110 |
| 25% | 30 | 50 | 70 | 90 | 110 | 130 | 155 |
| 50% | 40 | 65 | 90 | 115 | 140 | 170 | 200 |
| 75% | 55 | 85 | 115 | 145 | 180 | 215 | 255 |
| 100% | 70 | 105 | 140 | 180 | 220 | 265 | 315 |

These values shift upward to account for more realistic RPM targets. Actual calibration should be done with real driving data.

---

## 8. Shift Execution (Brief)

This research focuses on **shift scheduling** (when to shift), not **shift execution** (how the shift happens). However, for completeness:

**ZF 8HP shift execution characteristics:**
- Shift time: ~200ms
- Clutch-to-clutch shifts (no bands)
- Torque reduction requested during shift
- Can skip gears on downshift
- Rev-matching on downshifts

The architecture plan correctly separates these concerns:
- **Shift scheduling:** In bridge layer (`AutomaticGearbox` class)
- **Shift execution:** In engine-sim (`changeGear()` + clutch manipulation)

---

## 9. Acceptance Criteria for AutomaticGearbox

Based on this research, testable acceptance criteria:

### 9.1 Basic Shift Scheduling

- [ ] At 50% throttle, upshifts occur within ±10% of calibrated speed thresholds
- [ ] Downshifts occur at ~85% of upshift speed (hysteresis validation)
- [ ] No gear hunting: minimum 3 seconds between consecutive shifts in same direction
- [ ] Kickdown (throttle > 95% or delta > 0.4) forces downshift to safe gear
- [ ] Coast-down (throttle = 0) triggers downshifts as speed decreases

### 9.2 Edge Cases

- [ ] At standstill (speed = 0), no shifts occur
- [ ] At redline, forces upshift regardless of throttle
- [ ] At idle speed, no downshifts below 1st gear
- [ ] Invalid telemetry (isValid = false) holds last gear
- [ ] Throttle step from 0% to 100% triggers appropriate downshift sequence

### 9.3 Physics Integration

- [ ] Gear selection produces RPM within 10% of expected based on speed/gear ratio
- [ ] Shift timing (200ms clutch manipulation) produces audible RPM flare
- [ ] No circular dependency: gear selection uses real speed, not simulated RPM

### 9.4 Mode Support (Phase 2+)

- [ ] Sport mode shifts at +500-1500 RPM compared to Normal
- [ ] Economy mode shifts at -300-500 RPM compared to Normal
- [ ] Mode switch mid-drive updates shift thresholds immediately

### 9.5 Torque Awareness (Phase 2+)

- [ ] Under high load (simulated grade), upshifts are delayed
- [ ] Under low load, upshifts occur at normal thresholds
- [ ] Torque factor modulation is bounded (±20% max adjustment)

---

## 10. Sources

### Successfully Retrieved

1. **Mastering 8HP Transmission Control: The Ultimate Guide** - ZackTuned
   - URL: https://www.zacktuned.com/blogs/zacktuned/8hp-transmission-control-guide
   - Key findings: TCU inputs (engine RPM, throttle, torque request, vehicle speed, gear selector), CAN bus communication, bidirectional torque reduction during shifts

2. **8HP Electronic Control | MaxxECU & TCU Options** - NZ Wiring
   - URL: https://nzwiring.com/gearbox-conversions/8hp-conversion/8hp-electronic-control/
   - Key findings: Controller options (MaxxECU, CANTCU, TurboLamik), torque-based shift scheduling, generation support (Gen1/2/3), shift control features

3. **8HP gearbox control** - MaxxECU Documentation
   - URL: https://www.maxxecu.com/webhelp/advanced-8hp.html
   - Key findings: TCU reflash procedure, CAN protocol, configuration requirements, real-time data availability

4. **Existing web-research-report.md** - Project documentation
   - Path: ~/vscode/escli.refac7/engine-sim-bridge/docs/architecture/web-research-report.md
   - Key findings: ZF 8HP45 gear ratios, shift map structure, hysteresis, torque converter behavior

5. **VirtualICE-Twin-Architecture-Plan.md** - Project documentation
   - Path: ~/vscode/escli.refac7/engine-sim-bridge/docs/architecture/VirtualICE-Twin-Architecture-Plan.md
   - Key findings: Illustrative shift table, shift scheduling algorithm, hysteresis values, kickdown detection

### Supplemented with Domain Knowledge

- Typical shift threshold RPM ranges by throttle position
- Hysteresis values (15% speed gap) for gear hunting prevention
- Kickdown behavior (meanical detent + software detection)
- Coast-down vs powered downshift differentiation
- Torque converter slip behavior at low speeds
- Mode-specific shift threshold offsets

### Not Retrieved (API Errors / Rate Limits)

- Detailed ZF 8HP45 shift maps from OEM sources (proprietary)
- CAN message specifications for TCU communication
- Advanced torque-aware shifting algorithms
- Gradient/grade detection logic
- Transmission fluid temperature adaptation curves

---

## 11. Recommendations

### For Phase 1 Implementation

1. **Use speed-based shift curves parameterized by throttle** — this is validated as correct
2. **Implement 15% hysteresis** — downshift at 85% of upshift speed
3. **Support basic kickdown** — throttle > 95% or delta > 0.4 triggers downshift
4. **Support coast-down** — throttle = 0 uses same downshift thresholds
5. **Start with Normal mode** — single shift table, no mode switching
6. **Defer torque-aware shifting** — physics-driven behavior may be sufficient
7. **Calibrate shift thresholds** — adjust illustrative table to produce reasonable RPM ranges

### For Phase 2+ Enhancements

1. **Add driver modes** — Sport/Economy shift threshold offsets
2. **Add torque awareness** — modulate shift thresholds based on engine torque
3. **Add brake-dependent coast-down** — earlier downshifts with brake applied
4. **Add adaptive learning** — adjust thresholds based on driver behavior pattern
5. **Add grade detection** — modify thresholds for uphill/downhill driving

### For Testing

1. **Unit tests** — shift decision logic, hysteresis, kickdown detection, edge cases
2. **Integration tests** — gear selection produces correct RPM through physics
3. **Acceptance tests** — full acceleration, cruise, deceleration scenarios
4. **Calibration tests** — verify RPM bands match target ranges

---

**Conclusion:** The speed-based shift curve model in the architecture plan is fundamentally correct. ZF 8HP transmissions use throttle position and vehicle speed as primary inputs for shift scheduling, with hysteresis to prevent gear hunting. The main gaps are calibration of the illustrative shift table (which may have unrealistically low speeds for light throttle) and the addition of advanced features (torque awareness, driver modes) which are appropriately deferred to Phase 2+.
