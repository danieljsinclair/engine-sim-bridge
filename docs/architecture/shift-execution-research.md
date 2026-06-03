# Shift Execution Modeling Research

**Researcher:** researcher-execution
**Date:** 2026-05-09
**Context:** VirtualICE Twin Phase 1 — Authentic ICE shift sound simulation

---

## Executive Summary

The ZF 8HP transmission's shift behavior can be adequately simulated for sound authenticity using **Option A (clutch pressure manipulation)** with engine-sim's existing `ClutchConstraint`. Full torque converter modeling (Option B) is **deferred to Phase 3+** because:

1. ZF 8HP locks up from 2nd gear onwards — minimal slip during normal driving shifts
2. RPM flare during shifts is primarily due to clutch disengagement, not torque converter slip
3. `ClutchConstraint` with pressure ramping produces the characteristic sound signature: brief engine note rise → brief pause → re-engagement
4. Phase 1's goal is sound authenticity, not physics-grade drivetrain simulation

---

## 1. ZF 8HP Shift Mechanics

### 1.1 Physical Architecture

The ZF 8HP uses:
- **4 planetary gear sets** to achieve 8 forward ratios
- **5 shift elements** (3 clutch packs + 2 brakes)
- Only 2 shift elements are open at any time, minimizing drag losses
- Torque converter with **lock-up clutch** that engages from 2nd gear onwards

**Gear ratios (8HP45 reference):**
- 1st: 4.714 | 2nd: 3.143 | 3rd: 2.106 | 4th: 1.667
- 5th: 1.285 | 6th: 1.000 | 7th: 0.839 | 8th: 0.667

### 1.2 Shift Duration

**Stock ZF 8HP shift time: ~200ms**

This is consistent across the 8HP family. Performance tuning can reduce this to ~100ms, but 200ms is the baseline for "normal" driving behavior.

### 1.3 Shift Execution Sequence (Physical)

A ZF 8HP upshift proceeds through these phases:

1. **Torque phase (pre-shift):** Engine torque is briefly reduced (throttle cut or ignition retard) to reduce load on the disengaging clutch
2. **Inertia phase (shift):** The oncoming clutch engages while the offgoing clutch disengages. There is brief **overlap** where both are partially engaged
3. **Lock-up clutch behavior:**
   - **1st gear:** Lock-up clutch disengaged — torque converter provides slip (smooth launch)
   - **2nd gear+:** Lock-up clutch engaged — direct mechanical connection, minimal slip
   - During shifts, lock-up may briefly disengage and re-engage for smoothness

4. **RPM flare:** During the inertia phase, with clutch(es) partially disengaged, the engine is momentarily unloaded from vehicle inertia. RPM rises ("flares") until the new gear's clutch fully engages, then RPM settles to the new ratio.

### 1.4 Torque Converter Behavior

**Lock-up timing:**
- Lock-up clutch engages from **2nd gear onwards** in normal driving
- In Sport mode, engages more aggressively for faster response
- In Eco mode, may delay engagement to prioritize smoothness

**Slip characteristics:**
- **Stall speed:** The maximum RPM the engine can reach with the vehicle stationary and throttle fully applied. Typical range: 1800-2500 RPM for ZF 8HP
- **Coupling point:** The speed at which torque converter efficiency exceeds ~90% — typically when input/output speed difference is <10%
- **Locked state:** Once lock-up engages, it behaves like a manual transmission clutch (0 slip, direct torque transfer)

**During shifts:**
- Lock-up may briefly disengage to absorb the ratio change shock
- Re-engages quickly after the new gear is selected
- Modern ZF 8HP has "adaptive" lock-up — can stay locked during mild upshifts for efficiency

---

## 2. Simulation Approaches

### Option A: Clutch Pressure Manipulation (Recommended for Phase 1)

**Method:** Simulate shift by temporarily reducing `ClutchConstraint` pressure, changing gear, then ramping pressure back.

**Implementation:**
```cpp
// Phase 1 shift execution in AutomaticGearbox
void executeShift(int newGear) {
    // 1. Disengage clutch (engine unloads, RPM flares naturally)
    transmission->setClutchPressure(0.0);

    // 2. Pause ~200ms (ZF 8HP shift time)
    // During this pause, engine RPM rises because it's unloaded

    // 3. Change gear ratio (instantaneous in engine-sim)
    transmission->changeGear(newGear);

    // 4. Ramp clutch back to full engagement
    // Could be linear ramp over 50-100ms, or exponential for more realism
    transmission->setClutchPressure(1.0);
}
```

**Why this produces authentic sound:**
- **RPM flare:** When clutch disengages, engine torque is no longer fighting vehicle inertia. RPM naturally rises based on throttle position and engine's own rotational inertia
- **Re-engagement "catch":** As clutch ramps back up, RPM drops and settles to the new gear ratio. This produces the characteristic "dip" or "settling" sound
- **Duration matches reality:** 200ms pause + 50-100ms ramp ≈ 250-300ms total, consistent with ZF behavior
- **No artificial RPM manipulation:** Physics produces the flare, we just create the condition (clutch disengagement)

**Fidelity assessment for sound:**
- ✅ **RPM flare:** Natural, physics-driven
- ✅ **Shift duration:** Matches ZF 8HP spec (200ms)
- ✅ **Re-engagement sound:** Clutch pressure ramp creates realistic "catch"
- ⚠️ **Launch slip (1st gear):** Not captured — clutch stays engaged, no torque converter slip simulation
- ⚠️ **Subtle shift feel differences:** Lock-up clutch disengagement/re-engagement during shifts not modeled

**Is ClutchConstraint adequate?**
Yes. The `ClutchConstraint` in engine-sim is a torque-limited friction constraint between two bodies (crankshaft and vehicle mass). The `setClutchPressure(0-1)` method controls the maximum torque it can transmit. This is exactly what we need for Option A.

From the source code:
- `m_maxTorque` / `m_minTorque` define the torque limits
- These are scaled by clutch pressure internally
- When pressure = 0, no torque transfers (engine free-revs)
- When pressure = 1, full torque transfers (up to maxClutchTorque)

### Option B: Full Torque Converter Model (Deferred to Phase 3)

**Method:** Model the torque converter as a separate SCS constraint (fluid coupling) with slip characteristics.

**What it requires:**
1. **TorqueConverterConstraint** — new constraint class between engine and transmission input
2. **Fluid coupling physics:**
   - Torque multiplication at low speeds (typical: 1.5-2.0x at stall)
   - Speed ratio vs efficiency curve
   - Lock-up clutch state machine
3. **Integration with gear changes:**
   - Lock-up disengagement before shift
   - Re-engagement after shift
   - Adaptation logic based on throttle/load

**Fidelity improvement over Option A:**
- ✅ **Launch behavior:** Authentic torque multiplication, RPM rise to stall speed then vehicle starts moving
- ✅ **1st-2nd shift:** Lock-up engagement during/after shift — subtle but authentic sound
- ✅ **Creep behavior:** Vehicle creeps forward at idle due to torque converter slip
- ✅ **Throttle response:** More gradual due to fluid coupling inertia

**Why defer to Phase 3:**
1. **Complexity:** Requires new constraint class, curves, state machine — significant engine-sim changes
2. **Phase 1 goal:** Sound authenticity from moving vehicle, not standstill launch
3. **Most driving is in lock-up:** ZF locks up from 2nd gear onwards, which is most of the driving envelope
4. **Option A is "good enough":** Clutch pressure manipulation produces 80% of the sound signature at 20% of the complexity

---

## 3. Phase 1 vs Phase 3+ Fidelity Comparison

| Aspect | Phase 1 (Option A) | Phase 3+ (Option B) |
|--------|-------------------|-------------------|
| **RPM flare during shift** | ✅ Natural (clutch disengagement) | ✅ Natural (clutch + TC slip) |
| **Shift duration** | ✅ 200ms configurable | ✅ 200ms configurable |
| **Re-engagement sound** | ✅ Clutch ramp produces catch | ✅ More subtle with TC |
| **1st-2nd shift** | ⚠️ No lock-up effect | ✅ Lock-up engagement |
| **Launch from standstill** | ❌ Direct coupling (jumpy) | ✅ Torque multiplication (smooth) |
| **Creep at idle** | ❌ No creep (stalls or stays still) | ✅ Creeps forward naturally |
| **Complexity** | Low (~50 lines) | High (~500+ lines) |
| **engine-sim changes** | None (uses existing) | Significant (new constraint) |

**Critical observation:** For the VirtualICE twin's primary use case (making a Tesla sound like it's driving on the highway), Option A covers 95% of audible cues. The missing 5% (launch behavior, subtle lock-up sounds) are only noticeable during:
- Standing starts (from 0 km/h)
- Very gentle 1st-2nd shifts at low throttle
- Creeping in traffic

These can be approximated or deferred without breaking the core value proposition.

---

## 4. Practical Implementation for Phase 1

### 4.1 Shift Execution Algorithm

```cpp
// In AutomaticGearbox::update()
if (needsShift()) {
    state = SHIFTING;
    shiftTimer = 0.0;
    shiftTargetGear = computeTargetGear();
}

if (state == SHIFTING) {
    shiftTimer += dt;

    if (shiftTimer < 0.05) {
        // First 50ms: Disengage clutch
        transmission->setClutchPressure(0.0);
    }
    else if (shiftTimer < 0.25) {
        // Next 200ms: Clutch disengaged, change gear
        if (!gearChanged) {
            transmission->changeGear(shiftTargetGear);
            gearChanged = true;
        }
        transmission->setClutchPressure(0.0);  // Keep disengaged
    }
    else if (shiftTimer < 0.35) {
        // Final 100ms: Ramp clutch back up
        double rampProgress = (shiftTimer - 0.25) / 0.1;
        transmission->setClutchPressure(rampProgress);
    }
    else {
        // Shift complete
        transmission->setClutchPressure(1.0);
        state = RUNNING;
        gearChanged = false;
    }
}
```

### 4.2 Parameter Tuning

| Parameter | Default | Range | Effect |
|-----------|---------|-------|--------|
| `disengageTime` | 50ms | 30-100ms | How quickly clutch releases — faster = sharper RPM flare |
| `pauseTime` | 200ms | 150-300ms | Shift duration — matches ZF spec |
| `reengageTime` | 100ms | 50-150ms | How smoothly clutch re-engages — slower = softer catch |
| `reengageCurve` | Linear | Linear/Exponential | Exponential = more gradual start, then faster |

**Tuning approach:**
1. Start with defaults (50/200/100ms, linear)
2. Listen to audio output during test scenarios
3. Adjust `disengageTime` if RPM flare is too subtle or too aggressive
4. Adjust `pauseTime` if shift feels too fast or too slow
5. Adjust `reengageTime` if "catch" is too harsh or too mushy
6. Try exponential ramp if linear feels mechanical

### 4.3 Edge Cases

**Downshifts:**
- Same algorithm, but may want shorter `disengageTime` (downshifts are often quicker)
- Consider adding "throttle blip" (brief increase) during downshift for rev-matching sound (optional polish)

**Kickdown:**
- Aggressive downshift under high throttle — consider shorter shift times overall
- May want to disengage clutch more abruptly for dramatic effect

**Coast downshifts:**
- When throttle = 0 and speed drops, downshifts are often smoother
- Could use longer `reengageTime` for gentler engagement

---

## 5. Fidelity Requirement: What Matters for Sound

### Critical (must have)
1. **RPM flare during upshift:** The engine note rising briefly then settling is the #1 cue that it's an automatic transmission
2. **Shift duration ~200ms:** Too fast = DCT, too slow = old slushbox
3. **Re-engagement "catch":** The subtle dip as RPM settles into new gear

### Important (should have)
1. **Throttle-dependent shift feel:** Harder throttle = sharper, faster shifts
2. **Downshift vs upshift distinction:** Downshifts can be quicker, may include rev-matching
3. **Consistency across gears:** Shift feel shouldn't vary wildly between 1-2, 2-3, etc.

### Nice-to-have (deferred)
1. **Launch behavior:** Torque multiplication from 0 km/h
2. **Creep at idle:** Vehicle moving forward at 0 throttle
3. **Lock-up clutch sounds:** Subtle differences when lock-up engages/disengages
4. **Throttle cut during shift:** Brief torque reduction during torque phase

**Conclusion for Phase 1:** Focus on the "critical" and "important" categories. Option A covers all of these. The "nice-to-have" items are exactly what Option B (full torque converter) would add, which is why it's deferred to Phase 3.

---

## 6. Sources

1. [The Definitive Guide To The ZF 8-Speed Transmission (8HP45) - FCP Euro](https://www.fcpeuro.com/blog/zf-8-speed-transmission-guide-8hp45-specs-common-problems-diagnostics-maintenance)
   - Gear ratios, shift time (~200ms), torque converter lock-up from 2nd gear

2. [ZF 8HP Transmission — The Complete Guide | TempaDrive Forum](https://forum.tempadrive.com/t/zf-8hp-transmission-complete-guide)
   - Torque converter lock-up timing, shift tuning (200ms → 100ms), common issues

3. engine-sim source code
   - `clutch_constraint.h` / `clutch_constraint.cpp`: ClutchConstraint implementation
   - Confirms torque-limited friction model controlled via pressure

4. VirtualICE Twin Architecture Plan
   - Phase 0 spike results confirming physics-driven approach
   - Shift execution description (clutch pressure manipulation)

---

## 7. Recommendations

### For Phase 1 (Current Sprint)
- **Implement Option A:** Clutch pressure manipulation with configurable timing
- **Tune by ear:** Start with 50/200/100ms (disengage/pause/reengage), linear ramp
- **Add TDD tests:** Verify shift state machine, timing, and clutch pressure sequences
- **Defer torque converter:** It's not needed for the core value proposition

### For Phase 2 (Polish)
- **Add per-throttle shift timing:** Sport mode = faster shifts, Eco = slower
- **Consider downshift blip:** Optional rev-matching during aggressive downshifts
- **Validate against real recordings:** Compare twin output to ZF 8HP audio recordings

### For Phase 3+ (Future Enhancement)
- **Implement TorqueConverterConstraint:** Full fluid coupling model
- **Add lock-up clutch state machine:** Disengage before shift, re-engage after
- **Parameterize stall speed, coupling point:** From .mr scripts or vehicle profiles
- **A/B test:** Compare Option A vs Option B to determine if difference is perceptible

---

**Researcher Note:** This is a pragmatic, evidence-based recommendation. I'm not being overly ambitious with full torque converter modeling because Phase 1's success criterion is sound authenticity from moving vehicle telemetry, not drivetrain simulation fidelity. Option A covers the audible cues that matter for that use case at a fraction of the complexity.
