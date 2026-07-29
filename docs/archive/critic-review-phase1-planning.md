# Architecture Critic Review — Phase 1 Planning

**Reviewer:** critic (gearbox-planning team)
**Date:** 2026-05-09
**Files Reviewed:**
- VirtualICE-Twin-Architecture-Plan.md
- phase1-acceptance-criteria.md
- zf-shift-scheduling-research.md
- shift-execution-research.md

---

## Summary

**Result: CONDITIONAL PASS**

The planning is fundamentally sound and demonstrates excellent research depth. The separation of concerns (shift scheduling vs execution) and the calibrated ZF shift table represent significant improvements. However, **one substantive issue** was found that must be addressed before implementation begins, plus several minor inconsistencies that should be corrected.

---

## Issues Found

### Critical Issue

#### AC-03.2 and AC-03.3: Coast-down criteria reference "throttle history" — violates speed-based principle

**Where:** `phase1-acceptance-criteria.md`, lines 80-81

**What:**
```
- **AC-03.2:** At 50% throttle history, 4th→3rd downshift occurs at 76.5 ± 7.7 km/h (85% of upshift threshold, ±10%)
- **AC-03.3:** At 50% throttle history, 3rd→2nd downshift occurs at 55.3 ± 5.5 km/h (85% of upshift, ±10%)
```

**Why it matters:**
1. **Contradicts architecture plan:** The plan explicitly states "Road speed is an input from vehicle-sim, not a physics output — breaks the circular dependency" and uses real EV road speed as the primary input. Coast-down should be speed-based, not throttle-history-based.
2. **Contradicts research:** `zf-shift-scheduling-research.md` Section 3.3 states: "Coast-down logic: When throttle = 0 and speed drops below downshift threshold for current gear." There's no mention of using "throttle history."
3. **Tests implementation details:** "Throttle history" is a mechanism — the observable behavior is that downshifts occur at specific speeds when throttle = 0. The acceptance criteria should test the behavior, not the implementation.
4. **Ambiguous:** What does "50% throttle history" mean? Is it the last throttle value before going to 0? An average over N seconds? Time-decayed? This introduces unnecessary complexity for Phase 1.

**Recommended fix:**
Replace AC-03.2 and AC-03.3 with:
```
- **AC-03.2:** At 76.5 ± 7.7 km/h (85% of 4→3 upshift threshold), 4th→3rd downshift occurs when throttle = 0
- **AC-03.3:** At 55.3 ± 5.5 km/h (85% of 3→2 upshift threshold), 3rd→2nd downshift occurs when throttle = 0
```

This tests the observable behavior (downshift at speed X when throttle = 0) without specifying how "throttle history" should be tracked.

---

### Inconsistencies

#### Inconsistency 1: State machine transition diagram vs text

**Where:** `VirtualICE-Twin-Architecture-Plan.md`, lines 324-330 vs state machine table lines 332-338

**What:**
The state machine diagram shows:
```
OFF → (first telemetry) → CRANKING → (RPM > 550) → IDLE
IDLE → RUNNING (throttle > 5%)
RUNNING → SHIFTING (gear change triggered)
SHIFTING → RUNNING (shift complete)
RUNNING → IDLE (speed → 0 AND throttle → 0)
```

The transition to OFF is shown as `Any → OFF (no telemetry for N seconds)`.

However, AC-11.7 states:
```
- **AC-11.7:** Any state → OFF occurs when no valid telemetry for 5 seconds
```

**Why it matters:** Minor inconsistency — N vs 5 seconds. The acceptance criteria specify 5 seconds, which is testable. The diagram should be updated to match.

**Recommended fix:** Update line 330 in the architecture plan from `Any → OFF (no telemetry for N seconds)` to `Any → OFF (no telemetry for 5 seconds)`.

---

#### Inconsistency 2: Kickdown timing parameter reference

**Where:** `phase1-acceptance-criteria.md`, line 93

**What:**
```
When:** Throttle suddenly increases to 100% (step change > 0.4 delta) within 100ms
```

**Why it matters:** This specifies both a delta threshold (0.4) AND a time window (100ms). However, the research and plan don't specify the 100ms window. `zf-shift-scheduling-research.md` Section 4.2 mentions: "Throttle > 90% for > 500ms" for software kickdown detection, and Section 4.1 mentions "Rapid increase > 40% in < 100ms" — but this is for detection, not the "within 100ms" constraint in the acceptance criteria.

The architecture plan line 289 states: "Kickdown: throttle delta > 0.4 triggers immediate downshift" with no time constraint.

The acceptance criteria is introducing a new constraint: the delta must occur "within 100ms." This may be too restrictive for testing and doesn't align with the plan's simpler "throttle delta > 0.4 triggers immediate downshift."

**Recommended fix:** Clarify whether the 100ms window is a requirement or just an example. If it's a requirement, update the architecture plan to reflect it. If not, simplify AC-04 to: "When: Throttle suddenly increases with delta > 0.4" and let the research document's 100ms detection window be an implementation detail.

---

#### Inconsistency 3: Shift time timing vs range in AC-01.6

**Where:** `phase1-acceptance-criteria.md`, line 50

**What:**
```
- **AC-01.6:** Each shift completes within 250-350ms total (50ms disengage + 200ms pause + 100ms reengage)
```

**Why it matters:** This is a RANGE (250-350ms), not a specific value. However, AC-08.5 (shift execution quality) specifies the same range. The research document `shift-execution-research.md` Section 1.2 states "Stock ZF 8HP shift time: ~200ms" and the practical implementation uses 200ms as the pause time.

The range (250-350ms) accounts for variability in disengage (30-100ms) and reengage (50-150ms) times noted in Section 4.2 of shift-execution-research.md. However, the base ZF shift time is 200ms, not 200ms pause + 100ms reengage.

The inconsistency is subtle: AC-01.6 treats the 200ms pause as the "ZF shift time," but the research identifies the 200ms as the stock ZF shift time for the entire shift. The bridge's additional 50ms disengage and 100ms reengage are simulation-specific timing, not part of the ZF spec.

**Recommended fix:** Clarify in the acceptance criteria that 200ms is the ZF shift time from the spec, and the 250-350ms total includes bridge-specific disengage/reengage ramping. This is more of a documentation clarity issue than a bug.

---

### Architectural Concerns

#### Concern 1: No validation that gear selection produces RPM within expected bands

**Where:** Missing from acceptance criteria

**What:** While AC-07.2 states "Computed RPM from speed/gear ratio matches engine-sim's actual RPM output within ±10% tolerance," this is a physics pipeline correctness test, not a behavioral test.

The calibrated ZF shift table is designed to produce specific RPM bands at each throttle position (light throttle ~1500-2000 RPM, medium ~2500-3000 RPM, heavy ~4000-6500 RPM). However, none of the acceptance criteria verify that the twin actually produces these RPM bands during driving scenarios.

**Why it matters:** If the physics integration has issues (e.g., wrong vehicle mass, drag coefficient, or tire radius), the gear selection might be correct (speed-based) but the RPM output could be wrong, producing unrealistic sound.

**Recommended fix:** Add a criterion under AC-01 (Steady-State Acceleration) or AC-07 (Physics Pipeline):
```
- **AC-XX.1:** At 50% throttle, 1st→2nd upshift occurs at RPM between 2250-2750 RPM (±10% of 2500 RPM target)
- **AC-XX.2:** At 50% throttle, 2nd→3rd upshift occurs at RPM between 2250-2750 RPM (±10% of 2500 RPM target)
```

This validates that the calibrated shift table produces the intended RPM behavior, not just speed thresholds.

---

#### Concern 2: Coast-down criteria don't validate hysteresis

**Where:** `phase1-acceptance-criteria.md`, AC-03

**What:** AC-03.4 states "Downshifts occur later (lower speed) than upshifts for same gear (hysteresis validation)," but this is a qualitative statement, not a quantifiable test.

**Why it matters:** Hysteresis is critical to prevent gear hunting. The acceptance criteria should specify that downshift speeds are 85% of upshift speeds (as specified in AC-10.6), but AC-03 doesn't reference the specific hysteresis factor.

**Recommended fix:** Add to AC-03:
```
- **AC-03.X:** Downshift speeds are 85% of the corresponding upshift speeds (e.g., 4→3 downshift at 76.5 km/h = 85% × 90 km/h 3→4 upshift)
```

---

### Acceptance Criteria Issues

#### Issue 1: AC-03.2 and AC-03.3 reference "throttle history" (already documented as critical issue)

This is the same issue as the critical issue above. It's both a consistency problem (contradicts research) and an acceptance criteria quality issue (tests mechanism, not behavior).

---

#### Issue 2: AC-04.3 doesn't define "safe gear"

**Where:** `phase1-acceptance-criteria.md`, line 97

**What:**
```
- **AC-04.3:** If in 6th gear at 60 km/h, downshift to 3rd or 4th (multi-gear skip allowed)
```

**Why it matters:** "Safe gear" is not defined in the acceptance criteria. AC-04.2 states "Target gear is lowest safe gear (prevents RPM > 90% of redline at current speed)," but what is the exact calculation? The architecture plan doesn't specify this either.

The research document `zf-shift-scheduling-research.md` Section 4.2 states: "Force downshift to lowest gear that can handle current speed without over-revving," but doesn't give a formula.

**Recommended fix:** Either:
1. Specify the formula for "safe gear" in the acceptance criteria, or
2. Reference that the implementation uses a lookup/safety check to ensure RPM < 90% of redline, without specifying the exact algorithm (test behavior, not implementation)

---

#### Issue 3: AC-09 (Throttle Smoothing) tests implementation details

**Where:** `phase1-acceptance-criteria.md`, AC-09

**What:**
```
- **AC-09.1:** Filtered throttle reaches 63.2% of step change within TAU = 50ms (exponential response)
- **AC-09.5:** Filter formula: `filtered += (raw - filtered) × (1 - exp(-dt / 0.05))`
```

**Why it matters:** AC-09.5 tests the exact formula, which is an implementation detail. If the implementation changes to a different filter (e.g., Butterworth, moving average) while still producing the same observable behavior, AC-09.5 would fail.

The general principle from the project instructions is: "tests must test real code used by production, not test mock code, external code or truisms... tests to test specific business scenarios." Testing the exact exponential filter formula is testing an implementation detail.

**Recommended fix:** Remove AC-09.5 or change it to test behavior:
```
- **AC-09.5:** Filtered throttle response is exponential (time-constant ~50ms), producing smooth transitions without overshoot
```

Alternatively, keep AC-09.5 if the exponential filter is a documented interface contract (e.g., specified in the architecture plan as the required algorithm).

---

### Completeness Issues

#### Completeness 1: Q2 status in architecture plan not updated

**Where:** `VirtualICE-Twin-Architecture-Plan.md`, lines 577-583

**What:** Q2 is marked as "RESOLVED" but the resolution text doesn't explicitly state that research has been done and documented. It says: "Research validated that ZF 8HP transmissions use throttle position and vehicle speed as primary inputs... The original illustrative shift table was calibrated upward..."

This is correct, but it doesn't explicitly reference the new research files (`zf-shift-scheduling-research.md` and `shift-execution-research.md`). The resolution should explicitly state that the research is complete and documented.

**Why it matters:** Future readers should know that Q2 is not just "calibrated" but that full research documents exist for reference.

**Recommended fix:** Update Q2 resolution to:
"**RESOLVED — Automatic gearbox shift curve validation**: Research validated that ZF 8HP transmissions use throttle position and vehicle speed as primary inputs for shift scheduling. The original illustrative shift table was calibrated upward to produce realistic RPM bands. Full research documented in `zf-shift-scheduling-research.md` and `shift-execution-research.md`."

---

#### Completeness 2: State machine CRANKING state behavior not fully specified

**Where:** `VirtualICE-Twin-Architecture-Plan.md`, lines 335-336

**What:** The state machine table shows:
```
| CRANKING | 0 | 0.3 | starter ~200 |
```

But the acceptance criteria AC-11.2 states: "CRANKING → IDLE occurs when RPM > 550." What happens in CRANKING state? How long does it last? What if RPM never reaches 550 (e.g., engine won't start)?

**Why it matters:** CRANKING behavior is not specified in the acceptance criteria beyond the transition condition. If the twin gets stuck in CRANKING, what happens?

**Recommended fix:** Add edge case criterion for CRANKING:
```
- **AC-11.X:** CRANKING times out after N seconds (e.g., 5s) if RPM does not reach 550, transitioning to OFF or IDLE
```

Or document that CRANKING timeout is out of scope for Phase 1.

---

## What's Good

1. **Separation of concerns is excellent:** The plan clearly separates shift scheduling (Problem A) from shift execution (Problem B). This was a good improvement and aligns with SRP.

2. **Calibrated shift table is a significant improvement:** The move from illustrative to calibrated values based on realistic RPM bands shows careful research and attention to detail.

3. **Research documents are thorough:** Both `zf-shift-scheduling-research.md` and `shift-execution-research.md` demonstrate deep understanding of ZF 8HP behavior and provide clear, evidence-based recommendations.

4. **Acceptance criteria are mostly well-written:** The shift timing, hysteresis, and kickdown criteria are specific, testable, and align with the research. The manual QA gate is appropriately scoped.

5. **Architecture is SOLID-aligned:**
   - SRP: `VirtualIceTwin` maps telemetry, `AutomaticGearbox` selects gears, `VirtualIceInputProvider` adapts to simulation loop — clear separation.
   - OCP: New vehicle profiles can be added by creating new `IceVehicleProfile` instances, not modifying existing code.
   - DIP: Components depend on `UpstreamSignal` and `TwinOutput` abstractions, not concrete types.

6. **State machine fix is correct:** The unidirectional transition (RUNNING → SHIFTING → RUNNING) is now properly specified, fixing the earlier bidirectional confusion.

7. **Scope is appropriately limited:** Phase 1 excludes torque converter, driver modes, torque-aware shifting — the right MVP scope.

8. **Speed-based approach is validated:** The research confirms that ZF TCUs use speed as primary input, validating the architecture's approach and avoiding circular dependency.

---

## Overall Recommendation

**CONDITIONAL PASS — Address the critical issue before implementation begins.**

The critical issue (AC-03.2 and AC-03.3 referencing "throttle history") must be fixed because it:
1. Contradicts the speed-based principle that the architecture is built on
2. Contradicts the research findings
3. Tests implementation details rather than observable behavior
4. Introduces ambiguity that could lead to implementation disputes

The minor inconsistencies (state machine timing N vs 5 seconds, kickdown 100ms window) should be corrected for clarity but are not blockers.

The architectural concerns (RPM band validation, coast-down hysteresis quantification, "safe gear" definition) are more like suggestions for strengthening the acceptance criteria rather than blocking issues.

**Next steps:**
1. Fix AC-03.2 and AC-03.3 to remove "throttle history" reference
2. Update architecture plan state machine diagram to specify 5 seconds instead of N
3. Clarify kickdown 100ms window requirement (is it a hard constraint or implementation detail?)
4. Consider adding RPM band validation and "safe gear" definition to strengthen acceptance criteria

After these fixes, the planning is ready for implementation.

---

**Review Complete**
