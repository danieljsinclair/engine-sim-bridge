# Phase 1 Acceptance Criteria — VirtualICE Twin

**Version:** 1.1
**Date:** 2026-05-09
**Author:** product-owner (gearbox-planning team)
**Status:** Draft

---

## Phase 1 Scope Reminder

Phase 1 is the MVP: physics-driven RPM from EV telemetry feeding engine-sim's sound engine. Key components:
- Road-speed-based automatic gearbox (ZF 8HP45 reference profile)
- Clutch pressure manipulation for shift execution (disengage → pause → changeGear → ramp)
- Throttle smoothing (low-pass filter)
- State machine (OFF → CRANKING → IDLE → RUNNING ↔ SHIFTING)
- UpstreamSignal from vehicle-sim (throttle, speed)

**Phase 1 EXCLUDES:** Torque converter model, cranking sound modeling, driver modes (Sport/Eco), torque-aware shifting, brake-dependent coast-down, grade detection.

---

## Human Validation Checkpoint (Manual QA Gate)

**The following is NOT tested via TDD but MUST be validated manually by the project owner before Phase 1 is considered complete:**

- [ ] Audio output from the twin sounds like an ICE vehicle during all test scenarios (acceleration, cruise, deceleration, kickdown, standstill)
- [ ] Shift sounds have the characteristic RPM flare followed by settling (no artificial tone changes)
- [ ] There are no audible artifacts (clicks, pops, sudden cuts) during gear changes
- [ ] The ICE sound responds naturally to throttle changes (not delayed, not over-reactive)
- [ ] Different gears produce perceptibly different engine notes (pitch and character)

**Rationale:** "Sounds realistic" is subjective and cannot be automated. This manual QA gate is the final validation that the physics-driven approach produces authentic sound.

---

## Functional Acceptance Criteria

### AC-01: Steady-State Acceleration (0-100 km/h Throttle Ramp)

**Given:** The twin is receiving throttle ramped from 0% to 100% over 30 seconds with corresponding speed increase
**When:** The acceleration completes
**Then:**

- **AC-01.1:** Gear selections occur in sequence: 1st → 2nd → 3rd → 4th → 5th → 6th (minimum 5 upshifts)
- **AC-01.2:** At 50% throttle, 1st→2nd upshift occurs at 40 ± 4 km/h (±10% of calibrated threshold)
- **AC-01.3:** At 50% throttle, 2nd→3rd upshift occurs at 65 ± 6.5 km/h (±10%)
- **AC-01.4:** At 50% throttle, 3rd→4th upshift occurs at 90 ± 9 km/h (±10%)
- **AC-01.5:** At 100% throttle, 1st→2nd upshift occurs at 70 ± 7 km/h (±10%)
- **AC-01.6:** At 50% throttle, upshift RPM at point of gear change falls within 2250-2750 RPM (validates calibrated shift table produces intended RPM band)
- **AC-01.7:** Each shift completes within 250-350ms total (50ms disengage + 200ms pause + 100ms reengage)
- **AC-01.7:** RPM during each shift: flares upward during disengage/pause, then settles to new gear ratio within 100ms after reengage
- **AC-01.8:** No gear hunting: minimum 3 seconds between consecutive upshifts at steady throttle

**Testable via:** `AccelerationScenarioTest` with synthetic telemetry file

---

### AC-02: Highway Cruise (Constant Speed, Constant Throttle)

**Given:** The twin is receiving constant 80 km/h speed and constant 20% throttle for 60 seconds
**When:** The cruise period completes
**Then:**

- **AC-02.1:** Gear selection stabilizes on a single gear (6th or 7th depending on calibration)
- **AC-02.2:** No shifts occur after initial gear selection (steady-state conditions)
- **AC-02.3:** RPM remains within ±5% of the gear's expected RPM for 80 km/h
- **AC-02.4:** Speed variance ≤ 2 km/h produces no gear changes (hysteresis working)

**Testable via:** `CruiseScenarioTest` with synthetic telemetry file

---

### AC-03: Deceleration / Coast-Down (Lift Off at Speed)

**Given:** The twin is receiving 100 km/h speed, then throttle drops to 0% and speed decreases to 0 over 45 seconds
**When:** The deceleration completes
**Then:**

- **AC-03.1:** Downshifts occur sequentially as speed decreases (minimum 4 downshifts: 6th→5th→4th→3rd→2nd)
- **AC-03.2:** At 76.5 ± 7.7 km/h (85% of 3→4 upshift threshold), 4th→3rd downshift occurs when throttle = 0
- **AC-03.3:** At 55.3 ± 5.5 km/h (85% of 2→3 upshift threshold), 3rd→2nd downshift occurs when throttle = 0
- **AC-03.4:** Downshift speeds are 85% of the corresponding upshift speeds for the same gear (hysteresis factor = 0.85)
- **AC-03.5:** No downshift below 1st gear
- **AC-03.6:** At standstill (speed = 0), gear = 1st, throttle = 0%, no further shifts

**Testable via:** `DecelerationScenarioTest` with synthetic telemetry file

---

### AC-04: Kickdown (Sudden Throttle Application at Cruise)

**Given:** The twin is receiving constant 60 km/h speed and constant 20% throttle for 10 seconds
**When:** Throttle suddenly increases to 100% (step change > 0.4 delta) within 100ms
**Then:**

- **AC-04.1:** Downshift to lower gear occurs within 500ms of throttle step
- **AC-04.2:** Target gear produces RPM < 90% of redline at current speed (safe gear — prevents over-rev)
- **AC-04.3:** If in 6th gear at 60 km/h, downshift to 3rd or 4th (multi-gear skip allowed)
- **AC-04.4:** After kickdown, gear is held until throttle decreases below 80% OR speed approaches redline
- **AC-04.5:** Subsequent upshifts occur at higher speeds corresponding to high RPM (≥ 6000 RPM for 100% throttle, typically >140 km/h in calibrated shift table)

**Testable via:** `KickdownScenarioTest` with synthetic telemetry file

---

### AC-05: Standstill → Idle (Stationary, No Throttle)

**Given:** The twin is receiving 0 km/h speed and 0% throttle for 30 seconds
**When:** The idle period completes
**Then:**

- **AC-05.1:** Gear selection = 1st (or designated "P/N/D" neutral state)
- **AC-05.2:** RPM stabilizes at idle (700-800 RPM, per engine profile)
- **AC-05.3:** No shifts occur during idle period
- **AC-05.4:** Throttle input ≤ 5% produces no acceleration from idle (threshold prevents creep)

**Testable via:** `StandstillScenarioTest` with synthetic telemetry file

---

### AC-06: Standstill → Acceleration (From Rest, Throttle Applied)

**Given:** The twin is receiving 0 km/h speed and 0% throttle, then throttle increases to 50%
**When:** The vehicle accelerates from 0 to 60 km/h
**Then:**

- **AC-06.1:** Initial gear = 1st
- **AC-06.2:** 1st→2nd upshift occurs at 40 ± 4 km/h (±10%)
- **AC-06.3:** 2nd→3rd upshift occurs at 65 ± 6.5 km/h (±10%)
- **AC-06.4:** RPM tracks speed increase monotonically within each gear
- **AC-06.5:** No gear selection before speed > 0 (prevents shifts at standstill)

**Testable via:** `LaunchScenarioTest` with synthetic telemetry file

---

## Integration Acceptance Criteria

### AC-07: Physics Pipeline Correctness

**Given:** The twin is receiving any valid telemetry (throttle, speed)
**When:** The twin processes inputs and produces outputs for engine-sim
**Then:**

- **AC-07.1:** Gear selection uses `speedKmh` from UpstreamSignal (directly from vehicle-sim), NOT computed RPM
- **AC-07.2:** Computed RPM from speed/gear ratio matches engine-sim's actual RPM output within ±10% tolerance
- **AC-07.4:** No circular dependency: gear → RPM computation is diagnostic only; physics produces actual RPM independently
- **AC-07.5:** Clutch pressure = 1.0 (locked) during RUNNING state, < 1.0 only during SHIFTING state
- **AC-07.6:** Invalid telemetry (isValid = false) holds last valid gear and ramps throttle to 0 over 1 second

**Testable via:** `TwinPhysicsIntegrationTest`

---

### AC-08: Shift Execution Quality

**Given:** A shift is triggered by the gearbox
**When:** The shift executes through clutch pressure manipulation
**Then:**

- **AC-08.1:** Clutch pressure ramps to 0.0 within 50ms of shift trigger
- **AC-08.2:** Clutch pressure remains at 0.0 for 200ms (ZF 8HP shift time)
- **AC-08.3:** `changeGear(newGear)` is called during the 200ms pause
- **AC-08.4:** Clutch pressure ramps to 1.0 over 100ms after pause completes
- **AC-08.5:** Total shift duration = 250-350ms (50 + 200 + 100ms)
- **AC-08.6:** RPM during shift: rises during disengagement/pause (flare), then settles within 100ms after reengage

**Testable via:** `ShiftExecutionTest`

---

### AC-09: Throttle Smoothing

**Given:** Raw throttle input has step changes (e.g., 0% → 100% instant)
**When:** The twin processes throttle through the low-pass filter
**Then:**

- **AC-09.1:** Filtered throttle reaches 63.2% of step change within TAU = 50ms (exponential response)
- **AC-09.2:** Filtered throttle reaches 95% of step change within 150ms (3 × TAU)
- **AC-09.3:** Steady-state error = 0% (filtered throttle equals raw throttle after 200ms)
- **AC-09.4:** No overshoot (filtered throttle never exceeds raw throttle)
- **AC-09.5:** Response characteristic is exponential with time-constant ~50ms — smooth transitions without overshoot, agnostic to specific filter implementation

**Testable via:** `ThrottleSmootherTest`

---

## Edge Cases and Non-Functional Requirements

### AC-10: Gearbox Edge Cases

**Given:** Various edge case conditions
**When:** The gearbox processes these conditions
**Then:**

- **AC-10.1:** At speed = 0, no shifts occur regardless of throttle
- **AC-10.2:** At speed corresponding to RPM ≥ 95% of redline, upshift is forced even if below speed threshold
- **AC-10.3:** At speed corresponding to RPM ≤ 105% of idle, no downshift occurs below 1st gear
- **AC-10.4:** Kickdown detection triggers when throttle > 0.95 OR throttle delta > 0.4 within 100ms
- **AC-10.5:** Minimum 3 seconds between consecutive shifts in same direction (prevents rapid hunting)
- **AC-10.6:** Hysteresis: downshift threshold = 85% of upshift threshold for same gear/throttle

**Testable via:** `AutomaticGearboxTest` (edge case test suite)

---

### AC-11: State Machine Transitions

**Note:** State machine correctness is a non-functional requirement (NFR) for Phase 1. The following criteria verify state transitions are implemented correctly but do NOT test observable behavior.

**Given:** Various input conditions
**When:** The twin processes inputs through the state machine
**Then:**

- **AC-11.1:** OFF → CRANKING occurs on first valid telemetry (timestamp > 0)
- **AC-11.2:** CRANKING → IDLE occurs when RPM > 550
- **AC-11.3:** IDLE → RUNNING occurs when throttle > 5%
- **AC-11.4:** RUNNING → SHIFTING occurs when gearbox requests shift
- **AC-11.5:** SHIFTING → RUNNING occurs when shift completes (clutch = 1.0)
- **AC-11.6:** RUNNING → IDLE occurs when speed → 0 AND throttle → 0
- **AC-11.7:** Any state → OFF occurs when no valid telemetry for 5 seconds

**Testable via:** `VirtualIceTwinStateTest`

---

## Out of Scope for Phase 1

The following are **explicitly excluded** from Phase 1 acceptance criteria and must NOT be tested:

1. **Torque converter slip behavior** — No model for fluid coupling, launch creep, or torque multiplication
2. **Cranking sound modeling** — No validation that cranking sequence sounds realistic (audio only, NFR)
3. **Driver modes (Sport/Eco)** — Only Normal mode shift table is implemented
4. **Torque-aware shifting** — Shift thresholds do not modulate based on engine load or grade
5. **Brake-dependent coast-down** — Coast-down uses same downshift thresholds as powered downshifts
6. **Grade detection** — Uphill/downhill does not modify shift behavior
7. **Kickdown mechanical switch** — Only software kickdown detection (throttle delta) is implemented
8. **Rev-matching on downshifts** — No throttle blip during downshifts
9. **SpeedTrackingForce drift correction** — Not validated in Phase 1 (deferred until real telemetry available)
10. **Audio quality metrics** — No objective measurement of audio fidelity, only manual QA gate

---

## Notes on What Shifts to Phase 2+

Based on research findings, the following features are deferred to Phase 2+:

### Phase 2 (Polish):
- Driver mode adaptation (Sport/Economy shift threshold offsets)
- Per-throttle shift timing (faster shifts at high throttle)
- Downshift rev-matching (throttle blip)
- Validation against real ZF 8HP audio recordings

### Phase 3+ (Future Enhancement):
- Full torque converter model (TorqueConverterConstraint)
- Torque-aware shifting (modulate thresholds based on engine load)
- Brake-dependent coast-down (earlier downshifts with brake applied)
- Grade detection (modify thresholds for uphill/downhill)
- Adaptive learning (adjust thresholds based on driver behavior)

### Calibration Deferral:
- Shift table calibration with real driving data (Phase 1 uses illustrative ZF 8HP45 values)
- Redline and idle RPM per engine profile (hardcoded constants for MVP)
- Tire radius, diff ratio, vehicle mass per profile (hardcoded for MVP)

---

## Calibration Values (Phase 1 Defaults)

These values are used for Phase 1 testing and are **tunable parameters** marked as such:

| Parameter | Value | Tunable | Source |
|-----------|-------|---------|--------|
| Shift time (pause) | 200ms | Yes | ZF 8HP spec |
| Clutch disengage time | 50ms | Yes | Research recommendation |
| Clutch reengage time | 100ms | Yes | Research recommendation |
| Hysteresis factor | 0.85 | Yes | 15% speed gap |
| Kickdown throttle threshold | 0.95 | Yes | Research finding |
| Kickdown delta threshold | 0.4 | Yes | Research finding |
| Throttle smoothing TAU | 50ms | Yes | User preference |
| Minimum shift interval | 3.0s | Yes | Prevent hunting |
| Redline RPM | 6500 | Yes | Per engine profile |
| Idle RPM | 750 | Yes | Per engine profile |

**Note:** All tunable parameters should be exposed as constants or configuration to enable adjustment without code changes.

---

## Acceptance Criteria Summary

**Total criteria:** 38 (AC-01.1 through AC-11.7, including RPM band validation and coast-down hysteresis quantification)
**Testable via TDD:** 37 (all except manual QA gate)
**Manual validation:** 1 (audio quality checkpoint)
**Out of scope:** 10 explicitly deferred features

**Phase 1 Complete When:**
1. All 37 TDD-testable criteria pass with green tests
2. Manual QA gate validated by project owner (audio sounds authentic)
3. SOLID/DRY critic agent has reviewed and approved code
4. No regressions in existing engine-sim functionality
5. Code coverage ≥ 90% on new bridge layer code

---

## Appendix: Test File Mapping

| Test Suite | Acceptance Criteria | Purpose |
|------------|---------------------|---------|
| `AccelerationScenarioTest` | AC-01.1 through AC-01.8 | Full acceleration run validation |
| `CruiseScenarioTest` | AC-02.1 through AC-02.4 | Highway cruise stability |
| `DecelerationScenarioTest` | AC-03.1 through AC-03.6 | Coast-down downshift behavior |
| `KickdownScenarioTest` | AC-04.1 through AC-04.5 | Aggressive throttle response |
| `StandstillScenarioTest` | AC-05.1 through AC-05.4 | Idle behavior at rest |
| `LaunchScenarioTest` | AC-06.1 through AC-06.5 | Acceleration from standstill |
| `TwinPhysicsIntegrationTest` | AC-07.1 through AC-07.6 | Physics pipeline correctness |
| `ShiftExecutionTest` | AC-08.1 through AC-08.6 | Shift timing and clutch sequencing |
| `ThrottleSmootherTest` | AC-09.1 through AC-09.5 | Low-pass filter behavior |
| `AutomaticGearboxTest` | AC-10.1 through AC-10.6 | Gearbox logic edge cases |
| `VirtualIceTwinStateTest` | AC-11.1 through AC-11.7 | State machine transitions (NFR) |

---

**End of Phase 1 Acceptance Criteria**
