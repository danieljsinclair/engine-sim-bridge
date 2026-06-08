# Phase 2 Acceptance Criteria — Connect-Demo Wiring & Torque Display

**Version:** 1.0
**Date:** 2026-05-13
**Author:** ac-writer (phase2-fixes-4 team)
**Status:** Draft

---

## Phase 2 Scope Reminder

Phase 2 connects the twin layer to the simulator loop and adds a torque readout to the console display. Two operating modes:

- **Mode 1 (default):** `ManualTwin` passthrough — keyboard directly controls engine via engine-sim's native vehicle physics.
- **Mode 2 (`--connect-demo`):** `VirtualIceTwin` with ZF 8HP45 automatic gearbox — physics-driven auto-shifting, clutch manipulation, throttle smoothing.

Key components:
- `ManualTwin` / `ManualTwinProvider` — keyboard-driven twin with state machine (OFF, CRANKING, IDLE, RUNNING)
- `VirtualIceTwin` / `VirtualIceInputProvider` — physics-driven twin with ZF gearbox
- `BridgeSimulator::getStats()` — clutch-constraint torque extraction (engine + drivetrain sides)
- `ConsolePresentation` — ANSI-colored gear selector and torque display
- `GearConventions` — shared enum mapping (GearSelector, BridgeGear, EngineSimGear)

**Phase 2 EXCLUDES:** Torque converter model, driver modes (Sport/Eco), brake-dependent coast-down, grade detection, audio quality validation (Phase 1 manual QA gate covers this).

---

## Human Validation Checkpoint (Manual QA Gate)

**The following is NOT tested via TDD but MUST be validated manually by the project owner before Phase 2 is considered complete:**

- [x] Engine sound starts naturally on launch (cranking → idle hum, no abrupt onset)
- [x] Gear shifts produce audible RPM changes (no silent shifts, no double-shifts)
- [x] Throttle response feels immediate in manual mode (keys 1-9 → RPM change within one frame)
- [x] Auto mode shifts feel natural (no hunting at cruise, kickdown is responsive)
- [x] Torque display values change smoothly (no flickering between positive/negative at steady state)
- [x] Color changes in torque display are visually clear (green vs red distinction)
- [x] No audible artifacts (clicks, pops, buffer underruns) during mode operation
...verified by human QA!

**Rationale:** Control responsiveness, audio naturalness, and display readability are subjective qualities that cannot be fully automated.

---

## Mode 1: Default (no --connect-demo) — ManualTwin Passthrough

### AC-01: Engine Auto-Start on Launch

**Given:** The simulator is launched without `--connect-demo`
**When:** The warmup phase completes and the main loop begins
**Then:**

- [x] **AC-01.1:** Engine transitions through OFF -> CRANKING -> RUNNING via the ManualTwin state machine
- [x] **AC-01.2:** Starter motor engages during CRANKING and disengages when RPM exceeds 400
- [x] **AC-01.3:** Ignition is set to `true` once the engine is running

**Testable via:** `ManualTwinTest` (state transition sequence)

---

### AC-02: Gear Selector Display at Start

**Given:** The simulator launches in manual mode
**When:** The first frame is displayed
**Then:**

- [x] **AC-02.1:** Gear selector display shows `[Gear:NM0]` (N=Neutral, M=Manual, 0=gear zero/neutral)
- [x] **AC-02.2:** `gearAutoMode` is `false` in the display output
- [x] **AC-02.3:** `gearSelector` value maps to `GearSelector::NEUTRAL` (0)

**Testable via:** `ManualTwinProviderTest` (initial output verification)

---

### AC-03: Shift-Up Key Changes Gear

**Given:** The engine is running in manual mode at neutral (`[Gear:NM0]`)
**When:** The shift-up key is pressed
**Then:**

- [x] **AC-03.1:** Gear selector display changes to `[Gear:1M1]` (1=gear 1, M=Manual, 1=gear number)
- [x] **AC-03.2:** Subsequent shift-up presses increment: `[Gear:2M2]`, `[Gear:3M3]`, etc.
- [x] **AC-03.3:** Gear is clamped at 8th gear (no overflow beyond `[Gear:8M8]`)
- [x] **AC-03.4:** `engine-sim` `changeGear()` is called with the correct engine-sim convention value (`BridgeGear - 1`)

**Testable via:** `ManualTwinTest` (gear increment, boundary clamping)

---

### AC-04: Shift-Down Key Decreases Gear

**Given:** The engine is running in manual mode in gear 3 (`[Gear:3M3]`)
**When:** The shift-down key is pressed
**Then:**

- [x] **AC-04.1:** Gear selector display changes to `[Gear:2M2]`
- [x] **AC-04.2:** Subsequent shift-down presses decrement toward `[Gear:NM0]`
- [x] **AC-04.3:** Gear is clamped at neutral (no decrement below 0)
- [x] **AC-04.4:** At neutral, `engine-sim` gear is set to -1 (EngineSimGear::NEUTRAL)

**Testable via:** `ManualTwinTest` (gear decrement, neutral clamping)

---

### AC-05: Throttle Keys Directly Control RPM

**Given:** The engine is running in manual mode
**When:** Throttle keys (1-9) are pressed
**Then:**

- [x] **AC-05.1:** Throttle value is set proportionally: key 1 = ~11%, key 5 = ~56%, key 9 = 100%
- [x] **AC-05.2:** Engine RPM responds to throttle input within one simulation frame (16.7ms at 60Hz)
- [x] **AC-05.3:** Throttle 0% (no key pressed) returns engine to idle RPM
- [x] **AC-05.4:** Throttle is clamped to [0.0, 1.0] range

**Testable via:** `ManualTwinProviderTest` (throttle passthrough)

---

### AC-06: Vehicle Moves in Gear

**Given:** The engine is running in manual mode, gear is set to 1st (`[Gear:1M1]`)
**When:** Throttle is applied (>0%)
**Then:**

- [x] **AC-06.1:** Vehicle speed increases from 0 km/h
- [x] **AC-06.2:** Speed increase is proportional to throttle input
- [x] **AC-06.3:** In neutral (`[Gear:NM0]`), speed remains at 0 regardless of throttle

**Testable via:** `ManualTwinProviderTest` + integration test with engine-sim vehicle model

---

### AC-07: Clutch Lock/Unlock Behavior

**Given:** The engine is running in manual mode
**When:** Gear state changes
**Then:**

- [x] **AC-07.1:** Clutch pressure is 1.0 (locked) when any gear 1-8 is engaged
- [x] **AC-07.2:** Clutch pressure is 0.0 (disengaged) when in neutral
- [x] **AC-07.3:** Clutch pressure transition happens on the same frame as the gear change

**Testable via:** `ManualTwinTest` (clutch output per state)

---

### AC-08: Speed Display Format

**Given:** The engine is running in manual mode
**When:** Vehicle speed changes
**Then:**

- [x] **AC-08.1:** At standstill, display shows `[  0 mph]` (3-char right-aligned zero)
- [x] **AC-08.2:** Speed is converted from km/h to mph using factor 0.621371
- [x] **AC-08.3:** Speed is displayed as a whole number (rounded, not truncated)
- [x] **AC-08.4:** Speed increases with throttle when in gear

**Testable via:** `ConsolePresentation` unit test (format verification)

---

### AC-09: Torque Display Format

**Given:** The engine is running in manual mode with torque values
**When:** Torque state is displayed
**Then:**

- [x] **AC-09.1:** Display format is `[Eng: +XXXnm <--> +XXXnm: Drive]` (3-char signed integer, showpos)
- [x] **AC-09.2:** Positive torque is colored green (ANSI `\x1b[32m`)
- [x] **AC-09.3:** Negative torque is colored red (ANSI `\x1b[31m`)
- [x] **AC-09.4:** Color reset (ANSI `\x1b[0m`) is applied after the torque segment
- [x] **AC-09.5:** In neutral with no load, both torque values are approximately 0

**Testable via:** `ConsolePresentation` unit test (format + color verification)

---

## Mode 2: --connect-demo — VirtualIceTwin with ZF Auto Gearbox

### AC-10: Engine Starts in NEUTRAL Selector

**Given:** The simulator is launched with `--connect-demo`
**When:** The twin receives its first valid telemetry
**Then:**

- [x] **AC-10.1:** Engine transitions through OFF -> CRANKING -> IDLE
- [x] **AC-10.2:** Gear selector display shows `[Gear:NA0]` (N=Neutral, A=Auto, 0=gear zero)
- [x] **AC-10.3:** `gearAutoMode` is `true` in the display output
- [x] **AC-10.4:** Clutch is disengaged (pressure = 0.0) during NEUTRAL

**Testable via:** `VirtualIceTwinTest` (initial state verification)

---

### AC-11: Throttle Works in Neutral

**Given:** The twin is in IDLE state with selector at NEUTRAL
**When:** Throttle keys are pressed
**Then:**

- [x] **AC-11.1:** Throttle is passed to engine-sim (engine revs freely)
- [x] **AC-11.2:** Engine RPM increases above idle
- [x] **AC-11.3:** No gear changes occur while in NEUTRAL
- [x] **AC-11.4:** Vehicle speed remains at 0 (clutch disengaged, no power to wheels)

**Testable via:** `VirtualIceTwinTest` (idle state throttle passthrough)

---

### AC-12: Idle Throttle Behavior

**Given:** The twin is in IDLE state with no throttle input
**When:** The engine sustains on its own
**Then:**

- [x] **AC-12.1:** Throttle output is the profile's `idleThrottle` value (default 0% — engine idles from physics alone)
- [x] **AC-12.2:** Engine sustains idle RPM (700-800 RPM range) without additional throttle
- [x] **AC-12.3:** No gear engagement occurs at idle

**Testable via:** `VirtualIceTwinTest` (idle state output)

---

### AC-13: Shift-Up Changes Selector NEUTRAL to DRIVE

**Given:** The twin is in IDLE state with selector at NEUTRAL
**When:** The shift-up key is pressed (selector changes to DRIVE)
**Then:**

- [x] **AC-13.1:** Selector transitions from `GearSelector::NEUTRAL` to `GearSelector::DRIVE`
- [x] **AC-13.2:** State machine transitions IDLE -> RUNNING when throttle > idle threshold
- [x] **AC-13.3:** Gearbox starts in 1st gear when entering RUNNING
- [x] **AC-13.4:** Clutch engages (pressure ramps to 1.0)

**Testable via:** `VirtualIceTwinTest` (NEUTRAL -> DRIVE transition)

---

### AC-14: Shift-Down Changes Selector DRIVE to NEUTRAL

**Given:** The twin is in RUNNING state with selector at DRIVE
**When:** The shift-down key is pressed (selector changes to NEUTRAL)
**Then:**

- [x] **AC-14.1:** Selector transitions from `GearSelector::DRIVE` to `GearSelector::NEUTRAL`
- [x] **AC-14.2:** State machine transitions RUNNING -> IDLE
- [x] **AC-14.3:** Clutch disengages (pressure = 0.0)
- [x] **AC-14.4:** Gear output resets to NEUTRAL (BridgeGear::NEUTRAL = 0)

**Testable via:** `VirtualIceTwinTest` (DRIVE -> NEUTRAL transition)

---

### AC-15: Auto Gear Selection in DRIVE

**Given:** The twin is in RUNNING state with selector at DRIVE
**When:** Vehicle speed and throttle change
**Then:**

- [x] **AC-15.1:** Gearbox automatically selects gear based on speed and throttle from the shift table
- [x] **AC-15.2:** At standstill (speed = 0), gear is 1st
- [x] **AC-15.3:** Gear changes occur through the shift execution sequence (clutch disengage -> pause -> reengage)
- [x] **AC-15.4:** `gearAbsolute` output reflects the gearbox's current gear in BridgeGear convention

**Testable via:** `AutomaticGearboxTest` + `VirtualIceTwinTest` (gear selection in RUNNING)

---

### AC-16: Gear Display Updates with Auto Shifts

**Given:** The twin is in RUNNING state, accelerating
**When:** The gearbox auto-shifts from 1st to 2nd
**Then:**

- [x] **AC-16.1:** Display changes from `[Gear:DA1]` to `[Gear:DA2]` (D=Drive, A=Auto, gear number)
- [x] **AC-16.2:** Subsequent shifts update: `[Gear:DA3]`, `[Gear:DA4]`, etc.
- [x] **AC-16.3:** The gear number in display matches `gearbox_.getCurrentGear()`

**Testable via:** `VirtualIceTwinTest` + integration test (display output per shift)

---

### AC-17: Speed Increases Under Throttle in DRIVE

**Given:** The twin is in RUNNING state with selector at DRIVE, gear 1, vehicle at standstill
**When:** Throttle is applied (>5%)
**Then:**

- [x] **AC-17.1:** Vehicle speed increases from 0
- [x] **AC-17.2:** Speed increase is proportional to throttle and gear ratio
- [x] **AC-17.3:** `vehicleSpeedKmh` in EngineSimStats reflects the actual vehicle speed
- [x] **AC-17.4:** Display shows non-zero mph value

**Testable via:** Integration test with engine-sim vehicle model

---

### AC-18: ZF 8HP45 Shift Table Compliance

**Given:** The twin is in RUNNING state with the ZF 8HP45 profile
**When:** Vehicle accelerates at varying throttle levels
**Then:**

- [x] **AC-18.1:** At 20% throttle, 1st->2nd upshift occurs at 20 km/h (from shift table row 0)
- [x] **AC-18.2:** At 50% throttle, 1st->2nd upshift occurs at 50 km/h (from shift table row 2)
- [x] **AC-18.3:** At 100% throttle, 1st->2nd upshift occurs at 55 km/h (from shift table row 4)
- [x] **AC-18.4:** Upshift sequence proceeds 1st -> 2nd -> 3rd -> ... -> 8th (no skipped upshifts)
- [x] **AC-18.5:** Downshift hysteresis factor is 0.85 (downshift at 85% of upshift speed)

**Testable via:** `AutomaticGearboxTest` (shift table lookup, hysteresis)

---

### AC-19: Kickdown Works

**Given:** The twin is in RUNNING state at cruise (6th gear, 80 km/h, 20% throttle)
**When:** Throttle rapidly increases to 100% (delta > 0.4 within 100ms)
**Then:**

- [x] **AC-19.1:** Kickdown is detected (throttle delta exceeds `kickdownDelta` = 0.4)
- [x] **AC-19.2:** Downshift to a lower gear occurs within 500ms
- [x] **AC-19.3:** Target gear produces RPM below 95% of redline at current speed (safe gear)
- [x] **AC-19.4:** Gear is held until throttle decreases below 80% or speed approaches redline

**Testable via:** `AutomaticGearboxTest` (kickdown detection and gear selection)

---

## Torque Display

### AC-20: Engine Torque is Positive/Green During Acceleration

**Given:** The vehicle is in gear with throttle applied, accelerating
**When:** Torque values are read from `BridgeSimulator::getStats()`
**Then:**

- [x] **AC-20.1:** `engineTorqueNm` is positive (clutch reaction force opposing engine deceleration)
- [x] **AC-20.2:** Display shows green ANSI color (`\x1b[32m`) for the engine torque value
- [x] **AC-20.3:** Value represents the clutch constraint `F_t[0][0]` from the physics solver

**Testable via:** `BridgeSimulator` integration test (acceleration scenario)

---

### AC-21: Engine Torque is Negative/Red During Deceleration

**Given:** The vehicle is in gear with no throttle, decelerating (engine braking)
**When:** Torque values are read from `BridgeSimulator::getStats()`
**Then:**

- [x] **AC-21.1:** `engineTorqueNm` is negative (engine absorbing energy from drivetrain)
- [x] **AC-21.2:** Display shows red ANSI color (`\x1b[31m`) for the engine torque value
- [x] **AC-21.3:** Engine braking is perceptible as a negative torque at zero throttle in gear

**Testable via:** `BridgeSimulator` integration test (deceleration scenario)

---

### AC-22: Drive Torque Reflects Gear Multiplication

**Given:** The vehicle is in 1st gear (gear ratio 4.714) accelerating
**When:** Torque values are read from `BridgeSimulator::getStats()`
**Then:**

- [x] **AC-22.1:** `drivetrainTorqueNm` magnitude differs from `engineTorqueNm` (gear multiplication)
- [x] **AC-22.2:** Drivetrain torque = clutch torque * gear_ratio * diff_ratio
- [?] **AC-22.3:** At higher gears (lower ratio), the difference between engine and drivetrain torque decreases

**Testable via:** `BridgeSimulator` integration test (torque multiplication per gear)

---

### AC-23: Both Sides Show ~0 in Neutral

**Given:** The transmission is in neutral (EngineSimGear::NEUTRAL = -1)
**When:** Torque values are read from `BridgeSimulator::getStats()`
**Then:**

- [x] **AC-23.1:** `engineTorqueNm` is approximately 0 (clutch disengaged, no constraint force)
- [x] **AC-23.2:** `drivetrainTorqueNm` is approximately 0
- [x] **AC-23.3:** Display shows `[Eng: +000nm <--> +000nm: Drive]` with both values near zero

**Testable via:** `BridgeSimulator` integration test (neutral state)

---

### AC-24: Torque Values Match Physics Solver

**Given:** The simulator is running with a valid vehicle and transmission
**When:** The clutch constraint is evaluated by the physics solver
**Then:**

- [x] **AC-24.1:** `engineTorqueNm` equals `clutch.F_t[0][0]` (crankshaft reaction force)
- [x] **AC-24.2:** `drivetrainTorqueNm` equals `-clutch.F_t[0][1] * gearRatio * diffRatio` (wheel-side torque)
- [x] **AC-24.3:** Sign convention: positive = engine producing power, negative = engine braking

**Testable via:** `BridgeSimulatorTest` (torque extraction correctness)

---

## Code Quality

### AC-25: No Magic Numbers in ConsolePresentation

**Given:** The `ConsolePresentation::formatEngineState()` method
**When:** Reviewed for magic numbers
**Then:**

- [n] **AC-25.1:** Display format widths use named constants or struct field widths (not raw literals like `5`, `4`, `3`)
- [x] **AC-25.2:** Conversion factors (e.g., km/h to mph: `0.621371`) are named constants
- [ ] **AC-25.3:** Numeric thresholds (e.g., `rpm < 10`) are documented or named

**Testable via:** Code review (static analysis)

---

### AC-26: ANSI Color Codes in Shared Constants

**Given:** The `ANSIColors` namespace in `config/ANSIColors.h`
**When:** ANSI escape sequences are used in ConsolePresentation
**Then:**

- [x] **AC-26.1:** All ANSI color codes reference `ANSIColors::GREEN`, `ANSIColors::RED`, `ANSIColors::RESET` (no inline escape sequences)
- [x] **AC-26.2:** Color constants are declared as `const std::string` in the shared namespace
- [x] **AC-26.3:** No duplicate ANSI definitions across the codebase

**Testable via:** Code review (grep for inline `\x1b` in presentation code)

---

### AC-27: Gear Selector Char Mapping Uses Table/Switch

**Given:** The `gearSelectorChar()` function in `ConsolePresentation.cpp`
**When:** Mapping `GearSelector` enum values to display characters
**Then:**

- [x] **AC-27.1:** Mapping uses a `switch` statement over `GearSelector` enum values (not imperative `if/else if` chain on raw ints)
- [?] **AC-27.2:** All `GearSelector` enum values are handled: PARK('P'), REVERSE('R'), NEUTRAL('N'), DRIVE('D'), 1-8('1'-'8')
- [x] **AC-27.3:** Unknown values return '?' (safe fallback)

**Testable via:** Code review + `ConsolePresentation` unit test

---

### AC-28: -Werror Builds Clean

**Given:** The bridge library is compiled with `-Werror` and `-Wall`
**When:** `cmake --build build` is executed
**Then:**

- [x] **AC-28.1:** Zero warnings on Apple Silicon (arm64) with Clang
- [?] **AC-28.2:** Zero warnings on Intel (x86_64) with Clang (cross-compile or CI)
- [?] **AC-28.3:** No unused variable, sign conversion, or implicit cast warnings

**Testable via:** CI build pipeline (`cmake --build build` with `-Werror`)

---

### AC-29: No CMake Artifacts in Project Root

**Given:** The project directory structure
**When:** `cmake` is configured and built
**Then:**

- [x] **AC-29.1:** No `CMakeCache.txt`, `CMakeFiles/`, or `cmake_install.cmake` in the project root
- [x] **AC-29.2:** All build artifacts are contained within the `build/` directory
- [x] **AC-29.3:** `.gitignore` excludes build directories

**Testable via:** `find . -maxdepth 1 -name "CMakeCache.txt"` returns nothing

---

## Build

### AC-30: CMake Build Succeeds (Bridge)

**Given:** The engine-sim-bridge repository
**When:** `cmake -B build && cmake --build build` is executed
**Then:**

- [x] **AC-30.1:** Build completes with exit code 0
- [x] **AC-30.2:** All bridge library targets compile successfully
- [x] **AC-30.3:** No undefined symbol errors at link time

**Testable via:** CI build step

---

### AC-31: All Bridge Tests Pass

**Given:** The bridge test suite
**When:** `cd build && ctest --output-on-failure` is executed
**Then:**

- [x] **AC-31.1:** All tests pass with exit code 0
- [x] **AC-31.2:** No test failures or disabled tests
- [x] **AC-31.3:** Test coverage includes: ManualTwin, VirtualIceTwin, ManualTwinProvider, VirtualIceInputProvider, AutomaticGearbox, ThrottleSmoother, TwinPhysicsIntegration

**Testable via:** `ctest --output-on-failure` in build directory

---

### AC-32: All CLI Tests Pass

**Given:** The CLI (engine-sim-cli) test suite
**When:** `cd build && ctest --output-on-failure` is executed in the CLI build directory
**Then:**

- [x] **AC-32.1:** All CLI tests pass with exit code 0
- [x] **AC-32.2:** ConsolePresentation tests cover gear display format and torque color logic
- [x] **AC-32.3:** No regressions from bridge library changes

**Testable via:** `ctest --output-on-failure` in CLI build directory

---

## Calibration Values (Phase 2 Defaults)

| Parameter | Value | Source | Applies To |
|-----------|-------|--------|------------|
| Clutch locked pressure | 1.0 | Full engagement | Both modes |
| Clutch disengaged pressure | 0.0 | Full disengagement | Both modes |
| Cranking RPM threshold | 400 RPM (manual) / 500 RPM (auto) | ManualTwin / VirtualIceTwin | Per mode |
| Invalid feedback timeout | 5.0 seconds | State machine safety | Both modes |
| Km/h to mph factor | 0.621371 | Standard conversion | Display |
| Torque display width | 3 chars, signed | Format specification | Display |

---

## Acceptance Criteria Summary

**Total criteria:** 86 checkboxes across 32 ACs
**Testable via TDD:** 78 checkboxes (all except Manual QA Gate)
**Manual validation:** 7 checkboxes (audio quality, control responsiveness, display readability)
**Out of scope:** Torque converter, driver modes, brake-dependent coast-down, grade detection

**Phase 2 Complete When:**
1. All TDD-testable criteria pass with green tests
2. Manual QA gate validated by project owner (audio sounds authentic, controls feel responsive)
3. `-Werror` builds clean on Apple Silicon
4. All bridge and CLI tests pass via `ctest`
5. No CMake artifacts in project root
6. Code review confirms no magic numbers, proper ANSI constants, switch-based gear mapping

---

## Appendix: Test File Mapping

| Test Suite | Acceptance Criteria | Purpose |
|------------|---------------------|---------|
| `ManualTwinTest` | AC-01, AC-03, AC-04, AC-07 | Manual twin state machine and gear control |
| `ManualTwinProviderTest` | AC-02, AC-05, AC-06 | Keyboard-to-twin wiring |
| `VirtualIceTwinTest` | AC-10, AC-11, AC-12, AC-13, AC-14, AC-15, AC-16 | Auto twin state machine and gearbox |
| `VirtualIceInputProviderTest` | AC-10, AC-13 | Provider-to-twin wiring |
| `AutomaticGearboxTest` | AC-15, AC-18, AC-19 | ZF 8HP45 shift table logic |
| `ThrottleSmootherTest` | (Phase 1) | Low-pass filter behavior |
| `TwinPhysicsIntegrationTest` | AC-17, AC-22 | Physics pipeline with vehicle model |
| `BridgeSimulatorTest` | AC-20, AC-21, AC-22, AC-23, AC-24 | Torque extraction from clutch constraint |
| `ConsolePresentationTest` | AC-08, AC-09, AC-16, AC-25, AC-26, AC-27 | Display format and color verification |

---

**End of Phase 2 Acceptance Criteria**
