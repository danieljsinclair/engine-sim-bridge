# Preset Factory Redesign Plan

**Status:** Ready for next AI session
**Date:** 2026-05-14
**Context:** Phase 2 PoC committed. PresetEngineFactory has hardcoded magic numbers that must be eliminated.

---

## Problem Statement

`PresetEngineFactory` (614 lines) combines two responsibilities:
1. **Deserializing** JSON into engine objects
2. **Guessing** missing values with hardcoded defaults

The preset compiler (`.mr` → JSON) is incomplete — it doesn't serialize gear ratios, firing orders, ignition timing curves, port flow data, or vehicle parameters. PresetEngineFactory fills gaps with magic numbers (`createDefaultPortFlow()`, hardcoded timing curve line 486, hardcoded gear ratios line 556).

Result: every preset engine sounds partly like a generic small block, not like the specific engine the `.mr` file defines.

## Design Goal

**Pure data, no logic.** The workflow:

```
.mr script → Piranha compiler → serialized JSON → PresetEngineFactory (pure data holder)
```

PresetEngineFactory becomes a **data class with getters/setters only** — no initialization, no defaults, no fallbacks, no `createDefault*()` functions. The caller is responsible for:
1. Processing the `.mr` file (via Piranha compiler or future alternative)
2. Serializing the result to JSON
3. Loading that JSON into PresetEngineFactory
4. Validating that all required fields are present

This is OCP-compliant: swap the `.mr` processor for a different format, and PresetEngineFactory doesn't change.

---

## Coding Standards

- **TDD**: Write tests FIRST, blind to implementation. Tests assert correct behaviour, not implementation details.
- **SOLID**: SRP (one responsibility per class), OCP (open for extension, closed for modification), DIP (depend on abstractions).
- **DRY**: No duplicated logic. Extract shared behaviour into a single place.
- **KISS**: No clever abstractions. Prefer a simple solution over a flexible one.
- **No magic numbers**: All numeric values must be named constants. Better: values come from preset JSON, not constants.
- **No long functions**: Max ~30 lines. Break up for SRP.
- **Single return**: Structure functions so there's one return statement, not a handful.
- **Separation of concerns**: Data loading vs. validation vs. construction vs. orchestration — each is a separate class/function.

---

## Team Setup Instructions

Use the TeamCreate tool to create the team, then spawn agents with the Agent tool using `team_name` and `name` parameters.

```
TeamCreate: team_name="preset-redesign", description="Preset factory redesign + code quality"
```

### Agent 1: preset-compiler (Implementer — `subagent_type: "general-purpose"`)
**Scope:** W1 (complete .mr → JSON serialization)
**Can edit:** New file `src/simulator/PresetCompiler.cpp`, `src/simulator/PresetCompiler.h`
**Must NOT edit:** PresetEngineFactory, callers, KeyboardDemoThrottleSource

### Agent 2: preset-refactor (Implementer — `subagent_type: "general-purpose"`)
**Scope:** W2, W3, W4 (redesign PresetEngineFactory + EngineBuilder + callers)
**Can edit:** PresetEngineFactory, EngineBuilder (new), SimulatorFactory, related tests
**Must NOT edit:** PresetCompiler, KeyboardDemoThrottleSource, GearConventions

### Agent 3: quality-cleanup (Implementer — `subagent_type: "general-purpose"`)
**Scope:** AC1–AC4 (code quality)
**Can edit:** GearConventions.h, KeyboardDemoThrottleSource, component-diagram.md
**Must NOT edit:** PresetEngineFactory, PresetCompiler

### Agent 4: architect-critic (Critic — `subagent_type: "general-purpose"`)
**Scope:** Review only. NO code writes. NO file edits.
**Responsibilities:**
- Review all completed work items before marking them [x] DONE
- Block magic numbers, defaults, fallbacks
- Block `numberOr(fallback)` — every field must be explicitly provided
- Validate SRP: PresetEngineFactory = data only, EngineBuilder = construction only
- Validate OCP: new format support requires zero changes to PresetEngineFactory
- Validate DRY: no duplicated logic across agents' work
- Validate test quality: tests must test behaviour, not implementation
- Enforce single return, short functions, named constants

**Coordination:**
- preset-compiler and preset-refactor work in parallel (different files)
- quality-cleanup runs after preset-refactor (needs new data structures)
- architect-critic reviews all work, blocks completion on violations
- All agents write tests FIRST (TDD red-green-refactor)
- No commits until architect-critic approves
- Team lead (you) assigns tasks via TaskCreate, agents claim via TaskList

---

## Checklist

### W1: Complete the Preset Compiler (.mr → JSON serializer)

- [ ] W1.1: Create `PresetCompiler.h` with public API: `static std::string serializeToJson(const Engine* engine, const Vehicle* vehicle, const Transmission* transmission)`
- [ ] W1.2: Serialize ignition timing curve (sample points) — currently hardcoded at PresetEngineFactory line 486
- [ ] W1.3: Serialize firing order (per-cylinder ignition angles) — currently evenly-spaced assumption at line 501
- [ ] W1.4: Serialize transmission gear ratios — currently hardcoded `{2.97, 2.07, 1.43, 1.00, 0.84, 0.56}` at line 556
- [ ] W1.5: Serialize vehicle parameters (mass, drag, cross-section, diff ratio, tire radius, rolling resistance)
- [ ] W1.6: Serialize port flow samples (intake and exhaust per cylinder head) — currently uses `createDefaultPortFlow()` fallback
- [ ] W1.7: Serialize exhaust outlet flow rate — currently falls back to `k_carb(500)` at line 225
- [ ] W1.8: Serialize turbulence function parameters
- [ ] W1.9: Serialize combustion chamber starting conditions (pressure, temperature)
- [ ] W1.10: Serialize rev limiter parameters — currently `redline * 1.15` at line 492
- [ ] W1.11: Write unit test: compile known `.mr` file → serialize → verify all JSON fields present
- [ ] W1.12: Write unit test: compare serialized JSON against hand-verified expected output for at least one engine (e.g., Honda TRX520)

### W2: Redesign PresetEngineFactory as Pure Data Class

- [ ] W2.1: Create `PresetData.h` with plain data structs: `EngineParams`, `CrankshaftParams`, `CylinderBankParams`, `ExhaustParams`, `IntakeParams`, `FuelParams`, `IgnitionParams`, `VehicleParams`, `TransmissionParams`
- [ ] W2.2: Add `PresetData` aggregate struct containing all param structs
- [ ] W2.3: Add `bool isValid() const` and `std::string validationError() const` to PresetData — returns first missing required field
- [ ] W2.4: Write test: PresetData with all fields populated → `isValid()` returns true
- [ ] W2.5: Write test: PresetData with missing engine name → `isValid()` returns false, error mentions engine name
- [ ] W2.6: Write test: PresetData with zero cylinder count → `isValid()` returns false
- [ ] W2.7: Write test: PresetData with missing transmission gear ratios → `isValid()` returns false
- [ ] W2.8: Write test: PresetData with missing ignition timing curve → `isValid()` returns false
- [ ] W2.9: Implement `PresetEngineFactory::loadFromJson()` — populates PresetData from JSON, no defaults, no fallbacks
- [ ] W2.10: Write test: load from complete JSON → all PresetData fields populated correctly
- [ ] W2.11: Write test: load from JSON with missing field → `isValid()` returns false
- [ ] W2.12: Delete `createDefaultPortFlow()` and `createDefaultExhaustPortFlow()` — no replacements
- [ ] W2.13: Delete all `numberOr(fallback)`, `intOr(fallback)` calls — every field must be present in JSON
- [ ] W2.14: Verify: `grep -n "numberOr\|intOr\|stringOr\|boolOr\|createDefault"` in PresetEngineFactory.cpp returns nothing

### W3: Build Engine from PresetData (EngineBuilder)

- [ ] W3.1: Create `EngineBuilder.h` with static methods: `buildEngine()`, `buildVehicle()`, `buildTransmission()`
- [ ] W3.2: `buildEngine()` takes `EngineParams` + sub-params, constructs and initializes `Engine*`
- [ ] W3.3: `buildVehicle()` takes `VehicleParams`, constructs and initializes `Vehicle*`
- [ ] W3.4: `buildTransmission()` takes `TransmissionParams`, constructs and initializes `Transmission*`
- [ ] W3.5: Write test: build engine from known PresetData → engine cylinder count matches
- [ ] W3.6: Write test: build transmission from PresetData → gear ratios match
- [ ] W3.7: Write test: build vehicle from PresetData → mass and diff ratio match
- [ ] W3.8: No engine-sim types leak into PresetData or PresetEngineFactory — only EngineBuilder knows about `Engine*`, `Vehicle*`, etc.

### W4: Update Callers

- [ ] W4.1: Update `SimulatorFactory::create()` to use new two-step: load JSON → validate → build via EngineBuilder
- [ ] W4.2: Update existing PresetEngineFactory tests to use new API
- [ ] W4.3: Write integration test: load preset JSON → validate → build → verify engine runs (simulates one frame)
- [ ] W4.4: Remove old `loadFromFile()` / `loadFromString()` / `loadFromJson()` if superseded

### W5: Vehicle Speed Architecture

- [ ] W5.1: Document decision: Option C (UpstreamSignal.speedKmh is the speed source, caller chooses how to populate it)
- [ ] W5.2: Verify DemoVehiclePhysics populates UpstreamSignal.speedKmh correctly in demo mode
- [ ] W5.3: Document that in production, RealInputProvider will populate UpstreamSignal.speedKmh from OBD feed
- [ ] W5.4: Assess whether DemoVehiclePhysics needs gear-ratio-aware speed calculation or if current simplification is acceptable for PoC
- [ ] W5.5: Document engine-sim output data in component-diagram.md as feedback arrows

### AC1: GearSelector Simplification

- [ ] AC1.1: Change `GearSelector::DRIVE` from `99` to `1` — any positive value = forward gear
- [ ] AC1.2: Remove `PRNDL_ORDER[]` array — no longer needed with contiguous numeric sequence
- [ ] AC1.3: Remove `prndlIndex()` method — no longer needed
- [ ] AC1.4: Simplify `shiftUp()` to: if current < max (DRIVE), increment; else clamp
- [ ] AC1.5: Simplify `shiftDown()` to: if current > min (PARK), decrement; else clamp
- [ ] AC1.6: Update tests: verify PARK=-2, REVERSE=-1, NEUTRAL=0, DRIVE=1
- [ ] AC1.7: Update ConsolePresentation gear char mapping for DRIVE=1
- [ ] AC1.8: Add named constant for selector range bounds instead of raw ints

### AC2: KeyboardDemoThrottleSource SRP Fix

- [ ] AC2.1: Extract gear selector state into `GearSelectorInput` class (or fold into `DemoInputProvider`)
- [ ] AC2.2: Extract ignition state into same class or separate `IgnitionInput`
- [ ] AC2.3: `KeyboardDemoThrottleSource` becomes responsible for throttle ONLY
- [ ] AC2.4: `DemoInputProvider` reads gear selector from `GearSelectorInput`, ignition from `IgnitionInput`
- [ ] AC2.5: Update tests: `KeyboardDemoThrottleSource` tests only cover throttle behaviour
- [ ] AC2.6: Write tests for `GearSelectorInput` (cycling, clamping, state)
- [ ] AC2.7: Write tests for `IgnitionInput` (toggle, state)

### AC3: Remove Unauthorized Direct Key Mappings

- [ ] AC3.1: Remove P/R/N/D direct key handling from `KeyboardDemoThrottleSource` (or extracted input class)
- [ ] AC3.2: Keep only `[`/`]` cycling as specified — no direct key jumps
- [ ] AC3.3: Update tests to remove direct key assertions
- [ ] AC3.4: Verify 'R' key is no longer consumed (available for other uses)

### AC4: Document Engine-Sim Output Data

- [ ] AC4.1: Add feedback arrows to `docs/architecture/component-diagram.md` showing data read FROM engine-sim:
  - `EngineSimStats::vehicleSpeedKmh` (from vehicle model)
  - `EngineSimStats::engineTorqueNm` (from clutch F_t[0][0])
  - `EngineSimStats::drivetrainTorqueNm` (from clutch F_t[0][1] × ratios)
  - `EngineSimStats::currentRPM` (from crankshaft angular velocity)
- [ ] AC4.2: Label arrows as "feedback" (twin → engine-sim → stats → twin) not "forward"

---

## Acceptance Criteria (Unit Tests)

Each AC below must have a corresponding unit test. Tests are written FIRST, blind to implementation.

### PresetCompiler ACs

- [ ] **AC-PC-01:** Given a compiled Engine*, PresetCompiler produces JSON containing `ignition.timingCurveSamples` with at least 2 sample points
- [ ] **AC-PC-02:** Given a compiled Transmission*, PresetCompiler produces JSON containing `transmission.gearRatios` matching the transmission's actual ratios
- [ ] **AC-PC-03:** Given a compiled Vehicle*, PresetCompiler produces JSON containing `vehicle.mass`, `vehicle.diffRatio`, `vehicle.tireRadius`
- [ ] **AC-PC-04:** Given a compiled Engine* with port flow data, PresetCompiler produces JSON containing `intakePortFlowSamples` and `exhaustPortFlowSamples` per head
- [ ] **AC-PC-05:** Serialized JSON for a known engine (e.g., Honda TRX520) contains zero null/missing optional fields

### PresetEngineFactory ACs

- [ ] **AC-PEF-01:** `PresetData` with all required fields → `isValid()` returns true
- [ ] **AC-PEF-02:** `PresetData` missing engine name → `isValid()` returns false, `validationError()` contains "name"
- [ ] **AC-PEF-03:** `PresetData` with zero cylinder count → `isValid()` returns false
- [ ] **AC-PEF-04:** `PresetData` with zero gear ratios → `isValid()` returns false
- [ ] **AC-PEF-05:** `loadFromJson()` with complete JSON → all PresetData fields populated, no field left at default
- [ ] **AC-PEF-06:** `loadFromJson()` with missing field → PresetData isValid() false, validation error names the field
- [ ] **AC-PEF-07:** `grep -c "createDefault\|numberOr\|intOr\|stringOr\|boolOr" PresetEngineFactory.cpp` returns 0
- [ ] **AC-PEF-08:** No engine-sim `#include` in PresetEngineFactory.h (only in EngineBuilder)

### EngineBuilder ACs

- [ ] **AC-EB-01:** Built engine cylinder count matches PresetData
- [ ] **AC-EB-02:** Built transmission gear ratios match PresetData
- [ ] **AC-EB-03:** Built vehicle mass matches PresetData
- [ ] **AC-EB-04:** Built engine with ignition timing curve → curve matches PresetData (not hardcoded)

### Integration ACs

- [ ] **AC-INT-01:** Load preset JSON → validate → build → simulate one frame → no crash, RPM > 0
- [ ] **AC-INT-02:** SimulatorFactory uses new two-step API (no direct PresetEngineFactory::loadFromFile)

### Code Quality ACs

- [ ] **AC-CQ-01:** `GearSelector::DRIVE` value is `1`, not `99`
- [ ] **AC-CQ-02:** No `PRNDL_ORDER` array exists in codebase
- [ ] **AC-CQ-03:** `shiftUp()` and `shiftDown()` use increment/decrement + clamping, no lookup array
- [ ] **AC-CQ-04:** `KeyboardDemoThrottleSource` tests contain zero assertions about gear selector or ignition
- [ ] **AC-CQ-05:** No direct P/R/N/D key handling in any input class
- [ ] **AC-CQ-06:** Component diagram documents 4 engine-sim feedback outputs with arrows

---

## Build Verification

```bash
cd ~/vscode/escli.refac7/engine-sim-bridge/build
cmake --build .
ctest --output-on-failure -E 'NOT_BUILT'

# Verify no magic numbers remain
grep -n "numberOr\|intOr\|stringOr\|boolOr" src/simulator/PresetEngineFactory.cpp
# Expected: no output

grep -n "createDefault\|k_carb\|k_28inH2O" src/simulator/PresetEngineFactory.cpp
# Expected: no output

# Verify DRIVE is 1 not 99
grep "DRIVE" include/simulator/GearConventions.h
# Expected: DRIVE = 1

# Verify no PRNDL_ORDER
grep "PRNDL_ORDER" include/input/KeyboardDemoThrottleSource.h
# Expected: no output
```

---

## File Ownership Map

| Agent | Files | Tests |
|-------|-------|-------|
| preset-compiler | `PresetCompiler.h`, `PresetCompiler.cpp` | `PresetCompilerTest.cpp` |
| preset-refactor | `PresetData.h`, `PresetEngineFactory.h`, `PresetEngineFactory.cpp`, `EngineBuilder.h`, `EngineBuilder.cpp`, `SimulatorFactory.cpp` | `PresetDataTest.cpp`, `PresetEngineFactoryTest.cpp`, `EngineBuilderTest.cpp` |
| quality-cleanup | `GearConventions.h`, `KeyboardDemoThrottleSource.h`, `KeyboardDemoThrottleSource.cpp`, `DemoInputProvider.h`, `DemoInputProvider.cpp`, `component-diagram.md` | `GearSelectorInputTest.cpp`, `KeyboardDemoThrottleSourceTest.cpp` |
| architect-critic | None (review only) | None (review only) |
