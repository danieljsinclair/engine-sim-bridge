# VirtualICE Twin Architecture Plan

## Context

The goal: make a real Tesla EV produce realistic ICE vehicle sounds by mapping real OBD2 telemetry through a "digital twin" into engine-sim's physics-based sound engine. When you press the accelerator in a Tesla, the twin determines what an AMG C63 (or any .mr-defined vehicle) would be doing at that same road speed and throttle, feeds those conditions to engine-sim, and physics produces authentic sound.

The problem: EV and ICE drivetrains are fundamentally different. There's no 1:1 RPM mapping. A Tesla motor runs to 15,000+ RPM; road speed depends on gear ratio, not motor speed. The twin must compute which gear an automatic ICE transmission would select, apply appropriate throttle, and let engine-sim's physics engine determine the RPM and resulting sound.

---

## Phase 0 Spike Results (Evidence Collected)

### DynoTrackingSpike — PASS (dyno tracks RPM perfectly, but WRONG abstraction)
Real I6 engine with dyno hold=true. Mean RPM error = 0.0. Dyno can precisely hold target RPM.
- **However**: Dyno is a MEASUREMENT tool (sweep/hold), not a driving simulator.
- Dyno bypasses vehicle physics by directly controlling crankshaft angular velocity.

### PhysicsDrivenDriving — PASS (validated approach)
Ferrari F136 engine loaded via .mr script. Gear engaged via `changeGear()`, clutch locked via `setClutchPressure(1.0)`.
- RPM emerges naturally from engine torque vs vehicle inertia + drag through gear ratio
- Transmission encodes vehicle mass as effective rotational inertia: `I = m_car * (tire_radius / (diff_ratio * gear_ratio))^2`
- VehicleDragConstraint applies aero drag + rolling resistance automatically
- **This is how AngeTheGreat's original GUI does driving simulation** — NOT via dyno.

### Dyno Brake Mode — NOT VIABLE for gradual load
- `hold=false`, `m_rotationSpeed=0`: Kills engine at startup (velocity-dependent damping too aggressive)
- `hold=false`, `m_rotationSpeed=700` (idle): Cliff behavior — load 1-20% = 6400 RPM, 25%+ = stuck at idle
- Binary, not gradual — constraint solver saturates the cap too quickly

### AudioSweepSpike — PASS (pipeline validation)
Audio pipeline works end-to-end. Real engine audio confirmed with Ferrari .mr script.

### ClutchParameterSweep — PASS (runs but hand-built engine doesn't combust)
Hand-built C++ initialization produces 0 RPM — no combustion. .mr scripts through Piranha work perfectly.

### Key Decisions Validated
1. **Physics-driven approach is correct** — `changeGear()` + `setClutchPressure(1.0)` + throttle → RPM emerges
2. **Dyno is NOT for driving simulation** — AngeTheGreat's GUI uses dyno for sweep/hold TESTING only
3. **Vehicle + Transmission + VehicleDragConstraint provide realistic load** — no external torque needed
4. **RPM is strictly an OUTPUT** — never set RPM directly; set the conditions (throttle, gear, clutch) that produce it

---

## Key Finding: ForceGenerator Integration Point

The physics specialist traced through engine-sim's source code and identified the clean injection mechanism:

**`ForceGenerator::apply(SystemState* system)`** — called by `processForces()` before constraint solving. Writes to `system->t[body_index] += torque`.

This is the **intended extension point** in the SCS rigid body framework. It interacts correctly with all constraints (clutch, drag, etc.) because forces are applied before the constraint solver runs.

Two physics approaches are designed, sharing the same twin/gearbox layer:

### Approach A+ (Recommended): Full Vehicle + SpeedTrackingForce
Use engine-sim's complete vehicle model (Transmission + Vehicle + VehicleDragConstraint). Add a `SpeedTrackingForce` ForceGenerator on the vehicle mass body that gently corrects virtual speed toward real EV speed (P-controller, clamped). RPM emerges from physics.

- **Pros**: Authentic physics, natural engine braking, realistic load response
- **Cons**: Needs drift correction, startup speed initialization
- **Confidence**: High — physics specialist traced through source code, not speculative

### Approach B (Fallback): FixedLoadConstraint on Crankshaft
Replace vehicle model with a custom SCS constraint applying fixed braking torque. Computes target RPM from road speed + gear ratio, uses PD controller to derive load torque.

- **Pros**: Precise RPM tracking, no vehicle model complexity
- **Cons**: Bypasses vehicle physics, loses natural engine braking sound
- **When**: Only if Approach A+ shows unacceptable speed drift

Both approaches share the same `VirtualIceTwin`, `AutomaticGearbox`, `IceVehicleProfile`, and `VirtualIceInputProvider`. Only the physics injection layer differs.

---

## EV Telemetry Availability (from web research)

### Tesla-specific data sources

| Source | Rate | Fields | Feasibility |
|--------|------|--------|-------------|
| REST API (owner-api) | ~0.5 Hz | speed, power, shift_state | Easy but too slow for sound |
| Streaming API | ~2 Hz | speed, power, shift_state, soc | Usable with interpolation |
| CAN bus (drivetrain) | 50-100 Hz | motor RPM, throttle %, vehicle speed, brake | Best quality, needs CAN adapter |

### Minimum viable telemetry for Phase 1
- `throttleFraction` (EV pedal position, 0-1)
- `speedKmh` (road speed)
- `isValid` flag

These three fields are available even at the low-fidelity tier (REST/Streaming API). CAN bus gives 50-100 Hz for high-fidelity.

### Recommended strategy
- **MVP (demo)**: Streaming API at 2 Hz, interpolate to 60 Hz in adapter
- **Production**: CAN bus at 50-100 Hz via CAN adapter ($30-100 hardware)

---

## ZF Automatic Transmission Data (for AutomaticGearbox)

### ZF 8HP45 gear ratios (reference profile)

| Gear | Ratio | Step |
|------|-------|------|
| 1st | 4.714 | - |
| 2nd | 3.143 | 1.50:1 |
| 3rd | 2.106 | 1.49:1 |
| 4th | 1.667 | 1.26:1 |
| 5th | 1.285 | 1.30:1 |
| 6th | 1.000 | 1.29:1 |
| 7th | 0.839 | 1.19:1 |
| 8th | 0.667 | 1.26:1 |

### Shift map thresholds (calibrated for realistic RPM, km/h)

| Throttle | 1→2 | 2→3 | 3→4 | 4→5 | 5→6 | 6→7 | 7→8 |
|----------|-----|-----|-----|-----|-----|-----|-----|
| 10% | 20 | 35 | 50 | 65 | 80 | 95 | 110 |
| 25% | 30 | 50 | 70 | 90 | 110 | 130 | 155 |
| 50% | 40 | 65 | 90 | 115 | 140 | 170 | 200 |
| 75% | 55 | 85 | 115 | 145 | 180 | 215 | 255 |
| 100% | 70 | 105 | 140 | 180 | 220 | 265 | 315 |

**Footnote — Original illustrative values:** The earlier table (12/22/32 km/h at 10% throttle, 25/40/55 km/h at 50%, 45/70/95 km/h at 100%) was illustrative only and produced unrealistically low RPM. Research confirmed ZF 8HP45 typically shifts at ~1500-2000 RPM under light load, ~2500-3000 RPM under medium load, and ~4000-6500 RPM under heavy load. The calibrated table above produces these realistic RPM bands.

Shift time: ~0.2 seconds (ZF 8HP family). Hysteresis: downshift thresholds ~85% of upshift thresholds (15% speed gap prevents hunting).

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│ iOS App (integration point — depends on both libraries)         │
│                                                                  │
│  VehicleSimWrapper.mm          EngineSimWrapper.mm              │
│       │                              │                           │
│  VehicleSimulator ──► VehicleSimAdapter ◄── IOSRunner           │
│  (vehicle-sim C++)    (new adapter)     (engine-sim-bridge C++) │
│       │                    │                    │                 │
│  VehicleSignal      UpstreamSignal        BridgeSimulator       │
│  (10Hz BLE)         (interpolated 60Hz)   (ISimulator impl)     │
│                            │                    │                │
│                     VirtualIceInputProvider     │                │
│                     (IInputProvider impl)       │                │
│                            │                    │                │
│                     VirtualIceTwin              │                │
│                     (mapping model)             │                │
│                     ┌──────┴───────┐            │                │
│                     │ AutoGearbox  │            │                │
│                     │ ShiftCurves  │            │                │
│                     │ ThrottleSmo  │            │                │
│                     └──────┬───────┘            │                │
│                            │ throttle/gear/clutch               │
│                            ▼                    ▼                │
│                     ┌──────────────────────────┐                │
│                     │    engine-sim (physics)   │                │
│                     │  Engine + Transmission +  │                │
│                     │  Vehicle + VehicleDrag    │                │
│                     │  Dyno DISABLED for driving│                │
│                     └──────────┬───────────────┘                │
│                                │                                 │
│                     RPM = f(throttle, gear, mass, drag)         │
│                                │                                 │
│                          Audio Output                           │
└─────────────────────────────────────────────────────────────────┘
```

---

## Core Design Decision: Physics-Driven RPM (Validated)

**RPM is strictly an OUTPUT of the physics engine.** Feed throttle + gear + clutch pressure into engine-sim's existing physics pipeline. RPM emerges naturally from the interaction of engine torque, vehicle inertia, and drag.

### How engine-sim's physics produces RPM from inputs:
1. `changeGear(gear)` — sets effective vehicle inertia at crankshaft: `I = m_car * (tire_radius / (diff_ratio * gear_ratio))^2`. Lower gears = larger effective inertia = harder to accelerate.
2. `setClutchPressure(1.0)` — locks engine to drivetrain via ClutchConstraint (transmits up to maxClutchTorque)
3. `setThrottle(fraction)` — engine produces torque based on throttle position
4. `VehicleDragConstraint` — applies aero drag + rolling resistance as virtual torque (increases with speed^2)
5. RPM emerges from: engine torque vs (vehicle mass through gear ratio) + drag

This is exactly how AngeTheGreat's original GUI does driving simulation. The dyno is NOT involved — it's a separate measurement tool for sweep/hold testing.

### What the twin feeds engine-sim:
- **Throttle position** (from EV pedal, smoothed)
- **Gear selection** (computed by automatic gearbox from road speed + throttle)
- **Clutch pressure** (1.0 = locked, brief disengage during shifts)

### What the twin does NOT set:
- RPM (output, never input)
- Dyno target speed (dyno stays disabled for driving)
- Crankshaft velocity

---

## Gap Analysis: What Exists vs What's Missing

### What engine-sim provides (zero modification needed)

| Capability | Evidence |
|---|---|
| Full combustion physics (throttle -> RPM) | `CombustionChamber`, `GasSystem`, exhaust pipeline |
| Manual transmission with gear changes | `Transmission::changeGear()` — sets effective vehicle inertia |
| Clutch engagement/disengagement | `Transmission::setClutchPressure()` — locks engine to drivetrain |
| Vehicle inertia model (point mass via virtual rotating body) | `Vehicle`, `m_vehicleMass` RigidBody with `I = m * (tire/ratio)^2` |
| Aerodynamic drag + rolling resistance | `VehicleDragConstraint` — aero drag proportional to speed^2 |
| Starter motor for cranking | `StarterMotor` class |
| Ignition module | `IgnitionModule::m_enabled` |
| .mr script engine definitions | 236+ scripts, vehicle/transmission/engine parameters |
| Audio synthesis pipeline | `Synthesizer`, `writeToSynthesizer()` |
| Dynamometer (dyno sweep/hold testing) | `Dynamometer` — measurement tool, NOT for driving simulation |

### What engine-sim needs (small accessor additions on our fork)

~10 lines in `simulator.h` — no behavioral changes, no new classes:

```cpp
// Transmission control (already partially public)
// changeGear() and setClutchPressure() are already callable

// Vehicle state getters (needed for diagnostics)
double getVehicleAngularVelocity() const;
```

### Transmission getter gap (on our fork)

Missing getters needed for automatic gearbox:
- `getGearRatios()` — array of gear ratios
- `getGearCount()` — number of forward gears
- `getMaxClutchTorque()` — clutch torque capacity

These are pure accessors. The Piranha .mr parser already extracts these values during script loading.

### What's entirely missing (new bridge code)

| Component | Responsibility | Est. Lines |
|---|---|---|
| `VirtualIceTwin` | Core mapping model: state machine, gearbox, throttle smoothing | ~200 |
| `VirtualIceInputProvider` | `IInputProvider` impl, holds twin instance, applies outputs | ~100 |
| `IceVehicleProfile` | Gear ratios, diff, tire, shift points, redline, idle (from .mr) | ~80 |
| `AutomaticGearbox` | Shift curves, hysteresis, kickdown detection | ~150 |
| `UpstreamSignal` | Normalized EV telemetry struct (0-1 range) | ~20 |
| `VehicleSimAdapter` | iOS adapter: VehicleSignal → UpstreamSignal + interpolation | ~100 |
| ISimulator extension | `setGear()`, `setClutchPressure()`, `getEngineRpm()` virtual methods | ~30 |
| Tests | TDD for all new components | ~400 |

**Total new code: ~1000 lines C++, all in the bridge layer except the ~15-line accessor additions to engine-sim.**

---

## EV-to-ICE Mapping Model

### Twin Input (from vehicle-sim or CAN bus)

```cpp
struct UpstreamSignal {
    double throttleFraction = 0.0;   // 0.0 - 1.0
    double speedKmh = 0.0;
    double accelerationG = 0.0;
    double brakeFraction = 0.0;
    uint64_t timestampUtcMs = 0;
    bool isValid = false;
};
```

### Twin Output (to engine-sim)

```cpp
struct TwinOutput {
    double throttle;        // 0-1 (smoothed from EV throttle)
    int    gear;            // Selected gear index
    double clutchPressure;  // 0-1 (1.0 = locked in MVP)
    bool   starterMotor;
    bool   ignition;
};
```

### Automatic Gearbox Algorithm

Two distinct concerns: **shift scheduling** (when to shift) and **shift execution** (how the shift happens).

#### Shift scheduling

Road-speed-based shift curves, parameterized by throttle. Uses real EV road speed as the primary input (not engine RPM — RPM is a physics output, using it as gearbox input creates circular dependency):

```
upshiftSpeed(gear, throttle)  = ZF shift table lookup: speed threshold for gear → gear+1
downshiftSpeed(gear, throttle) = upshiftSpeed × 0.85  (15% hysteresis gap prevents hunting)
```

- Road speed is an input from vehicle-sim, not a physics output — breaks the circular dependency
- Shift table directly validated against published ZF 8HP45 data (km/h per throttle per gear)
- Kickdown: throttle delta > 0.4 within 100ms triggers immediate downshift (both thresholds are configurable)
- Coast-down downshift: throttle = 0 and speed drops below downshift threshold for current gear

**Deferred to Phase 2+**: Torque-aware shifting (engine torque vs gearbox torque). Real ZF TCU considers torque on both sides of the torque converter to modulate shift points under load. Phase 1 uses speed + throttle only, which covers steady-state acceleration, cruise, and basic kickdown.

#### Shift execution

engine-sim's `changeGear()` is instantaneous (ratio swap with energy conservation, no shift duration). The bridge simulates shift process via clutch manipulation:

1. Reduce clutch pressure to 0 (disengage drivetrain)
2. Pause ~200ms (ZF 8HP shift time)
3. Call `changeGear(newGear)`
4. Ramp clutch pressure back to 1.0 (re-engage)

During step 2-4, engine RPM flares naturally (unloaded) then settles as clutch re-engages — producing authentic shift sound without modifying engine-sim.

### RPM Computation (diagnostic/gear-selection only)

```
RPM = (speedMs / (2π × tireRadius)) × gearRatio × diffRatio × 60
```

Used to determine shift points and for display. NOT fed to engine-sim — physics computes actual RPM.

### Throttle Smoothing

EVs have instant response. Apply low-pass filter:
```
filteredThrottle += (rawThrottle - filteredThrottle) * (1 - exp(-dt / TAU))
```
TAU = 50ms. Prevents the ICE sound from feeling unnaturally sharp.

### State Machine

```
OFF → (first telemetry) → CRANKING → (RPM > 550) → IDLE
IDLE → RUNNING (throttle > 5%)
RUNNING → SHIFTING (gear change triggered)
SHIFTING → RUNNING (shift complete)
RUNNING → IDLE (speed → 0 AND throttle → 0)
Any → OFF (no telemetry for 5 seconds)
```

| State | Throttle | Clutch | RPM |
|---|---|---|---|
| OFF | 0 | disengaged | 0 |
| CRANKING | 0 | 0.3 | starter ~200 |
| IDLE | 0.05 | 0.5 | ~700-800 |
| RUNNING | mapped | 1.0 (locked) | physics-driven |
| SHIFTING | held | ramped 0→1 | natural flare |

---

## Integration Boundary: vehicle-sim → bridge

### Key constraint
vehicle-sim's PRODUCT_VISION prohibits build dependency on engine-sim. Both compile independently.

### Data flow
- Bridge owns `UpstreamSignal` (not shared from vehicle-sim)
- iOS app is the integration point — depends on both libraries
- `VehicleSimAdapter` (in iOS app target) maps `VehicleSignal` → `UpstreamSignal`
- Threading: vehicle-sim writes at 10Hz, bridge reads at 60Hz, adapter interpolates

### Lifecycle
- Bridge starts first (plays idle audio immediately)
- vehicle-sim connects when BLE is ready
- On disconnect: bridge ramps to idle, no audio artifacts
- On reconnect: ramps back up

### ESP32 target
- No vehicle-sim. `CANInputProvider` reads CAN frames → `UpstreamSignal`
- Bridge is source-agnostic

---

## Implementation Phases

### Phase 1: Core Twin (MVP — end-to-end pipeline)

**Goal**: Prove the physics-driven approach produces authentic sound from EV telemetry.

Components to build:
- [ ] `UpstreamSignal` struct (`bridge/include/io/UpstreamSignal.h`)
- [ ] `IceVehicleProfile` struct (`bridge/include/twin/IceVehicleProfile.h`) — all tunable constants centralized here, no magic numbers
- [ ] `AutomaticGearbox` class (`bridge/src/twin/AutomaticGearbox.cpp`) — road-speed-based shift scheduling, 85% hysteresis, kickdown
- [ ] `VirtualIceTwin` class — state machine + throttle smoothing (`bridge/src/twin/VirtualIceTwin.cpp`)
- [ ] `VirtualIceInputProvider` — `IInputProvider` impl (`bridge/src/input/VirtualIceInputProvider.cpp`)
- [ ] ISimulator extension — `setGear()`, `setClutchPressure()`, `getEngineRpm()` (`bridge/include/simulator/ISimulator.h`)
- [ ] Engine-sim accessor additions (~15 lines in `simulator.h`)
- [ ] `AutomaticGearboxTest` — TDD: shift decisions, hysteresis, kickdown, edge cases
- [ ] `VirtualIceTwinTest` — TDD: state machine transitions
- [ ] `ThrottleSmootherTest` — TDD: exponential response, no overshoot
- [ ] `IceVehicleProfileTest` — TDD: profile loading, parameter validation
- [ ] `TwinPhysicsIntegrationTest` — TDD: synthetic telemetry → twin → engine-sim → RPM within tolerance
- [ ] `AccelerationScenarioTest` — TDD: 0-100 km/h gear sequence, RPM bands, shift timing
- [ ] `DecelerationScenarioTest` — TDD: coast-down downshifts, hysteresis, no below-1st
- [ ] `KickdownScenarioTest` — TDD: throttle step → downshift within 500ms, safe gear
- [ ] `CruiseScenarioTest` — TDD: stable gear, no hunting at constant speed
- [ ] `StandstillScenarioTest` — TDD: idle behavior, no shifts at rest
- [ ] `LaunchScenarioTest` — TDD: 1st gear from rest, upshift at calibrated speed
- [ ] Shift execution via clutch manipulation (disengage → pause → changeGear → ramp)

**Acceptance criteria:**
- Comprehensive testable criteria defined in `docs/architecture/phase1-acceptance-criteria.md` (38 criteria total)
- Key headline criteria:
  - Steady-state acceleration produces correct gear sequence with calibrated shift speeds
  - Highway cruise stabilizes on appropriate gear with no hunting
  - Deceleration/coast-down triggers sequential downshifts with hysteresis
  - Kickdown forces downshift within 500ms of throttle step
  - Physics pipeline uses real EV speed for gear selection, not computed RPM
  - Shift execution completes within 250-350ms with characteristic RPM flare
  - Throttle smoothing provides realistic exponential response
- Manual QA gate: Audio output must sound authentic (realistic RPM changes, shift sounds, no artifacts)
- All new code has TDD test coverage (see Testing section)
- SOLID/DRY critic sign-off before commit

**MVP scope excludes**: torque converter slip, cranking sound modeling, driver modes (Sport/Eco), torque-aware shifting, brake-dependent coast-down, grade detection.

### Phase 2: Polish + iOS Integration

**Goal**: Connect vehicle-sim demo data to the twin on iPhone.

Components:
1. `VehicleSimAdapter` (iOS app target)
2. 10Hz→60Hz interpolation
3. Stale data handling (ramp to idle)
4. iOS UI for vehicle profile selection

**Acceptance criteria:**
- vehicle-sim demo data drives engine-sim audio through the twin
- No audio artifacts on BLE connect/disconnect
- Profile selection loads different .mr scripts

### Phase 3: Automatic Transmission in engine-sim (optional upstream contribution)

**Goal**: Add proper `AutomaticTransmission` + `TorqueConverter` to engine-sim.

Components (all in engine-sim):
1. `TorqueConverter` — new SCS constraint (fluid coupling between two bodies)
2. `AutomaticTransmission` — new class parallel to `Transmission`
3. Shift scheduling as part of `AutomaticTransmission::update()`

**Why defer**: Phase 1's bridge-level gearbox works by calling `changeGear()` externally. Proper automatic transmission in engine-sim is cleaner (physics-timestep shift scheduling) but is a larger change that benefits from Phase 1 validating the approach first.

**Acceptance criteria:**
- `AutomaticTransmission` follows same `initialize(Parameters)` pattern as `Transmission`
- Shift scheduling runs at physics timestep (10kHz), not bridge timestep (60Hz)
- Torque converter produces realistic slip behavior at launch
- Pre-generated C++ support (no .mr parser dependency)

### Phase 4: Pre-generation + Mobile Targets

**Goal**: Strip .mr parser for iPhone/ESP32; bake vehicle profiles as C++ constants.

---

## Testing Strategy (TDD)

### Red/Green/Refactor discipline
Every component is test-driven. RED phase tests MUST compile. Tests assert correct business behavior, not implementation details.

### Test categories

**Unit tests (pure logic, no engine-sim dependency):**

| Test Suite | Validates |
|---|---|
| `AutomaticGearboxTest` | Shift up/down decisions, hysteresis, kickdown, edge cases (redline, standstill) |
| `VirtualIceTwinTest` | State machine transitions (OFF→CRANKING→IDLE→RUNNING→SHIFTING→IDLE) |
| `ThrottleSmootherTest` | Exponential filter behavior, step response, steady-state accuracy |
| `IceVehicleProfileTest` | Profile loading, parameter validation |
| `UpstreamSignalTest` | Normalization (0-100 → 0-1), staleness detection |

**Integration tests (with engine-sim, no audio hardware):**

| Test Suite | Validates |
|---|---|
| `TwinPhysicsIntegrationTest` | Feed synthetic telemetry → twin → engine-sim → verify RPM tracks within tolerance |
| `GearChangeAudioTest` | Verify audio buffer contains audible difference between gears |
| `StartupSequenceTest` | Cranking → catch → idle produces correct RPM progression |

**Acceptance tests (end-to-end with mock telemetry):**

| Test Suite | Validates |
|---|---|
| `AccelerationScenarioTest` | Full throttle 0→100km/h: gears shift correctly, RPM behaves naturally |
| `DecelerationScenarioTest` | Lift off at 100km/h: engine braking sounds, downshifts occur |
| `StandstillScenarioTest` | 0 km/h with throttle: idle behavior, no RPM runaway |
| `DisconnectScenarioTest` | BLE disconnect mid-drive: ramp to idle gracefully |

### Test data
- Synthetic telemetry files (CSV): acceleration run, highway cruise, city driving, standstill
- No dependency on live BLE hardware
- No dependency on live audio hardware (mock the audio callback)

### Acceptance criteria for "done"
1. All unit + integration tests pass
2. SOLID compliance: each class has single responsibility, open for extension, depends on abstractions
3. DRY: no duplicated gear ratio logic, no duplicated throttle mapping
4. Test coverage ≥ 90% on new code (happy path + reasonable edge cases)
5. SOLID/DRY critic agent has reviewed and approved (see Quality Gate below)

---

## Quality Gate: SOLID/DRY Critic

Before any code is committed, a dedicated **architecture critic agent** must review and approve. The critic enforces:

- **SRP**: Each class has one reason to change. `VirtualIceTwin` maps telemetry to engine commands. `AutomaticGearbox` selects gears. `VirtualIceInputProvider` adapts to the simulation loop. No god classes.
- **OCP**: New vehicle profiles or gearbox algorithms are added by creating new types, not modifying existing ones.
- **LSP**: Any `IInputProvider` implementation works in the simulation loop without special-casing.
- **ISP**: `ISimulator` extensions are in a separate `ITwinControl` interface if they're twin-specific (keep base `ISimulator` clean for keyboard/sine use cases).
- **DIP**: Twin depends on `UpstreamSignal` (abstraction), not `VehicleSignal` (vehicle-sim concrete type). Bridge depends on `IInputProvider`, not concrete providers.
- **DRY**: Gear ratios live in `IceVehicleProfile` only. Speed-to-RPM formula is in one place. No copy-paste between test and production code.
- **No over-engineering**: No `TorqueConverter` until Phase 3. No shift mode enums until needed. No factory patterns for single implementations.
- **No magic numbers**: All numeric constants (shift thresholds, timing values, filter parameters, RPM limits) are defined in `IceVehicleProfile` or a dedicated configuration struct. No unexplained literals in logic code. Each value is named, documented with its source (ZF spec, research finding, tuning default), and grouped to indicate how it would translate to .mr script configuration in Phase 2+.

The critic MUST explicitly sign off in the commit workflow. If the critic rejects, the issue must be fixed before proceeding.

---

## Key Files Referenced

### Existing (to extend, not modify behaviorally)
- `engine-sim/include/simulator.h` — add ~15 lines of accessors
- `engine-sim-bridge/include/simulator/ISimulator.h` — add twin control virtuals
- `engine-sim-bridge/include/io/IInputProvider.h` — unchanged (twin uses it)
- `engine-sim-bridge/src/simulator/BridgeSimulator.cpp` — implement new virtuals

### New (bridge layer)
- `engine-sim-bridge/include/io/UpstreamSignal.h`
- `engine-sim-bridge/include/twin/IceVehicleProfile.h`
- `engine-sim-bridge/include/twin/TwinOutput.h`
- `engine-sim-bridge/include/twin/VirtualIceTwin.h`
- `engine-sim-bridge/include/twin/AutomaticGearbox.h`
- `engine-sim-bridge/include/physics/SpeedTrackingForce.h` (Approach A+)
- `engine-sim-bridge/include/physics/FixedLoadConstraint.h` (Approach B)
- `engine-sim-bridge/src/twin/IceVehicleProfile.cpp`
- `engine-sim-bridge/src/twin/VirtualIceTwin.cpp`
- `engine-sim-bridge/src/twin/AutomaticGearbox.cpp`
- `engine-sim-bridge/src/input/VirtualIceInputProvider.cpp`
- `engine-sim-bridge/src/physics/SpeedTrackingForce.cpp`
- `engine-sim-bridge/src/physics/FixedLoadConstraint.cpp`
- `engine-sim-bridge/test/twin/AutomaticGearboxTest.cpp`
- `engine-sim-bridge/test/twin/VirtualIceTwinTest.cpp`
- `engine-sim-bridge/test/twin/IceVehicleProfileTest.cpp`
- `engine-sim-bridge/test/integration/TwinPhysicsIntegrationTest.cpp`

### Existing docs to extend (not create orphans)
- `docs/BRIDGE_INTEGRATION_ARCHITECTURE.md` — update Phase 3 TODO with this plan's detail

### Research artifacts (in docs/architecture/)
- `docs/architecture/physics-specialist-analysis.md` — ForceGenerator integration point, constraint solver analysis, Approach A+/B design
- `docs/architecture/architecture-decisions.md` — Key architectural decisions, Phase 0 spike results, CLI control mappings
- `docs/architecture/zf-shift-scheduling-research.md` — ZF 8HP shift scheduling validation, calibrated shift thresholds, TCU behavior
- `docs/architecture/shift-execution-research.md` — Shift execution modeling, clutch pressure vs torque converter options, timing parameters
- `docs/architecture/phase1-acceptance-criteria.md` — Comprehensive testable acceptance criteria for Phase 1

### Archived (superseded by plan and newer research)
- `docs/archive/solution-architecture-proposal.md` — Earlier proposal; superseded by this plan
- `docs/archive/web-research-report.md` — EV telemetry data now in plan; ZF shift data superseded by `zf-shift-scheduling-research.md`
- `docs/archive/project-briefing.md` — Phase 0 specialist team briefing; no longer needed
- `docs/archive/critic-review-phase1-planning.md` — One-time planning review; findings incorporated

---

## Open Questions for User

1. **RESOLVED — ISimulator vs ITwinControl**: Phase 1 extends `EngineInput` with `gearAbsolute`, `clutchPressure`, and `vehicleSpeedTargetKmh` fields. Consistent with existing `gearDelta`/`dynoTorqueScale` pattern. ITwinControl deferred until a second twin-type provider emerges.

2. **RESOLVED — Physics-driven as primary**: Approach A+ (full vehicle + SpeedTrackingForce) is primary. Approach B (FixedLoadConstraint) is the documented fallback if speed drift is unacceptable.

3. **RESOLVED — Torque converter not needed for MVP**: ZF lockup clutch engages early (often 2nd gear+). Launch slip approximated via clutch pressure ramping. Full fluid coupling model deferred to Phase 3.

4. **Vehicle profile source**: MVP hardcodes 2-3 profiles (GM LS, Ferrari F136, Honda TRX520) as C++ constants. .mr script parsing for profile extraction is Phase 2+.

5. **RESOLVED — Telemetry source**: vehicle-sim (github.com/danielsinclair/vehicle-sim, submodule of the CLI, not Tesla API) is the intended data provider. Road speed, motor torque, throttle position etc. come from vehicle-sim at adequate rate. Out of scope for this repo to produce telemetry — vehicle-sim is the data provider. Phase 1 uses mocked/synthetic telemetry files (CSV) to validate the twin independently. Phase 2 connects the live vehicle-sim data stream.

6. **RESOLVED — SpeedTrackingForce tuning**: Out of scope for this repo. vehicle-sim provides real road speed and motor torque which the twin consumes. Speed drift correction (if needed) happens when we have real data flowing.

---

## Remaining Open Questions

### Q1: RESOLVED — .mr script compatibility for headless loading

**Problem**: Downloadable .mr files crash with segfault via `--script` because they use `run()` which expects a GUI environment, or lack a `main` node to wire engine/vehicle/transmission together.

**Solution**: Replace `run()` with `set_engine()` / `set_vehicle()` / `set_transmission()` pattern. Add a `main` node at the bottom of engine definition files that wires components together, then call `main()`. The pattern:
```
public node main {
    set_engine(MyEngine())
    set_vehicle(my_vehicle())
    set_transmission(my_transmission())
}
main()
```

**Fixed**: C63.mr (was already adapted), C63_M156.mr (added main node), C63_M156_V2.mr (replaced run() with set_engine pattern). All three load successfully via `--script`.

**General approach for new .mr files**: Any downloadable engine definition needs two changes: (1) ensure a `main` node exists that calls `set_engine`/`set_vehicle`/`set_transmission` instead of `run()`, (2) add `main()` call at the end.

### Q2: RESOLVED — Automatic gearbox shift curve validation

**Problem**: The plan specified shift curves parameterized by throttle, but the thresholds hadn't been validated against real ZF behavior.

**Impact**: Wrong shift points produce unrealistic sound — shifting too early sounds sluggish, too late sounds aggressive.

**Resolution**: Research validated that ZF 8HP transmissions use throttle position and vehicle speed as primary inputs for shift scheduling. The speed-based shift curve model is fundamentally correct. The original illustrative shift table was calibrated upward to produce realistic RPM bands (light throttle ~1500-2000 RPM, medium ~2500-3000 RPM, heavy ~4000-6500 RPM). Hysteresis of 15% (downshift at 85% of upshift speed) prevents gear hunting. Full research documented in `docs/architecture/zf-shift-scheduling-research.md` and `docs/architecture/shift-execution-research.md`.

### Q3: SpeedTrackingForce — is it needed at all?

**Problem**: Approach A+ includes a drift correction ForceGenerator, but we won't know if drift is significant until the pipeline runs end-to-end with real telemetry from vehicle-sim.

**Context**: When engine-sim's virtual vehicle runs, its simulated road speed may drift from the real EV's actual road speed because the virtual ICE engine produces different torque than the real EV motor. SpeedTrackingForce is a proposed `ForceGenerator` (engine-sim's extension point for injecting external forces) that applies gentle corrective torque to the vehicle mass body — pushing virtual speed toward real speed. It uses a P-controller: `correctionTorque = gain * (virtualSpeed - realSpeed)`, clamped to a max value. This prevents gear selection errors (gearbox uses real speed but physics uses virtual speed).

**Impact**: If drift is <2%, the ForceGenerator adds unnecessary complexity and potential audio artifacts (clicks/pops from sudden torque corrections).

**What's needed to answer**: Build the Phase 1 pipeline with SpeedTrackingForce disabled. Feed synthetic telemetry from vehicle-sim. Measure virtual vs real speed divergence over test scenarios (acceleration, cruise, deceleration). Enable only if drift exceeds 5%. Starting values if needed: gain=100, maxCorrection=500 N.

### Q4: Vehicle profile parameterization from .mr scripts

**Problem**: IceVehicleProfile needs gear ratios, diff ratio, tire radius, mass, idle/redline RPM. These are currently hardcoded as C++ constants. Can they be auto-extracted from .mr scripts?

**Context**: The proposed engine-sim accessors (`getGearRatios()`, `getGearCount()`, `getMaxClutchTorque()` on Transmission) would let the bridge read back what gear ratios a .mr script configured at runtime. However, for Phase 1 these accessors are **not required**.

**Phase 1 approach (PoC)**: Hardcode 2-3 profiles (GM LS, Ferrari F136, Honda TRX520) as C++ constants in `IceVehicleProfile`. This is a temporary measure to prove the twin model. The hardcoded profile must be manually validated against its corresponding .mr script to ensure gear ratios, diff ratio, tire radius, and mass match — these are two independent data sources and will not automatically stay in sync. The profile is selected at startup alongside the .mr script.

**Phase 2 approach**: Extract parameters from Piranha compiler output. Requires ~10 lines of read-only accessors on engine-sim's Transmission class (our fork, no upstream PR). Not blocking for Phase 1.

---

## Research Artifacts

The following research documents contain detailed findings that informed this architecture plan:

- `docs/architecture/zf-shift-scheduling-research.md` — ZF 8HP transmission shift scheduling validation, including TCU input signals, shift map structure, hysteresis mechanism, kickdown behavior, and calibrated shift thresholds
- `docs/architecture/shift-execution-research.md` — Shift execution modeling, recommending clutch pressure manipulation (Option A) over full torque converter modeling (deferred to Phase 3+), with detailed timing parameters and fidelity analysis
- `docs/architecture/phase1-acceptance-criteria.md` — Comprehensive Phase 1 acceptance criteria from product owner, including 38 testable criteria covering functional, integration, and edge case requirements

These documents should be consulted for detailed implementation guidance and rationale behind the decisions summarized in this architecture plan.
