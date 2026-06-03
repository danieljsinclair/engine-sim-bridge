# VirtualICE Twin -- Comprehensive Project Briefing

**Date:** 2026-05-06
**Branch:** preset-engine-config
**Status:** Phase 0 complete. Phase 1 (Core Twin MVP) ready to begin.
**Audience:** Specialist agents (Physics, Solution Architecture, Testing, Implementation)

---

## 1. Project Goal

Make a real Tesla EV produce realistic ICE vehicle sounds in real time. When the driver presses the accelerator, the system determines what a target ICE vehicle (e.g., AMG C63, Ferrari F136, GM LS) would be doing at that road speed and throttle position, feeds those conditions into engine-sim's physics-based sound engine, and physics produces authentic engine audio.

Why physics-based and not samples: Pre-recorded engine sounds loop and sound artificial. Engine-sim models real physics: combustion events, exhaust pulses, intake resonance, mechanical noise. The resulting audio is dynamic, responsive, and authentic because RPM and load emerge from actual physics simulation, not crossfaded WAV files.

The fundamental challenge: EV and ICE drivetrains are physically incompatible. A Tesla motor runs to 15,000+ RPM with a single reduction gear. An ICE V8 has 6-8 gears, a redline around 7,000 RPM, and a completely different torque curve. The "digital twin" must translate real EV telemetry into the conditions (throttle, gear, clutch pressure) that make the virtual ICE engine produce the right sound for the current driving situation.

---

## 2. Data Flow

```
Real EV (Tesla)
    |
    | OBD2 / BLE (10Hz)
    v
vehicle-sim (C++ data acquisition library, WIP)
    |  Produces VehicleSignal: throttle%, speedKmh, accelerationG, brake%, timestamp
    |
    v
VehicleSimAdapter (iOS app layer)
    |  Normalizes to UpstreamSignal (0-1 ranges)
    |  Interpolates 10Hz -> 60Hz
    |
    v
VirtualIceInputProvider (bridge layer, implements IInputProvider)
    |  Holds VirtualIceTwin instance
    |  Each tick: feeds EV telemetry + current engine RPM to twin
    |  Gets back: throttle, gear, clutchPressure, starterMotor, ignition
    |
    v
engine-sim-bridge (C++ wrapper library)
    |  BridgeSimulator wraps engine-sim's PistonEngineSimulator
    |  Applies twin outputs: setThrottle(), changeGear(), setClutchPressure()
    |
    v
engine-sim (forked physics engine, 10kHz timestep)
    |  Physics: Engine torque vs vehicle inertia through gear ratio + drag
    |  RPM emerges naturally from physics interaction
    |  Audio synthesis from exhaust/intake/combustion physics
    |
    v
Audio Output (AVAudioEngine on iOS, CoreAudio on macOS)
```

Key architectural constraint: vehicle-sim and engine-sim-bridge compile independently. Neither depends on the other at build time. The iOS app is the integration point. On ESP32, a CANInputProvider replaces vehicle-sim entirely.

---

## 3. Phase 0 Spike Results (Validated)

Five spikes validated core technical assumptions. All passed.

| Spike | Result | Significance |
|---|---|---|
| DynoTrackingSpike | PASS (0.0 RPM error) | Dyno tracks RPM perfectly -- but is the WRONG abstraction for driving |
| PhysicsDrivenDriving | PASS | `changeGear()` + `setClutchPressure(1.0)` + throttle produces physics-driven RPM via vehicle inertia. **Validated correct approach.** |
| RealEngineAudioSpike | PASS | Ferrari F136 engine produces real audio via .mr script |
| AudioSweepSpike | PASS | Audio pipeline works end-to-end |
| ClutchParameterSweep | PASS | Runs but hand-built engine init produces 0 RPM (no combustion). .mr scripts and C++ presets work correctly. |

### Key Validated Decisions

1. **Physics-driven approach is correct**: Feed throttle + gear + clutch pressure into engine-sim's physics pipeline. RPM emerges from engine torque vs vehicle inertia + drag.
2. **Dyno is NOT for driving simulation**: AngeTheGreat's original GUI uses dyno for sweep/hold TESTING only. The dyno bypasses vehicle physics.
3. **RPM is strictly an OUTPUT**: Never set RPM directly. Set the conditions (throttle, gear, clutch) that produce it.
4. **Vehicle + Transmission + VehicleDragConstraint provide realistic load**: No external torque injection needed for the primary approach.

### Why Dyno Fails for Driving

- `hold=true`: Bidirectional motor DRIVES crankshaft to target RPM, bypassing vehicle physics entirely.
- `hold=false`: Brake-only with cliff behavior (1-20% load = 6400 RPM, 25%+ = stuck at idle). Binary, not gradual.
- In the NsvOptimized solver, `ks` and `kd` are dead code. Only `v_bias`, `C`, and `limits` matter.

---

## 4. Core Unsolved Problem

How to translate real EV telemetry into engine-sim physics inputs so RPM and sound emerge naturally.

The twin must solve two coupled sub-problems:

**Sub-problem A: Which gear would an automatic ICE transmission select?**
The EV has a single reduction gear. The ICE vehicle has 6-8 forward gears. Given road speed and throttle position, the twin must determine which gear a ZF-style automatic would have selected. This determines the gear ratio, which determines effective vehicle inertia at the crankshaft, which determines how the engine responds to throttle.

**Sub-problem B: How much throttle should the virtual ICE engine receive?**
EV throttle response is instant. ICE throttle has natural lag. The twin must smooth the signal (exponential filter, TAU=50ms) so the ICE sound feels natural.

**The coupling:** Gear selection depends on RPM (diagnostic calculation from road speed), and RPM depends on gear (through the gear ratio). But we do NOT set RPM directly. We set conditions (throttle, gear, clutch) and let physics compute RPM. The gearbox algorithm uses a diagnostic speed-to-RPM formula for shift decisions; the actual engine RPM is always physics-driven.

**Additional challenge: Speed synchronization.** The physics engine computes virtual vehicle speed from forces. If virtual and real EV speeds drift (initial speed mismatch, numerical integration error), gear selection errors accumulate. A soft speed-tracking ForceGenerator on the vehicle mass body is the recommended correction mechanism (see physics-specialist-analysis.md).

---

## 5. Two Approaches

### Approach A: Full Physics with Vehicle Model (SELECTED)

Feed `setThrottle()`, `changeGear()`, `setClutchPressure(1.0)` into engine-sim. RPM emerges from engine torque vs vehicle inertia + drag through the gear ratio. This is exactly how AngeTheGreat's original GUI does driving simulation.

- **Pros**: Authentic physics, matches original application, uses existing vehicle model (mass, drag, gear ratios all work).
- **Cons**: Automatic gearbox must be built in bridge layer; speed drift needs correction.
- **Risk**: Low-medium. Physics is deterministic. Vehicle model is simple (point mass + drag). A soft speed-tracking ForceGenerator prevents drift.

### Approach B: Dyno Shortcut / Fixed Load Torque (FALLBACK)

Compute target RPM from EV road speed using speed-to-RPM formula, then use dyno or a custom FixedLoadConstraint to drive crankshaft toward that RPM.

- **Pros**: Simpler. Perfect RPM tracking.
- **Cons**: Bypasses vehicle physics. Sound decouples from drivetrain. No natural engine braking or load response. Binary/cliff behavior in dyno brake mode.
- **Status**: Dyno is rejected for primary use. A custom FixedLoadConstraint (applying velocity-independent torque to crankshaft via SCS constraint with biased limits) is a viable fallback if Approach A fails.

---

## 6. Key Constraints

1. **AngeTheGreat does NOT accept upstream PRs.** Our fork: `https://github.com/danieljsinclair/engine-sim.git`. Only clean, idiomatic additions on feature branches. Total engine-sim changes: ~15 lines of pure accessors.

2. **iOS cannot use Piranha** (the .mr script interpreter). Piranha requires Boost, which cannot be cross-compiled for iOS arm64. Two alternatives already exist:
   - `EnginePresets` -- hardcoded C++ factories (Honda TRX520, Subaru EJ25, GM LS)
   - `PresetEngineFactory` -- JSON-based presets compiled by macOS-only `engine-sim-preset-compiler` tool

3. **vehicle-sim is WIP.** The bridge must be source-agnostic, receiving `UpstreamSignal` (not vehicle-sim's `VehicleSignal`).

4. **Build independence.** engine-sim, engine-sim-bridge, and vehicle-sim each compile standalone. The iOS app links against bridge + vehicle-sim.

5. **Platform targets:** macOS CLI (working, CoreAudio), iOS (next, AVAudioEngine), ESP32 (future, direct audio).

---

## 7. Expected EV Telemetry Data

```cpp
struct UpstreamSignal {
    double throttleFraction = 0.0;   // 0.0 - 1.0 (normalized from OBD2 pedal position)
    double speedKmh = 0.0;           // Road speed from wheel speed sensors
    double accelerationG = 0.0;      // Longitudinal acceleration (-2.0 to +2.0 G)
    double brakeFraction = 0.0;      // 0.0 - 1.0 (brake pedal)
    uint64_t timestampUtcMs = 0;     // For staleness detection
    bool isValid = false;            // false = stale/no data (ramp to idle)
};
```

Source: OBD2 via BLE at ~10Hz. vehicle-sim handles BLE connection, polling, parsing. Bridge receives normalized data.

---

## 8. What engine-sim Has (Zero Modification Needed)

| Capability | Class/Method | What It Does |
|---|---|---|
| Full combustion physics | `CombustionChamber`, `GasSystem`, exhaust pipeline | Throttle -> airflow -> combustion -> torque -> RPM |
| Manual transmission | `Transmission::changeGear(int)` | Sets effective vehicle inertia: `I = m_car * (tire_radius / (diff_ratio * gear_ratio))^2`. Conserves kinetic energy on shift. |
| Clutch engagement | `Transmission::setClutchPressure(double)` | 0-1 range. 1.0 = locked. Transmits up to maxClutchTorque. |
| Vehicle inertia model | `Vehicle`, `m_vehicleMass` RigidBody | Point mass as virtual rotating body |
| Aerodynamic drag | `VehicleDragConstraint` | Drag proportional to speed^2 + rolling resistance |
| Starter motor | `StarterMotor` | `m_enabled` flag, cranks at ~200 RPM |
| Ignition | `IgnitionModule::m_enabled` | Enables/disables combustion |
| .mr scripts | 236+ via Piranha parser | Engine/vehicle/transmission definitions |
| Audio synthesis | `Synthesizer`, `writeToSynthesizer()` | Real-time audio from exhaust flow + cylinder events |
| Dynamometer | `Dynamometer` | Measurement tool for sweep/hold testing. NOT for driving. |
| Simulation loop | `startFrame()` / `simulateStep()` / `endFrame()` | Fixed 10kHz physics timestep |
| SCS constraint solver | `OptimizedNsvRigidBodySystem` | Handles all constraint math. Extension points: ForceGenerator, custom constraints. |

### Key physics insight

`changeGear()` conserves kinetic energy. On upshift, RPM drops automatically because the same KE is spread over a larger effective inertia. This produces natural-sounding gear changes without special logic.

### What engine-sim needs from our fork (~15 lines, pure accessors)

```cpp
// simulator.h -- add to public section
double getVehicleAngularVelocity() const;  // diagnostics
```

### Missing Transmission getters (~10 lines on fork)

- `getGearRatios()` -- array of gear ratios
- `getGearCount()` -- number of forward gears
- `getMaxClutchTorque()` -- clutch torque capacity

Needed for automatic gearbox shift point computation. Piranha already extracts these values during .mr script loading.

---

## 9. What's Missing (New Bridge Code)

~1000 lines C++ + ~400 lines tests, all in the bridge layer:

| Component | Responsibility | Est. Lines |
|---|---|---|
| `UpstreamSignal` | Normalized EV telemetry struct (0-1 range) | ~20 |
| `IceVehicleProfile` | Gear ratios, diff, tire, shift points, redline, idle (from .mr or presets) | ~80 |
| `AutomaticGearbox` | Shift curves parameterized by throttle, hysteresis, kickdown detection | ~150 |
| `VirtualIceTwin` | State machine (OFF/CRANKING/IDLE/RUNNING/SHIFTING), gearbox orchestration, throttle smoothing | ~200 |
| `VirtualIceInputProvider` | `IInputProvider` impl, holds twin, applies outputs to simulator | ~100 |
| ISimulator extension | `setGear()`, `setClutchPressure()`, `getEngineRpm()` virtuals | ~30 |
| Tests (TDD) | Unit + integration + acceptance | ~400 |

### Automatic Gearbox Algorithm

```
upshiftRPM(throttle)   = idleRpm + (redlineRpm - idleRpm) * (0.45 + 0.55 * throttle)
downshiftRPM(throttle) = idleRpm + (redlineRpm - idleRpm) * (0.25 + 0.20 * throttle)
```

| Throttle | Upshift RPM | Downshift RPM |
|---|---|---|
| 0.1 (light) | ~2,500 | ~1,500 |
| 0.5 (moderate) | ~4,600 | ~2,200 |
| 1.0 (full) | ~6,700 | ~3,000 |

Hysteresis gap prevents gear hunting. Kickdown: throttle delta > 0.4 triggers immediate downshift.

### State Machine

```
OFF -> (first valid telemetry) -> CRANKING -> (RPM > 550) -> IDLE
IDLE <-> RUNNING (throttle > 5%)
RUNNING <-> SHIFTING (gear change triggered, ~300ms with clutch pressure ramp)
RUNNING -> IDLE (speed -> 0, throttle -> 0)
Any -> OFF (no telemetry for N seconds)
```

| State | Throttle | Clutch | RPM |
|---|---|---|---|
| OFF | 0 | disengaged | 0 |
| CRANKING | 0 | 0.3 | starter ~200 |
| IDLE | 0.05 | 0.5 | ~700-800 |
| RUNNING | mapped from EV | 1.0 (locked) | physics-driven |
| SHIFTING | held | ramped 0 -> 1 | natural flare |

### Throttle Smoothing

```
filteredThrottle += (rawThrottle - filteredThrottle) * (1 - exp(-dt / TAU))
```
TAU = 50ms. Prevents artificially sharp ICE response from instant EV pedal.

---

## 10. Speed Synchronization Challenge

The physics-driven vehicle speed may drift from real EV speed. This is the primary risk with Approach A.

**Recommended solution:** Speed-tracking ForceGenerator on the vehicle mass body. A P-controller applies gentle correction torque pushing virtual speed toward real EV speed. The gain should be tuned so the correction is subtle (does not override engine physics) but prevents long-term drift.

**Alternative:** If drift is small enough in practice (the physics model is deterministic and simple), no correction may be needed. This needs empirical testing during Phase 1.

**Startup initialization:** When the twin starts, the EV may already be at 60 km/h. The vehicle mass body's angular velocity must be initialized to the corresponding value. This requires an accessor to set `m_vehicleMass.v_theta` on first telemetry receipt.

---

## 11. Current Codebase State

### Repository Structure

```
engine-sim-bridge/
  engine-sim/              -- git submodule (fork: danieljsinclair/engine-sim)
  include/
    common/                -- ILogging, JsonParser, CircularBuffer, wav_loader, Verification
    hardware/              -- IAudioHardwareProvider, CoreAudioHardwareProvider,
                            -- AVAudioEngineHardwareProvider, AudioTypes
    io/                    -- IInputProvider, IPresentation
    simulation/            -- SimulationLoop
    simulator/             -- ISimulator, BridgeSimulator, SimulatorFactory,
                            -- PresetEngineFactory, EnginePresets, SineSimulator,
                            -- SineEngine, SineTransmission, SineVehicle,
                            -- EngineSimTypes, ScriptLoadHelpers, SimulatorInitHelpers
    strategy/              -- SyncPullStrategy, ThreadedStrategy, IAudioBuffer,
                            -- AudioLoopConfig, AudioState, Diagnostics
    telemetry/             -- ITelemetryProvider, NullTelemetryWriter
  src/                     -- matching .cpp implementations for all above
  test/
    BridgeUnitTests.cpp
    PresetEngineTests.cpp         -- 15 tests (infrastructure + per-engine x 2)
    SineWaveRegressionTests.cpp
    fixtures/                     -- JSON preset files (Honda TRX520, Subaru EJ25)
    spikes/                       -- Phase 0: AudioSweep, ClutchParameterSweep,
                                   -- DynoTracking, DynoLaunchControl, RealEngineAudio,
                                   -- TelemetryJitter
  tools/
    preset_compiler.cpp           -- macOS-only: .mr -> JSON
```

### Existing Interfaces

- **ISimulator** -- pure virtual with lifecycle (`create/destroy/start/stop`), control (`setThrottle/setIgnition/setStarterMotor`), audio (`update/renderOnDemand/readAudioBuffer`), telemetry (`getStats`).
- **IInputProvider** -- abstract input source. `OnUpdateSimulation(dt)` returns `EngineInput` each tick.
- **BridgeSimulator** -- production ISimulator wrapping engine-sim's `Simulator*`. Exposes `getInternalSimulator()` for direct access to `getTransmission()->changeGear()` etc.
- **SimulationLoop** -- 60Hz main loop using IInputProvider and ISimulator.

### What ISimulator Does NOT Yet Expose

- `setGear(int)` -- needed for automatic gearbox
- `setClutchPressure(double)` -- needed for shift transitions
- `getEngineRpm()` -- needed for gearbox shift decisions

Workaround: `BridgeSimulator::getInternalSimulator()` provides raw access. Formalizing these into ISimulator (or a separate ITwinControl interface) is a Phase 1 task.

### Existing Test Coverage

- **Preset engine tests (15 tests)**: JSON -> physics -> audio pipeline for Honda and Subaru. Infrastructure, valid simulator creation, non-silent audio at idle, throttle response, clean shutdown, golden-file regression vs Piranha (macOS only).
- **Bridge unit tests**: Standalone, no engine-sim dependency.
- **Sine wave regression tests**: Validates int16/float buffer aliasing fix.
- **Phase 0 spikes**: 6 GTest-based executables with deterministic signal measurements.

### Build System

- CMake 3.20+, C++17
- macOS: shared library (.dylib), CoreAudio
- iOS: static library (.a), AVAudioEngine (ObjC++)
- GoogleTest via FetchContent
- Piranha disabled on iOS (no Boost)
- Phase 0 spikes: `BUILD_PHASE0_SPIKES` option

### Branch State

- Current branch: `preset-engine-config`
- Multiple stashes exist (see branch-stash-state.md) -- DO NOT DROP
- engine-sim submodule points to our fork: latest commit removes dead code, adds clean simulator hierarchy

---

## 12. Implementation Phases

### Phase 1: Core Twin MVP (NEXT)

**Goal**: End-to-end pipeline from synthetic EV telemetry to ICE audio.

Build: `UpstreamSignal`, `IceVehicleProfile`, `AutomaticGearbox`, `VirtualIceTwin`, `VirtualIceInputProvider`, ISimulator extensions.

**Acceptance criteria:**
- Synthetic telemetry file (throttle ramp 0-100%, speed ramp 0-100 km/h) produces correct gear selection (1->2->3->4->5->6)
- Engine RPM tracks road speed within 10% (physics-driven, not dyno)
- Audio output sounds like an ICE vehicle accelerating through gears
- TDD test coverage on all new components
- SOLID/DRY critic agent sign-off before commit

**MVP scope excludes**: torque converter slip, cranking sequence, kickdown, acceleration-informed throttle, dyno fallback.

### Phase 2: iOS Integration

**Goal**: Connect vehicle-sim demo data to twin on iPhone.

Build: `VehicleSimAdapter`, 10Hz->60Hz interpolation, stale data handling, iOS UI for profile selection.

### Phase 3: Automatic Transmission in engine-sim (optional)

**Goal**: Proper `AutomaticTransmission` + `TorqueConverter` as SCS constraints in engine-sim.

Deferred: Phase 1's bridge-level gearbox (calling `changeGear()` externally) is sufficient for MVP. Proper automatic transmission belongs in the physics engine for physics-timestep shift scheduling, but is a larger change.

### Phase 4: Pre-generation + Mobile Targets

**Goal**: Strip .mr parser for iOS/ESP32; bake profiles as C++ constants.

---

## 13. Open Questions

1. **ISimulator vs ITwinControl**: Should twin-specific methods (`setGear`, `setClutchPressure`, `getEngineRpm`) go directly on `ISimulator` or on a separate `ITwinControl` interface? ISP favors separate (keeps base ISimulator clean for keyboard/sine use cases). Simplicity favors combined. The existing `EngineInput.gearDelta` and `EngineInput.dynoTorqueScale` suggest the IInputProvider path may already support some of this.

2. **Speed synchronization**: If physics-driven vehicle speed drifts from real EV speed, how to correct? Options: (a) soft speed-tracking ForceGenerator on vehicle mass body, (b) periodic velocity override, (c) accept drift if empirically small enough. Needs Phase 1 testing.

3. **Initial vehicle speed at startup**: When the twin starts, the EV may already be at 60 km/h. How to initialize the physics engine's vehicle mass body to the correct speed? Requires accessor to set `m_vehicleMass.v_theta`.

4. **Vehicle profile source for MVP**: Hardcode a few profiles (GM LS, 2JZ, Ferrari) or start with .mr parsing from day one? Piranha parser already extracts 90% of needed parameters.

5. **AutomaticTransmission upstream**: Worth building as a fork contribution, or keep bridge-only indefinitely? The KISS argument: proper automatic transmission belongs in the physics engine for correct physics-timestep scheduling.

---

## 14. Key Files for Specialist Agents

### Architecture Plan (master document)
- `/Users/danielsinclair/.claude/plans/the-vehicle-sim-repo-has-foamy-dawn.md`

### Architecture Decisions Memory
- `/Users/danielsinclair/.claude/projects/-Users-danielsinclair-vscode-escli-refac7-engine-sim-bridge/memory/architecture-decisions.md`

### Physics Deep-Dive
- `/Users/danielsinclair/.claude/projects/-Users-danielsinclair-vscode-escli-refac7-engine-sim-bridge/memory/physics-specialist-analysis.md`

### Bridge Source (most relevant for implementation)
- `/Users/danielsinclair/vscode/escli.refac7/engine-sim-bridge/include/simulator/ISimulator.h`
- `/Users/danielsinclair/vscode/escli.refac7/engine-sim-bridge/include/simulator/BridgeSimulator.h`
- `/Users/danielsinclair/vscode/escli.refac7/engine-sim-bridge/include/io/IInputProvider.h`
- `/Users/danielsinclair/vscode/escli.refac7/engine-sim-bridge/include/simulator/PresetEngineFactory.h`
- `/Users/danielsinclair/vscode/escli.refac7/engine-sim-bridge/include/simulator/SimulatorFactory.h`
- `/Users/danielsinclair/vscode/escli.refac7/engine-sim-bridge/src/simulator/BridgeSimulator.cpp`
- `/Users/danielsinclair/vscode/escli.refac7/engine-sim-bridge/CMakeLists.txt`

### Engine-Sim Internals (for understanding physics)
- `engine-sim/include/simulator.h` -- Simulator class with m_dyno, m_vehicleMass, getTransmission()
- `engine-sim/include/transmission.h` -- changeGear(), setClutchPressure()
- `engine-sim/include/vehicle.h` -- Vehicle class, speed calculation
- `engine-sim/src/transmission.cpp` -- ClutchConstraint wiring, gear change energy conservation
- `engine-sim/src/dynamometer.cpp` -- Dyno constraint (measurement tool)
- `engine-sim/src/vehicle.cpp` -- Speed from rotational KE
- `engine-sim/src/vehicle_drag_constraint.cpp` -- Aero drag + rolling resistance

### Existing Tests (patterns to follow)
- `/Users/danielsinclair/vscode/escli.refac7/engine-sim-bridge/test/PresetEngineTests.cpp`
- `/Users/danielsinclair/vscode/escli.refac7/engine-sim-bridge/test/BridgeUnitTests.cpp`

### Reference Docs
- `/Users/danielsinclair/vscode/escli.refac7/docs/BRIDGE_INTEGRATION_ARCHITECTURE.md`
- `/Users/danielsinclair/vscode/escli.refac7/docs/BRIDGE_ARCHITECTURE_REVIEW.md`
