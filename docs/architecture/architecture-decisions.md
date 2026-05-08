---
name: VirtualICE Twin Architecture Decisions
description: Key architectural decisions for the VirtualICE twin (EV telemetry -> ICE sound engine)
type: project
originSessionId: 7046820a-eaba-4f82-a7c2-8cda7fb7c15a
---
## Data Flow Chain
```
Real Inputs (OBD: throttle, road speed, torque, acceleration) -> VirtualIceTwin -> engine-sim (via transmission + vehicle physics)
```
- Twin sits BETWEEN real inputs and engine-sim
- Real inputs: OBD (throttle position, road speed, EV motor torque/RPM, accelerometer)
- RPM is strictly an OUTPUT of engine-sim physics, never an input

## CRITICAL FINDING: Dyno is Wrong Abstraction for Driving Load

### Dyno is a measurement tool, not a load simulator
- The dynamometer constraint is for dyno sweep/hold TESTING (measuring engine output at controlled RPM)
- It is NOT used for driving simulation in AngeTheGreat's original GUI
- hold=true: Bidirectional motor that DRIVES crankshaft to target RPM — bypasses vehicle physics
- hold=false: Brake-only — cliff behavior (either engine overcomes load or it doesn't, no middle ground)

### The correct approach: Transmission + Vehicle + VehicleDragConstraint
engine-sim already has the full physics for simulated driving:
1. `changeGear(gear)` — sets effective vehicle inertia at crankshaft: `I = m_car * (tire_radius / (diff_ratio * gear_ratio))^2`
2. `setClutchPressure(1.0)` — locks engine to drivetrain via clutch constraint (transmits up to maxClutchTorque)
3. `VehicleDragConstraint` — applies aero drag + rolling resistance as virtual torque on vehicle mass body
4. `setThrottle()` — engine produces torque based on throttle position
5. RPM EMERGES from: engine torque vs vehicle inertia + drag through gear ratio

### Dyno brake mode evidence (why it doesn't work for load)
- `m_rotationSpeed=0`: Kills engine at startup — too aggressive for starter motor
- `m_rotationSpeed=700` (idle): Cliff behavior — load 1-20% = 6400 RPM, load 25%+ = stuck at idle
- Velocity-dependent damping saturates cap too quickly — binary, not gradual
- NOT how Ange's GUI does driving simulation

### Dyno in NsvOptimized solver
- `m_ks` and `m_kd` are COMPLETELY IGNORED
- Only `v_bias` (= m_rotationSpeed), `C`, and `limits` (= m_maxTorque) matter
- `m_maxTorque` caps the constraint force but doesn't provide gradual load

## How to Feed Real Vehicle Data into engine-sim
The twin must translate real EV telemetry into engine-sim physics inputs:
- **Throttle position** -> `setThrottle()` (direct mapping with smoothing)
- **Road speed** -> determines target gear (via automatic gearbox shift curves) and target RPM
- **Gear selection** -> `changeGear(gear)` + `setClutchPressure(1.0)`
- RPM then emerges from physics: engine torque vs (vehicle mass * gear ratio * diff ratio) + drag
- The twin does NOT set RPM directly — it sets the CONDITIONS (throttle, gear) that produce RPM

## Gearbox Strategy
- Automatic gearbox (ZF-like) lives in bridge layer initially
- Must stay DECOUPLED from .mr scripts — gearbox is a twin model, not an engine-sim concern
- Shift curves parameterized by throttle
- `changeGear()` conserves kinetic energy — RPM drops automatically on upshift

## Engine-Sim Fork Policy
- AngeTheGreat is NOT accepting upstream PRs
- Our fork: clean, idiomatic feature commits on feature branches, minimize divergence

## Vehicle Profiles / .mr Parsing
- Piranha parser already extracts 90% of needed parameters from .mr files
- Engine: getters for nearly everything. Vehicle: complete getter coverage.
- Transmission: MISSING getters for gear ratios, gear count, max clutch torque (need to add on our fork)
- No hardcoding profiles — leverage the parser from day one

## Phase 0 Spike Status
| Spike | Status | Proves |
|---|---|---|
| DynoTrackingSpike | PASS | Dyno tracks RPM perfectly (0.0 error) — but WRONG approach for twin |
| RealEngineAudioSpike | PASS | Real engine produces audio (Ferrari confirmed) |
| AudioSweepSpike | PASS | Audio pipeline works (sine wave only) |
| ClutchParameterSweep | PASS | Runs but final_rpm=0.0 — hand-built init lacks combustion |
| PhysicsDrivenDriving | PASS | Gear+clutch+throttle produces physics RPM via vehicle inertia |

## CLI Interactive Controls (current mapping)
- `i/I` — toggle ignition
- `s/S` — toggle starter motor
- `a/w/W`/UP — increase throttle
- `z/Z`/DOWN — decrease throttle
- SPACE — throttle cut
- `e` — decrease dyno torque
- `d` — increase dyno torque
- `c` — release dyno (free-revving)
- `]`/`[` — shift up/down (auto-locks clutch when gear > 0)
