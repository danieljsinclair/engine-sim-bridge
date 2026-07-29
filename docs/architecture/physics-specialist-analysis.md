---
name: Physics Chain Analysis for VirtualICE Twin
description: Deep-dive into engine-sim's physics chain, constraint solver mechanics, and EV telemetry injection strategy
type: physics-specialist-analysis
originSessionId: 7046820a-eaba-4f82-a7c2-8cda7fb7c15a
---
# Physics Chain Analysis: engine-sim -> VirtualICE Twin

## 1. The Complete Physics Chain

### 1.1 Rigid Bodies in the System

engine-sim creates the following rigid bodies (from `PistonEngineSimulator::loadSimulation`):

| Body | Mass | Moment of Inertia | Role |
|------|------|-------------------|------|
| `crankshaft[i].m_body` | crankshaft mass + flywheel mass | crankshaft MOI | Rotating assembly |
| `piston[i].m_body` | piston mass | 1.0 (irrelevant) | Translating mass |
| `connectingRod[i].m_body` | rod mass | rod MOI | Couples piston to crank |
| `m_vehicleMass` | vehicle mass (via `changeGear`) | `m_car * f^2` | Vehicle inertia at crankshaft |

### 1.2 Constraint Graph

```
CombustionChamber (ForceGenerator)
    |
    v
Piston body <---> CylinderWall (LineConstraint, constrains piston to cylinder axis)
    |
    v (via LinkConstraint at wrist pin)
ConnectingRod body
    |
    v (via LinkConstraint at crank pin)
Crankshaft body <--- FixedPositionConstraint (pins crank center in space)
    |                  RotationFrictionConstraint (bearing friction)
    |
    v (via ClutchConstraint: crankshaft <-> vehicleMass)
VehicleMass body <--- VehicleDragConstraint (aero drag + rolling resistance)
```

The output crankshaft (index 0) connects through the clutch to the vehicle mass body. Additional crankshafts are linked to the output via `ClutchConstraint`s (rigid links, no slip).

### 1.3 Force Generation Path (Throttle -> RPM)

```
setThrottle(position) or setSpeedControl(position)
    |
    v
Engine::update(dt) -> Throttle::update(dt, engine) [base class is no-op; script may override]
    |
    v
Intake::m_throttle = position  [sets throttle plate angle -> restricts airflow]
    |
    v
CombustionChamber::flow(dt)    [gas dynamics: intake -> cylinder -> exhaust]
    |                            airflow limited by throttle position
    v
CombustionChamber::ignite()    [ignition module fires at correct crank angle]
    |                            fuel burns based on AFR, turbulence, efficiency
    v
CombustionChamber::apply(system) [ForceGenerator]
    |  F = -A * (P_cylinder - P_crankcase)    [pressure force on piston]
    |  F_friction = Coulomb + viscous model     [piston ring friction]
    |  Applied to piston body via system->applyForce()
    v
Piston force -> LinkConstraint -> ConnectingRod -> LinkConstraint -> Crankshaft torque
    |  [Geometric coupling via rigid body constraint solver]
    v
alpha_crank = (sum of all forces + constraint forces) / I_crank
omega_crank += alpha_crank * dt     [angular velocity evolves]
    |
    v
ClutchConstraint links crankshaft.v_theta <-> vehicleMass.v_theta
    |  Constraint torque limited by: m_maxClutchTorque * m_clutchPressure
    v
VehicleMass body:
    I_vehicle = m_car * (tire_radius / (diff_ratio * gear_ratio))^2
    Drag torque = linearForceToVirtualTorque(F_drag + F_rolling)
    |
    v
RPM = |crankshaft.m_body.v_theta| * 60 / (2*pi)   [EMERGENT OUTPUT]
```

### 1.4 Key Equations

**Vehicle inertia encoding** (in `changeGear`):
```
f = tire_radius / (diff_ratio * gear_ratio)
I_vehicle = m_car * f^2

Energy conservation on gear change:
E_rot = 0.5 * I_old * omega_old^2
omega_new = sqrt(2 * E_rot / I_new)
```

This is the rotational equivalent of: the vehicle has linear KE `0.5 * m * v^2`, which maps to rotational KE `0.5 * I * omega^2` through the drivetrain ratio.

**Vehicle speed recovery** (in `Vehicle::getSpeed`):
```
E_rot = 0.5 * I_vehicle * omega_vehicle^2
v_vehicle = sqrt(2 * E_rot / m_car)
```

Inverts the inertia encoding: `omega_vehicle = v * f / tire_radius` where `f` is the drivetrain ratio.

**Drag force** (in `VehicleDragConstraint::calculate`):
```
F_drag = 0.5 * rho * v^2 * C_d * A
F_rolling = rolling_resistance
torque_drag = linearForceToVirtualTorque(F_drag + F_rolling)
           = sqrt(I_vehicle / m_car) * (F_drag + F_rolling)
```

**Clutch constraint** (simplified):
```
J = [-1, 1]   [on crankshaft.v_theta, vehicleMass.v_theta]
C = 0          [no position error, velocity-only coupling]
v_bias = 0
limits = [-maxClutchTorque * pressure, +maxClutchTorque * pressure]
```

When `pressure = 1.0` and `maxClutchTorque` is large enough, the clutch acts as a rigid link: `omega_crank = omega_vehicle` (enforced by constraint solver).

## 2. The SCS Constraint Solver: How It Actually Works

### 2.1 The Math

The solver uses Sequential Impulses (similar to Box2D). For each constraint row `j`:

```
q_dot_prime = q_dot + M_inv * F_ext * dt

right_j = -(J * q_dot_prime + v_bias + (bias_factor / dt) * C)_j

Solve: J * M_inv * J^T * lambda = right     [subject to limits]
```

Then constraint forces:
```
R = J^T * (lambda / dt)
a = M_inv * (F_ext + R)
```

### 2.2 What ks and kd Do (or Don't Do)

**CRITICAL**: In `OptimizedNsvRigidBodySystem::processConstraints`, `ks` and `kd` are **never read from the constraint output**. The solver builds its equations from `J`, `v_bias`, `C`, and `limits` only. This means:

- `ks` and `kd` fields on constraints are vestigial/dead code in the optimized solver path
- Only `J`, `J_dot` (also unused in NSV path), `C`, `v_bias`, and `limits` matter
- The `bias_factor` (default 1.0) controls position error correction via `C`

### 2.3 How Limits Work

```
limits[j] = [min_torque * dt, max_torque * dt]
```

The solver clamps `lambda_j` to these bounds. After solving:
```
constraint_force = lambda / dt
```

So `limits` directly cap the impulse-based constraint force. This is how torque-bounded constraints work.

### 2.4 External Forces (F_ext)

`processForces()` zeros `f_x`, `f_y`, `t` on all bodies, then calls each `ForceGenerator::apply()`. The combustion chamber is the main force generator -- it applies pressure forces and friction to piston bodies via `system->applyForce()`.

In `processConstraints()`, F_ext is built into `q_dot_prime`:
```
q_dot_prime = q_dot + M_inv * F_ext * dt
```

This means external forces effectively predict where velocities will go, and constraints correct from that prediction.

## 3. Approach A: Full Vehicle Model Analysis

### 3.1 How It Works

Use engine-sim's existing vehicle + transmission + drag chain. Feed throttle and gear position.

**Inputs**: throttle position, gear selection
**Output**: RPM emerges from physics

### 3.2 Can We "Lie" About Vehicle Mass?

Yes. `Vehicle::m_mass` is set once during `initialize()` and read by:
1. `changeGear()` to compute `I_vehicle = m_mass * f^2`
2. `Vehicle::getSpeed()` to recover speed from rotational KE
3. `linearForceToVirtualTorque()` for drag conversion
4. `VehicleDragConstraint::calculate()` for drag torque limits

If we set `m_mass` to a fake value:
- **Lighter mass** -> smaller `I_vehicle` -> engine revs faster for same torque (more responsive)
- **Heavier mass** -> larger `I_vehicle` -> engine revs slower (more sluggish)

The problem: `m_mass` is used consistently everywhere, so faking it makes the virtual vehicle accelerate at a different rate than the real EV. The virtual vehicle speed diverges from real EV speed.

### 3.3 Syncing Virtual Speed with Real Speed

This is the fundamental challenge. Options:

**Option A1: Periodic speed correction**
After each simulation step, compare `Vehicle::getSpeed()` with real EV speed. If they diverge, apply a correction force to the vehicle mass body for one step. This is essentially a soft constraint.

Problem: Introduces non-physical forces. Could cause audible artifacts if correction is too aggressive.

**Option A2: Adaptive mass**
Dynamically adjust `m_mass` (and hence `I_vehicle`) so that the engine's natural torque output produces the correct acceleration profile. Before `changeGear`, recompute mass from:
```
I_needed = T_engine / alpha_target
m_virtual = I_needed / f^2
```
This requires predicting engine torque, which varies with RPM and throttle.

Problem: Mass changes during `changeGear` reset `v_theta` via energy conservation, which is disruptive.

**Option A3: Additional constraint (speed tracking)**
Create a new SCS constraint that biases `vehicleMass.v_theta` toward the target angular velocity corresponding to real EV speed. Similar to dyno but on the vehicle body instead of crankshaft.

This is the most physically consistent approach but requires understanding that the constraint will interact with the existing clutch + drag constraints.

### 3.4 Verdict on Approach A

Approach A works well when the virtual ICE vehicle's power curve roughly matches the real EV's torque delivery. For a Ferrari V8 simulating a high-performance EV, the torque curves may be similar enough that throttle + gear produces plausible RPM.

The weakness is drift: without speed correction, virtual and real vehicle speeds diverge over time. A soft correction constraint is needed.

## 4. Approach B: Dyno/Custom Constraint Analysis

### 4.1 Why the Dyno Creates Cliff Behavior

The dyno constraint:
```
J = [0, 0, 1]   [acts on crankshaft v_theta only]
C = 0
v_bias = +/- m_rotationSpeed   [target angular velocity]
limits = [-m_maxTorque, +m_maxTorque] or [0, +/- m_maxTorque]
```

In non-hold mode (`hold=false`), the dyno only brakes (one-sided limits). The constraint tries to push `v_theta` toward `m_rotationSpeed` but can only apply force in one direction. Below target RPM, the engine is free to accelerate. Above target, the dyno clamps hard.

This creates cliff behavior because:
1. Below target: no load, engine revs freely (unrealistic)
2. Above target: maximum brake torque applied instantly (binary)
3. No gradual load proportional to speed deviation

In hold mode (`hold=true`), the dyno is bidirectional -- it can both drive and brake. This makes it a velocity servo, not a load simulator. It fights the engine to maintain target RPM, which is not what we want.

### 4.2 Custom Fixed-Torque Constraint

A custom constraint that applies a FIXED torque (independent of velocity) to the crankshaft:

```cpp
class FixedLoadConstraint : public atg_scs::Constraint {
    // 1 constraint row, 1 body (crankshaft)
    double m_loadTorque;  // the torque to apply

    void calculate(Output *output, SystemState *state) override {
        output->J[0][0] = 0;
        output->J[0][1] = 0;
        output->J[0][2] = 1;      // acts on v_theta

        output->J_dot[0][0] = 0;
        output->J_dot[0][1] = 0;
        output->J_dot[0][2] = 0;

        output->C[0] = 0;
        output->v_bias[0] = 0;
        output->ks[0] = 0;        // ignored anyway
        output->kd[0] = 0;        // ignored anyway

        // The trick: set limits to be exactly the desired torque
        output->limits[0][0] = -m_loadTorque;  // negative = braking torque
        output->limits[0][1] = 0;              // one-sided: only brakes
    }
};
```

**How this works**: The constraint solver solves `J * M_inv * J^T * lambda = right` with `lambda` clamped to `limits`. With `J = [0,0,1]` and no position/velocity error (C=0, v_bias=0), the constraint tries to produce zero force. But limits cap the force. The resulting constraint force is `lambda/dt`, clamped to the load torque.

Wait -- this is wrong. If there's no error (right side = 0), the solver will produce lambda = 0, and limits won't matter because 0 is within limits.

**Correct approach**: We need to create an error that the solver corrects, and cap the correction. Set `v_bias` to a large value:

```cpp
output->v_bias[0] = -1e6;   // huge bias toward deceleration
output->limits[0][0] = -m_loadTorque;  // cap the correction at load torque
output->limits[0][1] = 0;
```

The solver computes `right = -(J * q_dot_prime + v_bias)` which will be large positive (wanting to decelerate). But lambda is capped at `m_loadTorque * dt`. So the effective braking torque is exactly `m_loadTorque`.

**This gives us a fixed, velocity-independent load torque on the crankshaft.** We update `m_loadTorque` each frame from EV telemetry.

### 4.3 Deriving Load Torque from EV Telemetry

The real EV provides motor torque. For sound generation, we need the ICE crankshaft to spin at the RPM implied by the gear ratio and road speed. The load torque must balance:

```
T_engine - T_load = I_crank * alpha
```

At steady state (constant RPM): `T_load = T_engine`. During acceleration: `T_load = T_engine - I_crank * alpha`.

We don't know T_engine (it's the ICE output). We do know the EV motor torque. We can use EV motor torque as a proxy for the "load" that the road + vehicle mass presents to the engine. But the mapping is not direct because ICE and EV have different torque characteristics.

A practical approach:
1. Compute target RPM from: `RPM_target = v_road * gear_ratio * diff_ratio * 60 / (2*pi * tire_radius)`
2. Compute load torque needed to track this RPM: use a PD controller on RPM error
3. Apply as fixed load constraint

This blends physics (engine torque production is real) with control (load torque drives RPM toward target).

## 5. Torque Converter vs Rigid Clutch

### 5.1 What a Torque Converter Does

A ZF torque converter has three elements:
- **Impeller** (connected to engine)
- **Turbine** (connected to transmission input)
- **Stator** (one-way clutch, multiplies torque at low speed ratios)

Key characteristics:
- **Slip**: Turbine speed < impeller speed, especially at low speed ratios
- **Torque multiplication**: At stall, output torque can be 2-3x input torque
- **Lockup**: Above ~30-50 mph, a lockup clutch eliminates slip for efficiency

### 5.2 Do We Need Torque Converter Slip for Sound Quality?

**Probably not.** Here's why:

1. **Most driving is in lockup**: The ZF lockup clutch engages early (often in 2nd gear above ~20 mph). During normal driving, the converter is rigid.

2. **Slip is most relevant at low speed**: Launch from stop, creeping in traffic. At these speeds, the engine is near idle and sound is minimal.

3. **The clutch constraint already models partial engagement**: `setClutchPressure(0.0 to 1.0)` varies the torque capacity. At low pressure, the clutch slips -- the engine can spin faster than the transmission input. This is functionally similar to torque converter slip (though the physics differ: Coulomb friction vs fluid coupling).

4. **Sound impact**: Engine sound is dominated by RPM and load. Torque converter slip allows the engine to rev higher than wheel speed would imply. We can approximate this effect by modulating clutch pressure during launch:
   - `pressure = 0.0` at standstill -> engine free-revs to stall speed
   - `pressure = ramp(0.0 -> 1.0)` over ~1-2 seconds during launch
   - `pressure = 1.0` once moving -> rigid lockup

### 5.3 Recommendation

Start with rigid clutch lockup (`pressure = 1.0`). Add launch slip via pressure modulation only if sound testing reveals it matters. A full fluid coupling model is overengineering at this stage.

## 6. External Force Injection: F_ext Analysis

### 6.1 How External Forces Work

From `processForces()` and `processConstraints()`:

```
1. processForces(): zero f_x, f_y, t on all bodies, then call each ForceGenerator::apply()
   -> CombustionChamber::apply() writes to state->t[piston_body.index]

2. processConstraints(): builds F_ext from state->f_x, f_y, t arrays
   q_dot_prime = q_dot + M_inv * F_ext * dt
   -> This is the "predicted" velocity after external forces
   -> Constraints then correct from this prediction
```

### 6.2 Can We Inject Torque Directly?

Yes. We have two mechanisms:

**Mechanism 1: ForceGenerator**

Create a custom `ForceGenerator` that applies torque to the crankshaft body:

```cpp
class ExternalTorqueGenerator : public atg_scs::ForceGenerator {
    double m_torque;
    int m_bodyIndex;

    void apply(SystemState *system) override {
        system->t[m_bodyIndex] += m_torque;
    }
};
```

Add it to the system via `m_system->addForceGenerator(&myGen)`. This adds torque to `F_ext`, which is built into `q_dot_prime` before constraint solving. The constraint solver then enforces all constraints (clutch, drag, etc.) on top of this external torque.

This is the cleanest injection point. The torque is applied before constraint solving, so it interacts correctly with all constraints.

**Mechanism 2: Direct state mutation**

Write directly to `state->t[body_index]` during the simulation step. This is fragile because `processForces()` zeros the arrays first.

**Mechanism 1 is recommended** -- it uses the intended extension point.

### 6.3 Where to Inject

For the VirtualICE twin, we want to inject load torque (braking) to control RPM. The injection point depends on approach:

- **Approach A (full vehicle)**: Don't inject torque. Instead, let physics produce RPM naturally. Use a soft speed-tracking constraint on the vehicle mass body to prevent drift.

- **Approach B (custom load)**: Inject via a custom constraint on the crankshaft. The FixedLoadConstraint from section 4.2 is the right tool.

- **Hybrid**: Use the full vehicle model but add a ForceGenerator that applies a correction torque to the vehicle mass body when virtual speed diverges from real speed.

## 7. Automatic Gearbox Interface

### 7.1 How changeGear() Works

`changeGear(newGear)` does the following:
1. If `newGear == -1` (neutral): sets clutch torque limits to 0 (disengages)
2. Otherwise:
   - Computes new vehicle inertia: `I_new = m_car * (tire_radius / (diff_ratio * gear_ratio))^2`
   - Conserves kinetic energy: `omega_new = sqrt(E_rot * 2 / I_new)`
   - Updates `m_vehicleMass.I`, `m_vehicleMass.m`, `m_vehicleMass.v_theta`
   - Sets `m_gear = newGear`

The gear change is instantaneous -- there's no shift delay or slip phase. The clutch constraint limits are updated in `Transmission::update()` based on `m_clutchPressure`.

### 7.2 Modeling ZF Shift Behavior

A real ZF automatic has:
1. **Torque phase** (~100-300ms): one clutch releases while another applies, torque transfer is partial
2. **Inertia phase** (~100-200ms): speed ratio changes, engine RPM drops to match new gear
3. **Lockup**: fully engaged in new gear

To model this with engine-sim's manual transmission:

```
Phase 1 (torque phase):
  - m_clutchPressure ramps from 1.0 -> 0.3  (partial slip)
  - Wait ~150ms

Phase 2 (inertia phase):
  - changeGear(newGear)           (instantaneous ratio change + energy conservation)
  - m_clutchPressure = 0.3       (still slipping, allows RPM to settle)

Phase 3 (lockup):
  - m_clutchPressure ramps 0.3 -> 1.0 over ~200ms
  - Full lockup
```

This gives a plausible shift feel without modifying engine-sim internals. The clutch slip during the shift allows the engine RPM to smoothly transition between gear ratios.

### 7.3 Gear Selection Logic

The bridge layer (not engine-sim) owns the shift logic:
- Input: real EV road speed + throttle position
- Shift curves: map (speed, throttle) -> target gear
- Throttle-heavy driving -> hold lower gears longer (higher RPM, more aggressive sound)
- Cruise/light throttle -> shift up early (lower RPM, efficient sound)

Call `changeGear(target)` and `setClutchPressure(phase)` from the bridge layer each frame.

## 8. Recommended Integration Point

### 8.1 Recommended: Approach A+ (Full Vehicle + Soft Speed Tracking)

**Rationale**: Approach A leverages engine-sim's existing physics chain, which already handles the ICE torque curve, combustion dynamics, and exhaust sound generation correctly. The only addition needed is a speed-tracking mechanism to prevent drift between virtual and real vehicle speeds.

**Architecture**:

```
Real EV Telemetry (OBD)
  |
  v
VirtualICE Twin (bridge layer)
  |- Gear selector: (speed, throttle) -> gear, via shift curves
  |- Throttle mapper: EV throttle -> ICE throttle (may need curve adjustment)
  |- Speed tracker: compares virtual vs real speed
  |    |
  |    v  (if divergence > threshold)
  |    Apply correction torque via ForceGenerator on vehicleMass body
  |
  v
engine-sim inputs:
  - Engine::setThrottle(throttle)
  - Engine::getIgnitionModule()->m_enabled = true
  - Transmission::changeGear(gear)
  - Transmission::setClutchPressure(1.0)  [or modulated during shifts]

engine-sim outputs:
  - RPM = Engine::getRpm()  [EMERGENT from physics]
  - Audio = Synthesizer output
```

### 8.2 Speed Tracking ForceGenerator

```cpp
class SpeedTrackingForce : public atg_scs::ForceGenerator {
    Vehicle *m_vehicle;
    RigidBody *m_vehicleMass;
    double m_gain;           // PD gain (start with ~100)
    double m_maxCorrection;  // cap correction force

    void apply(SystemState *system) override {
        double virtualSpeed = m_vehicle->getSpeed();
        double speedError = virtualSpeed - m_targetSpeed;

        // Convert speed error to torque on vehicle mass body
        double correctionTorque = m_vehicle->linearForceToVirtualTorque(
            -m_gain * speedError    // P-controller: push toward target speed
        );

        // Clamp
        correctionTorque = std::max(-m_maxCorrection,
                           std::min(m_maxCorrection, correctionTorque));

        system->t[m_vehicleMass->index] += correctionTorque;
    }
};
```

This adds a restoring force that pushes the virtual vehicle speed toward the real EV speed. The gain should be tuned so the correction is subtle (doesn't override engine physics) but prevents long-term drift.

### 8.3 Alternative: Approach B (Fixed Load Constraint)

Use Approach B only if Approach A+ proves unable to track real EV behavior. Approach B gives direct RPM control but at the cost of decoupling from vehicle physics:

```
EV Telemetry -> target RPM -> PD controller -> load torque -> FixedLoadConstraint
```

This is simpler but less physically authentic. The engine sound will track RPM accurately but may lack the dynamic response that comes from full vehicle simulation (e.g., engine braking on deceleration, lugging under load at low RPM).

### 8.4 Files to Create/Modify

| File | Purpose |
|------|---------|
| `include/constraints/SpeedTrackingForce.h` | ForceGenerator for speed correction |
| `src/constraints/SpeedTrackingForce.cpp` | Implementation |
| `include/constraints/FixedLoadConstraint.h` | Alternative: fixed torque constraint |
| `src/constraints/FixedLoadConstraint.cpp` | Implementation |
| `src/twin/VirtualIceTwin.cpp` | Bridge logic: telemetry -> engine-sim inputs |
| `src/twin/GearSelector.cpp` | Shift curve logic |
| Existing `BridgeSimulator.cpp` | Add twin integration hooks |

No modifications to engine-sim core (constraint solver, combustion, etc.) are needed. All injection uses existing extension points (ForceGenerator, constraint API, Transmission/Vehicle accessors).

## 9. Summary of Key Findings

1. **RPM is emergent** from the constraint solver balancing engine torque against vehicle inertia + drag through the drivetrain ratio. It is never set directly.

2. **ks/kd are dead code** in the NsvOptimized solver. Only J, C, v_bias, and limits matter for constraints.

3. **Approach A+ (full vehicle + speed tracking)** is recommended. It preserves engine-sim's authentic combustion physics and only adds a gentle correction layer.

4. **Torque converter modeling is unnecessary** initially. Clutch pressure modulation during launch approximates the key behavior (engine slip at low speed).

5. **External forces inject cleanly** via ForceGenerator. This is the intended extension point and interacts correctly with the constraint solver.

6. **ZF shift behavior** can be modeled by ramping clutch pressure around `changeGear()` calls, all from the bridge layer.

7. **The dyno is the wrong abstraction** for driving simulation. Its velocity-dependent behavior is designed for measurement, not load simulation.
