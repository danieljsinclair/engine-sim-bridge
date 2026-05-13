# Torque Display Analysis

**Researcher:** physics-researcher
**Date:** 2026-05-13
**Context:** Torque transfer display `[Eng: +000nm <--> +000nm: Drive]`

---

## 1. Power vs Torque: A Practical Explanation

**Torque** is rotational force -- how hard the engine twists. Measured in Nm (Newton-metres). It's what you feel when a car pushes you back in your seat.

**Power** is how fast you can deliver that twist. Power = Torque x Angular Velocity. A diesel engine producing 500 Nm at 2000 RPM makes less power than a petrol engine producing 300 Nm at 8000 RPM. The diesel has more *push*; the petrol has more *sustained push at high speed*.

For this display, we care about **torque**, not power. Torque is the physical quantity that flows through the drivetrain. Power is a derived metric.

**Key intuition**: Torque is what the engine *does right now*. A stalled engine at 5000 RPM producing 0 Nm has no torque transfer. An engine at 2000 RPM producing 400 Nm is doing real work.

---

## 2. How Torque Flows Through the Drivetrain

```
Engine crankshaft (combustion produces torque)
    |
    v
Clutch / Torque converter (couples engine to gearbox)
    |                        can slip (partial transfer)
    v
Gearbox input shaft
    |
    v  (torque multiplied by gear ratio)
Gearbox output shaft
    |
    v
Driveshaft / Propeller shaft
    |
    v  (torque multiplied by final drive / diff ratio)
Differential
    |
    v  (split between left and right wheel)
Wheels (torque converted to linear force via tyre radius)
    |
    v
Road (tractive force pushes vehicle)
```

**Key relationship:**
- `T_gearbox_output = T_engine * gear_ratio` (torque multiplication)
- `T_wheel = T_gearbox_output * diff_ratio` (further multiplication)
- `F_traction = T_wheel / tyre_radius` (torque becomes force)

During **acceleration**: Engine produces positive torque -> clutch transfers it -> gearbox multiplies it -> wheels push vehicle forward.

During **engine braking**: Wheels are turning faster than the engine wants to -> vehicle inertia drives the engine through the drivetrain -> engine resists (compression + pumping losses) -> this resistance *decelerates* the vehicle. The torque direction reverses through the drivetrain.

---

## 3. What engine-sim Actually Exposes

### 3.1 The Constraint Solver's Torque Data

After each physics step, the SCS constraint solver populates `Constraint::F_t[row][body]` with the reaction torques for each body involved in a constraint. This is the actual physical torque the constraint exerted.

**The clutch constraint** (`Transmission::m_clutchConstraint`) is the key:
- `m_clutchConstraint.F_t[0][0]` = torque applied to **engine crankshaft** (body 1)
- `m_clutchConstraint.F_t[0][1]` = torque applied to **vehicle mass** (body 2)

The J matrix is `[-1, 1]` on `v_theta`, so by Newton's third law, if the clutch exerts torque T on the engine, it exerts -T on the vehicle mass (equal and opposite).

**Access issue**: `m_clutchConstraint` is `protected` in `Transmission`. We need to add a public accessor method to read the clutch torque.

### 3.2 The Dynamometer's Torque Data

`Simulator::m_dyno` connects to the output crankshaft and measures the torque the dyno applies. `m_dyno.getTorque()` returns `F_t[0][0]` adjusted for rotation direction. This is already used in `getStats()`.

When the dyno is **disabled** (normal driving mode), it applies zero torque. When enabled, it's a braking/driving force on the crankshaft.

### 3.3 Vehicle Mass Body

The `Simulator::m_vehicleMass` rigid body represents the vehicle's inertia as seen at the clutch output, transformed through the gear ratio. Its `v_theta` relates to vehicle speed via `Vehicle::getSpeed()`. The drag constraint applies braking torque to this body.

### 3.4 What We Can Read (Currently)

| Source | Method | What it gives | Available? |
|--------|--------|---------------|------------|
| Engine crankshaft | `Engine::getSpeed()` -> RPM | Angular velocity | Yes |
| Engine throttle | `Engine::getThrottle()` | Throttle position 0-1 | Yes |
| Dyno torque | `m_dyno.getTorque()` | Dyno braking torque | Yes (dyno mode only) |
| Clutch torque on engine | `m_clutchConstraint.F_t[0][0]` | Torque clutch exerts ON engine | Need accessor |
| Clutch torque on vehicle | `m_clutchConstraint.F_t[0][1]` | Torque clutch exerts ON vehicle mass | Need accessor |
| Clutch pressure | `Transmission::getClutchPressure()` | 0-1 | Yes |
| Current gear | `Transmission::getGear()` | Gear index | Yes |
| Gear ratios | `Transmission::m_gearRatios[]` | Ratio per gear | Need accessor |
| Vehicle speed | `Vehicle::getSpeed()` | m/s | Yes |

### 3.5 Internal Units

engine-sim uses SI internally. Torque values in `F_t` are already in **Nm** (Newton-metres). The `units::torque(v, units::ft_lb)` function converts ft-lb to Nm. When engine parameters specify `MaxClutchTorque` in ft-lb, it's converted at initialization time.

---

## 4. Recommended Display Model

### 4.1 What We Can Actually Show

Given engine-sim's architecture, we can show the torque at **two points**:

1. **Engine-side torque** (Eng:) -- the torque the clutch constraint exerts on the engine crankshaft. This is the net torque transfer between engine and drivetrain at the clutch.

2. **Drivetrain-side torque** (Drive:) -- the torque the clutch constraint exerts on the vehicle mass body, scaled by the gear ratio and diff ratio to represent the torque at the wheel hub.

**Important nuance**: In engine-sim, the vehicle mass body's inertia already encodes the gear ratio (`I_vehicle = m_car * (tire_radius / (diff_ratio * gear_ratio))^2`). The clutch constraint torque on the vehicle mass is the torque *at the clutch output*, not at the wheels. To get wheel torque, we multiply by `diff_ratio * gear_ratio`.

However, for the display format `[Eng: +000nm <--> +000nm: Drive]`, showing the clutch-side torques makes more physical sense than showing wheel torques (which would be huge numbers in low gears due to multiplication).

**Recommendation**: Show both values as torque at the clutch:
- **Eng:** = torque the clutch exerts on the engine crankshaft
- **Drive:** = torque the clutch exerts on the vehicle mass body

These are equal in magnitude but opposite in sign (Newton's third law), so the display would show something like `[Eng: +350nm <--> -350nm: Drive]` where the sign convention is:
- Positive Eng = engine is being loaded (drivetrain is absorbing engine torque)
- Positive Drive = drivetrain is being pushed forward (engine driving)

Actually, let me reconsider the sign conventions to match the user's desired display.

### 4.2 Sign Convention for Display

The user wants:
- **Eng: green/positive** when engine is producing power
- **Eng: red/negative** when engine is being dragged/braked
- **Drive: green/positive** when pushing vehicle forward
- **Drive: red/negative** when wheels are pushing back (engine braking)

The clutch constraint's J matrix is `J = [-1, 1]` on `v_theta`. The constraint force `R = J^T * lambda`. So:
- `F_t[0][0]` (on engine, body 1) = `-lambda/dt` 
- `F_t[0][1]` (on vehicle, body 2) = `+lambda/dt`

Where `lambda` is the constraint impulse.

When the engine is driving the vehicle (acceleration):
- The clutch constrains the engine and vehicle mass to rotate together
- The engine wants to go faster than the vehicle mass (combustion is pushing it)
- The clutch transfers positive torque from engine to vehicle mass
- `F_t[0][0]` = negative (clutch brakes the engine)
- `F_t[0][1]` = positive (clutch drives the vehicle mass)

When engine braking (deceleration):
- Vehicle mass wants to go faster than the engine (vehicle inertia)
- The clutch resists, transferring torque from vehicle mass to engine
- `F_t[0][0]` = positive (clutch tries to speed up the engine)
- `F_t[0][1]` = negative (clutch brakes the vehicle mass)

**Display mapping:**
- `Eng: = -F_t[0][0]` (negate: positive = engine producing power, negative = engine being dragged)
- `Drive: = F_t[0][1]` (as-is: positive = pushing vehicle forward, negative = engine braking)

Both should show the same magnitude but same-sign values during normal driving:
- Acceleration: `Eng: +350nm <--> +350nm: Drive` (both green)
- Engine braking: `Eng: -80nm <--> -80nm: Drive` (both red)

Wait -- that's not right. During acceleration, `F_t[0][0]` is negative (clutch opposes engine rotation) and `F_t[0][1]` is positive (clutch pushes vehicle forward). If we negate the engine side:

- `Eng = -F_t[0][0]` = positive (green) -- correct
- `Drive = F_t[0][1]` = positive (green) -- correct

During engine braking:
- `Eng = -F_t[0][0]` = negative (red) -- correct  
- `Drive = F_t[0][1]` = negative (red) -- correct

This gives us the desired display behavior where both values have the same sign and the sign indicates the direction of power flow.

---

## 5. How to Compute Each Value

### 5.1 Engine-Side Torque

```cpp
// In BridgeSimulator::getStats() or a new helper method
double engineTorque = 0.0;
if (m_simulator && m_simulator->getTransmission()) {
    // Need to add this accessor to Transmission:
    // const atg_scs::ClutchConstraint& getClutchConstraint() const
    //   { return m_clutchConstraint; }
    
    const auto& clutch = m_simulator->getTransmission()->getClutchConstraint();
    // F_t[0][0] = torque on engine crankshaft from clutch
    // Negate for display convention: positive = engine producing power
    engineTorque = -clutch.F_t[0][0];
}
stats.engineTorqueNm = engineTorque;
```

### 5.2 Drivetrain-Side Torque

```cpp
double drivetrainTorque = 0.0;
if (m_simulator && m_simulator->getTransmission()) {
    const auto& clutch = m_simulator->getTransmission()->getClutchConstraint();
    // F_t[0][1] = torque on vehicle mass from clutch
    // Positive = pushing vehicle forward, negative = engine braking
    drivetrainTorque = clutch.F_t[0][1];
}
stats.drivetrainTorqueNm = drivetrainTorque;
```

### 5.3 When Each Should Be Green vs Red

| Condition | Eng sign | Drive sign | Eng color | Drive color |
|-----------|----------|------------|-----------|-------------|
| Accelerating (throttle on) | positive | positive | green | green |
| Cruising (steady speed) | ~0 | ~0 | neutral | neutral |
| Engine braking (throttle off, in gear) | negative | negative | red | red |
| Neutral / clutch disengaged | ~0 | ~0 | neutral | neutral |
| Clutch slip during shift | reduced | reduced | transitional | transitional |

**Color logic:**
```cpp
auto torqueColor = [](double torque) -> Color {
    if (std::abs(torque) < threshold) return Color::Neutral;  // grey/white
    return torque > 0 ? Color::Green : Color::Red;
};
```

### 5.4 Required Code Changes

1. **Add accessor to `Transmission`** (engine-sim layer):
   ```cpp
   // transmission.h, in public section:
   const atg_scs::ClutchConstraint& getClutchConstraint() const 
       { return m_clutchConstraint; }
   ```

2. **Update `BridgeSimulator::getStats()`** (bridge layer):
   Read clutch F_t values and populate `engineTorqueNm` and `drivetrainTorqueNm`.

3. **Update display code** (presentation layer):
   Read the torque values and format as `[Eng: %+03.0fnm <--> %+03.0fnm: Drive]` with color.

---

## 6. Edge Cases

### 6.1 During Gear Changes (Clutch Disengaged)

When `m_clutchPressure = 0.0` (during shift):
- `ClutchConstraint` limits are `[-0, +0]` -- no torque transfer
- Both `F_t[0][0]` and `F_t[0][1]` will be ~0
- Display: `[Eng: +000nm <--> +000nm: Drive]` (neutral/grey)

During partial clutch engagement (shift ramp):
- Torque transfers proportionally to pressure
- Both values will be reduced from full-engagement values
- Display: transitional values, possibly flickering color

### 6.2 At Idle (No Torque Transfer)

When the engine is idling in neutral (`gear == -1`):
- Clutch limits are zeroed in `Transmission::update()`
- No torque transfer
- Display: `[Eng: +000nm <--> +000nm: Drive]`

When idling in gear with clutch engaged:
- Engine produces just enough torque to overcome internal friction
- Vehicle might be stationary (brakes on) or creeping
- Small positive engine torque balanced against drag
- Display: very small positive values `[Eng: +015nm <--> +015nm: Drive]`

### 6.3 During Engine Braking (Deceleration)

When throttle is released while in gear and moving:
- Engine compression and pumping losses create resistance
- Vehicle inertia drives the engine through the drivetrain
- Clutch torque reverses: `F_t[0][0]` becomes positive (clutch pushes engine faster), `F_t[0][1]` becomes negative (clutch slows vehicle)
- Display: `[Eng: -080nm <--> -080nm: Drive]` (both red)

The magnitude of engine braking torque depends on:
- Engine speed (higher RPM = more pumping resistance)
- Engine displacement (larger = more braking torque)
- Throttle position (closed throttle = maximum vacuum = maximum braking)

### 6.4 What the Torque Converter Does to the Relationship

engine-sim does NOT model a torque converter -- it uses a friction clutch (`ClutchConstraint`). This matters because:

**Real torque converter:**
- Can multiply torque at low speed ratios (stall): up to 2-3x
- Allows slip continuously (not just binary engaged/disengaged)
- Creates different torques on input vs output sides

**engine-sim's clutch:**
- No torque multiplication -- same torque on both sides (Newton's 3rd law)
- Slip only when torque demand exceeds `maxClutchTorque * pressure`
- Binary-ish behavior (engaged or slipping)

**Impact on display:**
- During normal driving (clutch fully engaged): `|Eng| == |Drive|` exactly
- During clutch slip: `|Eng|` and `|Drive|` may differ because the constraint hits its torque limit
- We will NOT see torque multiplication during launch -- this is a known limitation

For the ZF 8HP simulation context:
- Lock-up clutch is engaged from 2nd gear onwards in normal driving
- In lock-up mode, torque converter is effectively a rigid coupling (same as engine-sim's clutch)
- Only 1st gear launch would differ, and we're not modeling that torque converter behavior

---

## 7. Alternative Approach: Computing Torque from Angular Acceleration

If the F_t accessor proves problematic (e.g., timing issues with constraint solver updates), we can compute torque indirectly:

**Engine torque estimate** (from Newton's 2nd law for rotation):
```
T_net = I_crank * alpha_crank
T_engine = T_net + T_clutch + T_friction
```

Where:
- `I_crank` = crankshaft moment of inertia (known from `Crankshaft::getMomentOfInertia()`)
- `alpha_crank` = crankshaft angular acceleration (derivative of `v_theta`)
- `T_clutch` = clutch torque on engine (from F_t)
- `T_friction` = bearing friction (from `Crankshaft::getFrictionTorque()`)

This is more complex and less direct than reading `F_t`. **Prefer the F_t approach.**

---

## 8. Summary

| Question | Answer |
|----------|--------|
| Can we show engine-side torque? | Yes, via `m_clutchConstraint.F_t[0][0]` (negate for display) |
| Can we show drivetrain-side torque? | Yes, via `m_clutchConstraint.F_t[0][1]` |
| Are values in Nm? | Yes, engine-sim uses SI internally |
| Is there torque multiplication? | No -- clutch constraint obeys Newton's 3rd law (same magnitude both sides) |
| What about during shifts? | Clutch disengaged = ~0 torque on both sides |
| What about engine braking? | Both values go negative (same sign) |
| Code changes needed? | Add `getClutchConstraint()` accessor to Transmission; update `getStats()` |

**Bottom line**: engine-sim's constraint solver gives us exactly what we need via `F_t`. The only code change required in engine-sim itself is a single public accessor method on Transmission.
