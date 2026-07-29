# X-Engineer Ch7: Simulation Results

**Source**: [Modeling and simulation of a vehicle with automatic transmission - Chapter 7](https://x-engineer.org/modeling-simulation-vehicle-automatic-transmission/7/)

**Platform**: Scilab/Xcos

**Simulation duration**: 50 s per scenario

**Vehicle**: Mercedes 5.0L V8 (225 kW @ 5600 rpm, 460 Nm @ 3000-4250 rpm), RWD, 7-speed automatic with torque converter (Sachs s244, always unlocked in this model)

---

## 1. Acceleration Performance (Full Throttle)

### 0-100 kph Time

- **8 seconds** from standstill to 100 kph under full throttle acceleration.
- Accelerator pedal goes from 0% to 100% between t=0s and t=1s (ramp input).

### Quarter Mile (~402 m)

- Not directly stated. Distance at t=28s is approximately **1000 m**.
- Interpolating: quarter mile (~402 m) is reached at roughly **12-14 s** based on the acceleration profile.

### 1000 m Time

- **28 seconds** from standstill to cover 1000 m during full acceleration.

### Top Speed

- **Not reached** within 50 s. Speed at t=50s is approximately **230 kph**.
- The article notes: "The maximum speed is not reached in this scenario due to limited simulation time. To get the maximum speed value increase the simulation time to 100 s."
- Based on the speed curve flattening, top speed is estimated near **240-250 kph**.

### Maximum Acceleration

- **4.5 m/s^2** in first gear (approximately 0.46 g).
- The acceleration is flat at maximum for several seconds because traction force is limited by the **friction limit** (wheel slip limit), not engine torque.

---

## 2. Gear Shift Points (Full Throttle)

Gear shifts during full-throttle acceleration occur at the following times:

| Shift Event | Time (s) | From Gear | To Gear | Time in Gear (s) |
|-------------|----------|-----------|---------|-------------------|
| 1-2 upshift | ~5       | 1         | 2       | ~5                |
| 2-3 upshift | ~10      | 2         | 3       | ~5                |
| 3-4 upshift | ~15      | 3         | 4       | ~5                |
| 4-5 upshift | ~30      | 4         | 5       | ~15               |
| 5-6 upshift | >50      | 5         | 6       | not in sim window |
| 6-7 upshift | >50      | 6         | 7       | not in sim window |

Key observations:
- First three shifts are evenly spaced at ~5 s intervals.
- The 4-5 upshift occurs at ~30 s, meaning 4th gear is held for ~15 s (the vehicle spends longer in higher gears due to reduced acceleration).
- Only 4 upshifts occur within 50 s; 5th, 6th, and 7th gears are not reached.
- All shifts are sequential (1-2, 2-3, 3-4, 4-5).

### Estimated Vehicle Speed at Each Shift Point

Based on the speed chart and gear ratio analysis:

| Shift Event | Approx. Speed (kph) | Approx. Speed (mph) |
|-------------|---------------------|---------------------|
| 1-2 upshift | ~50-55              | ~31-34              |
| 2-3 upshift | ~85-90              | ~53-56              |
| 3-4 upshift | ~120-125            | ~75-78              |
| 4-5 upshift | ~170-180            | ~106-112            |

These can be cross-referenced with the upshift line data at 100% throttle from the shift map (see Section 8 below).

---

## 3. RPM Behavior During Acceleration

### Saw-Tooth Pattern

The engine RPM trace shows a **classic saw-tooth pattern** during full-throttle acceleration:
- RPM climbs from ~1000 rpm (idle) toward peak as the vehicle accelerates in each gear.
- At each upshift, RPM drops sharply due to the lower gear ratio of the next gear.
- RPM then climbs again in the new gear.

### Peak and Drop Values

| Shift Event | Peak RPM (before shift) | RPM Drop | Post-Shift RPM |
|-------------|-------------------------|----------|----------------|
| 1-2 upshift | ~6200                   | ~1800    | ~4400          |
| 2-3 upshift | ~6000                   | ~1600    | ~4400          |
| 3-4 upshift | ~6200                   | ~1800    | ~4400          |
| 4-5 upshift | ~6300                   | ~1500    | ~4800          |

RPM axis range in the simulation: **1000 to 6500 rpm**.

The engine max RPM is set to 7000 rpm (hard limit) but shift points keep it below this. The idle speed floor is 1000 rpm.

### Key Insight for Our Model

The RPM drop magnitude varies with the gear ratio step between gears. Larger ratio steps (e.g., 1st to 2nd at 4.381 -> 2.860) cause larger RPM drops. The drop is less severe in 4-5 (1.363 -> 1.000) because the ratio step is smaller.

---

## 4. Shift Timing

### Instantaneous Shifts (Simplification)

The simulation uses **instantaneous gear changes** -- the gear ratio steps are modelled as discrete jumps, not continuous ramps.

Consequences of this simplification:
- **Torque spikes** occur at each shift point in the turbine torque and traction force plots.
- **Acceleration spikes** appear in the vehicle acceleration trace at shift points.
- The article acknowledges this limitation: "In reality, the gearshift happens through clutch-to-clutch handover, the gear ratio ramps up/down linearly and the torque output is smooth."

### Minimum Time in Gear

The TCU enforces a **minimum time in gear** correction:
- **2 seconds** minimum for upshifts.
- **1 second** minimum for downshifts.

This prevents rapid successive shifts and ensures gear holding behavior.

### Other Shift Corrections Active During Full Acceleration

1. **Accelerator pedal tip-in** at t=1s: inhibits a 1-2 upshift briefly when the driver floors the pedal.
2. **Minimum time in gear (upshift)**: active at every shift, enforces the 2-second hold.

---

## 5. Braking Scenario Results

### Scenario Setup

- Phase 1 (0-15s): Accelerator ramps to 60%, vehicle picks up speed.
- Phase 2 (15-20s): Accelerator released gradually.
- Phase 3 (20-40s): Brake pedal pressed to 35%. Road slope = 0%.
- Phase 4 (40-50s): Small re-acceleration to ~40% throttle.

### Downshift Behavior

- **4-3 downshift at t=37s** during braking deceleration.
- Only one downshift observed. The 3-2 downshift is delayed by the "minimum time in gear - downshift" correction at 37s.
- No 2-1 downshift occurs within the simulation window.

### Dynamic Corrections Active

1. **Accelerator pedal tip-out** at t=15s: inhibits 3-4 upshift.
2. **Engine brake inhibit** between t=20s and t=40s: inhibits any upshift during overrun (engine braking). This keeps the current gear for engine braking effect.
3. **Minimum time in gear (downshift)** at t=37s: delays a potential 3-2 downshift.

### Engine Speed During Braking

- When the accelerator is released, engine speed drops to approximately **idle speed (1000 rpm)**.
- The engine remains connected to the gearbox through the unlocked torque converter, so there is a small **drag torque** keeping engine speed slightly above idle.
- Engine torque goes negative (around **-5 Nm**) during overrun due to torque converter slip.

### Vehicle Speed

- Steep decrease during braking (35% brake = ~3500 N braking force, since transfer function gain K=100, max brake force = 10000 N at 100%).
- Small re-acceleration when throttle is re-applied at t=40s.

---

## 6. Hill Climb Scenario Results

### Scenario Setup

- Accelerator pedal: 0% -> 30% over 2s, held at 30% until t=30s, then drops to 20%.
- Brake pedal: 0% for entire simulation.
- Road slope: 0% until t=5s, then jumps to **12% grade** at t=10s and holds.

### Downshift Behavior

- **4-3 downshift at t=35s** due to increased road load from the 12% grade.
- The downshift increases traction torque at the wheels, reducing the rate of deceleration.

### Gear Holding Behavior

- The transmission holds gear until the road load forces the vehicle speed to drop below the downshift line threshold.
- No special "hill hold" logic -- the shift is purely driven by the static shift map intersecting the reduced vehicle speed.

### Engine Speed

- Engine speed gradually decreases, tracking vehicle speed decline.
- Since positive torque is always applied (throttle never released to 0%), engine speed stays well above idle.
- A small **spike in engine speed** occurs at t=35s due to the 4-3 downshift (momentarily higher gear ratio drives engine faster through the driveline).

### Torque Converter Behavior

- Significant **speed slip** between engine (impeller) and turbine during hill climb.
- This represents substantial friction loss and heat generation in the torque converter.
- The article notes: "All modern torque converters have a lock-up clutch, which has the role to mechanically link the impeller with the turbine in order to improve efficiency. In order to keep it simple, this example considers that the torque converter is always unlocked."

### Dynamic Corrections Active

1. **Accelerator pedal tip-in** at t=1-2s.
2. **Minimum time in gear** corrections.
3. Same correction pattern as the braking scenario for the downshift event.

---

## 7. Numerical Data Points for Validation

### Vehicle Parameters

| Parameter | Value | Unit |
|-----------|-------|------|
| Vehicle mass | 2255 | kg |
| Wheel radius | 0.31587 | m |
| Final drive ratio | 2.769 | - |
| Driving axle load coefficient | 0.6 | - (60% rear) |
| Rotational inertia coefficient | 1.25 | - |
| Aerodynamic drag coefficient | 0.29 | - |
| Frontal area | 2.138 | m^2 |
| Air density | 1.202 | kg/m^3 |
| Friction coefficient (wheel) | 1.0 | - |
| Longitudinal shaft efficiency | 0.994 | - |

### Gear Ratios and Efficiencies

| Gear | Ratio (ix) | Efficiency |
|------|-----------|------------|
| 1    | 4.381     | 0.930      |
| 2    | 2.860     | 0.948      |
| 3    | 1.917     | 0.973      |
| 4    | 1.363     | 0.987      |
| 5    | 1.000     | 1.000      |
| 6    | 0.822     | 0.987      |
| 7    | 0.731     | 0.987      |

### Engine Parameters

| Parameter | Value | Unit |
|-----------|-------|------|
| Engine inertia | 0.08 | kg*m^2 |
| Max engine speed | 7000 | rpm |
| Min engine speed (idle) | 1000 | rpm |
| Max torque | 460 | Nm |
| Max power | 225 | kW |

### Torque Converter (Sachs s244)

| Parameter | Value | Unit |
|-----------|-------|------|
| Active diameter | 0.244 | m |
| Fluid density | 900 | kg/m^3 |
| Max torque coefficient (stall) | 2.163 | - |
| Max performance coefficient | 0.169 | - |

### Key Performance Benchmarks

| Metric | Value |
|--------|-------|
| 0-100 kph | ~8 s |
| 1000 m from standstill | ~28 s |
| Distance at t=50s (full accel) | ~2200 m |
| Max speed at t=50s | ~230 kph |
| Max acceleration (1st gear) | 4.5 m/s^2 |
| Peak RPM (WOT shifts) | ~6000-6300 rpm |
| Post-shift RPM recovery | ~4400-4800 rpm |
| Traction force friction limit | mv * g * mu * cw = 2255 * 9.81 * 1.0 * 0.6 = ~13,270 N |
| Max braking force (100% pedal) | 10,000 N |
| Brake transfer function | 100 / (0.1s + 1) |

### Traction Limit Calculation

```
F_lim = mv * g * mu * cw
      = 2255 * 9.81 * 1.0 * 0.6
      = 13,270 N
```

This is the maximum traction force that can be applied at the wheel before slip. The article notes that in reality ESP would intervene.

---

## 8. Shift Map Data (from Chapter 6)

### Upshift Speed Limits at 100% Throttle

| Upshift | Speed (kph) |
|---------|-------------|
| 1 -> 2  | 58          |
| 2 -> 3  | 91          |
| 3 -> 4  | 135         |
| 4 -> 5  | 192         |
| 5 -> 6  | 263         |
| 6 -> 7  | 320         |

### Downshift Speed Limits at 0% Throttle (Coast)

| Downshift | Speed (kph) |
|-----------|-------------|
| 2 -> 1    | 0           |
| 3 -> 2    | 10          |
| 4 -> 3    | 16          |
| 5 -> 4    | 24          |
| 6 -> 5    | 37          |
| 7 -> 6    | 50          |

### Upshift Speed Limits at 85% Throttle (Hill Climb Approx.)

| Upshift | Speed (kph) |
|---------|-------------|
| 1 -> 2  | 55          |
| 2 -> 3  | 89          |
| 3 -> 4  | 135         |
| 4 -> 5  | 192         |

Note: the shift lines at 85% and 100% throttle are nearly identical for the upper gears, indicating the shift map is relatively flat at high throttle positions.

### Downshift Speed Limits at 20% Throttle (Hill Climb Phase 2 Approx.)

| Downshift | Speed (kph) |
|-----------|-------------|
| 4 -> 3    | 17          |
| 5 -> 4    | 27          |
| 3 -> 2    | 11          |

---

## 9. Charts and Graphs Described

The article contains the following charts for each scenario. All charts have time (0-50 s) on the x-axis.

### Full Acceleration Scenario Charts

| Chart | Y-Axis Range | Description |
|-------|-------------|-------------|
| Input pedals | 0-100% | Accel pedal 0->100% at t=1s, brake = 0% |
| Vehicle speed | 0-240 kph | S-curve reaching ~230 kph at 50s, 100 kph at ~8s |
| Current gear | 1-7 | Step function: 1(0-5s), 2(5-10s), 3(10-15s), 4(15-30s), 5(30-50s) |
| Gear inhibits | binary flags | Tip-in inhibit at 1s, min-time holds at each shift |
| Speed shift limits | 0-350 kph | Shows upshift/downshift thresholds vs actual speed |
| Engine speed | 1000-6500 rpm | Saw-tooth pattern with 4 peaks |
| Engine torque | -50 to 460 Nm | Max torque in 1st gear, decreasing in higher gears |
| Engine power | 0-~200 kW | Follows torque profile scaled by RPM |
| Turbine speed | 0-6500 rpm | Below engine speed during acceleration (positive slip) |
| Turbine torque | 0-~1000 Nm | Torque amplification at low speed ratio; spikes at shifts |
| Traction force | 0-~13000 N | Capped at friction limit in 1st gear |
| Vehicle acceleration | -1 to 5 m/s^2 | Max 4.5 m/s^2, decreasing with each upshift |
| Vehicle offset | 0-2200 m | Distance traveled; reaches 1000m at ~28s |

### Braking Scenario Charts

| Chart | Key Features |
|-------|-------------|
| Vehicle speed | Rise to ~100 kph, then steep drop during braking, then small rise |
| Current gear | Upshifts during accel, 4-3 downshift at t=37s |
| Gear inhibits | Tip-out at 15s, engine brake inhibit 20-40s |
| Engine speed | Drops to ~1000 rpm (idle) during coast/braking |
| Engine torque | Goes negative (~-5 Nm) during overrun |

### Hill Climb Scenario Charts

| Chart | Key Features |
|-------|-------------|
| Vehicle speed | Rises initially, then declines after grade applied at t=10s |
| Current gear | 4-3 downshift at t=35s |
| Engine speed | Gradual decline with small spike at 35s downshift |
| Turbine speed | Significant slip vs engine speed (inefficiency) |
| Road slope | 0% until t=10s, then 12% |

---

## 10. Key Takeaways for Our Gearbox Model Validation

### Behavior to Replicate

1. **Saw-tooth RPM pattern**: Engine RPM should show clear rise-drop-rise behavior at each shift, with drops proportional to the gear ratio step.

2. **Shift schedule**: At WOT, shifts should occur at specific speed thresholds from the shift map (58, 91, 135, 192, 263, 320 kph). Our model should hit these same thresholds.

3. **Acceleration profile**: Max ~4.5 m/s^2 in 1st gear, decreasing with each upshift. The first gear acceleration should be flat (traction-limited) not torque-limited.

4. **Downshift on deceleration**: A 4-3 downshift should occur when speed drops below the downshift threshold for the current throttle position.

5. **Engine braking**: When throttle is released, engine speed should drop to idle (~1000 rpm) and engine torque should go slightly negative.

6. **Minimum time in gear**: 2s for upshifts, 1s for downshifts prevents shift hunting.

### Simplifications to Be Aware Of

- **Instantaneous shifts**: The x-engineer model uses instant gear changes. Our model should consider implementing ramped shifts (clutch-to-clutch) for more realistic torque traces.
- **No lock-up clutch**: The torque converter is always unlocked, which overestimates fuel consumption and slip losses, especially at highway speeds.
- **Sequential shifts only**: The model only allows sequential shifts (N -> N+1 or N -> N-1). Real transmissions can skip gears on heavy braking (e.g., 6-4 downshift).
- **Simple friction limit**: Wheel traction is hard-capped. A real model would use ESP/TCS to modulate engine torque.

### Torque Converter Amplification

At stall (speed ratio = 0), the torque converter provides **2.163x** torque multiplication. This is significant for launch performance and means the turbine torque at standstill launch can exceed the engine's rated torque.
