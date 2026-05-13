# ZF 8HP45 Torque-Aware Shifting Analysis

**Date:** 2026-05-12
**Purpose:** Phase 2 analysis of torque-based shift modulation for the VirtualICE twin automatic gearbox
**Cross-reference:** `zf-shift-scheduling-research.md` (Phase 1 research)

---

## 1. Real ZF TCU Shift Decision Inputs

The ZF 8HP TCM processes the following real-time data for shift decisions:

### Primary inputs
- **Engine RPM** -- determines shift timing and torque management
- **Throttle position** -- driver intent, maps to shift thresholds
- **Torque request / output** -- ensures transmission operates within safe torque limits
- **Vehicle speed** -- gear selection and shift strategies
- **Gear selector position** -- mode (P/R/N/D/S)

### Advanced inputs (beyond speed + throttle)
- **Torque converter turbine speed** -- monitored by TCM to calculate converter clutch slip and internal clutch slip; controls slip timing during shifts and adjusts clutch application/release pressure
- **Inertia sensor (G-sensor)** -- central to "fuzzy logic" driver type assessment; detects longitudinal acceleration for grade detection (uphill = hold gear longer, downhill = earlier downshift for engine braking)
- **Throttle rate of change (d_throttle/dt)** -- rapid increase triggers early downshift (kickdown) before full throttle
- **Transmission fluid temperature** -- cold = smoother shifts, less lockup; hot = modified shift points
- **Vehicle acceleration/deceleration rates** -- stored and evaluated for driver pattern recognition

### Key TCU behaviors relevant to torque awareness

From the Gears Magazine analysis of ZF 8HP shift strategy:

1. **Driver Type Assessment** -- "fuzzy logic" uses inertia sensor data, engine torque demand, and acceleration/deceleration rates to build a driver behavior pattern. This tailors the selected driving program (economy/normal/sport) automatically.

2. **Drag Recognition** -- compares shift points going uphill vs downhill. The TCM detects grade via the G-sensor and adjusts shift thresholds: uphill = later shifts, downhill = earlier shifts.

3. **Warm-up Mode** -- delays shifts until transmission fluid reaches operating temperature (~86F+), which affects torque capacity of the clutches.

4. **Bidirectional torque coordination** -- During shifts, the TCM requests torque reductions from the engine ECU. The engine trims output to protect the transmission. This means the TCU knows current engine torque and factors it into shift decisions.

5. **Torque-based shift modulation formula** (from Phase 1 research):
   ```
   shift_speed_adjusted = shift_speed_base * (1 - torque_factor * load_factor)
   ```
   Where `torque_factor = current_engine_torque / max_engine_torque` and `load_factor` is a calibration constant (0.1-0.2).

---

## 2. What Our Current Model Uses

The current `AutomaticGearbox` class (`include/twin/AutomaticGearbox.h`) uses:

- **Speed (km/h)** -- primary axis for shift map lookup
- **Throttle fraction (0-1)** -- secondary axis for shift map lookup
- **Throttle rate of change** -- for kickdown detection (stored in `throttleDeltaHistory_`, `throttleDeltaTimeS_`)
- **Time since last shift** -- minimum shift interval to prevent hunting
- **Hysteresis** -- downshift thresholds at ~85% of upshift thresholds

**Not currently used:**
- Engine torque
- Engine load / manifold pressure
- Vehicle acceleration (the G-sensor equivalent)
- Grade / incline detection
- Transmission temperature

The `AutomaticGearbox::update()` signature is:
```cpp
void update(double dt, double speedKmh, double throttleFraction);
```

---

## 3. Torque Data Available from engine-sim

### 3.1 Engine torque (indirect, via crankshaft angular velocity)

engine-sim does NOT expose a direct `getEngineTorque()` method. The engine is a physics simulation that computes torque internally through combustion forces acting on pistons, but the result is not surfaced as a single scalar getter.

**Available indirect proxies:**

| Method | Location | Returns | Usefulness |
|--------|----------|---------|------------|
| `Engine::getRpm()` | `engine.h:397` | Current crankshaft RPM (from `v_theta`) | Already used via feedback loop |
| `Engine::getManifoldPressure()` | `engine.h:69` | Intake manifold pressure (Pa) | Good proxy for engine load |
| `Engine::getIntakeFlowRate()` | `engine.h:65` | Total intake air flow | Proportional to torque demand |
| `Engine::getThrottle()` | `engine.h:154` | Current throttle plate value (0-1) | Already available |
| `Engine::getSpeed()` | `engine.h:402` | Crankshaft angular velocity (rad/s) | Raw RPM source |

### 3.2 Dyno torque (only when dyno is active)

| Method | Location | Returns | Condition |
|--------|----------|---------|-----------|
| `Simulator::getFilteredDynoTorque()` | `simulator.h:77` | Rolling average of dyno brake torque (512 samples) | Only when `m_dyno.m_enabled == true` |
| `Dynamometer::getTorque()` | `dynamometer.h:15` | Instantaneous dyno reaction force | Only when dyno connected |
| `Simulator::getDynoPower()` | `simulator.h:78` | `torque * engine_speed` | Only when dyno active |

**Problem:** Dyno torque measures the *brake* force applied to hold the engine at a target RPM -- this is NOT the same as engine output torque during normal driving. When the dyno is disabled (normal driving mode), these values return 0.

### 3.3 Clutch torque

| Field | Location | Returns |
|-------|----------|---------|
| `Transmission::m_maxClutchTorque` | `transmission.h:41` | Maximum clutch torque capacity (static, not runtime) |
| `ClutchConstraint` reaction force | `constraint.h:40` | `F_t[0][0]` -- the solved constraint force | Accessible via `m_clutchConstraint.F_t` but requires reaching into SCS internals |

### 3.4 Vehicle kinetic energy and speed

| Method | Location | Returns |
|--------|----------|---------|
| `Vehicle::getSpeed()` | `vehicle.h:30` | Linear speed derived from rotational kinetic energy |
| `Vehicle::getMass()` | `vehicle.h:24` | Vehicle mass (constant) |

The vehicle speed is computed from `0.5 * I * v_theta^2 / mass`, which means the rigid body's `v_theta` (angular velocity) encodes the vehicle's current kinetic state.

### 3.5 Crankshaft friction torque

| Method | Location | Returns |
|--------|----------|---------|
| `Crankshaft::getFrictionTorque()` | `crankshaft.h:44` | Friction torque parameter (constant, not dynamic) |

### 3.6 Summary: What's practically accessible for torque-aware shifting

For the AutomaticGearbox, which runs in the twin layer (above the simulator, not inside it), the available data is what flows through `TwinFeedback`:

```cpp
struct TwinFeedback {
    double engineRpm = 0.0;
    double vehicleSpeedKmh = 0.0;
    bool isValid = false;
};
```

And what the twin receives directly from the input source:
```cpp
struct UpstreamSignal {
    double throttleFraction = 0.0;
    double speedKmh = 0.0;
    double accelerationG = 0.0;
    double brakeFraction = 0.0;
    uint64_t timestampUtcMs = 0;
    bool isValid = false;
};
```

**Key finding:** `UpstreamSignal.accelerationG` is already present in the data model but is not currently used by `AutomaticGearbox`. This is the G-sensor equivalent for grade/incline detection.

---

## 4. Recommended Code Changes for Torque-Aware Shifting

### 4.1 Short-term: Use existing data (no engine-sim changes needed)

**Change 1: Add accelerationG to AutomaticGearbox**

File: `include/twin/AutomaticGearbox.h`
```cpp
// Before:
void update(double dt, double speedKmh, double throttleFraction);

// After:
void update(double dt, double speedKmh, double throttleFraction, double accelerationG = 0.0);
```

Add a `loadFactor_` member that accumulates acceleration data:
```cpp
double loadFactor_ = 0.0;  // 0 = normal, > 0 = heavy load (hill, towing)
static constexpr double ACCEL_SMOOTHING_RC = 0.5;  // Low-pass filter time constant
```

In the shift speed lookup, apply modulation:
```cpp
double loadModulation = 1.0 + loadFactor_ * 0.15;  // Max 15% shift delay under heavy load
double adjustedShiftSpeed = baseShiftSpeed * loadModulation;
```

**Change 2: Wire accelerationG through VirtualIceTwin**

File: `src/twin/VirtualIceTwin.cpp`
```cpp
TwinOutput VirtualIceTwin::update(double dt, const UpstreamSignal& signal) {
    // Existing:
    gearbox_.update(dt, signal.speedKmh, signal.throttleFraction);
    // Becomes:
    gearbox_.update(dt, signal.speedKmh, signal.throttleFraction, signal.accelerationG);
}
```

**Change 3: Add manifold pressure to TwinFeedback (requires bridge layer change)**

File: `include/twin/IVehicleTwin.h`
```cpp
struct TwinFeedback {
    double engineRpm = 0.0;
    double vehicleSpeedKmh = 0.0;
    double manifoldPressure = 0.0;  // Pa -- proxy for engine load
    bool isValid = false;
};
```

File: `src/simulator/BridgeSimulator.cpp` -- populate in `getStats()`:
```cpp
if (m_simulator->getEngine()) {
    stats.manifoldPressure = m_simulator->getEngine()->getManifoldPressure();
}
```

File: `src/input/VirtualIceInputProvider.cpp` -- forward to twin:
```cpp
void VirtualIceInputProvider::provideFeedback(const EngineSimStats& stats) {
    twin_->setEngineRpmFeedback(stats.currentRPM);
    twin_->setManifoldPressureFeedback(stats.manifoldPressure);  // NEW
}
```

### 4.2 Medium-term: Derive a torque estimate

If manifold pressure and RPM are available, engine torque can be approximated:

```
estimated_torque = manifold_pressure * displacement * volumetric_efficiency / (RPM * constant)
```

This is the standard "speed-density" approach used by aftermarket ECUs. For our purposes, a simpler normalized load metric suffices:

```cpp
double engineLoad = manifoldPressure / atmosphericPressure;  // 0 = vacuum, 1 = atmospheric, >1 = boost
```

Add to AutomaticGearbox:
```cpp
void setEngineLoad(double load);  // 0-1+ normalized
```

Modulate shift thresholds:
```cpp
double loadModulation = 1.0 + std::max(0.0, loadFactor_ * 0.10) + std::max(0.0, (engineLoad - 0.8) * 0.05);
// loadFactor from accelerationG (grade detection)
// engineLoad from manifold pressure (torque proxy)
// Combined effect: max ~20% shift delay under heavy load + high torque
```

---

## 5. Scenarios That Benefit from Torque Awareness

| Scenario | Current Behavior | Torque-Aware Behavior | Improvement |
|----------|-----------------|----------------------|-------------|
| **Launch from standstill** | Shifts to 2nd at speed threshold regardless of load | Holds 1st longer if acceleration is low (heavy vehicle, incline) | Prevents bogging down after early 1-2 shift |
| **Hill climbing** | Same shift thresholds as flat road | Detects negative accelerationG, delays upshifts | Maintains power band on grades |
| **Towing simulation** | Not distinguishable from normal driving | Higher load factor from acceleration deficit | Prevents gear hunting under load |
| **Coast-down on decline** | Standard downshift thresholds | Detects positive accelerationG with zero throttle, earlier downshift for engine braking | Better speed control on descents |
| **High-RPM power band** | Shifts at fixed speed regardless of whether engine is making power | Manifold pressure indicates whether engine is in its power band; delays shift if pressure is still rising | Keeps engine in optimal RPM range |
| **Kickdown under load** | Kickdown based on throttle delta only | Adds load check: if already under heavy load, trigger kickdown at lower throttle threshold | More responsive overtaking behavior |
| **Variable road surface** | No adaptation | Acceleration noise could indicate rough surface, hold current gear | Prevents unnecessary shifts on rough terrain |

### Priority ranking for implementation

1. **AccelerationG-based load detection** -- data already flows through `UpstreamSignal`, just needs wiring to `AutomaticGearbox`. Biggest bang for buck.
2. **Manifold pressure as engine load proxy** -- requires adding to `TwinFeedback` and wiring through the feedback loop. Good second step.
3. **Combined load + pressure torque model** -- merges both signals for robust load detection. Natural evolution of steps 1-2.

---

## 6. What Phase 1 Research Identified vs What's New

### From Phase 1 research (zf-shift-scheduling-research.md)
- Speed + throttle as primary shift inputs (validated, implemented)
- Hysteresis at 85% of upshift thresholds (implemented)
- Kickdown via throttle delta > 0.4 or throttle > 0.95 (implemented)
- Torque awareness deferred to Phase 2 (this analysis)

### New findings in this analysis
- **Real ZF TCU uses bidirectional torque coordination** with the engine ECU during shifts -- the TCM requests torque reductions and the engine trims output. This is a shift *execution* concern, not shift *scheduling*, so it's out of scope for the gearbox model.
- **Driver Type Assessment (fuzzy logic)** uses G-sensor + acceleration patterns to build a driver profile. The `accelerationG` field in `UpstreamSignal` is the direct equivalent.
- **Drag Recognition** adjusts shift points for uphill/downhill. Implementable via `accelerationG` without any engine-sim changes.
- **engine-sim does NOT expose direct engine torque** -- there is no `getEngineTorque()` method. The closest proxies are `getManifoldPressure()` (load), `getFilteredDynoTorque()` (only with dyno active), and crankshaft angular velocity (already used as RPM).
- **Clutch constraint reaction forces** are accessible via `F_t[0][0]` on the `ClutchConstraint`, but this requires reaching into the SCS solver internals -- fragile and not recommended.

---

## 7. Summary of Recommendations

1. **Wire `accelerationG` to `AutomaticGearbox`** -- highest value, lowest risk, data already exists.
2. **Add manifold pressure to `TwinFeedback`** -- requires small changes to feedback loop wiring.
3. **Modulate shift thresholds by load factor** -- bounded to +/-20% max adjustment to prevent runaway behavior.
4. **Do NOT try to extract raw engine torque** -- engine-sim doesn't expose it, and manifold pressure is a sufficient proxy for our purposes.
5. **Keep shift scheduling in the twin layer** -- don't push torque awareness into engine-sim. The twin layer has all the data it needs through the feedback loop.
