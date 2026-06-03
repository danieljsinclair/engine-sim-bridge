# VirtualICE Twin -- Solution Architecture Proposal

**Date:** 2026-05-06
**Branch:** preset-engine-config
**Status:** Design (pre-Phase 1)
**Audience:** Implementation agents

---

## 1. Architecture Overview

```
                              ┌─────────────────────────────┐
                              │   Real EV (Tesla OBD2/BLE)  │
                              │   ~10Hz: throttle, speed,   │
                              │   accel, brake               │
                              └──────────────┬──────────────┘
                                             │
                              ┌──────────────▼──────────────┐
                              │    vehicle-sim (separate)    │
                              │  BLE parsing, VehicleSignal  │
                              └──────────────┬──────────────┘
                                             │ VehicleSignal (10Hz)
                                             │
                     ┌───────────────────────▼───────────────────────┐
                     │            iOS App / ESP32 App                 │
                     │        (integration + adapter layer)           │
                     │                                                │
                     │  ┌─────────────────────────────────┐          │
                     │  │  VehicleSimAdapter / CANAdapter   │          │
                     │  │  - Normalize to UpstreamSignal   │          │
                     │  │  - Interpolate 10Hz -> 60Hz      │          │
                     │  │  - Staleness detection           │          │
                     │  └──────────────┬──────────────────┘          │
                     │                 │ UpstreamSignal (60Hz)        │
                     │                 │                              │
                     │  ┌──────────────▼──────────────────┐          │
                     │  │   VirtualIceInputProvider        │          │
                     │  │   (IInputProvider impl)          │          │
                     │  │   - Holds VirtualIceTwin         │          │
                     │  │   - OnUpdateSimulation(dt)       │          │
                     │  │     -> feeds signal to twin      │          │
                     │  │     -> returns EngineInput        │          │
                     │  └──────────────┬──────────────────┘          │
                     │                 │                              │
                     │  ┌──────────────▼──────────────────┐          │
                     │  │       VirtualIceTwin             │          │
                     │  │  State: OFF/CRANKING/IDLE/       │          │
                     │  │         RUNNING/SHIFTING         │          │
                     │  │  ┌──────────┐ ┌──────────────┐  │          │
                     │  │  │Throttle  │ │Automatic     │  │          │
                     │  │  │Smoother  │ │Gearbox       │  │          │
                     │  │  └──────────┘ └──────┬───────┘  │          │
                     │  │     │    ┌───────────┘          │          │
                     │  │     │    │ Uses IceVehicleProfile│          │
                     │  │     ▼    ▼                      │          │
                     │  │   TwinOutput                    │          │
                     │  │   { throttle, gear,             │          │
                     │  │     clutchPressure,             │          │
                     │  │     ignition, starter }         │          │
                     │  └──────────────┬──────────────────┘          │
                     │                 │                              │
                     │  ┌──────────────▼──────────────────┐          │
                     │  │      BridgeSimulator             │          │
                     │  │      (ISimulator impl)           │          │
                     │  │  ┌─────────────────────────────┐│          │
                     │  │  │  engine-sim (physics)        ││          │
                     │  │  │  Engine + Transmission +     ││          │
                     │  │  │  Vehicle + VehicleDrag +     ││          │
                     │  │  │  SpeedTrackingForce (A+)     ││          │
                     │  │  │  Dyno: DISABLED              ││          │
                     │  │  └──────────────┬──────────────┘│          │
                     │  └─────────────────┼───────────────┘          │
                     │                    │ RPM (emergent)            │
                     │                    ▼                           │
                     │              Audio Output                     │
                     └──────────────────────────────────────────────┘
```

### Integration Modes (same architecture, swapped physics layer)

```
 ┌─ Approach A+ (recommended) ─────────────────────────────────────────┐
 │  Uses engine-sim's full vehicle model.                              │
 │  SpeedTrackingForce (ForceGenerator) gently corrects virtual speed  │
 │  toward real EV speed to prevent drift.                             │
 │  RPM emerges from: engine torque vs vehicle inertia + drag.         │
 │                                                                      │
 │  Engine-sim calls:                                                   │
 │    setThrottle(), changeGear(), setClutchPressure()                  │
 │    + SpeedTrackingForce::setTargetSpeed(evSpeedKmh)                  │
 │                                                                      │
 │  Pros: Authentic physics, natural load response, engine braking.     │
 │  Cons: Needs drift correction, startup speed initialization.         │
 └──────────────────────────────────────────────────────────────────────┘

 ┌─ Approach B (fallback) ─────────────────────────────────────────────┐
 │  Replaces vehicle model with FixedLoadConstraint on crankshaft.     │
 │  Computes target RPM from road speed, uses PD controller to derive  │
 │  load torque that drives physics toward target RPM.                  │
 │                                                                      │
 │  Engine-sim calls:                                                   │
 │    setThrottle(), changeGear(), FixedLoadConstraint::setLoadTorque() │
 │                                                                      │
 │  Pros: Precise RPM tracking, no vehicle model complexity.            │
 │  Cons: Bypasses vehicle physics, loses natural engine braking.       │
 └──────────────────────────────────────────────────────────────────────┘

 Both modes share the SAME VirtualIceTwin, AutomaticGearbox,
 IceVehicleProfile, and VirtualIceInputProvider. Only the physics
 injection strategy differs (ForceGenerator vs Constraint).
 The twin is agnostic to which mode is active.
```

---

## 2. Component Interfaces

### 2.1 UpstreamSignal -- Normalized EV Telemetry

Owned by the bridge. Never imported from vehicle-sim. All fields normalized
to 0-1 range (except speed in km/h and timestamp).

```cpp
// include/twin/UpstreamSignal.h

#pragma once
#include <cstdint>

struct UpstreamSignal {
    double throttleFraction = 0.0;   // 0.0 - 1.0 (EV pedal position)
    double speedKmh         = 0.0;   // Road speed (km/h)
    double accelerationG    = 0.0;   // Longitudinal acceleration (-2.0 to +2.0 G)
    double brakeFraction    = 0.0;   // 0.0 - 1.0 (brake pedal)
    uint64_t timestampUtcMs = 0;     // For staleness detection
    bool isValid            = false;  // false = stale/no data
};
```

**Minimum viable telemetry:** `throttleFraction`, `speedKmh`, `isValid`.
All other fields enhance fidelity but are not required for Phase 1.

**Degradation strategy:**

| Missing field       | Fallback behavior                          |
|---------------------|--------------------------------------------|
| `accelerationG`     | Compute from speed delta (numerical deriv) |
| `brakeFraction`     | Infer from speed decrease + throttle=0     |
| `isValid = false`   | Twin transitions toward IDLE (ramp down)   |
| `speedKmh` missing  | Cannot compute gear. Twin goes to IDLE.    |
| `throttleFraction`  | Assume 0 (coast)                           |

### 2.2 IceVehicleProfile -- ICE Vehicle Parameters

Immutable value object loaded once at startup. Contains all physical
parameters the gearbox and twin need. Sourced from .mr script parsing
or C++ presets.

```cpp
// include/twin/IceVehicleProfile.h

#pragma once
#include <vector>
#include <string>

struct IceVehicleProfile {
    // Identity
    std::string name;                  // e.g. "GM LS3"

    // Drivetrain
    std::vector<double> gearRatios;    // [0]=N/A, [1]=3.27, [2]=2.04, ...
    double diffRatio         = 3.42;
    double tireRadiusM       = 0.326;  // meters
    double vehicleMassKg     = 1700.0;

    // Engine limits
    double idleRpm           = 700.0;
    double redlineRpm        = 6600.0;

    // Shift curve coefficients
    // upshiftRpm(t) = idleRpm + (redlineRpm - idleRpm) * (upBase + upThrottleCoeff * t)
    // downshiftRpm(t) = idleRpm + (redlineRpm - idleRpm) * (downBase + downThrottleCoeff * t)
    double upshiftBaseFraction       = 0.45;
    double upshiftThrottleCoeff      = 0.55;
    double downshiftBaseFraction     = 0.25;
    double downshiftThrottleCoeff    = 0.20;

    // Derived: shift thresholds for a given throttle position
    double upshiftRpm(double throttleFraction) const;
    double downshiftRpm(double throttleFraction) const;

    // Diagnostic: what RPM would the engine be at for given speed + gear
    double rpmForSpeedAndGear(double speedKmh, int gear) const;

    // Diagnostic: what road speed corresponds to given RPM + gear
    double speedForRpmAndGear(double rpm, int gear) const;

    // Gear count (excludes neutral index 0)
    int forwardGearCount() const;

    // Accessors for engine-sim Transmission setup
    double maxClutchTorque    = 500.0; // ft-lbs
};
```

### 2.3 TwinOutput -- What the twin produces each tick

```cpp
// include/twin/TwinOutput.h

#pragma once

struct TwinOutput {
    double throttle        = 0.0;  // 0.0 - 1.0 (smoothed)
    int    gear            = 0;    // 0=neutral, 1-N=forward gears
    double clutchPressure  = 1.0;  // 0.0 - 1.0 (1.0 = locked)
    bool   ignition        = true;
    bool   starterMotor    = false;

    // Diagnostics (for telemetry/logging, not consumed by engine-sim)
    double targetSpeedKmh  = 0.0;  // What the twin believes EV speed is
    int    targetGear      = 0;    // What gear the gearbox wants
};
```

### 2.4 AutomaticGearbox -- Shift Decision Engine

Pure function of (currentGear, roadSpeed, throttle) -> targetGear.
Owns no state beyond the hysteresis latch. Stateful aspects (shift timing,
clutch ramp) belong to VirtualIceTwin.

```cpp
// include/twin/AutomaticGearbox.h

#pragma once
#include "twin/IceVehicleProfile.h"

class AutomaticGearbox {
public:
    explicit AutomaticGearbox(const IceVehicleProfile& profile);

    // Compute desired gear. Does NOT change internal state.
    // Uses shift curves + hysteresis to prevent hunting.
    int computeTargetGear(
        int currentGear,
        double speedKmh,
        double throttleFraction) const;

    // Detect kickdown: large sudden throttle increase -> force downshift
    bool isKickdown(
        double previousThrottle,
        double currentThrottle) const;

    // Diagnostic: what RPM would result from speed + gear
    double diagnosticRpm(double speedKmh, int gear) const;

private:
    const IceVehicleProfile& profile_;

    // Kickdown threshold
    static constexpr double KICKDOWN_DELTA = 0.4;
};
```

**Shift algorithm:**

```
currentRPM = diagnosticRpm(speedKmh, currentGear)

if currentRPM >= upshiftRpm(throttle):
    return min(currentGear + 1, maxGear)

if currentRPM <= downshiftRpm(throttle):
    return max(currentGear - 1, 1)

if isKickdown(prevThrottle, throttle):
    return max(currentGear - 1, 1)

return currentGear  // hold
```

Hysteresis is inherent: the gap between upshift and downshift thresholds
(e.g., upshift at 6700, downshift at 3000 for full throttle) prevents
oscillation. No explicit timer needed for MVP.

### 2.5 VirtualIceTwin -- State Machine + Orchestration

The twin is the central orchestrator. It holds state, manages transitions,
smooths throttle, and coordinates gearbox shifts with clutch modulation.

```cpp
// include/twin/VirtualIceTwin.h

#pragma once
#include "twin/UpstreamSignal.h"
#include "twin/TwinOutput.h"
#include "twin/IceVehicleProfile.h"
#include "twin/AutomaticGearbox.h"
#include <chrono>

class VirtualIceTwin {
public:
    enum class State {
        OFF,
        CRANKING,
        IDLE,
        RUNNING,
        SHIFTING
    };

    explicit VirtualIceTwin(IceVehicleProfile profile);

    // Main tick. Called once per simulation update (60Hz).
    // Receives EV telemetry, returns what engine-sim should do.
    TwinOutput update(const UpstreamSignal& signal, double dt);

    // State queries
    State getState() const;
    const IceVehicleProfile& getProfile() const;

    // Configuration
    static constexpr double THROTTLE_SMOOTH_TAU  = 0.050;  // 50ms
    static constexpr double CRANKING_RPM_THRESHOLD = 550.0;
    static constexpr double IDLE_THROTTLE         = 0.05;
    static constexpr double RUNNING_THROTTLE_THRESHOLD = 0.05;

    // Shift timing (Phase 1: simple ramp)
    static constexpr double SHIFT_CLUTCH_RAMP_DURATION = 0.30;  // seconds
    static constexpr double SHIFT_CLUTCH_MIN = 0.0;   // disengage during shift

private:
    // State machine handlers
    TwinOutput handleOff(const UpstreamSignal& signal, double dt);
    TwinOutput handleCranking(const UpstreamSignal& signal, double dt);
    TwinOutput handleIdle(const UpstreamSignal& signal, double dt);
    TwinOutput handleRunning(const UpstreamSignal& signal, double dt);
    TwinOutput handleShifting(const UpstreamSignal& signal, double dt);

    // Smoothing
    double smoothThrottle(double rawThrottle, double dt);
    double clampThrottle(double throttle) const;

    // Shift management
    void beginShift(int targetGear);
    double shiftClutchPressure(double dt);  // returns pressure for current shift phase

    // State
    State state_ = State::OFF;
    IceVehicleProfile profile_;
    AutomaticGearbox gearbox_;

    double smoothedThrottle_ = 0.0;
    double prevThrottle_     = 0.0;
    int    currentGear_      = 0;      // actual gear in transmission
    int    shiftTargetGear_  = 0;      // gear we're shifting toward
    double shiftTimer_       = 0.0;    // elapsed time in SHIFTING state

    // For cranking detection (RPM comes from engine-sim feedback)
    // The twin does NOT read RPM directly -- it uses throttle == 0 + state
    // to know the engine is cranking. RPM is read by VirtualIceInputProvider.
    std::chrono::steady_clock::time_point lastValidTelemetryTime_;
};
```

**State maintained by VirtualIceTwin:**

| State variable       | Purpose                                         |
|----------------------|-------------------------------------------------|
| `state_`             | Current state machine position                  |
| `smoothedThrottle_`  | Low-pass filtered throttle (removes EV snap)    |
| `prevThrottle_`      | For kickdown detection (delta threshold)        |
| `currentGear_`       | Actual gear engaged in transmission             |
| `shiftTargetGear_`   | Destination gear during SHIFTING                |
| `shiftTimer_`        | Duration into current shift (for clutch ramp)   |
| `lastValidTelemetryTime_` | Staleness detection for OFF transition    |

### 2.6 VirtualIceInputProvider -- Bridge to SimulationLoop

Implements `IInputProvider`. Holds the twin. Each tick: receives upstream
signal (from adapter), feeds twin, reads engine RPM back from simulator,
returns `EngineInput`.

```cpp
// include/input/VirtualIceInputProvider.h

#pragma once
#include "io/IInputProvider.h"
#include "twin/VirtualIceTwin.h"
#include "twin/UpstreamSignal.h"
#include <mutex>
#include <atomic>

class ISimulator;  // forward (for RPM feedback)

class VirtualIceInputProvider : public input::IInputProvider {
public:
    // Simulator is needed for RPM feedback (cranking detection, diagnostics)
    VirtualIceInputProvider(
        IceVehicleProfile profile,
        ISimulator& simulator);

    // IInputProvider lifecycle
    bool Initialize() override;
    void Shutdown() override;
    bool IsConnected() const override;

    // Called by adapter (vehicle-sim, CAN) from any thread
    void postUpstreamSignal(const UpstreamSignal& signal);

    // Called by SimulationLoop at 60Hz from simulation thread
    input::EngineInput OnUpdateSimulation(double dt) override;

    std::string GetProviderName() const override;
    std::string GetLastError() const override;

private:
    VirtualIceTwin twin_;
    ISimulator& simulator_;

    // Thread-safe signal passing (adapter writes, sim thread reads)
    std::mutex signalMutex_;
    UpstreamSignal latestSignal_;

    // RPM feedback for cranking detection
    double getEngineRpm() const;

    // Convert TwinOutput -> EngineInput
    input::EngineInput toEngineInput(const TwinOutput& output);
};
```

**Why ISimulator reference?** The twin needs RPM for cranking-to-idle
transition detection. RPM is read from `ISimulator::getStats().currentRpm`.
The twin itself is agnostic to RPM -- the provider bridges this gap.

### 2.7 SpeedTrackingForce -- Approach A+ Drift Correction

A ForceGenerator that applies gentle corrective torque to the vehicle
mass body, pushing virtual speed toward real EV speed.

```cpp
// include/physics/SpeedTrackingForce.h

#pragma once
#include "engine-sim/include/scs.h"

class Vehicle;

class SpeedTrackingForce : public atg_scs::ForceGenerator {
public:
    SpeedTrackingForce(
        Vehicle* vehicle,
        atg_scs::RigidBody* vehicleMassBody);

    // Called each tick before physics (from VirtualIceInputProvider)
    void setTargetSpeedKmh(double speedKmh);

    // ForceGenerator interface -- called by SCS during processForces()
    void apply(atg_scs::SystemState* state) override;

    // Tuning
    void setGain(double gain);
    void setMaxCorrectionTorque(double maxTorque);

private:
    Vehicle* vehicle_;
    atg_scs::RigidBody* vehicleMassBody_;
    double targetSpeedMs_ = 0.0;
    double gain_ = 100.0;              // P-controller gain
    double maxCorrection_ = 500.0;     // Cap (ft-lbs equivalent)
};
```

**Physics interaction:** `processForces()` zeros all body forces, then
calls each ForceGenerator. Our `apply()` adds correction torque to
`state->t[vehicleMassBody_->index]`. The constraint solver then enforces
clutch, drag, etc. on top of this external force. Correction is applied
*before* constraint solving, so it interacts correctly.

### 2.8 FixedLoadConstraint -- Approach B Fallback

A custom SCS constraint that applies a fixed (velocity-independent) braking
torque to the crankshaft body.

```cpp
// include/physics/FixedLoadConstraint.h

#pragma once
#include "engine-sim/include/scs.h"

class FixedLoadConstraint : public atg_scs::Constraint {
public:
    FixedLoadConstraint();

    void initialize(atg_scs::RigidBody* crankshaft);

    // Set the braking torque (updated each tick from PD controller)
    void setLoadTorque(double torqueNm);

    // Constraint interface
    void calculate(Output* output, atg_scs::SystemState* state) override;

private:
    atg_scs::RigidBody* crankshaft_;
    double loadTorque_ = 0.0;
    static constexpr int BODY_COUNT = 1;
    static constexpr int CONSTRAINT_ROWS = 1;
};
```

**Implementation note (from physics analysis):** The constraint sets
`v_bias` to a large negative value and caps `limits` to the desired torque.
The solver produces a correction impulse, clamped by limits, resulting in
a fixed braking torque independent of crankshaft velocity.

---

## 3. Three-Phase Shift Sequence

The AutomaticGearbox returns the *target* gear. VirtualIceTwin manages
the physical shift through clutch pressure modulation:

```
SHIFTING state (triggered when gearbox computes new target gear):

  Phase 1: CLUTCH RAMP DOWN  (~100ms)
    clutchPressure: 1.0 -> 0.0  (linear ramp)
    throttle: held at current smoothed value
    gear: unchanged (still in old gear)

  Phase 2: GEAR CHANGE  (instantaneous, at clutch = 0.0)
    changeGear(targetGear)
    clutchPressure: 0.0 (brief hold, ~50ms)
    Engine free-revs briefly (natural RPM flare)

  Phase 3: CLUTCH RAMP UP  (~150ms)
    clutchPressure: 0.0 -> 1.0  (linear ramp)
    throttle: held
    Clutch engages, RPM settles to new gear ratio

  Total shift duration: ~300ms (tunable via SHIFT_CLUTCH_RAMP_DURATION)
```

**Why this works:** `changeGear()` conserves kinetic energy. On upshift,
the larger effective inertia at the crankshaft causes RPM to drop naturally.
The brief disengagement period lets the engine flare slightly (realistic)
before the clutch grabs in the new gear. No torque converter needed for MVP.

**Engine-sim mechanics during shift:**
- `setClutchPressure(0.0)`: ClutchConstraint limits drop to zero. Engine
  decouples from vehicle mass. Engine free-revs against only its own inertia.
- `changeGear(newGear)`: Recomputes vehicle mass inertia via new gear ratio.
  Conserves KE: `omega_new = sqrt(2 * E_rot / I_new)`.
- `setClutchPressure(1.0)`: ClutchConstraint re-engages. Engine torque
  now works against the new (larger for upshift) vehicle inertia.

---

## 4. Integration Points with engine-sim

### 4.1 Existing engine-sim APIs (no modification needed)

| API | Used by | Purpose |
|-----|---------|---------|
| `Engine::setThrottle(double)` | VirtualIceInputProvider | Set throttle position |
| `Engine::setSpeedControl(double)` | BridgeSimulator::setThrottle | Already wired |
| `Engine::getRpm()` | VirtualIceInputProvider (via ISimulator stats) | RPM feedback |
| `Engine::getIgnitionModule()` | BridgeSimulator::setIgnition | Enable combustion |
| `Transmission::changeGear(int)` | VirtualIceInputProvider | Gear changes |
| `Transmission::setClutchPressure(double)` | VirtualIceInputProvider | Shift modulation |
| `Transmission::getGear()` | BridgeSimulator::getStats | Current gear state |
| `Simulator::m_starterMotor.m_enabled` | BridgeSimulator::setStarterMotor | Cranking |
| `Simulator::getEngine()` | BridgeSimulator | Engine access |
| `Simulator::getTransmission()` | BridgeSimulator | Transmission access |
| `Simulator::getVehicle()` | SpeedTrackingForce setup | Vehicle access |
| `Simulator::getSystem()` | SpeedTrackingForce setup | RigidBodySystem access |
| `Vehicle::getSpeed()` | SpeedTrackingForce | Virtual speed for comparison |
| `Vehicle::linearForceToVirtualTorque()` | SpeedTrackingForce | Speed->torque conversion |
| `RigidBodySystem::addForceGenerator()` | Setup | Register SpeedTrackingForce |

### 4.2 Accessor additions needed on engine-sim fork (~15 lines)

```cpp
// engine-sim/include/transmission.h -- add to public section:

// Gear ratio accessors (for automatic gearbox shift point computation)
const double* getGearRatios() const { return m_gearRatios; }
int getGearCount() const { return m_gearCount; }
double getMaxClutchTorque() const { return m_maxClutchTorque; }
```

No behavioral changes. Pure read-only accessors for values already stored.

### 4.3 ISimulator extension options

**Recommendation: ITwinControl interface (ISP-compliant).**

The base `ISimulator` serves keyboard, sine wave, and dyno use cases.
Twin-specific methods would pollute it. Instead:

```cpp
// include/simulator/ITwinControl.h

#pragma once

class ITwinControl {
public:
    virtual ~ITwinControl() = default;

    virtual void setGear(int gear) = 0;
    virtual void setClutchPressure(double pressure) = 0;
    virtual double getEngineRpm() const = 0;
    virtual void setVehicleSpeed(double speedKmh) = 0;  // startup init
};
```

`BridgeSimulator` implements both `ISimulator` and `ITwinControl`.
`VirtualIceInputProvider` holds `ITwinControl*` (queried from the
`ISimulator` via dynamic_cast or static factory method).

**Alternative (simpler, less ISP):** Extend `EngineInput` to carry
gear, clutch pressure, and speed target directly. The provider sets
these fields; the simulation loop applies them to the simulator.
This avoids the cast entirely but couples EngineInput to twin concerns.

**Phase 1 decision:** Start with extending `EngineInput`. It already has
`gearDelta` and `dynoTorqueScale`. Adding `gearAbsolute`, `clutchPressure`,
and `vehicleSpeedTarget` is consistent. Revisit ITwinControl if/when the
base `ISimulator` interface shows strain.

---

## 5. EngineInput Extension (Phase 1 approach)

```cpp
// Extend io/IInputProvider.h EngineInput struct:

struct EngineInput {
    double throttle = 0.1;
    bool ignition = true;
    bool starterMotor = false;
    bool shouldContinue = true;

    // Existing gear control (keyboard: relative)
    int gearDelta = 0;

    // New: absolute gear control (twin: set exact gear)
    int gearAbsolute = -1;           // -1 = use gearDelta, 0+ = set this gear

    // New: clutch pressure control
    double clutchPressure = -1.0;    // -1 = unchanged, 0.0-1.0 = set pressure

    // New: vehicle speed target (for startup initialization + speed sync)
    double vehicleSpeedTargetKmh = -1.0;  // -1 = no target, 0+ = sync target

    // Dyno control (unchanged)
    double dynoTorqueScale = -1.0;
};
```

**Why this approach for Phase 1:**
- `IInputProvider::OnUpdateSimulation()` already returns `EngineInput`.
- The simulation loop already processes it.
- We only need to add processing for the new fields in the loop.
- No new interfaces, no casts, no architectural changes.
- `VirtualIceInputProvider` is the ONLY provider that sets these fields.
- Keyboard provider leaves them at defaults (-1), no impact.

---

## 6. Data Contract: Minimum Telemetry

### Required fields (hard failure without these)

| Field | Why required |
|-------|-------------|
| `speedKmh` | Cannot compute gear or RPM. Twin goes to IDLE. |
| `throttleFraction` | Cannot determine engine load. Defaults to 0 (coast). |
| `isValid` | Must be true. False triggers graceful degradation. |

### Enhancement fields (improve fidelity)

| Field | Enhancement |
|-------|------------|
| `accelerationG` | Detects aggressive launches, informs throttle smoothing rate |
| `brakeFraction` | Distinguishes engine braking from throttle lift |
| `timestampUtcMs` | Staleness detection, interpolation timestamps |

### Degradation matrix

```
isValid = false:
  -> Twin ramps throttle toward 0
  -> After N seconds timeout, transitions to OFF
  -> Audio continues at idle

speedKmh present, throttle missing:
  -> throttle = 0 (coast/deceleration sound)

throttle present, speed missing:
  -> Cannot compute gear
  -> Hold last gear, clamp throttle to idle
  -> If persists > 2s, transition to IDLE

Both missing (total signal loss):
  -> Immediate transition to IDLE state
  -> Engine idles with ignition on (audio continues)
```

---

## 7. File Organization

### New files (all in engine-sim-bridge)

```
include/
  twin/
    UpstreamSignal.h           ~20 lines   (struct)
    IceVehicleProfile.h        ~60 lines   (struct + inline methods)
    TwinOutput.h               ~20 lines   (struct)
    AutomaticGearbox.h         ~30 lines   (class declaration)
    VirtualIceTwin.h           ~80 lines   (class declaration)
  physics/
    SpeedTrackingForce.h       ~30 lines   (Approach A+ ForceGenerator)
    FixedLoadConstraint.h      ~25 lines   (Approach B fallback)

src/
  twin/
    IceVehicleProfile.cpp      ~40 lines   (rpm/speed calculations)
    AutomaticGearbox.cpp       ~60 lines   (shift logic)
    VirtualIceTwin.cpp         ~200 lines  (state machine + smoothing)
  input/
    VirtualIceInputProvider.cpp ~100 lines (IInputProvider impl)
  physics/
    SpeedTrackingForce.cpp      ~30 lines  (ForceGenerator impl)
    FixedLoadConstraint.cpp     ~25 lines  (Constraint impl)

test/
  twin/
    AutomaticGearboxTests.cpp   ~150 lines
    VirtualIceTwinTests.cpp      ~200 lines
    IceVehicleProfileTests.cpp   ~60 lines
  integration/
    TwinPhysicsIntegrationTests.cpp  ~150 lines
```

### Modified files

```
include/io/IInputProvider.h        +5 lines  (extend EngineInput)
src/simulation/SimulationLoop.cpp   +20 lines (process new EngineInput fields)
engine-sim/include/transmission.h   +3 lines  (gear ratio accessors)
```

---

## 8. Recommended Implementation Order

### Stage 1: Value types + Gearbox (no engine-sim dependency)

Build and test these in isolation. They are pure functions/structs.

```
1. UpstreamSignal.h           (struct, trivial)
2. IceVehicleProfile.h/.cpp   (struct + rpm/speed math)
3. AutomaticGearbox.h/.cpp    (shift decision logic)
4. Tests: AutomaticGearboxTests, IceVehicleProfileTests
```

**Test cases for AutomaticGearbox:**
- Standstill (speed=0): gear stays at 1
- Acceleration ramp: upshifts at correct RPM thresholds
- Deceleration: downshifts at correct thresholds
- Hysteresis: speed oscillating near shift point does not cause hunting
- Kickdown: throttle delta > 0.4 triggers immediate downshift
- Full throttle at redline: holds gear, does not exceed max gear
- Light throttle cruising: early upshifts, stays in high gear

### Stage 2: Twin state machine (still no engine-sim dependency)

```
5. TwinOutput.h               (struct, trivial)
6. VirtualIceTwin.h/.cpp      (state machine + throttle smoother)
7. Tests: VirtualIceTwinTests (mock gearbox, verify state transitions)
```

**Test cases for VirtualIceTwin:**
- OFF -> CRANKING on first valid signal
- CRANKING -> IDLE (simulated: tick enough times with ignition)
- IDLE -> RUNNING on throttle > threshold
- RUNNING -> IDLE on throttle=0 + speed=0
- RUNNING -> SHIFTING on gear change trigger
- SHIFTING -> RUNNING after shift duration
- Any -> OFF on invalid signal timeout
- Throttle smoothing: step input produces exponential ramp
- Stale signal: throttle ramps to 0 over timeout period

### Stage 3: Engine-sim integration

```
8. Extend EngineInput struct  (+gearAbsolute, +clutchPressure, +vehicleSpeedTarget)
9. VirtualIceInputProvider     (wire twin to IInputProvider)
10. BridgeSimulator extensions (apply new EngineInput fields)
11. SimulationLoop extensions  (process new fields)
12. SpeedTrackingForce.h/.cpp  (Approach A+ ForceGenerator)
```

### Stage 4: End-to-end validation

```
13. TwinPhysicsIntegrationTests (synthetic telemetry -> twin -> engine-sim -> RPM)
14. Manual listening test (throttle ramp 0-100, verify sound)
```

**Integration test scenarios:**
- Acceleration: 0 -> 100 km/h at full throttle. Gears 1->2->3->4->5->6.
  RPM stays between idle and redline. No stalls, no overrevs.
- Cruising: Constant 80 km/h, light throttle. Stable gear, stable RPM.
  Virtual speed within 5% of target.
- Deceleration: Lift throttle at 100 km/h. Engine braking sound.
  Downshifts as speed drops.
- Startup at speed: Begin with EV already at 60 km/h. Vehicle mass
  initialized to correct speed. No transient spike.

---

## 9. Speed Synchronization Detail (Approach A+)

### The drift problem

Engine-sim computes virtual vehicle speed from physics:
```
E_rot = 0.5 * I_vehicle * omega_vehicle^2
v_virtual = sqrt(2 * E_rot / m_car)
```

If the virtual ICE engine produces different torque than the real EV motor
at the same throttle, virtual and real speeds diverge. Over time, this
causes gear selection errors (gearbox uses real speed, physics uses virtual).

### SpeedTrackingForce solution

```cpp
void SpeedTrackingForce::apply(atg_scs::SystemState* state) {
    if (!vehicle_ || !vehicleMassBody_) return;

    double virtualSpeed = vehicle_->getSpeed();   // m/s
    double speedError = virtualSpeed - targetSpeedMs_;

    // P-controller: gentle correction toward target
    double correctionForce = -gain_ * speedError;  // Newtons

    // Clamp to prevent physics disruption
    double absCorrection = std::min(std::abs(correctionForce), maxCorrection_);
    correctionForce = (correctionForce >= 0) ? absCorrection : -absCorrection;

    // Convert linear force to virtual torque on vehicle mass body
    double correctionTorque = vehicle_->linearForceToVirtualTorque(correctionForce);

    // Inject before constraint solving
    state->t[vehicleMassBody_->index] += correctionTorque;
}
```

### Startup initialization

When the twin receives the first valid signal and the EV is already moving,
the vehicle mass body's angular velocity must be initialized:

```cpp
// In VirtualIceInputProvider::OnUpdateSimulation (first valid signal):
// v_theta = speedMs * diffRatio * gearRatio / tireRadius
simulator.setVehicleSpeed(signal.speedKmh);
```

This requires a one-time setter on the vehicle mass body's `v_theta`.
Approximately 5 lines in BridgeSimulator, using `getInternalSimulator()`.

### Tuning guidance

| Parameter | Starting value | Effect of increasing |
|-----------|---------------|---------------------|
| `gain_` | 100 | Faster correction, risk of oscillation |
| `maxCorrection_` | 500 N | Stronger correction, risk of audible artifacts |

Start with gain=100, maxCorrection=500. Empirically verify:
1. Virtual speed tracks real speed within 5% during steady cruise
2. No audible artifacts (clicks, pops) from correction torque
3. Gear changes occur at correct speeds

If drift is empirically negligible (< 2% in normal driving), consider
disabling the ForceGenerator entirely and accepting open-loop physics.

---

## 10. Open Design Decisions

### 10.1 ITwinControl vs EngineInput extension (resolved for Phase 1)

**Decision:** Extend `EngineInput` with absolute gear, clutch pressure,
and speed target fields. This avoids interface proliferation and is
consistent with the existing `gearDelta` / `dynoTorqueScale` pattern.
Revisit `ITwinControl` if a second twin-type provider emerges.

### 10.2 SpeedTrackingForce activation strategy (deferred to Phase 1 testing)

Two options:
- **Always on:** Apply correction every frame. Simple, continuous.
- **Threshold-gated:** Only apply when drift exceeds threshold (e.g., 5%).
  Less interference with natural physics, but introduces discontinuity.

Recommend testing always-on first. If audible artifacts appear,
switch to threshold-gated.

### 10.3 Cranking RPM feedback (design detail)

The twin needs to know when to transition CRANKING -> IDLE (RPM > 550).
The twin itself should NOT know about RPM (SRP: it maps telemetry to
engine commands). Instead:

Option A: VirtualIceInputProvider checks RPM and sets a flag on the signal.
Option B: Twin takes an optional `engineRpm` parameter in `update()`.

**Recommendation:** Option B. The twin's `update()` signature becomes:
```cpp
TwinOutput update(const UpstreamSignal& signal, double engineRpm, double dt);
```
This is explicit, testable (inject RPM in tests), and does not couple
the twin to ISimulator.

### 10.4 Vehicle profile loading (deferred)

Phase 1 hardcodes 2-3 profiles (GM LS, Ferrari F136, Honda TRX520) as
C++ constants. .mr script parsing for profile extraction is Phase 2+.
The `IceVehicleProfile` struct is the same either way.

---

## 11. SOLID Compliance Summary

| Principle | How this design complies |
|-----------|------------------------|
| **SRP** | Each class has one job: AutomaticGearbox selects gear. VirtualIceTwin manages state. VirtualIceInputProvider bridges twin to simulation loop. SpeedTrackingForce corrects speed drift. |
| **OCP** | New vehicle profiles are added by creating new `IceVehicleProfile` instances. New gearbox algorithms by implementing a new class with `computeTargetGear()`. Approach B is added by swapping ForceGenerator for Constraint. |
| **LSP** | Any `IInputProvider` works in the simulation loop. VirtualIceInputProvider is interchangeable with KeyboardInputProvider. |
| **ISP** | EngineInput extension is additive (defaults = no-op for non-twin providers). ITwinControl deferred but available if needed. |
| **DIP** | Twin depends on `UpstreamSignal` (abstraction), not `VehicleSignal` (vehicle-sim concrete). Provider depends on `ISimulator` (interface), not `BridgeSimulator` (concrete). |

---

## 12. Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Speed drift > 10% | Medium | Wrong gear selection, poor sound | SpeedTrackingForce (tested in Stage 4) |
| Shift transients (clicks/pops) | Low-Medium | Audio artifacts | Clutch ramp timing tuning |
| EV telemetry latency > 100ms | Medium | Sound lags visual | Interpolation + prediction in adapter |
| Engine stalls at low speed | Low | Audio dropout | StarterMotor auto-restart, minimum idle throttle |
| Approach A+ insufficient | Low | Need Approach B | Both designed, B is drop-in replacement |

---

## 13. Dependency Graph (build order)

```
UpstreamSignal.h          (no deps)
TwinOutput.h              (no deps)
IceVehicleProfile.h/.cpp  (no deps)
AutomaticGearbox.h/.cpp   (depends: IceVehicleProfile)
VirtualIceTwin.h/.cpp     (depends: UpstreamSignal, TwinOutput,
                            IceVehicleProfile, AutomaticGearbox)
EngineInput extension     (depends: IInputProvider.h)
VirtualIceInputProvider   (depends: VirtualIceTwin, ISimulator,
                            EngineInput extension)
SpeedTrackingForce        (depends: engine-sim Vehicle + SCS)
FixedLoadConstraint       (depends: engine-sim SCS)
Integration tests         (depends: all above)
```

No circular dependencies. Twin layer is independent of engine-sim.
Only the provider and physics layer touch engine-sim APIs.
