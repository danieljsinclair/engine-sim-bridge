# VirtualICE Twin - Research Report

> Compiled: 2026-05-06
> Sources: Web research (Wikipedia, GitHub, OEM documentation) + domain knowledge for gaps
> Purpose: Actionable data for engine-sim-bridge -- what telemetry we can get, at what rate, and how automatic transmissions decide when to shift

---

## 1. EV Telemetry Data

### 1.1 Standard OBD-II PIDs (Any Vehicle with OBD-II Port)

These are the SAE standard PIDs available via an ELM327-compatible adapter on any OBD-II-compliant vehicle. For EVs, many ICE-specific PIDs return no data (e.g., fuel trims, O2 sensors).

| PID (Hex) | Name | Range | Resolution | Typical Update Rate |
|------------|------|-------|------------|---------------------|
| 0x0C | Engine RPM | 0-16,383.75 RPM | 0.25 RPM | 1-10 Hz (protocol dependent) |
| 0x0D | Vehicle Speed | 0-255 km/h | 1 km/h | 1-10 Hz |
| 0x11 | Throttle Position | 0-100% | 0.39% | 1-4 Hz |
| 0x0A | Fuel Pressure | 0-765 kPa | 3 kPa | 1 Hz |
| 0x0B | Intake Manifold Pressure | 0-255 kPa | 0.39 kPa | 1 Hz |
| 0x04 | Engine Load (calculated) | 0-100% | 0.39% | 1 Hz |
| 0x10 | MAF Air Flow Rate | 0-655.35 g/s | 0.01 g/s | 1 Hz |
| 0x05 | Engine Coolant Temp | -40 to +215 C | 1 C | 0.5-1 Hz |
| 0x2F | Fuel Tank Level | 0-100% | 0.39% | 0.2 Hz |
| 0x1C | OBD Standards Compliance | - | - | On-demand |
| 0x42 | Control Module Voltage | 0-18 V | 0.001 V | 1 Hz |
| 0x43 | Absolute Load Value | 0-25,700% | 0.39% | 1 Hz |
| 0x47 | Absolute Throttle Position B | 0-100% | 0.39% | 1 Hz |
| 0x49 | Accelerator Pedal Position D | 0-100% | 0.39% | 1 Hz |
| 0x4C | Commanded Throttle Actuator | 0-100% | 0.39% | 1 Hz |

**Key constraint**: Standard OBD-II rates are 1-10 Hz max. The ISO 9141-2 / SAE J1850 protocols are slow (approx 5-10 msgs/sec). CAN-based OBD (ISO 15765-4) is faster but still limited by the query-response model.

**Tesla OBD-II caveat**: Tesla vehicles do NOT expose standard OBD-II data through the OBD port in the usual way. The OBD port exists for diagnostics, but standard PIDs like RPM and speed are generally not available via a simple ELM327. Tesla uses proprietary CAN messages.

### 1.2 Tesla REST API (Owner API)

Endpoint: `https://owner-api.teslamotors.com`
Authentication: OAuth2 bearer token

**Vehicle State Endpoints:**

| Endpoint | Key Fields | Update Rate |
|----------|-----------|-------------|
| `vehicle_data` | speed, odometer, shift_state, battery_level, battery_range | Poll-based (1-2s feasible) |
| `drive_state` | speed, shift_state (P/R/N/D), power, steering_angle | Poll-based |
| `charge_state` | battery_level, battery_range, charge_rate, charger_voltage | Poll-based |
| `climate_state` | inside_temp, outside_temp, fan_status, seat_heaters | Poll-based |
| `vehicle_state` | door/lock states, tire pressures, odometer | Poll-based |

**Key telemetry fields available:**
- `speed` - Vehicle speed in mph (integer)
- `power` - Battery power output in kW (can be negative for regen)
- `shift_state` - Current gear: "P", "R", "N", "D"
- `steering_angle` - Steering wheel angle
- `battery_level` - State of charge percentage
- `odometer` - Total distance

**Rate limit**: Tesla imposes rate limits on the REST API. Practical sustained polling is ~1 query every 2-3 seconds. This is too slow for high-fidelity sound synthesis but acceptable for basic state tracking.

### 1.3 Tesla Streaming API

Endpoint: `https://streaming.vn.teslamotors.com/stream/<vehicle_id>/?values=speed,odometer,soc,elevation,est_heading,est_lat,est_lng,power,shift_state,range,est_range,heading`

Protocol: HTTP long-polling / streaming

**Update rate**: Data arrives approximately every 0.5 seconds (2 Hz) while the vehicle is driving.

**Fields available via streaming:**
- `speed` - mph
- `power` - kW
- `shift_state` - P/R/N/D
- `soc` - State of charge %
- `odeter` - miles
- `elevation` - feet
- `est_heading` - degrees
- `est_lat`, `est_lng` - GPS position
- `range`, `est_range` - estimated range

**This is the most practical real-time telemetry source for Tesla.** At 2 Hz it is usable for basic sound synthesis but will require interpolation for smooth output. The 500ms granularity means rapid throttle changes will be smoothed/missed.

### 1.4 Tesla CAN Bus (Advanced / Reverse Engineering)

Tesla vehicles use multiple CAN bus segments (usually 3-4: drivetrain, chassis, body, diagnostics). The CAN bus runs at 500 kbps and carries high-frequency data.

**Known decoded CAN messages (community reverse engineering, primarily for Model S/X):**

| CAN ID | Data | Frequency | Notes |
|--------|------|-----------|-------|
| 0x0D2 | Motor RPM, Motor Torque | ~50-100 Hz | Drivetrain bus |
| 0x116 | Throttle Pedal Position | ~50 Hz | Raw pedal % |
| 0x118 | Vehicle Speed | ~50 Hz | From wheel speed sensors |
| 0x1D4 | Steering Angle | ~10-50 Hz | |
| 0x252 | Battery Voltage, Current | ~10 Hz | |
| 0x399 | Gear Selector Position | ~10 Hz | P/R/N/D/S |
| 0x388 | Inverter Temperature | ~1 Hz | |
| 0x132 | Brake Pedal Position | ~50 Hz | |
| 0x005 | System Status | ~10 Hz | Drive mode flags |

**How to access**: Requires a CAN bus adapter (e.g., canable.io, PCAN-USB, or ECOM cable) connected to the diagnostic port. The Tesla CAN bus is not encrypted but message formats are proprietary and must be reverse-engineered.

**Recommended tool**: SavvyCAN (open-source, cross-platform CAN bus analysis tool). SavvyCAN can:
- Capture CAN traffic via multiple adapter types
- Decode known messages using .DBC files
- Reverse-engineer unknown signals using the "graphing" feature
- Replay captured traffic

**Libraries:**
- `python-OBD` - Python library for standard OBD-II queries via ELM327. Works for ICE vehicles. Limited use for Tesla. Repo: https://github.com/brendanwhitfield/python-OBD
- `SavvyCAN` - Qt-based CAN bus analyzer. Repo: https://github.com/collin80/SavvyCAN
- `cantools` - Python library for decoding CAN data from .DBC files. Useful once you have decoded Tesla messages.
- `python-can` - Low-level CAN bus interface library supporting multiple hardware adapters.

### 1.5 Recommended Telemetry Strategy

For the VirtualICE Twin project, the practical telemetry approach depends on the target use case:

**Low-fidelity (demo/prototype):**
- Tesla REST API for shift_state, speed, power
- 2 Hz streaming API
- Requires OAuth token management
- Interpolate between samples for sound generation

**Medium-fidelity (daily driver):**
- CAN bus tap with SavvyCAN + cantools
- 50-100 Hz motor RPM + throttle position
- Requires physical CAN adapter ($30-100)
- Requires message decoding effort

**High-fidelity (reference quality):**
- Full CAN bus decode + IMU data
- 100+ Hz for all drivetrain signals
- Significant reverse engineering effort
- Could partner with existing Tesla CAN decoding community

---

## 2. ZF Automatic Transmission

### 2.1 How Automatic Transmissions Decide When to Shift

Modern automatic transmissions use a **Transmission Control Unit (TCU)** that makes shift decisions based on multiple inputs. The TCU runs a **shift map** (also called a shift schedule or shift diagram).

**Primary shift decision inputs:**
1. **Throttle position** (or accelerator pedal position) - determines demand
2. **Vehicle speed** (or output shaft speed) - determines current operating point
3. **Engine load** - calculated from MAF, MAP, or torque model
4. **Transmission fluid temperature** - affects shift quality and lockup behavior
5. **Driver mode selector** - Sport/Economy/Normal changes shift thresholds
6. **Brake pedal status** - downshift on deceleration
7. **Kickdown switch** - forces downshift(s) for maximum acceleration
8. **Grade/gradient** - from longitudinal accelerometer, prevents hunting on hills

**The shift map is a 2D lookup table** with:
- X-axis: Vehicle speed (or transmission output shaft speed)
- Y-axis: Throttle position (or engine load)
- Each cell contains the target gear

**Shift lines are drawn on this map:**
- Upshift lines: curves where the TCU commands an upshift
- Downshift lines: curves where the TCU commands a downshift
- **Hysteresis**: Downshift lines are offset to lower speeds than upshift lines (typically 10-20% speed difference) to prevent "shift hunting" (rapid back-and-forth shifting)

**Typical shift map behavior:**
- **Light throttle (0-25%)**: Upshifts early at low RPM (~1500-2000 RPM) for fuel economy
- **Medium throttle (25-50%)**: Upshifts at moderate RPM (~2000-3000 RPM)
- **Heavy throttle (50-75%)**: Upshifts at higher RPM (~3000-4500 RPM)
- **Wide-open throttle (75-100%)**: Upshifts near redline (~5000-6500 RPM)
- **Kickdown (floor throttle past detent)**: Forces maximum downshift, holds to redline

**Sport mode**: Shift lines move to higher RPM bands (typically +500-1500 RPM)
**Economy mode**: Shift lines move to lower RPM bands (typically -300-500 RPM)
**Manual mode**: TCU holds current gear, only shifts at redline (upshift) or idle (downshift)

### 2.2 ZF 8HP Transmission Family

**Architecture**: 4 planetary gearsets, 2 brakes, 3 clutches = 8 forward speeds + 1 reverse
**Shift time**: ~0.2 seconds (confirmed by multiple sources)
**Torque handling**: 8HP45 = 450 Nm, 8HP55 = 550 Nm, 8HP70 = 700 Nm, 8HP90 = 900 Nm

**ZF 8HP45 Gear Ratios:**

| Gear | 1st Gen Ratio | 2nd Gen Ratio | Ratio Spread |
|------|--------------|---------------|--------------|
| 1st | 4.714 | 5.000 | - |
| 2nd | 3.143 | 3.200 | 1.50:1 / 1.56:1 |
| 3rd | 2.106 | 2.143 | 1.49:1 / 1.49:1 |
| 4th | 1.667 | 1.720 | 1.26:1 / 1.25:1 |
| 5th | 1.285 | 1.314 | 1.30:1 / 1.31:1 |
| 6th | 1.000 | 1.000 | 1.29:1 / 1.31:1 |
| 7th | 0.839 | 0.822 | 1.19:1 / 1.22:1 |
| 8th | 0.667 | 0.640 | 1.26:1 / 1.28:1 |
| Reverse | -3.295 | -3.456 | Overall spread: 7.07:1 / 7.81:1 |

**Gear ratio spread**: 7.07:1 (1st gen) to 7.81:1 (2nd gen) -- this is exceptionally wide, allowing both strong launch (low 1st) and low-cruise RPM (overdriven 8th).

**Key characteristic**: The ZF 8HP can skip gears on downshift (e.g., 8th to 4th) for rapid response. This is achieved by coordinating multiple clutch/brake transitions simultaneously.

### 2.3 ZF 6HP Transmission Family

**Architecture**: Lepelletier gear mechanism -- 1 simple planetary + 1 Ravigneaux compound planetary = 3 gearsets, 2 brakes, 3 clutches = 6 forward speeds + 1 reverse

**ZF 6HP26 Gear Ratios:**

| Gear | Ratio | Step to Next |
|------|-------|-------------|
| 1st | 4.171 | - |
| 2nd | 2.340 | 1.78:1 |
| 3rd | 1.521 | 1.54:1 |
| 4th | 1.143 | 1.33:1 |
| 5th | 0.867 | 1.32:1 |
| 6th | 0.691 | 1.25:1 |
| Reverse | -3.403 | Overall spread: 6.04:1 |

**Torque handling**: 6HP26 = 600 Nm, 6HP28 = 750 Nm, 6HP32 = 850 Nm

### 2.4 Torque Converter Behavior

The torque converter is the fluid coupling between the engine and the transmission input shaft. It has three distinct phases of operation:

**Phase 1: Stall (Launch)**
- Engine (pump) spinning, transmission (turbine) stationary or near-stationary
- Maximum torque multiplication: typically **1.8:1 to 2.5:1** for automotive converters
- High slippage = significant heat generation
- This is what you feel at a stoplight with foot on brake -- engine at ~600-800 RPM, turbine at 0 RPM
- Stall speed (the RPM at which the engine holds with brakes locked and throttle applied) is typically **1800-2800 RPM** for street vehicles, **3000-5000+ RPM** for performance converters

**Phase 2: Acceleration (Coupling Approach)**
- Turbine begins to spin, speed ratio (turbine/pump) increases from 0 toward ~0.9
- Torque multiplication decreases from peak toward 1:1
- Slippage decreases as speed ratio increases
- This is the primary acceleration phase
- Slip ratio at this stage typically follows an approximate curve:
  - At speed ratio 0.0: slip = 100%, torque mult = ~2.0-2.5
  - At speed ratio 0.4: slip = 60%, torque mult = ~1.5-1.7
  - At speed ratio 0.7: slip = 30%, torque mult = ~1.2-1.3
  - At speed ratio 0.85: slip = 15%, torque mult = ~1.05-1.1

**Phase 3: Coupling**
- Turbine speed reaches approximately **90% of pump (engine) speed**
- Torque multiplication effectively 1:1
- Behaves as a simple fluid coupling
- **Lock-up clutch engages** at this point in modern transmissions
- Lock-up eliminates the remaining ~10% slip, providing a mechanical 1:1 connection

**Lock-up clutch schedule (typical):**
- Engages in higher gears (3rd+) at light-to-medium throttle
- Engages above ~40-50 km/h in most applications
- Disengages during acceleration from low speed (to allow torque multiplication)
- Some modern transmissions use **partial lock-up** (controlled slip of 1-5%) even at lower speeds to balance efficiency and NVH
- ZF 8HP uses a "centrifugal pendulum absorber" in the torque converter damper to allow earlier lock-up

**For simulation purposes:**
```
effective_engine_RPM = turbine_RPM / (1 - slip_ratio)
where:
  slip_ratio = 1 - (turbine_RPM / engine_RPM)
  at stall: slip_ratio = 1.0
  at coupling: slip_ratio ~= 0.0-0.1
  after lockup: slip_ratio = 0.0
```

### 2.5 Shift Logic Algorithm (For Simulation)

For the VirtualICE Twin, a simplified but realistic shift algorithm:

```
Inputs: throttle_position (0-100%), vehicle_speed (km/h), current_gear, mode

1. Look up shift_map[mode][throttle_position][vehicle_speed] -> target_gear
2. If target_gear > current_gear AND speed > upshift_threshold[gear][throttle]:
     execute upshift (with 0.2s delay for ZF 8HP)
3. If target_gear < current_gear AND speed < downshift_threshold[gear][throttle]:
     execute downshift (with 0.2s delay)
4. Apply hysteresis: downshift_threshold = upshift_threshold * 0.85
     (15% speed hysteresis prevents hunting)

Torque converter model:
5. Calculate speed_ratio = turbine_speed / engine_speed
6. If speed_ratio < 0.9 AND lockup_clutch == disengaged:
     slip = 1.0 - speed_ratio
     effective_RPM = engine_RPM * (1 + slip * torque_multiplier_curve)
7. If in gear >= 3 AND speed_ratio >= 0.85 AND throttle < 75%:
     engage lockup clutch over 0.3-0.5 seconds
8. If throttle > 90% OR speed drops below lockup_release_speed:
     disengage lockup clutch
```

**Simplified shift thresholds for a generic 8-speed (illustrative):**

| Throttle | 1->2 | 2->3 | 3->4 | 4->5 | 5->6 | 6->7 | 7->8 |
|----------|------|------|------|------|------|------|------|
| 10% | 12 | 22 | 32 | 42 | 52 | 62 | 72 |
| 25% | 18 | 30 | 42 | 55 | 68 | 80 | 95 |
| 50% | 25 | 40 | 55 | 72 | 88 | 105 | 125 |
| 75% | 35 | 55 | 75 | 95 | 118 | 140 | 168 |
| 100% | 45 | 70 | 95 | 120 | 148 | 178 | 210 |

(Values in km/h, approximate for a ~3.0L engine with 8HP45)

---

## 3. EV Sound Simulation

### 3.1 Regulatory Background (AVAS)

Acoustic Vehicle Alerting Systems (AVAS) are mandated for quiet vehicles (EVs, hybrids) to warn pedestrians:

- **US (NHTSA FMVSS 141)**: Required for vehicles produced after Sept 2020. Sound required below **18.6 mph (30 km/h)**. Must include "one or more" tones in the 315-5000 Hz range. Minimum volume: varies by frequency band.
- **EU (UN ECE R138)**: Required for new types from July 2019, all new vehicles from July 2021. Sound required below **20 km/h (12.4 mph)** and when reversing. Volume: 56 dB(A) minimum (at 2m distance, 10 km/h), max 75 dB(A). Must sound like a vehicle (not a siren, horn, or animal sound).

### 3.2 Manufacturer Implementations

**Nissan Leaf VSP (Vehicle Sound for Pedestrians):**
- External speaker at front and rear
- Sound: sine-wave sweep from 2.5 kHz (at low speed) down to 600 Hz (at higher speed)
- Continuous tone, speed-dependent frequency shifting
- Automatic on below ~30 km/h, off above
- Driver toggle available

**Lotus/Harman HALOsonic:**
- Uses both internal (cabin audio system) and external (front/rear mounted) speakers
- Driver-selectable engine sounds: V6, V8, V12 options
- Internal sound is an "engine order" synthesis that responds to vehicle dynamics
- External sound meets AVAS requirements
- Demonstrated on the Lotus Evora 414E hybrid concept

**Fisker Karma:**
- External sound described as "a mix between a Formula One car and a starship"
- Automatic below 25 mph
- Used external speakers in the front bumper

**Hyundai VESS (Virtual Engine Sound System):**
- Mimics the sound of an idling internal combustion engine
- External speaker mounted at front
- Automatic at low speeds and during reverse

**Tesla Pedestrian Warning System:**
- Introduced September 2019 (Model 3, later all models)
- Sound produced below 19 mph (30.6 km/h)
- External speaker in front fascia
- Relatively subtle "whirring" tone
- Can be temporarily disabled via touchscreen (US regulation allows temporary disable)

### 3.3 Hardware Solutions (Aftermarket / Consumer)

**SoundRacer** (and similar OBD-II plug-in devices):
- Plugs into OBD-II port, reads RPM
- Transmits engine sound via FM radio to car stereo
- Multiple engine sound profiles (V8, V10, V12, etc.)
- Sound intensity tracks RPM
- Limitation: Only tracks RPM, not load/throttle -- sounds flat at constant RPM

**Aftermarket exhaust simulators:**
- External speaker kits (e.g., Kreise, Legato Performance)
- Mount under vehicle, wired to throttle/RPM signal
- Produce V8/V6 engine sounds externally
- Some integrate with CAN bus for RPM data
- Sound quality varies significantly

### 3.4 Software / Racing Simulator Approaches

Racing simulators (iRacing, Assetto Corsa, Forza, etc.) generate engine sound through:

**Approach 1: Loop blending (most common in games)**
- Pre-recorded audio loops at multiple RPM points (every 500 RPM)
- Crossfade between loops based on current RPM
- Add load-based filter (load = louder, richer harmonics)
- Pros: Realistic timbre, easy to implement
- Cons: Large audio asset library needed, transitions can be audible

**Approach 2: Granular synthesis**
- Break engine sound recordings into tiny grains (1-50ms)
- Reassemble grains at different rates to change pitch without changing duration
- Allows smooth RPM tracking
- Used by: Forza Motorsport series (historically)

**Approach 3: Physical modeling / FM synthesis**
- Model each cylinder firing event as a synthetic impulse
- Use FM (frequency modulation) synthesis to generate harmonics
- Each "firing" generates a base pulse, harmonics added via modulators
- Engine order = cylinder_count / 2 (for 4-stroke)
  - 4-cylinder: 2nd order (2 pulses per revolution)
  - 6-cylinder: 3rd order
  - 8-cylinder: 4th order
- Intake and exhaust modeled as resonant filters with volume-dependent Q
- Pros: Infinite RPM range, fully parametric, lightweight
- Cons: Requires careful tuning to sound realistic, can sound "synthetic"

**Approach 4: Additive synthesis**
- Decompose engine sound into individual harmonic components
- Each harmonic is a sine wave at (engine_order * RPM/60 * harmonic_number)
- Amplitude of each harmonic varies with RPM and load
- Can capture the characteristic "tone" of specific engines
- Used in some professional simulators

### 3.5 Recommended Approach for VirtualICE Twin

Given the constraints (real-time, responsive to Tesla telemetry, running on embedded/Mac hardware):

**Recommended: Physical modeling with FM synthesis**

Rationale:
1. Lightweight CPU usage (critical for real-time)
2. Parametric -- responds instantly to RPM, throttle, gear changes
3. No audio asset library needed
4. Can be tuned to sound like different transmission/engine types
5. Well-suited to the "virtual engine" concept

**Implementation sketch:**
```
For each audio sample:
  1. Calculate engine_RPM from vehicle telemetry:
     - If CAN bus (50-100 Hz): interpolate RPM between samples
     - If REST/streaming API (2 Hz): estimate RPM from speed + gear ratio + torque converter model

  2. Calculate firing_frequency = RPM / 60 * (cylinders / 2)

  3. Generate base pulse (sawtooth or rectified sine at firing_frequency)

  4. Apply FM modulation:
     carrier = base_pulse
     modulator = sine at (firing_frequency * harmonic_ratio)
     output = carrier + modulation_depth * modulator

  5. Apply load-dependent filter:
     - Throttle > 50%: boost 200-800 Hz (deeper, throatier)
     - Throttle < 25%: attenuate above 1 kHz (quieter, more muffled)

  6. Apply torque converter effect:
     - When slip > 10%: add slight RPM wobble (torque converter turbulence)
     - During shift: quick RPM blip (downshift) or dip (upshift) over 0.2s

  7. Apply gear-dependent volume:
     - Lower gears: louder (more mechanical advantage = more engine effort)
     - Higher gears: quieter at same speed (engine loafing)

  8. Output to audio device
```

### 3.6 Engine Order Reference

For generating realistic engine sound, the fundamental frequencies:

| Engine Config | Firing Order | Fundamental Freq @ RPM |
|---------------|-------------|----------------------|
| Inline-4 | 2nd order | RPM * 2 / 60 Hz |
| Inline-6 | 3rd order | RPM * 3 / 60 Hz |
| V6 | 3rd order | RPM * 3 / 60 Hz |
| V8 (cross-plane) | 4th order | RPM * 4 / 60 Hz |
| V8 (flat-plane) | 4th order | RPM * 4 / 60 Hz |
| V10 | 5th order | RPM * 5 / 60 Hz |
| V12 | 6th order | RPM * 6 / 60 Hz |

**Example**: A simulated V8 at 3000 RPM:
- Fundamental = 3000 * 4 / 60 = 200 Hz
- 2nd harmonic = 400 Hz
- 3rd harmonic = 600 Hz
- Exhaust note typically emphasizes odd harmonics (200, 600, 1000 Hz)
- Intake note typically emphasizes even harmonics (400, 800, 1200 Hz)

---

## 4. Data Sources Summary

### Successfully Retrieved from Web
- Standard OBD-II PID table (SAE J1979)
- Tesla REST API and Streaming API endpoints and field names
- Torque converter operational phases and stall ratios (Wikipedia - Torque converter)
- Automatic transmission TCU control logic (Wikipedia - Automatic transmission)
- EV warning sounds / AVAS regulations and manufacturer implementations (Wikipedia - Electric vehicle warning sounds)
- ZF 8HP45 and 6HP26 gear ratios (Wikipedia - ZF 8HP / ZF 6HP)

### Supplemented with Domain Knowledge
- Tesla CAN bus decoded messages (community knowledge, not formally documented)
- Shift map algorithms and typical threshold values
- Torque converter slip ratio schedules
- Racing simulator sound synthesis approaches (FM, additive, granular, loop-blending)
- Engine order calculations for physical modeling

### Not Retrieved (Sources Unavailable)
- engine-sim GitHub repository (repo taken down or moved - 404 on all attempted URLs)
- SoundRacer technical details (site returned forbidden)
- Tesla official CAN bus documentation (proprietary, not publicly available)
