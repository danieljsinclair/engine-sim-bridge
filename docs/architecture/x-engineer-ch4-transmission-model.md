# x-engineer.org Ch4: Automatic Transmission Plant Model

Source: https://x-engineer.org/modeling-simulation-vehicle-automatic-transmission/4/
Reference vehicle: Mercedes 5.0L V8 with 7-speed automatic, Sachs s244 torque converter

---

## 1. Torque Converter Model

The torque converter couples the engine (impeller) to the gearbox (turbine). It allows disconnection at standstill, provides smooth launch coupling, and amplifies torque at high slip ratios.

### 1.1 Speed Ratio

```
i_tc = N_t / N_i = N_t / N_e                        ... (7)
```

- `i_tc` [-] -- speed ratio (0 at stall, ~0.97 at coupling point)
- `N_t` [rpm] -- turbine speed
- `N_i` = `N_e` [rpm] -- impeller speed (equals engine speed)

### 1.2 Impeller Torque (pump torque absorbed from engine)

```
T_i = rho * lambda_tc * N_i^2 * D_tc^5               ... (6)
```

- `T_i` [Nm] -- impeller torque (load the converter places on the engine)
- `rho` [kg/m^3] -- converter fluid density = **900 kg/m^3**
- `lambda_tc` [-] -- performance coefficient (lookup from table vs `i_tc`)
- `N_i` [rpm] -- impeller speed (= engine speed)
- `D_tc` [m] -- impeller active diameter = **0.244 m**

Note: The `N_i^2` term makes impeller torque quadratic in engine speed. At higher RPM the converter absorbs significantly more torque from the engine.

### 1.3 Turbine Torque (torque delivered to gearbox)

```
T_t = k_tc * T_i                                      ... (5)
```

- `T_t` [Nm] -- turbine torque (output to gearbox input shaft)
- `k_tc` [-] -- torque coefficient (lookup from table vs `i_tc`)
- `T_i` [Nm] -- impeller torque

This is the key torque multiplication equation. At stall (`i_tc = 0`), `k_tc = 2.163`, meaning the turbine delivers more than double the impeller torque. As speed ratio approaches 1, `k_tc` drops below 1, and the converter transitions from amplification to a slight loss.

### 1.4 Sachs s244 Lookup Tables

| `i_tc` | `k_tc` | `lambda_tc` |
|--------|--------|-------------|
| 0.000  | 2.163  | 0.169       |
| 0.110  | 2.063  | 0.169       |
| 0.220  | 1.943  | 0.168       |
| 0.330  | 1.804  | 0.166       |
| 0.440  | 1.652  | 0.163       |
| 0.550  | 1.493  | 0.159       |
| 0.660  | 1.323  | 0.154       |
| 0.770  | 1.152  | 0.146       |
| 0.825  | 1.072  | 0.138       |
| 0.861  | 1.018  | 0.126       |
| 0.897  | 0.972  | 0.112       |
| 0.906  | 0.965  | 0.108       |
| 0.915  | 0.963  | 0.104       |
| 0.924  | 0.963  | 0.099       |
| 0.932  | 0.961  | 0.094       |
| 0.941  | 0.959  | 0.087       |
| 0.950  | 0.952  | 0.078       |
| 0.959  | 0.935  | 0.067       |
| 0.968  | 0.908  | 0.051       |
| 0.977  | 0.852  | 0.033       |

**Key observations:**
- At stall (`i_tc = 0`): torque multiplication is **2.163x**
- Coupling point (~`i_tc = 0.86`): `k_tc` crosses ~1.0, multiplication ends
- At high speed ratio (`i_tc > 0.95`): `k_tc < 1.0`, the converter actually absorbs more torque than it delivers
- `lambda_tc` is roughly constant (~0.169) at low speed ratios, then drops off rapidly above 0.85 -- this means the engine load from the converter decreases as it approaches coupling

### 1.5 Torque Converter Parameters

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Impeller active diameter | `D_tc` | 0.244 | m |
| Fluid density | `rho` | 900 | kg/m^3 |

---

## 2. Gearbox and Differential Model

### 2.1 Gearbox Output Torque

```
T_g = T_t * i_x * eta_x                             ... (8)
```

- `T_g` [Nm] -- gearbox output torque (to driveshaft)
- `T_t` [Nm] -- turbine torque (from converter)
- `i_x` [-] -- gear ratio for current gear
- `eta_x` [-] -- mechanical efficiency for current gear

Lower gears have higher ratios (4.381 for 1st) but lower efficiency (0.930). Top gear (5th) is direct drive at ratio 1.000 and 100% efficiency.

### 2.2 Turbine Speed (from wheel speed)

```
N_t = N_w * i_x * i_0                                ... (9)
```

- `N_t` [rpm] -- turbine speed
- `N_w` [rpm] -- wheel speed
- `i_x` [-] -- gear ratio
- `i_0` [-] -- final drive (differential) ratio = **2.769**

### 2.3 Gear Ratios and Efficiencies

| Gear | `i_x` | `eta_x` |
|------|-------|---------|
| 1    | 4.381 | 0.930   |
| 2    | 2.860 | 0.948   |
| 3    | 1.917 | 0.973   |
| 4    | 1.363 | 0.987   |
| 5    | 1.000 | 1.000   |
| 6    | 0.822 | 0.987   |
| 7    | 0.731 | 0.987   |

### 2.4 Differential Parameter

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Final drive ratio | `i_0` | 2.769 | - |

---

## 3. Engine Model (Ch3, for context)

The engine model provides the torque that feeds into the torque converter impeller.

### 3.1 Engine Dynamics

```
J_e * d(omega_e)/dt = T_e - T_i                      ... (1)
omega_e = (1/J_e) * integral(T_e - T_i) dt           ... (2)
N_e = 30 * omega_e / pi                               ... (3)
P_e = T_e * omega_e / 1000                            ... (4)
```

- `J_e` = **0.08 kg*m^2** -- engine + impeller inertia
- `T_e` [Nm] -- engine torque (from throttle/RPM map)
- `T_i` [Nm] -- impeller torque (load from converter, equation 6)
- `N_e` [rpm] -- engine speed (clamped 1000-7000)

### 3.2 Engine Torque Map (Mercedes 5.0L V8)

Torque is a 2D lookup: function of accelerator pedal position (%) and engine speed (rpm). Full-load peak: 460 Nm at 3000-4250 rpm. Negative torque (engine braking) at 0% throttle above 1000 rpm.

---

## 4. Complete Torque Path (Engine to Wheels)

Combining all equations, the full torque flow is:

```
T_e (engine torque from map)
  |
  v
T_i = rho * lambda_tc(i_tc) * N_e^2 * D_tc^5       (impeller load on engine)
  |
  v
T_t = k_tc(i_tc) * T_i                              (turbine torque, multiplied)
  |
  v
T_g = T_t * i_x * eta_x                             (gearbox output torque)
  |
  v
T_d = T_g * i_0 * eta_0                             (differential output to wheels)
```

Speed relationships (inverse direction):

```
N_w (wheel speed)
  -> N_t = N_w * i_x * i_0                           (turbine speed)
  -> i_tc = N_t / N_e                                (speed ratio for lookup)
```

---

## 5. Mapping to Our AutomaticGearbox Model

### 5.1 What We Have

Our `AutomaticGearbox` (in `src/twin/AutomaticGearbox.cpp`) is a **shift scheduler**, not a torque physics model. It:

- Determines **which gear** to be in based on vehicle speed and throttle
- Uses a 2D shift table (`shiftTable[throttleLevel][gearIndex]`) for upshift/downshift speed thresholds
- Implements kickdown detection, hysteresis, redline forced upshift, coast-down shifts
- Computes `getEngineRpm()` as: `wheelRpm * gearRatios[gear] * diffRatio`
- Has **no torque converter model** -- no slip, no torque multiplication, no impeller load

Our `IceVehicleProfile` (ZF 8HP45) has 8 gear ratios `{4.714, 3.143, ..., 0.667}` with `diffRatio = 3.15`.

### 5.2 What the x-engineer Model Adds

The x-engineer model introduces three physics layers we are missing:

1. **Torque converter slip**: Our model assumes engine speed = wheel speed * gear ratio * diff ratio. The real relationship has slip -- at low speed ratios the turbine spins much slower than the impeller. This means our RPM calculation overestimates gearbox input RPM at low vehicle speeds (especially during launch).

2. **Torque multiplication**: At stall, the Sachs s244 multiplies torque by 2.163x. This is why automatic cars launch hard despite the engine not being directly coupled. Our model passes torque straight through with no multiplication.

3. **Impeller load on engine**: The converter loads the engine with `T_i = rho * lambda_tc * N_e^2 * D_tc^5`. This quadratic load term affects engine dynamics (equation 1). Our model has no concept of converter load feedback to the engine.

### 5.3 Integration Approach

To add torque converter physics, the minimum viable additions would be:

| Component | What to Add | Where |
|-----------|-------------|-------|
| Speed ratio | Compute `i_tc = N_t / N_e` | New `TorqueConverter` class or inline in `VirtualIceTwin` |
| k_tc / lambda_tc tables | Store as interpolated lookup tables | `IceVehicleProfile` or separate converter profile |
| Turbine torque | `T_t = k_tc * T_i` | Torque path in physics update |
| Impeller load | `T_i = rho * lambda_tc * N_e^2 * D_tc^5` | Engine dynamics equation |
| Slip-corrected RPM | Use `N_t` instead of `N_e` for gearbox RPM | Replace `getEngineRpm()` with turbine-speed-aware version |

The shift scheduling logic (gear selection based on speed/throttle tables) can remain unchanged -- torque converter physics affect the torque values and speed relationships but not the gear selection strategy itself.

### 5.4 Parameter Mapping (x-engineer to our model)

| x-engineer Parameter | Our Model | Notes |
|---------------------|-----------|-------|
| `i_x` gear ratios (7 gears) | `gearRatios` (8 gears, ZF 8HP45) | Different transmission, same concept |
| `i_0` = 2.769 | `diffRatio` = 3.15 | Different final drive |
| `eta_x` per gear | Not modeled | Could add as `std::vector<double> gearEfficiencies` |
| `D_tc` = 0.244 m | Not modeled | New parameter |
| `rho` = 900 kg/m^3 | Not modeled | New parameter |
| `k_tc`, `lambda_tc` tables | Not modeled | New lookup tables |
| `J_e` = 0.08 kg*m^2 | Not modeled | Affects engine dynamics |
| `N_e` clamped 1000-7000 | `idleRpm` = 750, `redlineRpm` = 6500 | Similar concept, different values |

---

## 6. Summary of All Equations

| Eq | Formula | Description |
|----|---------|-------------|
| (1) | `J_e * d(omega_e)/dt = T_e - T_i` | Engine rotational dynamics |
| (2) | `omega_e = (1/J_e) * integral(T_e - T_i) dt` | Engine speed integration |
| (3) | `N_e = 30 * omega_e / pi` | rad/s to rpm conversion |
| (4) | `P_e = T_e * omega_e / 1000` | Engine power [kW] |
| (5) | `T_t = k_tc * T_i` | Turbine torque (torque multiplication) |
| (6) | `T_i = rho * lambda_tc * N_i^2 * D_tc^5` | Impeller torque (converter load on engine) |
| (7) | `i_tc = N_t / N_i` | Torque converter speed ratio |
| (8) | `T_g = T_t * i_x * eta_x` | Gearbox output torque |
| (9) | `N_t = N_w * i_x * i_0` | Turbine speed from wheel speed |

---

## 7. Key Takeaways for Implementation

1. **Launch behavior**: The torque converter is the primary reason automatics feel different from manuals at launch. The 2.163x stall torque multiplication means significant torque is available even when the turbine is nearly stationary. Our current model misses this entirely.

2. **Speed ratio drives everything**: Both `k_tc` and `lambda_tc` are functions of `i_tc` alone. The converter's behavior is fully characterized by this single variable and the three lookup tables. Implementation requires only 1D interpolation.

3. **Quadratic impeller load**: The `N_i^2` term in equation (6) means converter load scales quadratically with engine speed. At high RPM, the converter is a significant load on the engine. This affects engine dynamics and should be included in any engine speed integration.

4. **Efficiency varies by gear**: The x-engineer model uses per-gear efficiency values (0.930 to 1.000). We currently assume 100% efficiency in all gears. Adding per-gear efficiency is straightforward.

5. **Our shift scheduling is independent**: The gear selection logic (speed+throttle tables, kickdown, hysteresis) operates at a higher level of abstraction and does not need to change when torque converter physics are added. The converter sits between the engine and the gearbox in the torque path, not in the control path.
