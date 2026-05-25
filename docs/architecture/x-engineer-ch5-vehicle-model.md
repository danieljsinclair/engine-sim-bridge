# X-Engineer Chapter 5: Plant Model — Vehicle

**Source**: https://x-engineer.org/modeling-simulation-vehicle-automatic-transmission/5/
**Part of**: "Modeling and simulation of a vehicle with automatic transmission" tutorial series
**Equation range**: Equations 10–23

---

## 1. Overview

The vehicle is modeled as Rear Wheel Drive (RWD). The model treats the vehicle as a mass in translation: it receives gearbox torque (converted to traction force) as the driving input and the sum of road loads as the resistive force.

The fundamental longitudinal dynamics equation is:

```
F_g = F_w                              ... (10)
```

where:
- `F_g [N]` — Gearbox (traction) force
- `F_w [N]` — Wheel (resistive) force

At equilibrium, traction equals resistance. The vehicle accelerates when traction exceeds resistance and decelerates when resistance exceeds traction.

---

## 2. Gearbox (Traction) Force

The gearbox force is the force at the wheel hubs after the final (differential) gear:

```
F_g = (T_g * i_0 * eta_0 * eta_l) / r_w    ... (11)
```

where:
- `T_g [Nm]` — Gearbox output torque (from transmission model)
- `i_0 [-]` — Final drive (differential) gear ratio
- `eta_0 [-]` — Final drive (differential) efficiency
- `eta_l [-]` — Longitudinal (propeller) shaft efficiency
- `r_w [m]` — Wheel (rolling) radius

**Key insight**: Transmission output torque is multiplied by the final drive ratio and efficiency losses, then divided by the wheel radius to convert from torque to linear force at the tire contact patch.

---

## 3. Traction Force Limit (Tire Grip Constraint)

The traction force cannot exceed the friction limit of the driving wheels:

```
F_lim = m_v * g * mu_w * c_w               ... (12)
```

where:
- `m_v [kg]` — Vehicle mass
- `g [m/s^2]` — Gravitational acceleration
- `mu_w [-]` — Driving wheels friction coefficient
- `c_w [-]` — Driving axle load coefficient

The load coefficient `c_w` represents the fraction of total vehicle weight on the driving axle. For example, `c_w = 0.65` means 65% of the vehicle weight sits on the rear (driving) wheels.

**Applied traction force**: `F_trac = min(F_g, F_lim)`

---

## 4. Road Load (Resistive Forces) Model

The total resistive force is the sum of five components:

```
F_w = F_i + F_a + F_r + F_s + F_b          ... (13)
```

where:
- `F_i [N]` — Inertia force
- `F_a [N]` — Aerodynamic drag force
- `F_r [N]` — Rolling resistance force
- `F_s [N]` — Road slope (gradient) force
- `F_b [N]` — Braking force

### 4.1 Inertia Force

```
F_i = m_v * c_i * (dv_v / dt)              ... (14)
```

where:
- `m_v [kg]` — Vehicle mass
- `c_i [-]` — Rotational components inertia coefficient
- `v_v [m/s]` — Vehicle speed

The coefficient `c_i` accounts for the inertia of all rotating components (wheels, propeller shaft, gears, etc.). A value of `c_i = 1.1` means the effective mass is 10% higher than the static vehicle mass to capture rotational inertia effects.

### 4.2 Aerodynamic Drag Force

```
F_a = 0.5 * C_d * A * v_v^2 * rho          ... (15)
```

where:
- `C_d [-]` — Aerodynamic drag coefficient
- `A [m^2]` — Vehicle frontal area
- `rho [kg/m^3]` — Air density
- `v_v [m/s]` — Vehicle speed

Drag is proportional to the square of vehicle speed — the dominant resistance at high speeds.

### 4.3 Rolling Resistance Force

```
F_r = m_v * g * cos(alpha) * f              ... (16)
```

where:
- `alpha [rad]` — Road slope angle
- `f [-]` — Rolling resistance coefficient

The road slope angle is derived from the gradient percentage:

```
alpha = arctan(s / 100)                     ... (17)
```

where `s [%]` is the road slope (gradient).

The rolling resistance coefficient is speed-dependent:

```
f = C_0 + C_1 * v_v + C_2 * v_v^2          ... (18)
```

where:
- `C_0 [-]` — Rolling resistance constant coefficient
- `C_1 [s/m]` — Rolling resistance linear coefficient
- `C_2 [s^2/m^2]` — Rolling resistance quadratic coefficient

### 4.4 Road Slope (Gradient) Force

```
F_s = m_v * g * sin(alpha)                  ... (19)
```

### 4.5 Braking Force

The braking force is modeled as a first-order transfer function:

```
H(s) = K / (T * s + 1) = 100 / (0.1 * s + 1)   ... (20)
```

where:
- `K = 100` — Gain
- `T = 0.1 s` — Time constant

The input is brake pedal position in [%]. At 100% pedal press, the braking force is 10,000 N. The time constant provides mild filtering of the driver input.

---

## 5. Vehicle Speed Calculation

Vehicle acceleration is derived from Newton's second law (rearranging equation 10):

```
a_v = (F_trac - F_w) / (m_v * c_i)
```

Speed is obtained by integrating acceleration:

```
v_v = integral(a_v) dt     [m/s]
```

Vehicle offset (distance traveled):

```
x_v = integral(v_v) dt                       ... (21)
```

where `x_v [m]` — vehicle offset from start.

### 5.1 Speed Unit Conversions

Vehicle speed in kph:

```
V_v = 3.6 * v_v                             ... (22)
```

where `V_v [kph]` — vehicle speed in kilometers per hour.

### 5.2 Wheel Speed in RPM

```
N_w = (v_v / r_w) * (30 / pi)               ... (23)
```

where `N_w [rpm]` — wheel speed.

This is derived from `v = omega * r_w` and converting from rad/s to rpm: `rpm = (rad/s) * 30 / pi`.

---

## 6. RPM-Speed-Gear Ratio Relationship

Combining the equations from this chapter with those from earlier chapters, the relationship between engine RPM, gear ratio, and vehicle speed is:

```
N_e = N_w * i_x * i_0
    = (v_v / r_w) * (30 / pi) * i_x * i_0
```

where:
- `N_e [rpm]` — Engine speed
- `N_w [rpm]` — Wheel speed
- `i_x [-]` — Current gear ratio
- `i_0 [-]` — Final drive ratio
- `r_w [m]` — Wheel radius
- `v_v [m/s]` — Vehicle speed

Rearranging to get vehicle speed from engine RPM:

```
v_v = (N_e * pi * r_w) / (30 * i_x * i_0)   [m/s]
```

---

## 7. Complete Parameter Table

### Traction Force Parameters

| Parameter | Symbol | Value | Unit | Description |
|-----------|--------|-------|------|-------------|
| `VehM_kg_C` | m_v | 2255 | kg | Vehicle mass |
| `VehGrvygA_mps2_C` | g | 9.81 | m/s^2 | Gravitational acceleration |
| `VehWhlFricCoeff_z_C` | mu_w | 1.0 | - | Driving wheels friction coefficient |
| `VehReWghtCoeff_z_C` | c_w | 0.6 | - | Driving axle load coefficient (60% rear weight) |
| `LgtShaftEff_z_C` | eta_l | 0.994 | - | Longitudinal (propeller) shaft efficiency |
| `DftlGearRat_z_C` | i_0 | 2.769 | - | Final drive (differential) gear ratio |
| `DftlEff_z_C` | eta_0 | 0.93 | - | Final drive (differential) efficiency |
| `VehWhlRollgRd_m_C` | r_w | 0.31587 | m | Wheel (rolling) radius |

### Road Resistance Parameters

| Parameter | Symbol | Value | Unit | Description |
|-----------|--------|-------|------|-------------|
| `VehM_kg_C` | m_v | 2255 | kg | Vehicle mass |
| `VehGrvygA_mps2_C` | g | 9.81 | m/s^2 | Gravitational acceleration |
| `VehDragCoeff_z_C` | C_d | 0.29 | - | Aerodynamic drag coefficient |
| `VehFrntAr_m2_C` | A | 2.138 | m^2 | Vehicle frontal area |
| `VehAirRho_kgpm3_C` | rho | 1.202 | kg/m^3 | Air density |
| `C0` | C_0 | 1.3295e-2 | - | Rolling resistance constant coefficient |
| `C1` | C_1 | -2.8664e-5 | s/m | Rolling resistance linear coefficient |
| `C2` | C_2 | 1.8036e-7 | s^2/m^2 | Rolling resistance quadratic coefficient |

### Speed Calculation Parameters

| Parameter | Symbol | Value | Unit | Description |
|-----------|--------|-------|------|-------------|
| `VehM_kg_C` | m_v | 2255 | kg | Vehicle mass |
| `VehRotCmpJCoeff_z_C` | c_i | 1.25 | - | Rotational components inertia coefficient |
| `VehWhlRollgRd_m_C` | r_w | 0.31587 | m | Wheel (rolling) radius |

### Derived Constants

| Constant | Value | Derivation |
|----------|-------|------------|
| Traction limit force | F_lim = 2255 * 9.81 * 1.0 * 0.6 = **13,278.9 N** | From eq. (12) |
| Effective mass | m_eff = 2255 * 1.25 = **2,818.75 kg** | Accounting for rotational inertia |
| Max braking force | **10,000 N** | At 100% pedal, gain = 100 |

---

## 8. Xcos Block Diagram Structure

The vehicle model is decomposed into three subsystems:

1. **Traction Force** — Takes gearbox torque, computes `F_g` via eq. (11), limits to `F_lim` via eq. (12), outputs `min(F_g, F_lim)`.
2. **Road Resistances** — Takes road slope, brake pedal, and vehicle speed; computes all five resistive force components via eqs. (13)–(20); outputs total drag force.
3. **Speed Calculation** — Takes traction force and total drag force; computes net force, divides by effective mass, integrates to get speed in m/s, converts to kph and wheel RPM via eqs. (21)–(23).

---

## 9. Implications for Virtual Twin Implementation

For our engine-sim-bridge project, the key takeaways are:

- **Torque-to-force conversion**: Engine torque flows through gear ratio, final drive, efficiency losses, and is divided by wheel radius to become tractive force.
- **Speed feedback loop**: Vehicle speed feeds back into road load calculations (aerodynamic drag is quadratic in speed, rolling resistance is speed-dependent), creating a natural feedback loop.
- **RPM-speed coupling**: Engine RPM and vehicle speed are rigidly coupled through the gear ratio and final drive — useful for computing expected RPM from speed (or vice versa) in the twin.
- **Traction limiting**: The model caps tractive force at the tire grip limit, preventing unphysical acceleration. This is a useful sanity check for the twin.
- **Effective mass**: The 1.25 rotational inertia coefficient means the vehicle behaves as if it weighs 25% more during acceleration — important for realistic simulation feel.
