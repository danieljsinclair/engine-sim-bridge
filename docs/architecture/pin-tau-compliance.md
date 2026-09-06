# PIN Target Compliance — the `--pin-tau-ms` Map

Status: empirical map bench-verified 2026-08-30/31 (owner-tagged `pin_no_staircase`
chain). Mechanism reference: `include/twin/PinTargetChase.h`. Owner decisions:
default **150 ms** (2026-09-04), warn-only guards (no value is rejected).

## What tau is

The CAN road-speed signal arrives at ~5.5 Hz in ~0.9 km/h held steps. A rigid
pin (tau <= 0) teleports the wheel-speed target between those steps — the
audible "piano keys" staircase. `--pin-tau-ms` sets the time constant tau of a
critically-damped second-order low-pass (double pole at -1/tau) that the PIN
**target** chases; the ramp trail is ~2·tau·v. Scope: the pin target ONLY —
the gearbox shift map and the slip-lock math still see the raw road speed.
tau <= 0 is EXACTLY the rigid pin, bit-identical (the regression contract:
explicit `--pin-tau-ms 0`).

## Empirical stability map (deterministic bench, PinFixDrive3-class captures)

| tau (ms) | behavior |
|---|---|
| < 8.3 | below dt/2 — wrong-by-construction (filter cannot resolve a step) |
| 20–50 | **drivetrain bifurcation**: 50 ms ran away to ~207 mph / 15.5k rpm |
| 60–1000 | **stable window** (recommended). 60 ms = minimum usable, RMSE 0.63 |
| > 3000 | over-damped: 15000 ms halved road speed |
| 0 / negative | rigid passthrough (OFF), bit-identical legacy |

## Default and guards (owner-set 2026-09-04)

- CLI default: **150 ms** (`args.twin.pinTauMs` in `engine-sim-cli
  src/config/CLIconfig.h`). The bridge provider default (never-set) remains
  rigid so "disabled config == never-set" stays inert.
- Guards are **warn-only** (`pinTauWarningText` in `engine-sim-cli
  src/config/TelemetryProviderFactory.h`, printed yellow at parse time):
  warn when `0 < tau < 60` (bifurcation territory) and when `tau > 3000`
  (over-damped). 1001–3000 is odd but legal and silent. Every value is
  accepted — a tuning toggle must never be restricted from experimentation.
- **iOS app: tau is NOT exposed.** The best default (150 ms today) is baked
  into the app's provider wiring; the knob exists for CLI bench tuning only.

## Known interaction

The startup-crackle family is separate from tau (owner: "NO its two different
things"). Loud-attach emission onset is fixed by the audio envelope attack
(`EmissionOnsetEnvelope`), not by tau. A tau of 150 with a bad envelope still
crackles at attach; a good envelope with tau 50 still bifurcates.
