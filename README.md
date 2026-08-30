# Engine Sim Bridge

C bridge layer for engine-sim, providing a stable C API for .NET P/Invoke integration and other language bindings.

## Purpose

This repository contains the bridge layer that wraps the core engine-sim C++ library in a C API. This allows the simulation to be used from:
- .NET applications via P/Invoke
- Other languages with C FFI support
- CLI tools that prefer a C API

## Architecture

- **engine-sim-bridge** (this repo): C bridge API
- **engine-sim**: Core C++ simulation library

## Building

```bash
mkdir build && cd build
cmake ..
make
```

## API

The bridge provides a C API defined in `include/engine_sim_bridge.h`. Key features:
- Zero-allocation render path
- Thread-safe for single-threaded audio callbacks
- 48kHz sample rate support
- Script loading support
- Real-time parameter control (throttle, RPM, load, etc.)

## Replay `--start-from` (instant arrival state)

For file-trace replay (`--replay-telemetry` + `--start-from T`), rows before
`T` are NEVER simulated — whatever the offset (start, middle, or end of a
capture). The state at the offset is synthesized instead:

1. **Provider prime** (`input::IArrivalStatePrimer`, implemented by
   `ReplayTelemetryProvider`): the VirtualICE twin warm-boots seeded from the
   first row at/after `T` (throttle, road speed, selector), the gearbox and
   coupling settle at that operating point, and the replay clock anchors on
   the arrival row's timecode.
2. **Core settle** (`SimulationLoop::settleAtArrivalPoint`): a bounded,
   offset-independent window of suppressed full-tick stepping at the HELD
   arrival row. The wheel pin drags the drivetrain to the recorded road speed
   (the engine catches through its own physics), and the gas path relaxes to
   the quasi-steady state consistent with that rpm/throttle.
3. **Handoff**: hold released, loop clock anchored at `T`, audio ring reset —
   rows emit from the arrival row onward.

Cost is constant (~seconds, not `T x real-time`): a 527 s offset settles in
the same time as a 10 s one. Guarantees: from-0 runs (no `--start-from`) are
byte-identical to runs without this code path, repeated runs at the same
offset are byte-identical under `--deterministic`, and the live-stream path
(`--live-telemetry`, unseekable) keeps its own unchanged instant contract.

## License

Same license as upstream engine-sim project.
