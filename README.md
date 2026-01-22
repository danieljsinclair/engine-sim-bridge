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

## License

Same license as upstream engine-sim project.
