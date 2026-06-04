# .mr → C++ Preset Code Generation Pipeline

## Overview

Build-time tool that compiles .mr engine scripts via Piranha, walks the resulting
object graph, and generates C++ source files implementing `IEnginePreset`. Each
generated .cpp creates a concrete `Simulator` subclass matching what `EnginePresets.cpp`
does by hand — but automatically.

## Prerequisite: Public API Gaps

Several engine-sim classes lack public getters needed for code generation.
These must be added to the engine-sim submodule before the pipeline works.

### Critical (code gen cannot work without these)

| Class | Getter to add | Returns |
|-------|--------------|---------|
| `Function` | `int getSampleCount() const` | `m_size` |
| `Function` | `double getX(int i) const` | `m_x[i]` |
| `Function` | `double getY(int i) const` | `m_y[i]` |
| `Function` | `double getFilterRadius() const` | `m_filterRadius` |
| `Transmission` | `int getGearCount() const` | `m_gearCount` |
| `Transmission` | `double getGearRatio(int i) const` | `m_gearRatios[i]` |
| `Transmission` | `double getMaxClutchTorque() const` | `m_maxClutchTorque` |

### High (most engine params are inaccessible)

| Class | Getter to add | Returns |
|-------|--------------|---------|
| `Intake` | `double getVolume() const` | plenum volume |
| `Intake` | `double getInputFlowK() const` | `m_inputFlowK` |
| `Intake` | `double getIdleFlowK() const` | `m_idleFlowK` |
| `Intake` | `double getIdleThrottlePlatePosition() const` | raw param |
| `CylinderHead` | `Function* getIntakePortFlow() const` | `m_intakePortFlow` |
| `CylinderHead` | `Function* getExhaustPortFlow() const` | `m_exhaustPortFlow` |

### Low (one value, inferrable, or already defaulted)

| Class | Getter to add | Returns |
|-------|--------------|---------|
| `ExhaustSystem` | `double getOutletFlowRate() const` | `m_outletFlowRate` |
| `Camshaft` | `int getLobeCount() const` | `m_lobes` |

## Pipeline Architecture

```
  .mr scripts ──► preset_codegen ──► generated_preset_foo.cpp
  (Piranha)       (build-time tool)     (compiled into bridge)
```

### Step 1: Build-time code generator (`tools/preset_codegen.cpp`)

Replaces the existing `preset_compiler.cpp` (JSON intermediate).
Runs on macOS only (needs Piranha). Takes:
- `--script <path>` — .mr script to compile
- `--output <path>` — generated .cpp output file
- `--asset-base <path>` — engine-sim assets directory
- `--classname <name>` — C++ class name (e.g., `HondaTrx520Preset`)

The generator:
1. Compiles .mr via Piranha (same wrapper technique as existing compiler)
2. Walks Engine/Vehicle/Transmission object graph using public getters
3. Emits a C++ class that:
   - Extends `Simulator` (or creates one internally like `EnginePresets.cpp`)
   - Has a `create()` static method returning `Simulator*`
   - Hardcodes all engine parameters as C++ literals
   - Uses `EnginePresetsHelper::createFlowFunction()` for port flow
   - Uses `EnginePresetsHelper::createMeanPistonSpeedToTurbulence()` etc.
   - Calls `intake->m_system.reset(airMix)` to initialize manifold with air

### Step 2: CMake integration

```cmake
# In engine-sim-bridge/CMakeLists.txt
if(NOT IOS AND TARGET engine-sim-script-interpreter)
    # Collect all .mr scripts to generate presets for
    file(GLOB PRESET_SCRIPTS "${ENGINE_SIM_PATH}/assets/engines/**/*.mr")

    # Generated source directory
    set(GENERATED_PRESETS_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated_presets")
    file(MAKE_DIRECTORY ${GENERATED_PRESETS_DIR})

    foreach(script ${PRESET_SCRIPTS})
        # Derive class name and output path from script path
        get_filename_component(script_name "${script}" NAME_WE)
        set(output_cpp "${GENERATED_PRESETS_DIR}/preset_${script_name}.cpp")

        add_custom_command(
            OUTPUT "${output_cpp}"
            COMMAND engine-sim-preset-codegen
                --script "${script}"
                --output "${output_cpp}"
                --asset-base "${ENGINE_SIM_PATH}/assets"
                --classname "${script_name}_preset"
            DEPENDS "${script}" engine-sim-preset-codegen
            COMMENT "Generating C++ preset from ${script_name}.mr"
        )
        list(APPEND GENERATED_PRESET_SOURCES "${output_cpp}")
    endforeach()

    # Add generated sources to the bridge library
    target_sources(engine-sim-bridge PRIVATE ${GENERATED_PRESET_SOURCES})
endif()
```

### Step 3: PresetRegistry lookup

The generated presets register themselves with a `PresetRegistry`:

```cpp
// In each generated file:
static bool _registered = PresetRegistry::registerPreset(
    "engines/atg-video-1/01_honda_trx520.mr",  // key = relative script path
    []() -> Simulator* { return HondaTrx520Preset::create(); }
);
```

The CLI `--script` flag on the baked build looks up this registry:

```cpp
// In CLI main():
Simulator* sim = PresetRegistry::create(scriptPath);
if (!sim) {
    fprintf(stderr, "Unknown preset: %s\n", scriptPath);
    return 1;
}
```

## Generated Code Example

The generator emits code that looks like this (similar to EnginePresets.cpp):

```cpp
// Auto-generated from engines/atg-video-1/01_honda_trx520.mr
// DO NOT EDIT — regenerate with preset_codegen

#include "engine.h"
#include "simulator/EnginePresetsHelper.h"
// ... (all needed headers)

class HondaTrx520Preset : public Simulator {
public:
    static Simulator* create() {
        auto* sim = new HondaTrx520Preset();
        // ... initialize and return
        return sim;
    }
private:
    void buildEngine() {
        Engine::Parameters params;
        params.name = "Honda TRX520 (ATV)";
        params.cylinderCount = 1;
        params.cylinderBanks = 1;
        // ... all params as literals
        
        m_engine = new Engine();
        m_engine->initialize(params);
        
        // Crankshaft
        Crankshaft* cs = m_engine->getCrankshaft(0);
        Crankshaft::Parameters cp;
        cp.crankThrow = 0.03575;
        cp.flywheelMass = 2.26796;
        // ...
        cs->initialize(cp);
        
        // Intake with air mixture
        Intake* intake = m_engine->getIntake(0);
        Intake::Parameters ip;
        ip.volume = 0.0015;
        ip.InputFlowK = 0.0001;  // k_carb(100)
        // ...
        intake->initialize(ip);
        GasSystem::Mix airMix{0.0, 0.79, 0.21};
        intake->m_system.reset(101325.0, 298.15, airMix);
        
        // ... etc for all subsystems
    }
};
```

## Function Serialization

Functions are serialized as arrays of (x, y, filterRadius) tuples:

```cpp
// Generated code:
Function* portFlow = new Function();
portFlow->initialize(15, 50 * units::distance(1, units::thou));
portFlow->addSample(0.0, 0.0);
portFlow->addSample(0.00127, 50.0 * 2.0);
portFlow->addSample(0.00254, 150.0 * 2.0);
// ... etc
```

## Implementation Order

1. **Add public getters to engine-sim** — PR to engine-sim submodule
2. **Replace preset_compiler.cpp with preset_codegen.cpp** — new tool
3. **CMake integration** — auto-generate from .mr scripts
4. **PresetRegistry** — simple key → factory map
5. **CLI integration** — `--script` flag lookup on baked build
6. **Tests** — verify generated presets match Piranha output
