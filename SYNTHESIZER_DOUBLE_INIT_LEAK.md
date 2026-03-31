# Synthesizer Double-Initialization Memory Leak

**Status:** PARKED - Known issue, deferred fix
**Date:** 2026-03-24
**Priority:** Medium (memory leak on each engine load, but small and bounded)

## Problem Description

The synthesizer is initialized **twice** during `EngineSimLoadScript`, causing a memory leak:

1. **First initialization:** `PistonEngineSimulator::loadSimulation()` calls `initializeSynthesizer()` at line 184
   - Uses hardcoded audio buffer sizes (44100 samples)
   - Uses correct `inputChannelCount = m_engine->getExhaustSystemCount()`
   - Allocates `m_inputChannels[]` and `m_filters[]` arrays

2. **Second initialization:** Bridge immediately calls `synthesizer().initialize()` again with user config
   - Uses user-provided buffer sizes and sample rates
   - Uses correct `inputChannelCount = m_engine->getExhaustSystemCount()`
   - Allocates NEW `m_inputChannels[]` and `m_filters[]` arrays
   - **First allocation is leaked** (not freed before overwriting pointers)

## Root Cause

### Architectural Mismatch

| Layer | Responsibility | Current Behavior |
|-------|----------------|------------------|
| **engine-sim** | Engine physics simulation | `loadSimulation()` calls `initializeSynthesizer()` with **hardcoded audio defaults** |
| **engine-sim-bridge** | C API wrapper with user config | Needs to apply **user-provided audio configuration** (sample rates, buffer sizes, etc.) |

### The Bridge's Dilemma

The bridge must initialize the synthesizer with user configuration, but:
1. In `EngineSimCreate`: Engine not loaded yet, can't know `exhaustSystemCount`
2. In `EngineSimLoadScript`: `loadSimulation()` already initializes with hardcoded values
3. Re-initializing applies user config but leaks the first allocation

## Current State (After Partial Fix)

The bridge now:
1. **Defers** synthesizer initialization until `EngineSimLoadScript` (no longer initializes in `Create`)
2. **Re-initializes** after `loadSimulation()` to apply user config
3. **Result:** Correct exhaust count + correct user config, but small memory leak

### Leak Size Estimate

Per engine load:
- `sizeof(InputChannel) * exhaustCount` (~48 bytes * N channels)
- `sizeof(ProcessingFilters) * exhaustCount` (~10,000+ bytes * N channels with convolution data)

For dual exhaust: ~20KB leak per engine load (bounded, occurs once per session)

## Possible Solutions

### Recommended: Option C (Add Synth Params to Simulator)

This is the cleanest architectural fix because it follows the existing pattern of configuration flowing through `Simulator::Parameters`.

**engine-sim files to modify:**

1. **`engine-sim/include/simulator.h`** - Add `synthesizerParams` to `Parameters` struct:
```cpp
struct Parameters {
    SystemType systemType;
    int simulationFrequency = 10000;
    int fluidSimulationSteps = 8;
    double targetSynthesizerLatency = 0.1;

    // NEW: Synthesizer audio configuration
    Synthesizer::Parameters synthesizerParams;  // Has sensible defaults
};
```

2. **`engine-sim/src/simulator.cpp`** - Remove `initializeSynthesizer()`, inline it:
```cpp
void Simulator::loadSimulation(Engine *engine, Vehicle *vehicle, Transmission *transmission) {
    m_engine = engine;
    m_vehicle = vehicle;
    m_transmission = transmission;
}

void Simulator::initializeSynthesizer() {
    Synthesizer::Parameters params = m_params.synthesizerParams;
    params.inputChannelCount = m_engine->getExhaustSystemCount();
    params.inputSampleRate = static_cast<float>(getSimulationFrequency());
    m_synthesizer.initialize(params);
}
```

3. **`engine-sim/src/piston_engine_simulator.cpp`** - Update initialization call:
```cpp
void PistonEngineSimulator::loadSimulation(...) {
    Simulator::loadSimulation(engine, vehicle, transmission);
    m_engine = engine;
    // ... existing setup ...

    initializeSynthesizer();  // Now uses m_params.synthesizerParams
}
```

4. **`engine-sim-bridge/src/engine_sim_bridge.cpp`** - Set params before `initialize()`:
```cpp
// In EngineSimCreate()
Simulator::Parameters simParams;
simParams.synthesizerParams.inputBufferSize = ctx->config.inputBufferSize;
simParams.synthesizerParams.audioBufferSize = ctx->config.audioBufferSize;
simParams.synthesizerParams.audioSampleRate = static_cast<float>(ctx->config.sampleRate);
// ... other audio params ...
// Note: inputChannelCount will be overridden by initializeSynthesizer()

ctx->simulator->initialize(simParams);
```

**Why this is best:**
- Single initialization (no leak)
- Config flows through proper layers (Simulator::Parameters)
- Bridge doesn't reach into Synthesizer internals
- Minimal changes to engine-sim (extend existing struct pattern)
- Backward compatible (synthesizerParams has defaults)

---

### Option A: Modify loadSimulation Signature (Alternative)

**Change:** `PistonEngineSimulator::loadSimulation()` accepts optional synth parameters

```cpp
// In engine-sim/src/piston_engine_simulator.cpp
void PistonEngineSimulator::loadSimulation(
    Engine* engine,
    Vehicle* vehicle,
    Transmission* transmission,
    const Synthesizer::Parameters* synthParams = nullptr)
{
    Simulator::loadSimulation(engine, vehicle, transmission);
    m_engine = engine;
    // ...

    if (synthParams) {
        m_synthesizer.initialize(*synthParams);
    } else {
        initializeSynthesizer(); // Use defaults
    }
}
```

**Pros:** Clean, no leak, proper layering
**Cons:** Requires modifying engine-sim submodule, affects all consumers

### Option B: Add Reinitialize Method to Synthesizer

**Change:** Add a method that reconfigures without leaking

```cpp
// In engine-sim/src/synthesizer.h
void Synthesizer::reinitialize(const Parameters& p) {
    // Clean up old allocations first
    destroy();

    // Re-initialize with new parameters
    initialize(p);
}
```

**Pros:** Smaller change, contained to Synthesizer class
**Cons:** Still requires engine-sim modification

### Option C: Store Config in Simulator (Deferred)

**Change:** Add synth params to `Simulator::Parameters`

```cpp
// In engine-sim/src/simulator.h
struct Parameters {
    SystemType systemType;
    int simulationFrequency = 10000;
    int fluidSimulationSteps = 8;
    Synthesizer::Parameters synthesizerParams; // NEW
};

// In PistonEngineSimulator::initializeSynthesizer()
void PistonEngineSimulator::initializeSynthesizer() {
    Synthesizer::Parameters params = m_simulatorParams.synthesizerParams;
    params.inputChannelCount = m_engine->getExhaustSystemCount();
    m_synthesizer.initialize(params);
}
```

**Pros:** Cleanest architecture, config flows naturally
**Cons:** Requires engine-sim changes, larger refactor

## Workaround (Current Implementation)

The bridge currently uses Option 2 pattern:
- Store user config in `EngineSimContext` during `Create`
- Apply it in `LoadScript` after engine is loaded
- Accept small memory leak as trade-off

This is acceptable because:
- Leak is bounded (occurs once per session on engine load)
- Leak size is small (~20KB for typical dual exhaust)
- Alternative is using hardcoded 44.1kHz/44100 buffer sizes regardless of user config

## Related Files

- `engine-sim-bridge/src/engine_sim_bridge.cpp` - Bridge implementation
- `engine-sim-bridge/engine-sim/src/piston_engine_simulator.cpp` - Calls `initializeSynthesizer()`
- `engine-sim-bridge/engine-sim/src/synthesizer.cpp` - Synthesizer initialization

## Related Issues

- Misleading `// Stereo` comment on `inputChannelCount = 2` (was actually exhaust systems, not L/R audio)
- Original bridge code also had leak (initialized in Create, re-initialized in loadSimulation)

## Next Steps (When Unparking)

1. **Verify leak impact:** Run with Valgrind/Instruments to quantify actual leak
2. **Choose solution:** If leak is problematic, implement Option A or C
3. **Coordinate with engine-sim:** If modifying engine-sim, ensure upstream alignment
4. **Add tests:** Verify synthesizer configuration is correctly applied

## References

- Original investigation discussion about stereo production and channel count
- Architecture discussion about SOLID principles and initialization order
- BRIDGE_INTEGRATION_ARCHITECTURE.md for broader context
