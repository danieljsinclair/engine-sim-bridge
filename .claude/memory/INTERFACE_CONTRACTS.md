# VirtualICE Twin — Interface Contracts

**Version:** 1.0
**Date:** 2026-04-24
**Scope:** Preconditions, postconditions, error handling, and thread-safety guarantees for all module boundaries.

---

## Introduction

Each interface boundary in the VirtualICE Twin architecture has a **contract**: an explicit statement of what the caller must guarantee and what the callee promises. These contracts enable:

- **Substitutability (LSP)**: Any implementation fulfilling the contract can replace any other.
- **Testability**: Mocks can be built that faithfully reproduce production behavior.
- **Reasoning**: Callers know what to expect without reading implementation code.

Contracts follow the **Design by Contract** pattern but simplified for C++:
- **Preconditions**: Caller responsibilities (enforced via `ASSERT()` or `EXPECT()` in debug builds)
- **Postconditions**: Callee guarantees (valid for caller to rely on after return)
- **Invariants**: Conditions that hold throughout the object lifetime

---

## 1. `ISimulator` → BridgeSimulator Boundary

**Location:** `include/simulator/ISimulator.h`

### 1.1 `bool create(const ISimulatorConfig& config, ILogging* logger, telemetry::ITelemetryWriter* telemetryWriter)`

**Preconditions:**
- `config.sampleRate > 0` (valid Hz, must match audio hardware capability)
- `config.simulationFrequency > 0` (physics step rate Hz)
- `logger` may be `nullptr` — implementation must use `ConsoleLogger` default
- `telemetryWriter` may be `nullptr` — implementation must use `NullTelemetryWriter` default

**Postconditions:**
- Returns `true` on success, `false` on failure with descriptive `getLastError()`
- On success: `simulator->getStats()` returns valid `EngineSimStats` (non-NaN, RPM ≥ 0)
- Audio buffer allocated internally; `renderOnDemand()` can be called

**Error handling:**
- Returns `false` (does not throw) on configuration errors, missing scripts, hardware init failure
- `getLastError()` returns human-readable `std::string` — test may assert substring presence
- **No exceptions thrown**: All errors reported via return code + error string

**Thread-safety:**
- `create()` **must be called from a single thread only** (not concurrent with any other method)
- After successful return, `update()`, `renderOnDemand()`, `getStats()` are **thread-safe for concurrent read** (const methods use atomics where needed)

---

### 1.2 `void update(double deltaTime)`

**Preconditions:**
- `deltaTime > 0` and `deltaTime ≤ 0.1` seconds (100 ms max step to avoid instability)
- `create()` has returned `true` (simulator initialized)
- `start()` has been called if audio mode requires running state

**Postconditions:**
- Internal physics state advanced by `deltaTime` seconds
- `getStats().currentRPM` reflects post-update engine speed (≥ 0)
- If `telemetryWriter_` was provided in `create()`, telemetry callbacks fire (non-blocking)

**Error handling:**
- If `deltaTime` is out-of-range, implementation may assert in debug or clamp silently in release (defined behavior)
- Does not throw exceptions — keeps real-time thread safe

**Thread-safety:**
- **Not thread-safe**: Must be called from simulation thread only
- Caller must serialize calls (no concurrent `update()` invocations)

---

### 1.3 `bool renderOnDemand(float* buffer, int32_t frames, int32_t* written)`

**Preconditions:**
- `buffer` points to allocated memory of at least `frames * channels * sizeof(float)` bytes
- `frames > 0` and `frames ≤ MAX_AUDIO_CHUNK_FRAMES` (`EngineSimDefaults::MAX_AUDIO_CHUNK_FRAMES`)
- `written` points to valid `int32_t` storage
- `create()` returned `true`
- Simulator audio synthesizer buffer has sufficient pre-filled samples (call `update()` at least once before first render)

**Postconditions:**
- Returns `true` on success, `false` on underrun/error
- On success: `*written` equals `frames` (full buffer filled) or less if partial data available
- On return, `buffer[0..frames*2-1]` contain interleaved stereo float32 samples (L, R, L, R, …)
- Samples normalized to `[-1.0, 1.0]`, silence = all zeros

**Error handling:**
- Returns `false` if synthesizer buffer empty (need more `update()` calls)
- Returns `false` on allocation failure internally
- **Never throws**

**Thread-safety:**
- **Thread-safe for concurrent calls** if each call uses independent `buffer` (no internal state mutation)
- Internal conversion buffer is thread-local or protected

---

### 1.4 `EngineSimStats getStats() const`

**Preconditions:** None (const method)

**Postconditions:**
- Returns a **copy** of internal stats snapshot
- All fields non-NaN:
  - `currentRPM ≥ 0`
  - `currentLoad ∈ [0.0, 1.0]`
  - `exhaustFlow ≥ 0`
  - `manifoldPressure ≥ 0`
  - `processingTimeMs ≥ 0`

**Thread-safety:**
- **Thread-safe**: Uses atomics or mutex-protected reads; safe to call from any thread at any time

---

## 2. `IInputProvider` → Simulation Loop Boundary

**Location:** `include/io/IInputProvider.h`

### 2.1 `EngineInput OnUpdateSimulation(double dt)`

**Preconditions:**
- `dt > 0` seconds (typically 0.016s = 1/60s, 0.0001s = 1/10000s)
- `Initialize()` has returned `true` and `IsConnected()` is `true`

**Postconditions:**
- Returns `EngineInput` struct with fields:
  - `throttle ∈ [0.0, 1.0]` (clamped by implementation if needed)
  - `ignition` is `true` or `false`
  - `starterMotorEngaged` is `true` or `false`
  - `shouldContinue` is `false` **only** when the input source has ended (EOF, user quit)
- If `shouldContinue == false`, the simulation loop **must terminate**

**Error handling:**
- Does **not throw**
- `GetLastError()` returns non-empty string **only if** `IsConnected()` is `false`
- On disconnect, implementations must either:
  - Return last-known input values with `shouldContinue = false`, or
  - Return `throttle=0, ignition=false, shouldContinue=false` and set error

**Thread-safety:**
- Called from **simulation thread only** (not audio thread)
- Implementations **must be lock-free or use caller-serialized access** (no internal mutex requirement)

---

## 3. `IAudioHardwareProvider` → Audio Strategy Boundary

**Location:** `include/hardware/IAudioHardwareProvider.h`

### 3.1 `bool initialize(const AudioStreamFormat& format)`

**Preconditions:**
- `format.sampleRate > 0` (valid Hz; must be supported by platform: 44100, 48000)
- `format.channels ≥ 1` and `≤ 8`
- `format.isFloat` must be `true` for current implementations (int16 not supported in this interface)
- `format.isInterleaved` must be `true` (non-interleaved not supported yet)

**Postconditions:**
- Returns `true` if platform-specific audio resources allocated
- On return, `getHardwareState().isInitialized == true`
- No audio callback has been invoked yet (startPlayback needed)

**Error handling:**
- Returns `false` if platform audio unavailable (device busy, unsupported format)
- Does **not throw** — audio initialization failure is recoverable or fatal at runtime, not exceptional

**Thread-safety:**
- Must be called **before** any other method, single-threaded initialization phase

---

### 3.2 `bool registerAudioCallback(const AudioCallback& callback)`

**Preconditions:**
- `initialize()` returned `true`
- `callback` is a callable object (not null)
- `startPlayback()` has **not** been called yet

**Postconditions:**
- Returns `true` if callback registered
- The callback **will be invoked** on the platform's real-time audio thread after `startPlayback()`
- Multiple calls replace previous callback (only one active at a time)

**Error handling:**
- Returns `false` if callback cannot be registered (platform limitation)

**Thread-safety:**
- May be called from any thread (typically main thread before audio starts)
- Must synchronize internally against concurrent `startPlayback()`

---

### 3.3 `bool startPlayback()`

**Preconditions:**
- `initialize()` returned `true`
- A callback has been registered via `registerAudioCallback()`

**Postconditions:**
- Returns `true` if audio streaming begins
- On return, `getHardwareState().isPlaying == true` and `isCallbackActive == true`
- The registered callback begins invocation on the real-time audio thread at the configured sample rate

**Error handling:**
- Returns `false` if platform fails to start (device unavailable, permission denied)
- **Does not throw**

**Thread-safety:**
- Called from main/control thread
- Must coordinate with callback thread for shutdown races (see `stopPlayback()`)

---

### 3.4 `int callback(AudioBufferView& buffer)` (platform override)

This is the method the platform subclass implements — invoked by platform audio system.

**Preconditions:**
- Called on **real-time audio thread** at high priority
- `buffer` descriptor valid; `buffer.asFloat()` returns writable memory
- `buffer.frameCount` equals platform's callback size (typically 128–512 frames)

**Postconditions:**
- Returns 0 on success (non-zero = error to platform, may stop audio)
- On return, `buffer` filled with `buffer.frameCount × buffer.channelCount` float samples
- Samples must be within `[-1.0, 1.0]` (clipping causes distortion but not UB)

**Error handling:**
- Return 0 for all paths — **never throw** from audio callback
- If internal error occurs, fill buffer with silence and return 0

**Thread-safety:**
- **Real-time thread**: Must not block, allocate heap, or call non-RT-safe APIs
- `getHardwareState()` called from callback must be lock-free or atomics-only

---

### 3.5 `void stopPlayback()`

**Preconditions:**
- `startPlayback()` returned `true` (currently playing)

**Postconditions:**
- Audio stops; `getHardwareState().isPlaying == false`
- Callback **no longer invoked** after return
- Platform audio resources remain allocated (for potential restart via `startPlayback()`)

**Error handling:**
- May be called even if already stopped — must be idempotent

**Thread-safety:**
- **Critical race**: Can be called **from the audio callback thread itself** (consumer signals stop)
- Implementation **must not deadlock** if called from within `callback()`
- Use atomic flags without mutexes; if mutex needed, use `std::mutex` with `std::try_lock` and fallback

---

### 3.6 `void cleanup()`

**Preconditions:**
- `stopPlayback()` has been called (or playback never started)

**Postconditions:**
- All platform audio resources released
- `getHardwareState().isInitialized == false`
- Object is ready for destruction or re-initialization

**Thread-safety:**
- Called from control thread only
- Must ensure callback thread has fully terminated before resource free

---

## 4. `ITelemetryWriter` / `ITelemetryReader` Boundary

**Location:** `include/telemetry/ITelemetryProvider.h`

### 4.1 `void writeEngineState(const EngineStateTelemetry& state)`

**Preconditions:**
- `state` fields valid (RPM ≥ 0, load ∈ [0, 1])
- Called from any thread (typically simulation thread)

**Postconditions:**
- Latest engine state visible via `ITelemetryReader::getEngineState()`
- Readers see either **old value or new value** — never partial/internally inconsistent struct

**Error handling:**
- Does **not throw** (telemetry is fire-and-forget, dropped if backing store full)

**Thread-safety:**
- **Lock-free**: Each field stored in `std::atomic<T>` (double → `std::atomic<double>` on supported platforms, else `std::mutex`)
- Multiple writers: Last-writer-wins (caller must serialize if ordering matters)

---

### 4.2 Component Write Methods (all six)

All six `write*()` methods follow same contract:
- Precondition: caller owns the struct; data valid for domain
- Postcondition: data atomically visible to `get*()` readers
- Never throws, may drop if telemetry pipeline backed up
- Thread-safe via atomic per-field storage

---

## 5. `IAudioBuffer` (Strategy) → BridgeSimulator Boundary

**Location:** `include/strategy/IAudioBuffer.h`

### 5.1 `bool render(AudioBufferView& buffer)`

**Preconditions:**
- `startPlayback()` has been called on this strategy
- `buffer` points to writable memory of `frameCount × channelCount` floats
- Called from **real-time audio callback thread**

**Postconditions:**
- Returns `true` on successful audio generation
- On success: `buffer` filled with `frameCount` frames of interleaved stereo samples
- Audio is **time-coherent** with simulator state at the moment of call

**Error handling:**
- Returns `false` on underrun (not enough audio data ready)
- No exceptions

**Thread-safety:**
- Called continuously from audio thread
- Must not allocate, lock mutexes, or call non-RT-safe code
- Internal state must use lock-free structures or pre-allocated pools

---

### 5.2 `bool AddFrames(float* buffer, int frameCount)`

**Preconditions:**
- `buffer` points to `frameCount × channels` valid float samples
- Called from **simulation thread** (not audio thread)

**Postconditions:**
- Returns `true` if samples enqueued into internal buffer
- On return, strategy's internal buffer size increased by `frameCount`

**Error handling:**
- Returns `false` if buffer full (samples dropped)

**Thread-safety:**
- Called from simulation thread concurrently with `render()` on audio thread
- Must use **lock-free ring buffer** or mutex with minimal contention
- Each frame either fully enqueued or dropped; never partially enqueued

---

## 6. `ITwinStrategy::update()` → Strategy Contract

**Location:** `INTERFACE_DESIGN.md` (interfaces committed to implementation)

### 6.1 `TwinOutput update(const UpstreamSignal& signal, double currentRpm, double dt)`

**Preconditions:**
- `signal` fields valid:
  - `vehicleSpeedKmh ≥ 0`
  - `throttlePercent ∈ [0.0, 100.0]`
  - `gear ∈ [-2, 6]` (reverse to 6th)
- `currentRpm ≥ 0` (may be stale by ≤ `dt`)
- `dt > 0` and `dt ≤ 0.1` seconds

**Postconditions:**
- Returns `TwinOutput` within reasonable bounds:
  - `recommendedThrottle ∈ [0.0, 1.0]`
  - `recommendedGear ∈ [-2, 6]` (or current gear if no change)
  - `clutchEngagement ∈ [0.0, 1.0]`
  - `shiftRequested` is `true` only if a gear change should occur
  - `derivedRpm ≥ 0` (model's predicted next-tick RPM)
  - `estimatedWheelTorque` can be any double (including negative for engine braking)

**Error handling:**
- **Must not throw** — guaranteed lock-free and real-time safe
- On invalid `signal` (NaN, out-of-range), strategy may:
  - Clamp silently
  - Return previous output
  - Set `shiftRequested = false` as safe default
  - **Never crash or assert**

**Thread-safety:**
- Called from **twin update thread** (not audio callback, not main UI)
- Strategy must be internal lock-free; no heap allocation, no blocking I/O
- `update()` may be called concurrently with `resetToInitialState()` (rare path) — strategy must handle via atomic flag

---

## 7. `ITelemetrySource::read()` → Telemetry Source Contract

**Location:** `INTERFACE_DESIGN.md`

### 7.1 `bool read(UpstreamSignal& signal)`

**Preconditions:**
- `Initialize()` has returned `true`
- Source connected (`IsConnected() == true`)

**Postconditions:**
- Returns `true` if a fresh sample was read and `signal` filled
- Returns `false` if no data available (non-blocking) or unrecoverable error (device unplugged)
- On `false`, `getLastError().code` indicates reason (`DeviceDisconnected`, `Timeout`, …)

**Error handling:**
- For transient errors (timeout, checksum fail), may return `false` without setting fatal error
- For fatal errors (device removed), set `code = DeviceDisconnected` and return `false`

**Thread-safety:**
- May be called from simulation thread or twin thread
- Not necessarily lock-free — may use internal mutex (blocking ≤ 10ms acceptable)

---

## 8. `IEngineController` → Engine Control Contract

**Location:** `INTERFACE_DESIGN.md` (proposed)

### 8.1 `void setThrottle(double position)`

**Preconditions:**
- `position ∈ [0.0, 1.0]`
- `initialize()` completed

**Postconditions:**
- Simulator's throttle set to `position` (clamped by simulator if needed)
- Next `simulator->update(dt)` uses this throttle

**Error handling:**
- Does not throw; clamps to `[0,1]` if out of range

**Thread-safety:**
- Called from twin update thread only
- Lock-free; updates atomic throttle state

---

### 8.2 `void setGear(int gear)`

**Preconditions:**
- `gear ∈ [-2, 6]`
- In `DynoAssisted` mode: clutch automatically disengaged during shift (controller handles)
- In `PhysicsDriven` mode: clutch pressure already dropped by strategy

**Postconditions:**
- Simulator transmission gear changes; engine RPM may jump per energy conservation
- `getCurrentGear()` returns `gear` after change completes

**Error handling:**
- Silently ignored if already in target gear
- Not an error if gear change fails (returns `false` status — use `getLastError()`)

**Thread-safety:**
- Called from twin thread; may internally lock transmission (brief critical section within `Transmission::changeGear`)

---

### 8.3 `double getEngineRpm() const`

**Preconditions:** None

**Postconditions:**
- Returns `≥ 0` RPM (clamped to `0` during pre-init)
- Value is **subject to 1–2 tick latency** (last `simulator->update()` call)

**Thread-safety:**
- **Thread-safe**: Reads atomically-protected stats

---

## 9. Cross-Cutting Contract: All Exceptions

**Rule: Only throw from non-real-time threads.**

| Interface | Throws? | Exception Types | Caller Must Catch? |
|---|---|---|---|
| `ISimulator::create()` | **Yes** — configuration load failure | `std::runtime_error`, `std::filesystem::filesystem_error` | Yes |
| `TwinFactory::create()` | **Yes** — dependency injection failure | `std::runtime_error` | Yes |
| All `IInputProvider` methods | No | N/A | Never — use `GetLastError()` |
| All `IAudioHardwareProvider` methods | No | N/A | Never — use return codes |
| All `IAudioBuffer::render()` | No | N/A | Never — return `false` |
| `ITwinStrategy::update()` | No | N/A | Never — return safe defaults |

**Exception safety levels:**
- `create()` paths: **Strong guarantee** — on throw, object state unchanged (not created)
- `destroy()` paths: **No-throw guarantee** — destructors never throw

---

## 10. Thread-Safety Matrix

| Interface | Method | Thread Context | Blocking Allowed? | Notes |
|---|---|---|---|---|
| `ISimulator` | `create()` | Main thread | Yes (file loading) | Serialize with other calls |
| `ISimulator` | `update()` | Sim thread | No | RT safe |
| `ISimulator` | `renderOnDemand()` | Audio RT thread | No | No allocations |
| `ISimulator` | `getStats()` | Any | No (lock-free) | Atomic snapshot |
| `IInputProvider` | `OnUpdateSimulation()` | Sim thread | No (RT safe) | Max 1ms |
| `ITelemetryWriter` | `write*()` | Any | No | Atomic per-field |
| `ITelemetryReader` | `get*()` | Any | No | Lock-free read |
| `IAudioHardware` | `callback()` | Audio RT | No | Priority high |
| `IAudioHardware` | `start/stopPlayback()` | Main thread | Yes | Must not race callback |
| `ITwinStrategy` | `update()` | Twin thread | No | Alloc-free, <50 µs |

**Guideline**: If a method is called from the audio callback, it costs us 1–2 ms of the 11 ms audio period. Err on the side of non-blocking.

---

## 11. Lifetime & Ownership Contracts

| Object | Created By | Owned By | Destroyed By | Notes |
|---|---|---|---|---|
| `BridgeSimulator` | `SimulatorFactory` or `main()` | `SimulationLoop` or caller | `destroy()` then `delete`/`unique_ptr` | Not owned by twin |
| `ISimulator` (subclass) | `SimulatorFactory` | `BridgeSimulator` (composed) | `BridgeSimulator::destroy()` | Pre-composed |
| `ITwinModel` | `TwinFactory` | `BridgeSimulator` or `SimulationLoop` | `delete`/`unique_ptr` | Twin module owns its parts |
| `ITelemetrySource` | `TwinFactory` | `ITwinModel` | `ITwinModel` destructor | Borrowed or owned based on config |
| `IEngineController` | `TwinFactory::createEngineController` | `ITwinModel` | `ITwinModel` destructor | Owns embedded `ISimulator` |
| `ITwinStrategy` | `TwinFactory` or `activeStrategy->clone()` | `ITwinModel` | `ITwinModel` destructor or `switchStrategy()` | Clone on swap preserves state |

**Rule**: If `TwinFactory::create()` returns a `unique_ptr<T>`, the factory **transfers ownership** to the caller. The caller becomes responsible for destruction.

---

## 12. Invariants

### 12.1 `InMemoryTelemetry` Invariants

For all `t ≥ 0`:
- `getEngineState().currentRPM ≥ 0`
- `getVehicleInputs().throttlePosition ∈ [0, 1]`
- `getAudioTiming().callbackRateHz > 0` after playback starts
- Values monotonically increase within valid bounds (no sudden jumps unless warranted by input)

### 12.2 `SyncPullStrategy` Invariants

- While `isPlaying() == true`: `audioState_.bufferFillFrames ≥ PRE_ROLL_FRAMES`
- While `isPlaying() == true`: `simulator_ != nullptr` (attached to live simulator)
- After `stopPlayback()`: `isPlaying() == false` and `simulator_ == nullptr`

### 12.3 `DynoAssistedStrategy` Invariants

- `currentStrategyOutput.clutchEngagement == 0.0` during normal dyno operation
- `targetRpm` never exceeds `EngineSimDefaults::REDLINE_RPM` (6500)
- `currentGear` ∈ defined vehicle profile gear set (after profile load)

---

## 13. Debug-Accesible State

For test inspection, production interfaces expose **read-only accessors**:

```cpp
// ISimulator (already present)
EngineSimStats getStats() const;

// ITwinModel (added for debugging)
double getCurrentRpm() const;
int getCurrentGear() const;
double getVehicleSpeedKmh() const;

// SimpleGearbox (internal debug)
int getCurrentGear() const;
double getCurrentRpm() const;
int getShiftPhase() const;  // 0=stable, 1=upshift, 2=downshift
```

These are **never part of the production control flow** — twins control via `IEngineController::set*()` methods only. Read accessors exist for tests and diagnostics.

---

## 14. Contract Violations & What to Do

### 14.1 Precondition Violation

**Caller broke the contract** (e.g., called `OnUpdateSimulation()` before `Initialize()`).

**Response**: Implementation may `ASSERT(false)` in debug builds. In release builds, behavior is undefined (code may crash or silently misbehave). Tests must ensure preconditions are met — contract violation means test bug, not implementation bug.

**Example**: Unit test forgets to initialize `MockTelemetryProvider` before calling. Fix the test, not the mock.

### 14.2 Postcondition Violation

**Implementation broke its guarantee** (e.g., `renderOnDemand()` returns `true` but leaves buffer uninitialized).

**Response**: This is a **production bug**. Test should `EXPECT_TRUE(written > 0)` and validate buffer content.

### 14.3 Invariant Violation

A class invariant broken (e.g., `InMemoryTelemetry` allows negative RPM).

**Response**: Bug in the code; test catching it is correct.

---

## 15. Summary Checklist for Implementers

Before marking an interface implementation "done":

- [ ] All preconditions clearly documented in header comments (with `@param` and `@pre` tags where applicable)
- [ ] All postconditions documented (`@post`, `@return`)
- [ ] Return codes documented for error paths (what `false` means for each method)
- [ ] Thread-safety annotated: `// Thread-safe` / `// Simulation thread only` / `// Audio RT thread`
- [ ] No exceptions thrown from any RT-path method
- [ ] All invariants documented in class-level comment
- [ ] Mock implementation exists that honors same contract (for tests)

---

## 16. Future Extensions

When adding a new interface or method:

1. **Write the contract first** — before implementation
2. **Write RED-phase tests** that assert postconditions
3. **Then write GREEN implementation**
4. **Run full suite** — all existing tests must pass

**Never add a method to an interface without updating this contracts document** if:
- It changes thread-safety guarantees
- It introduces allocation on the real-time path
- It adds a new error condition callers must handle

---

*Contracts are the formal specification. Code is the concrete realization. When they disagree, the contract takes precedence (fix implementation).*
