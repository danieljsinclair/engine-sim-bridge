# VirtualICE Twin — Test Strategy & Quality Framework

**Version:** 1.0
**Date:** 2026-04-24
**Scope:** VirtualICE Twin module + engine-sim-bridge integration
**Author:** Test Architect (Claude Code)

---

## Executive Summary

This document defines the test strategy for the VirtualICE Twin project. The strategy advances from the existing `TEST_HARNESS_DESIGN.md` deterministic twin validation approach to cover **four distinct test layers**:

| Layer | Scope | Tool | Failures Indicate |
|---|---|---|---|
| **Unit** | Single class/function, no external deps | GoogleTest | Logic errors, contract violations |
| **Integration** | 2+ modules, mock collaborators | GoogleTest + mocks | Interface boundary breaks, DI wiring failures |
| **E2E Smoke** | Full pipeline: telemetry → twin → engine → audio | TwinTestRunner | Regression in end-to-end signal flow |
| **Performance** | Real-time thread safety, latency budgets | Custom timing assertions | Audio glitches, dropped frames, priority inversion |

**Core principle**: Tests exist to enable **safe, confident refactoring**. They must fail when behavior changes, but not when implementation details change. We assert **what the code does**, not **how it does it**.

---

## 1. Unit Test Philosophy

### 1.1 TDD Red→Green→Refactor Cycle

Every new feature begins with a **RED** (failing) test:

```cpp
// RED PHASE — test fails, code doesn't exist yet
TEST_F(DynoAssistedStrategyTest, ComputesTargetRpmFromSpeedKmh) {
    DynoAssistedStrategy strategy;
    strategy.setGearRatios({4.1, 2.8, 1.9, 1.4, 1.0, 0.8}); // example 6-speed
    strategy.setFinalDrive(3.73);
    strategy.setTireRadius(0.33); // meters

    UpstreamSignal signal{ .vehicleSpeedKmh = 80.0 };
    double dt = 0.016; // 60Hz tick

    TwinOutput output = strategy.update(signal, 1800.0, dt);

    // Expected: RPM = (speed_mps / (2πr)) × gearRatio × finalDrive × 60
    // 80 km/h = 22.22 m/s; wheel_rpm = 22.22/(2π×0.33) × 60 ≈ 643 RPM
    // gear=5 (ratio=1.0), final=3.73 → engine_rpm ≈ 643 × 1.0 × 3.73 ≈ 2400 RPM
    EXPECT_NEAR(output.derivedRpm, 2400.0, 50.0);
}
```

**RED phase rules:**
- Code stub must compile to satisfy the linker (`g++` rule: all referenced symbols must exist)
- Use **forward declarations** and **minimal implementations** to get RED tests to compile
- The RED phase proves the test actually exercises new code

**GREEN phase**: Write minimal implementation to make test pass. No design yet — just enough to satisfy the assertion.

**REFACTOR phase**: Apply SRP/KISS/DRY. Run full suite continuously — green must remain green.

### 1.2 Assert Behavior, Not Implementation

**BAD** — implementation detail assertion (fragile):
```cpp
// Enforces specific internal structure; any refactor breaks test
EXPECT_EQ(strategy->m_gearBox->m_gearRatios.size(), 6);
EXPECT_STREQ(strategy->m_gearbox->getName(), "SimpleGearbox");
```

**GOOD** — observable behavior assertion (robust):
```cpp
// Tests what strategy actually does: shifts at expected RPM
StrategyShiftsAtRpm(strategy, 4000.0, gear1, gear2);
EXPECT_EQ(strategy.getCurrentGear(), gear2);
```

### 1.3 Determinism Is Non-Negotiable

Every unit test must be **fully deterministic**:

| Anti-pattern | Why It's Bad | Fixed By |
|---|---|---|
| `std::rand()` | Non-repeatable, CI flakes | Use constant/fixed input data |
| `std::chrono::steady_clock::now()` | Wall-clock dependent | Inject clock or use simulation time |
| `std::this_thread::sleep_for()` | Slows suite, timing-dependent | Use condition variables or mocks |
| Reading from filesystem | IO flakiness | Embed test fixtures as `constexpr` or inline strings |
| Network calls | External dependency failure | Mock `ITelemetrySource` |

**Real-world example — the buffer-aliasing regression tests** (`SineWaveRegressionTests.cpp`):
- Uses mathematical sine properties (smoothness, adjacent-sample deltas)
- Zero randomness
- Self-contained, no assets
- Compiles and fails on buggy code, passes on correct code

---

## 2. Integration Test Philosophy

### 2.1 Test Module Boundaries

Unit tests assure individual classes work. Integration tests assure **two modules wired together** exchange data correctly at the interface boundary.

**Targeted integration points:**

| Boundary | Contract Test | Mock Type | Example Assertion |
|---|---|---|---|
| `IInputProvider` → `BridgeSimulator` | Input values propagate to simulator throttle | Fake `IInputProvider` | `sim->getStats().currentRPM` reflects input throttle |
| `ISimulator` → `IAudioBuffer` (ThreadedStrategy) | Rendered audio captures physics output | Fake `IAudioBuffer` | AudioRMS matches expected engine power at given throttle |
| `ITwinModel` → `IEngineController` | Strategy outputs correctly set simulator state | Spy `IEngineController` | `setThrottle()` called with strategy-computed value |
| `ITelemetrySource` → `ITwinStrategy` | UpstreamSignal fields correctly ingested | `MockTelemetrySource` | Strategy internal state matches signal speed/throttle |

### 2.2 Mock Taxonomy

| Mock Kind | Purpose | Example |
|---|---|---|
| **Stub** | Return canned values, no verification | `MockTelemetrySource` returns `vehicleSpeedKmh = 100.0` always |
| **Fake** | Working but simplified implementation | `InMemoryTelemetry` vs real UDP telemetry |
| **Spy** | Record calls for later verification | `SpyEngineController` captures all `setThrottle()` invocations |
| **Dummy** | Satisfy constructor, never used | `nullptr` for optional `IPresentation*` in tests |

**Mock design rule**: Mocks implement the **production interface** (`IInputProvider`, `ITelemetrySource`), never private methods. This ensures tests exercise the same call sites as production.

### 2.3 Integration Test Structure

Integration tests follow **Arrange-Act-Assert** with injectable collaborators:

```cpp
class TwinIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        logger = std::make_unique<ConsoleLogger>();

        // Construct real twin, mock outside world
        TwinFactory factory;
        TwinConfig config;
        config.mode = TwinMode::DynoAssisted;
        // ... configure mock telemetry source type
        twin = factory.create(config);

        // Spy on engine controller to observe effects
        spyController = std::make_unique<SpyEngineController>();
        twin->initialize(mockTelemetrySource.get(), spyController.get(), strategy.get());
    }

    std::unique_ptr<ITwinModel> twin;
    std::unique_ptr<MockTelemetrySource> mockTelemetrySource;
    std::unique_ptr<SpyEngineController> spyController;
    std::unique_ptr<ILogging> logger;
};
```

---

## 3. E2E Smoke Tests

### 3.1 Purpose

E2E tests validate the **full signal path** from telemetry input to audio output. They are slower and more brittle than unit tests, so we keep them minimal. They catch:

- **Configuration drift**: build-system files out of sync (CMake, headers)
- **Runtime coupling bugs**: `BridgeSimulator` not calling `ITwinModel::update()`
- **Audio pipeline breaks**: format conversion corruption, buffer size mismatches
- **Threading deadlocks**: real-time audio callback + simulation thread coordination

### 3.2 Minimal Smoke Test Matrix

| Scenario | Duration | Key Assertion | Pass Threshold |
|---|---|---|---|
| Full-throttle 0→100 km/h | 12 s | 6 virtual shifts detected | ≥5 shifts, shift error < ±0.3 s |
| Steady cruise 80 km/h | 10 s | RPM variance < 3% | σ_RPM < 60 @ 1800 RPM |
| Deceleration 100→0 km/h | 15 s | RPM monotonic decrease | All derivatives ≤ 0 |
| Standstill launch | 5 s | Starter disengages at target RPM | 3400 ± 5% after 1 s |
| Immediate throttle kickdown | 3 s | Downshift triggered < 200 ms | Δgear ≥ -2 within 0.2 s |

Each smoke test runs through `TwinTestRunner` (see `TEST_HARNESS_DESIGN.md`) and produces two artifacts:

1. **Validation report** (JSON): `rpm.l2Error`, `shift.timingErrors[]`, `audio.cosineSimilarity`
2. **Audio fingerprint** (JSON): spectral centroid sweep, harmonic energy profile

### 3.3 Golden File Regression

Golden files are committed baseline fingerprints:

```
test/golden/
├── full_throttle_golden.json    # Baseline audio + telemetry fingerprint
├── steady_cruise_golden.json
├── deceleration_golden.json
└── standstill_launch_golden.json
```

**Regression thresholds** (configurable via `TWIN_TEST_TOLERANCE_MULTIPLIER` env var):

```json
{
  "rpm_tracking": { "l2_error_rpm_threshold": 200.0 },
  "shift_timing":  { "max_abs_error_threshold_s": 0.35 },
  "audio": {
    "cosine_similarity_min": 0.90,
    "harmonic_correlation_min": 0.85
  }
}
```

**When a golden file breaks:**
1. Run the failing test locally with `--artifacts-dir=./tmp` to inspect JSON diffs
2. If the change is **expected** (new gearshift behavior, different RPM curve), update the golden file and commit with explanation
3. If the change is **unexpected** (audio corruption, excessive shift lag), fix the bug

Golden files are **code artifacts**, not hand-tuned data. They result from a known-good build and are updated deliberately.

---

## 4. Test Data Management

### 4.1 Deterministic Scenarios

All test scenarios use **mathematical functions**, never random data:

| Scenario | Input Function | Expected Behavior |
|---|---|---|
| `FullThrottleAcceleration` | `throttle(t) = 1/(1+exp(-(t-4)/1.5))` | Smooth sigmoid ramp from 0→1 |
| `SteadyCruise` | `throttle(t) = constant 0.192` | RPM = 1800 ± 3% |
| `DecelerationBraking` | `throttle(t)=0`, RPM exponential decay `6500·exp(-t/8)` | Monotonic decrease |
| `StandstillLaunch` | Step at t=0.5s: starter→throttle=0.5 | Hold 800 RPM then rise to 3400 |

Each scenario supplies its own `expectedRPMAt(t)` function (used by test assertions). No `srand()` or `rand()` in test code paths.

### 4.2 Synthetic Telemetry

`MockTelemetrySource` generates deterministic `UpstreamSignal` streams. Parameters:

```cpp
struct ScenarioConfig {
    double simulationDt = 1.0 / 10000.0;  // 10 kHz physics step
    double realtimeFactor = 0.0;          // as-fast-as-possible
    double duration = 12.0;               // seconds
};
```

Phase 1: Mathematical generators. Phase 2+: CSV-replay from recorded drives (deterministic due to fixed file).

### 4.3 Numerical Constants as Single Source

All test thresholds reference constants defined in one place:

```cpp
// test/constants/EngineTestConstants.h
namespace TwinTestConstants {
    constexpr double IDLE_RPM = 800.0;
    constexpr double REDLINE_RPM = 6500.0;
    constexpr double THROTTLE_TO_RPM_SLOPE = 5200.0; // (RPM_max - RPM_idle)
    constexpr double GEAR_SHIFT_HYSTERESIS_RPM = 150.0;
    constexpr double MAX_SHIFT_TIMING_ERROR_S = 0.35;
    constexpr double RPM_TRACKING_L2_TOLERANCE = 200.0;
    // ... more
}
```

If parameters change (e.g., idle RPM), only this file updates.

---

## 5. Performance & Real-Time Tests

### 5.1 Timing Budgets

| Operation | Max Duration | Thread | Rationale |
|---|---|---|---|
| `ITwinStrategy::update()` | 50 µs | Twin thread | Must not block audio |
| `BridgeSimulator::update(dt)` | 500 µs | Sim thread | 10 kHz step = 100 µs budget → 500 µs headroom |
| `IAudioBuffer::render()` | 2 ms | Audio RT | Frames @ 44.1kHz = 11.3 ms per 512-frame buffer |
| `MockTelemetrySource::read()` | 10 ms | Sim thread | Avoid starving simulation |

**Enforcement**: Use `std::chrono::steady_clock` inside test wrappers:

```cpp
auto start = std::chrono::steady_clock::now();
strategy->update(signal, rpm, 0.016);
auto elapsed = std::chrono::duration<double, std::micro>(...);
EXPECT_LE(elapsed.count(), 50.0) << "Strategy update took " << elapsed.count() << " µs";
```

### 5.2 Thread-Safety Tests

Verify lock-free access under contention:

```cpp
TEST_F(ConcurrencyTest, NoDataRaceOnTwinUpdate) {
    ITwinModel twin = createDynoTwin();
    twin->initialize(...);

    std::atomic<bool> done{false};
    std::thread t1([&]{
        while (!done) twin->update(0.016);
    });
    std::thread t2([&]{
        while (!done) twin->update(0.016);
    });
    std::this_thread::sleep_for(100ms);
    done = true;
    t1.join(); t2.join(); // Must not crash or assert
}
```

Run under ThreadSanitizer (`-fsanitize=thread`). Any data race → test failure.

---

## 6. Error Handling vs. Correct Behavior

### 6.1 Fail-Fast Principle

Production code should **throw exceptions or assert preconditions** on invalid input. Tests should:

1. **Assert correct behavior under valid inputs** — primary focus
2. **Assert expected failure modes only** for documented recoverable cases

**Example** — `IGearbox::selectGear()`:

```cpp
// Valid input tests
TEST_F(GearboxTest, AcceptsForwardGears) {
    EXPECT_NO_THROW(gearbox.selectGear(3));
    EXPECT_EQ(gearbox.getCurrentGear(), 3);
}

// Do NOT test every invalid value unless the API promises specific handling
TEST_F(GearboxTest, RejectsInvalidGear) {
    // Only test this IF the interface documents specific exception type
    // If it's "undefined behavior", don't test it
    EXPECT_THROW(gearbox.selectGear(999), std::out_of_range);
}
```

### 6.2 What NOT to Test

| Item | Reason |
|---|---|
| Trivial getters/setters (`getVolume()` returns `volume_`) | No business logic — covered indirectly via behavior tests |
| Third-party library wrappers (`FAudioEngine::start()`) | Test our code, not library correctness |
| `constexpr` math functions (`convertInt16ToStereoFloat`) | Deterministic; integration tests exercise indirectly |
| Compiler-generated default constructors | Implicitly covered by compilation |

### 6.3 Test the Contract, Not the Message

**BAD** — brittle message match:
```cpp
EXPECT_THROW(parser.load(""), ParseException)
    << "Expected 'empty file' error message";
```

**GOOD** — assert intent:
```cpp
try {
    parser.load("");
    FAIL() << "Expected ParseException for empty file";
} catch (const ParseException& e) {
    // Message may change; we assert that parsing failed, not the exact wording
    EXPECT_TRUE(e.isRecoverable() == false);
}
```

---

## 7. Coverage Philosophy

### 7.1 Coverage Goals

- **New code**: ≥ 90% line coverage (enforced via CI)
- **Behavioral coverage**: ≥ 80% mutation score (using `mutmut` or equivalent)
- **Critical path**: 100% of twin update loop, audio callback path exercised

**Do not pursue 100% line coverage at the expense of test value**. Trivial one-liner setters are not worth a test file.

### 7.2 What Constitutes "Enough" Coverage

A test is sufficient if it:
1. Exercises the **observable behavior contract** (not internal variables)
2. Covers the **happy path** and at least **one edge case** per parameter boundary
3. Would fail if the underlying business logic was broken

**Example** — `SimpleGearbox::update(dt)`:

```cpp
// Happy path: throttle=0.5, RPM rises gradually into upshift zone
TEST_F(SimpleGearboxTest, UpshiftsWhenRpmExceedsThreshold) { ... }

// Edge case: kickdown — rapid throttle increase triggers immediate downshift
TEST_F(SimpleGearboxTest, KickdownDownshiftsImmediately) { ... }

// Edge case: already in top gear — no upshift occurs
TEST_F(SimpleGearboxTest, WillNotUpshiftPastTopGear) { ... }

// NOT required: test every single throttle value from 0.0 to 1.0 in 0.001 increments
```

---

## 8. Regression Test Selection Strategy

### 8.1 Test Tiers by Runtime

| Tier | Tests | Estimated Runtime | When to Run |
|---|---|---|---|
| **Fast** | Unit tests only (all `*Test.cpp` in `test/unit/`) | ~5 s | Every `git commit` (pre-commit hook) |
| **Integration** | Twin+simulator mocks (`test/integration/`) | ~30 s | Every `git push` (pre-push) or CI |
| **E2E** | `TwinTestRunner` scenarios (`test/e2e/`) | ~2 min | CI on every PR |
| **Performance** | Real-time benchmark sweeps (`test/perf/`) | ~5 min | Nightly or on `main` branch only |

### 8.2 Running Subsets

```bash
# Fast unit only
ctest -R unit -j$(sysctl -n hw.ncpu)

# All except performance
ctest -E perf -j$(sysctl -n hw.ncpu)

# Single scenario
./twin_test_runner --gtest_filter="*FullThrottle*"
```

---

## 9. Cross-Platform Test Guarantees

### 9.1 macOS (Intel/ARM)

Run on GitHub Actions `macos-latest` (Intel runner) and `macos-14` (ARM M-series) matrix.

### 9.2 iOS (Device + Simulator)

`ios_adapter_tests` target built only when `BUILD_IOS_ADAPTER_TESTS=ON`. Tests `AudioBufferView` and `AVAudioEngineHardwareProvider` lifecycle.

### 9.3 Linux (Future ESP32)

ESP32-toolchain builds with `-DESP32_PLATFORM`. Tests compile-only (hardware mocked).

---

## 10. Test Fixtures & Shared Test Infrastructure

### 10.1 Common Test Helpers

Place reused helpers in `test/utils/`:

| Helper | Purpose |
|---|---|
| `TestHelpers.h` | `MakeMockTelemetry(double speedKmh, double throttlePct)` |
| `AudioTestUtils.h` | `extractLeftChannel()`, `computeRMS()`, `spectralCentroid()` |
| `EngineTestUtils.h` | `waitForRpm(double target, double tolerance, double timeout)` |
| `SimulationTestRunner.h` | One-liner: `TwinTestResult r = runScenario(&FullThrottle{});` |

### 10.2 Parameterized Tests

Use `::testing::WithParamInterface<T>` for boundary sweeps:

```cpp
class GearShiftThresholdTest : public ::testing::TestWithParam<std::tuple<double, int>> {
    TEST_P(GearShiftThresholdTest, UpshiftsAtExpectedRpm) {
        auto [throttlePct, expectedRpm] = GetParam();
        // ... configure, assert
    }
};

INSTANTIATE_TEST_SUITE_P(AllThrottles, GearShiftThresholdTest,
    ::testing::Combine(
        ::testing::Values(0.2, 0.5, 0.8, 1.0),
        ::testing::Values(  // (throttle → expected upshift RPM from ZF formula)
            std::make_pair(0.2, 750 + 5750*(0.20 + 0.60*0.2)),
            std::make_pair(0.5, 750 + 5750*(0.20 + 0.60*0.5)),
            std::make_pair(0.8, 750 + 5750*(0.20 + 0.60*0.8)),
            std::make_pair(1.0, 750 + 5750*(0.20 + 0.60*1.0))
        )
    )
);
```

### 10.3 Test Fixture Inheritance

Base class `TwinTestFixture` provides common setup:

```cpp
class TwinTestFixture : public ::testing::Test {
protected:
    void SetUp() override {
        logger = std::make_unique<ConsoleLogger>();
        telemetry = std::make_unique<InMemoryTelemetry>();
        // Each subclass overrides createTwin()
    }

    std::unique_ptr<ILogging> logger;
    std::unique_ptr<telemetry::InMemoryTelemetry> telemetry;
    std::unique_ptr<ITwinModel> twin;
};
```

---

## 11. Filing Bugs from Test Failures

When a test fails:

1. **Run individually** with `--gtest_filter` and `--gtest_verbose=info` to isolate
2. **Check golden files** — if the artifact changed legitimately, update and document
3. **Inspect artifact diff** — audio fingerprint JSON shows which metric changed
4. **If unexpected**, create `docs/test-failures/` entry with:
   - Test name
   - Observed vs. expected values
   - Prerequisites and `git bisect` range
   - Likely culprit (e.g., `BridgeSimulator::update()`, `DynoAssistedStrategy`)

---

## 12. Continuous Integration (CI)

### 12.1 GitHub Actions Workflow

`.github/workflows/ci.yml` runs on every PR:

```yaml
jobs:
  unit-tests:
    runs-on: ${{ matrix.os }}
    strategy:
      matrix:
        os: [macos-latest, ubuntu-latest]
    steps:
      - uses: actions/checkout@v4
      - name: Install deps
        run: |
          brew install cmake ninja  # mac
          sudo apt-get install cmake ninja-build  # linux
      - name: Build
        run: cmake -B build -GNinja -DCMAKE_BUILD_TYPE=Debug
      - name: Run unit tests
        run: ctest --test-dir build -R unit --parallel $(nproc)
      - name: Upload artifacts
        if: always()
        uses: actions/upload-artifact@v4
        with:
          name: unit-test-results
          path: build/Testing/*.xml

  integration-tests:
    needs: unit-tests
    runs-on: macos-latest
    steps:
      - ... build ...
      - name: Run integration tests
        run: ctest --test-dir build -R integration --parallel $(sysctl -n hw.ncpu)

  e2e-tests:
    needs: integration-tests
    runs-on: macos-latest
    steps:
      - ... build ...
      - name: Run twin smoke tests
        run: ./build/tests/twin_test_runner --gtest_filter="*FullThrottle*:*SteadyCruise*:*Deceleration*:*Standstill*"
      - name: Upload validation reports
        if: always()
        uses: actions/upload-artifact@v4
        with:
          name: twin-validation-reports
          path: test_artifacts/validation_reports/*.json

  # Performance tests: nightly only, not on every PR
  performance-tests:
    if: github.event_name == 'schedule'
    runs-on: macos-latest
    steps:
      - ... build ...
      - name: Run performance benchmarks
        run: ./build/perf/engine_perf_sweep
```

### 12.2 Test Result Reporting

- JUnit XML uploaded → GitHub Checks tab shows pass/fail
- Flaky test detection: if test X fails on CI but passes locally 5×, mark as flaky and quarantine until fixed
- Performance regression alerts: if `RPM_TRACKING_L2_ERROR` increases > 10% vs baseline, annotate PR

---

## 13. Test Hygiene & Maintenance

### 13.1 When to Delete Tests

Delete tests if:
- The feature they tested was **removed from the product** (not just refactored)
- The test is **duplicate** of another with different name only
- The test **always passes** because it asserts a tautology (e.g., `EXPECT_EQ(1+1, 2)`)

Never delete tests just because they're "annoying" — they encode requirements.

### 13.2 When to Update Tests

Update tests when:
- **Business rules change** (e.g., new upshift formula from ZF research)
- **Golden files drift** with legitimate behavior changes (commit update with PR rationale)
- **Test data** becomes insufficient (e.g., previously untested edge case discovered)

### 13.3 Naming Conventions

```
TEST_F/FixtureName, TestName)             // Fixture-prefixed, CamelCase
TEST(UnitTestSuite, ComputesRpmFromSpeed) // Suite-prefixed
```

**Suite names**: `BridgeUnitTests`, `GearboxUnitTests`, `TwinIntegrationTests`, `E2eSmokeTests`.

**Test names**: Must communicate the **expected behavior**:
- ✅ `ProducesSixVirtualGearShiftsDuringFullThrottle`
- ✅ `TracksTargetRpmWithinFiftyMilliseconds`
- ✅ `RejectsNegativeGearSelection`
- ❌ `TestUpdate1`, `TestShiftLogic`

---

## 14. Test-Driven Development in This Project

### 14.1 TDD Workflow Checklist (per feature)

1. **RED**: Write **failing** test that asserts the new behavior
2. **RED**: Make it compile (declarations in headers, stub methods)
3. **GREEN**: Write **minimal** implementation to make test pass
4. **REFACTOR**: Clean up code while green, run full suite after each change
5. **COMMIT**: Message links to test and requirement (e.g., "Add DynoAssistedStrategy update() — see TEST_STRATEGY.md §4.2")

### 14.2 When to Break TDD

Rare exceptions:
- **Spike/exploratory code**: Write prototype first, then delete and TDD from scratch
- **Refactoring legacy code without tests**: Add characterization tests first, then enable TDD

---

## 15. Next Steps

Once this strategy is approved, the team proceeds with:

1. Create `test/` directory structure per Section 8
2. Implement `TwinTestRunner` and `MockTelemetryProvider` (per `TEST_HARNESS_DESIGN.md`)
3. Write **RED-phase** tests for `DynoAssistedStrategy` and `SimpleGearbox`
4. Green → Refactor → Golden file baseline

The test strategy document lives alongside the code — update it when the testing approach evolves.
