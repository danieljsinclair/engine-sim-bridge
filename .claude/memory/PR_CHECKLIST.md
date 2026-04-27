# Pre-Commit / Pre-PR Checklist

**Project:** VirtualICE Twin — engine-sim-bridge
**Applies to:** All commits on `main`, all pull requests to `master`
**Enforcement:** Mandatory — merge-blocking

---

## Purpose

This checklist catches common testability, determinism, and architecture violations before they enter `main`. It codifies the team's quality gate: **Every change is tested, deterministic, and SOLID-compliant**.

**How to use:**
1. Complete this checklist for every PR (self-review is OK)
2. The **PR reviewer verifies** checklist items before approving
3. If a CI test is added, run it locally first: `ctest --test-dir build`
4. If any item fails, the PR **must not merge** until resolved

---

## Section A: Tests & TDD

### A1 — All New Code Has a Corresponding Test

- [ ] Every new class has at least **one** `TEST()` or `TEST_F()` covering its core contract
- [ ] Every new public method has at least **happy-path + one edge case** test
- [ ] Bug fixes include a **reproducing-test-first** (RED fails, GREEN passes, bug fixed)

**Exception:** Trivial one-liner getters/setters with no logic — OK to skip. Document why skipped in PR description.

**Example of passing:**
```cpp
// New class: SimpleGearbox
TEST_F(SimpleGearboxTest, SelectsCorrectGearFromRpmThresholds) { ... }
TEST_F(SimpleGearboxTest, DoesNotUpshiftPastTopGear) { ... }
TEST_F(SimpleGearboxTest, TriggersKickdownOnRapidThrottleIncrease) { ... }
```

**Example of failing:**
```cpp
// New class: DynoAssistedStrategy (no tests)
class DynoAssistedStrategy { ... }  ///❌ No test file → BLOCKED
```

---

### A2 — Tests Compile in RED Phase

- [ ] New test file compiles **before** the implementation stubs are added (linker errors OK, compile errors not OK)
- [ ] Implementation stub exists to link function symbols (even if empty body)

**Why:** Ensures tests are written first, not added as afterthought.

---

### A3 — Tests Are Deterministic

**These are NEVER allowed in any test:**
- [ ] `std::rand()`, `rand_r()`, `arc4random()` — use fixed values
- [ ] `std::this_thread::sleep_for()` as synchronization — use condition variables or mocks
- [ ] `std::chrono::steady_clock::now()` (wall-clock timing) — inject clock or use simulation time
- [ ] Reading from filesystem for test data — embed as `constexpr` or inline strings
- [ ] Reading from network/serial port — mock `ITelemetrySource`
- [ ] `std::random_device`, `std::mt19937` without fixed seed — seed with constant

**Verified by:** Visual inspection of test file + `grep -r "sleep_for\|rand\|random_device" test/`

---

### A4 — Tests Are Self-Contained

- [ ] No test depends on another test's side effects (no global mutable state)
- [ ] Each test sets up its own fixtures in `SetUp()` or test body
- [ ] Each test cleans up in `TearDown()` (files, sockets, temp dirs)
- [ ] `TearDown()` is `noexcept` (does not throw)

**Anti-pattern:**
```cpp
// ❌ GLOBAL STATE POLLUTION — flaky in parallel run
static int g_globalCounter;
TEST(FeatureA) { g_globalCounter = 42; ... }
TEST(FeatureB) { EXPECT_EQ(g_globalCounter, 0); } // flaky if FeatureA runs first
```

---

### A5 — Tests Are Fast

- [ ] Unit tests ≤ 1 second each (average < 200 ms)
- [ ] No slow sleeps (max sleep_for = 10 ms; replace with mocks for "wait for condition")
- [ ] No unbounded loops (tests must have built-in timeouts)

**Rule of thumb**: A failing CI run should report within 5 minutes, E2E within 15 minutes.

---

### A6 — Integration Tests Use Fakes/Mocks for External Dependencies

- [ ] **Audio hardware**: `MockAudioHardwareProvider` or `FakeAudioHardware` — never real device in CI
- [ ] **OBD2/CAN**: `MockTelemetrySource` — never serial port in CI
- [ ] **Filesystem**: Test assets committed to repo (`test/fixtures/`), no network mounts
- [ ] **Network**: All external services mocked (no `connect()` in tests)

**Example of passing:**
```cpp
// Integration test — twin + simulator + mock source
auto mockSource = MockTelemetrySource::create(FullThrottleScenario{});
auto twin = TwinFactory::createDynoTwin();
twin->initialize(mockSource.get(), ...);
```

---

## Section B: Code Quality & Architecture

### B1 — SOLID Compliance Check

**SRP (Single Responsibility Principle):**
- [ ] Each class has one, well-scoped reason to change
- [ ] No "God classes" with >3 major responsibilities

**OCP (Open/Closed Principle):**
- [ ] New behavior added via new classes, not `if (type == X)` conditionals
- [ ] Strategies use polymorphic dispatch, not switch-on-enum over all modes

**LSP (Liskov Substitution Principle):**
- [ ] Subclasses honor base class contracts (no weakening preconditions, no strengthening postconditions)
- [ ] Mocks behave identically to production for tested call paths

**ISP (Interface Segregation Principle):**
- [ ] No "fat interfaces" — interfaces have minimal cohesive methods
- [ ] Callers depend only on the methods they actually use

**DIP (Dependency Inversion Principle):**
- [ ] High-level modules depend on abstractions (`ITwinModel`, `IInputProvider`), not concrete implementations
- [ ] No `#include "ConcreteClass.h"` in header files; forward-declare abstractions

**Check:** Run `clang-tidy` with `readability/` and `bugprone/` checks. Fix or justify suppressions.

---

### B2 — No Implementation Details in Tests

- [ ] Tests do not access `private` or `protected` members of SUT (unless via `FRIEND_TEST` —rarely justified)
- [ ] Tests do not assert on internal state variable values (assert on observable outputs)
- [ ] Tests do not depend on specific code paths (e.g., caching layer presence)

**BAD:**
```cpp
// ❌ Tests internal cache hit counter — implementation detail
EXPECT_EQ(gearbox->m_cacheHits, 5);
```

**GOOD:**
```cpp
// ✅ Tests behavior — shift timing (cache affects performance but not correctness unless timing-critical)
EXPECT_NEAR(shift.timestamp, expectedTime, 0.1);
```

---

### B3 — Thread Safety Verified

- [ ] All methods called from audio callback thread are marked `// RT safe` in comments
- [ ] Any lock used in audio callback path is `std::atomic` or lock-free (no `std::mutex`)
- [ ] Data shared between threads uses `std::atomic` or `std::mutex` with clear ownership
- [ ] `ThreadSanitizer` run on new thread code (`-fsanitize=thread`), no races reported

**Run:**
```bash
TSAN_OPTIONS="suppressions=test/.tsan-suppress" ./build/tests/twin_test_runner --gtest_filter="*Concurrency*"
```

---

### B4 — No Hidden Side Effects

- [ ] Const methods do not modify `mutable` state (only for caching/lazy init, documented)
- [ ] No global/static mutable state modified by methods under test
- [ ] No singleton pattern (anti-pattern for testability)

---

### B5 — Memory Safety

- [ ] No raw `new`/`delete` — use `std::unique_ptr`, `std::shared_ptr`, or stack allocation
- [ ] No C-style arrays — use `std::vector` or `std::array`
- [ ] No pointer arithmetic on user-facing APIs
- [ ] AddressSanitizer run: `-fsanitize=address` passes cleanly

---

## Section C: Regression Protection

### C1 — Golden Files Updated Legitimately

If test golden JSON files changed:

- [ ] Change is **expected** (new behavior, minor metric drift within tolerance)
- [ ] Artifact diff inspected for anomalies (e.g., `cosineSimilarity` dropped from 0.96→0.70 indicates problem)
- [ ] PR description explains why golden changed (e.g., "New ZF shift curve updates upshift RPM thresholds")

---

### C2 — No Silent Behavior Changes

- [ ] All assertion messages describe expected vs. actual (use `EXPECT_NEAR(a, b, tol)` not `EXPECT_EQ` for floating point)
- [ ] No assertions commented out or `// TODO: fix me` in test bodies
- [ ] No `SUCCEED()` macros standing alone as the only assertion in a test (must have at least one real assertion)

**BAD:**
```cpp
TEST(Foo, DoesSomething) {
    // TODO: fill in asserts
    SUCCEED();  // ❌ test always passes regardless
}
```

---

## Section D: Documentation & Naming

### D1 — Interfaces Documented

- [ ] Every virtual method has Doxygen-style comment block: `@param`, `@return`, `@pre`, `@post` where meaningful
- [ ] Every interface lists thread-safety guarantees in header: `// Thread-safe` or `// Called from sim thread only`
- [ ] Exceptions documented (`@throws` if applicable)

**Minimal viable comment:**
```cpp
/**
 * @brief Set engine throttle position.
 * @param throttle Normalized throttle 0.0 (idle) to 1.0 (full)
 * @pre throttle ∈ [0, 1]
 * @post simulator throttle set; takes effect on next update()
 * @threadsafe No — sim thread only
 */
virtual void setThrottle(double throttle) = 0;
```

---

### D2 — Tests Named Clearly

- [ ] Test name is sentence fragment describing expected behavior
- [ ] Fixture name ends with `Test` or `Tests` (e.g., `DynoAssistedStrategyTest`, not `DynoHelper`)
- [ ] No generic names: `Test1`, `CheckStuff`, `DoesItWork`

**Pattern:** `[Unit_Under_Test]_[Scenario_Under_Test]_[Expected_Behavior]`

**Examples:**
- `SimpleGearbox_UpshiftsAtConfiguredRpmThreshold`
- `DynoAssistedStrategy_TracksTargetRpmWithinFiftyMilliseconds`
- `VirtualIceInputProvider_ForwardsTwinOutputToSimulator`

---

### D3 — Inter-module Interface Changes Noted

If PR modifies any of these files, mandatory cross-review:
- `include/io/IInputProvider.h`
- `include/hardware/IAudioHardwareProvider.h`
- `include/simulator/ISimulator.h`
- `include/strategy/IAudioBuffer.h`
- `include/telemetry/ITelemetryProvider.h`
- `INTERFACE_DESIGN.md` (if contracts change)

**Check:** Reviewer confirms backward compatibility or justified breaking change.

---

## Section E: CI & Build Health

### E1 — Builds Cleanly

- [ ] `cmake -B build -GNinja` succeeds on macOS (Intel and ARM)
- [ ] No compiler warnings under `-Wall -Wextra -Werror` (or justified suppressions)
- [ ] No static analysis (`clang-tidy`) warnings (or `NOLINT`-justified)

---

### E2 — All Tests Pass Locally Before Push

- [ ] Run `ctest --test-dir build -j$(sysctl -n hw.ncpu)` before `git push`
- [ ] All unit and integration tests **PASS** (no flakes)
- [ ] If a test flakes, quarantine it (rename `*_FLAGGY` and add issue link) before merging anything else

---

### E3 — Pre-Commit Hook Passes (if configured)

If `.git/hooks/pre-commit` or `.claude/settings.json` has hooks:

- [ ] Linting (`clang-format`) applied to all modified `.cpp`, `.h`, `.mm` files
- [ ] Fast unit test subset runs (e.g., only touched test files)
- [ ] Commit message follows conventional format: `type(scope): subject` (e.g., `feat(twin): add DynoAssistedStrategy::update()`)

---

## Section F: Twin-Specific Quality Gate

### F1 — ITwinStrategy Implementation Pattern

For new strategies (e.g., `PhysicsDrivenStrategy`, `RecordedPlayback`):

- [ ] Strategy extends `ITwinStrategy` base class
- [ ] Implements `clone()` method (deep copy or shared-ptr clone)
- [ ] `update()` is deterministic, no allocations, thread-safe
- [ ] Strategy registered in `TwinFactory::createStrategy()` switch

---

### F2 — Mock Telemetry Scenario Validity

For new `ScenarioDefinition` subclasses:

- [ ] `throttleAt(t)` is continuous and bounded `[0,1]` (no discontinuities)
- [ ] `expectedRPMAt(t)` mathematically consistent with throttle model: `800 + 5200 × throttle(t)`
- [ ] `duration()` ≥ 0 and finite (no infinite loops)
- [ ] Scenario named descriptively: `HighwayPass`, `MountainClimb`, `TrackDay`

---

### F3 — Audio Pipeline Hygiene

Any change touching these files requires audio pipeline smoke test:
- `src/simulation/SimulationLoop.cpp`
- `src/strategy/SyncPullStrategy.cpp` / `ThreadedStrategy.cpp`
- `src/hardware/CoreAudioHardwareProvider.cpp` / `AVAudioEngineHardwareProvider.mm`

- [ ] Run `twin_test_runner` locally with `--gtest_filter="*Audio*"` and verify no underruns
- [ ] Capture audio and listen for clicks/pops (manual verification)

---

## Section G: Breaking Changes

### G1 — Breaking Changes Identified and Justified

If PR breaks backward compatibility (interface changed, behavior changed):

- [ ] Breaking change is **required** (not cosmetic)
- [ ] Alternatives considered and rejected with rationale
- [ ] All call-sites updated atomically in the same PR (or coordinated via deprecation cycle)
- [ ] Deprecation warnings added if supporting both old and new APIs during transition
- [ ] Version bump planned in `CMakeLists.txt` (major version for breaking changes)

---

## Section H: Final Validation

### H1 — Self-Review Complete

- [ ] I reviewed every modified file
- [ ] I ran all affected tests locally
- [ ] I verified checklist items above
- [ ] PR description includes:
  - What changed (summary)
  - Why (rationale linked to requirement or bug)
  - How to test (commands, expected outputs)
  - Breaking changes (if any)

---

### H2 — Reviewer Sign-Off

**Reviewer name:** ________________
**Date:** ________________

- [ ] All checklist items A–H pass
- [ ] Tests are meaningful (not just coverage padding)
- [ ] Architecture remains clean (no shortcuts taken)
- [ ] Documentation updated alongside code (headers, `INTERFACE_DESIGN.md` if impacted)
- [ ] Ready to merge → **APPROVED**

---

## Automated Enforcement (Future Enhancement)

When the team adopts pre-commit automation:

| Check | Tool | Status |
|---|---|---|
| Format (clang-format) | `pre-commit` hook | TODO |
| Fast unit tests | `pre-push` hook | TODO |
| TreadSanitizer (new threads) | CI job | TODO |
| Mutation score ≥ 80% | `mutmut` CI step | TODO |
| Golden file diff size | CI verification | TODO |

---

## Appendix A: `ctest` Invocation Reference

```bash
# All tests (fast + integration + E2E)
ctest --test-dir build --parallel $(sysctl -n hw.ncpu) --output-on-failure

# Unit tests only
ctest --test-dir build -R unit --parallel $(sysctl -n hw.ncpu)

# E2E twin validation with verbose artifacts
./build/tests/twin_test_runner \
    --gtest_filter="*FullThrottle*:*Cruise*" \
    --artifacts_dir=./tmp/artifacts

# ThreadSanitizer check (run single-threaded)
TSAN_OPTIONS="suppressions=test/.tsan-suppress" \
    ./build/tests/twin_test_runner
```

---

## Appendix B: Quick Banned Patterns

```
❌ sleep_for(100ms)               → use condition_variable or mocks
❌ rand() / random_device         → use fixed seed or deterministic function
❌ ASSERT(condition)              → use GoogleTest EXPECT_/ASSERT_ macros
❌ System("rm -rf ...")           → use filesystem API safely
❌ reinterpret_cast<float*>(...)  → use memcpy or proper conversion
❌ assert() in release            → expect exceptions or error codes
❌ Global mutable variables      → inject dependencies
❌ Singleton pattern             → use dependency injection
```

---

**Last updated:** 2026-04-24
**Next review:** After Phase 1 twin implementation complete
