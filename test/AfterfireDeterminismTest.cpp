// AfterfireDeterminismTest.cpp
//
// Deterministic, SILENT acceptance test for the afterfire ("pop on overrun")
// feature. It drives the real SimulatorFactory -> BridgeSimulator path — the same
// path the CLI uses — and proves that a throttle-cut overrun increases the
// per-chamber afterfire event counters.
//
// TDD PHASE: RED. The engine-sim skeleton stubs the firing decision
// (CombustionChamber::shouldTriggerAfterfire always declines, and
// PistonEngineSimulator::tickAfterfire never asks a chamber to fire), so the
// summed eventCount stays 0 and the final assertion MUST fail. The test compiles
// and links cleanly against the skeleton API, so the RED is a BEHAVIOUR failure,
// not a missing symbol.
//
// NO AUDIO: the event counter is the non-audio proof that pops occurred. Nothing
// here renders, reads or asserts on audio, so the result never depends on a
// human ear and cannot flake on audio-thread timing.
//
// DETERMINISM: the drive loop is a fixed number of fixed-size ticks over a
// fixed-step integrator; no wall-clock, no time-seeded RNG, and no audio thread
// (BridgeSimulator::start() is deliberately NOT called, so nothing renders in the
// background). Verified empirically: two consecutive in-process runs of this
// scenario produced bitwise-identical RPM traces (maxAbsDiff = 0.000000000).
//
// ---------------------------------------------------------------------------
// THROTTLE POLARITY — measured, not assumed
// ---------------------------------------------------------------------------
// C63_M156_V3.mr resolves to a DirectThrottleLinkage engine. It sets no explicit
// `throttle:`, so it inherits objects.mr's default
// `direct_throttle_linkage(gamma: throttle_gamma)`; a dynamic_cast on the built
// engine confirms it. DirectThrottleLinkage::setSpeedControl(s) sets plate
// position = 1 - s^gamma, so the SPEED-CONTROL input is inverted relative to
// plate position.
//
// That inversion is INTERNAL to engine-sim and already yields correct pedal
// behaviour. Measured end-to-end through BridgeSimulator on this engine:
//
//     setThrottle(0.0) -> idles ~1020 RPM   (Engine::getThrottle() == 1.0)
//     setThrottle(1.0) -> revs  ~7290 RPM   (Engine::getThrottle() == 0.0)
//
// So the bridge's CURRENT setThrottle already acts as a pedal: 1.0 revs, 0.0
// cuts. The trap is that `Engine::getThrottle()` on this linkage is a PLATE
// RESTRICTION value (1.0 = closed/cut), i.e. the opposite sense to the pedal —
// reading it as a pedal value is what makes the polarity look broken when it is
// not.
//
// This test therefore asserts the PEDAL INTENT contract (1.0 revs, 0.0 cuts) and
// reads the event counter. It deliberately does NOT assert any internal
// speed-control or plate value, so it stays correct however GREEN wires the
// internals — while still failing loudly if the pedal gets inverted.
// ---------------------------------------------------------------------------

#include <gtest/gtest.h>

#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "simulator/BridgeSimulator.h"
#include "simulator/EngineSimTypes.h"
#include "simulator/SimulatorFactory.h"

// Paths are injected by CMake (same convention as TEST_ES_DIR / TEST_FIXTURE_DIR
// elsewhere in this suite) and resolved at CONFIGURE time, so the test never has
// to guess the checkout layout at runtime. Deriving them from __FILE__ was tried
// and rejected: __FILE__ is relative whenever the compiler is invoked with a
// relative source path, which silently breaks resolution.
#ifndef AFTERFIRE_TEST_SCRIPT_PATH
#define AFTERFIRE_TEST_SCRIPT_PATH ""
#endif
#ifndef AFTERFIRE_TEST_ASSET_DIR
#define AFTERFIRE_TEST_ASSET_DIR ""
#endif

namespace {

// --- Scenario constants (single source of truth; no magic numbers inline) ----

constexpr double kControlTickSeconds = 1.0 / 60.0;  // 60 Hz control tick
constexpr int    kCrankTicks         = 120;         // ~2.0 s on the starter
constexpr int    kIdleSettleTicks    = 90;          // ~1.5 s settling at idle
constexpr int    kRevTicks           = 180;         // ~3.0 s at full pedal
constexpr int    kCoastTicks         = 240;         // ~4.0 s of overrun coast

// Measured on this scenario: idle ~1020 RPM, revved ~7290 RPM, coast bottoms out
// near idle. These bounds are deliberately loose — they prove "it really revved"
// and "it really decelerated". They are NOT tuning assertions.
constexpr double kMinRevRpm     = 4000.0;
constexpr double kMinRpmDropRpm = 1500.0;

// Resolve a path from an env override first (lets a developer or CI point the
// test at a different checkout without recompiling), else the CMake-injected
// compile-time default.
std::filesystem::path resolvePath(const char* environmentVariable, const char* compiledDefault) {
    const char* pathOverride = std::getenv(environmentVariable);
    const bool hasOverride = (pathOverride != nullptr) && (std::strlen(pathOverride) > 0);
    return std::filesystem::path(hasOverride ? pathOverride : compiledDefault);
}

// The engine script: C63_M156_V3.mr, which lives in the CLI repo's es_new/.
std::filesystem::path resolveScriptPath() {
    return resolvePath("ATG_AFTERFIRE_SCRIPT", AFTERFIRE_TEST_SCRIPT_PATH);
}

// The directory that DIRECTLY contains sound-library/ (the impulse-response
// WAVs). The bridge's own es/ carries them; the CLI's es_new/ does NOT (it ships
// only impulse_responses.mr), so the script's own directory cannot double as the
// asset base here — the factory throws when the WAVs are missing.
std::filesystem::path resolveAssetBasePath() {
    return resolvePath("ATG_AFTERFIRE_ASSETS", AFTERFIRE_TEST_ASSET_DIR);
}

int sumAfterfireEvents(const BridgeSimulator& bridge) {
    int total = 0;
    for (const AfterfireDiagnostics& chamber : bridge.getAfterfireDiagnostics()) {
        total += chamber.eventCount;
    }
    return total;
}

// Hold a fixed pedal position for a fixed number of control ticks.
void driveFor(ISimulator& simulator, double pedal, int ticks) {
    for (int tick = 0; tick < ticks; ++tick) {
        simulator.setThrottle(pedal);
        simulator.update(kControlTickSeconds);
    }
}

// Afterfire tuning for the acceptance run: permissive on purpose. The subject is
// "does a pop ever happen on overrun", not how a tuned engine sounds, so every
// gate that could mask a working implementation is opened.
AfterfireConfig acceptanceAfterfireConfig() {
    AfterfireConfig config;
    config.enabled = true;
    config.diagnostics = true;
    config.probability = 1.0;         // no stochastic gate — keeps the run deterministic
    config.cooldownMs = 0.0;          // no per-chamber rate limit
    config.globalPopIntervalMs = 0.0; // no cross-chamber spacing
    config.rpmMin = 1500.0;           // comfortably under the measured coast range
    config.throttleCutoff = 0.5;      // permissive gate
    config.maxEventsPerDecel = 1000;  // effectively uncapped
    return config;
}

} // namespace

// ---------------------------------------------------------------------------
// ACCEPTANCE: rev the engine, snap the pedal shut, coast on overrun, and require
// that the afterfire event counter increased.
//
// The EXPECTs before the final assertion are SCENARIO GUARDS, not the subject of
// the test. They exist so a failure says WHICH thing broke — "the engine never
// revved" / "it never decelerated" (scenario invalid) versus "the overrun
// happened but nothing popped" (the real RED, and later the real regression).
// Without them a pedal-polarity regression would surface as a confusing
// zero-event failure instead of naming its own cause.
// ---------------------------------------------------------------------------
TEST(AfterfireDeterminismTest, OverrunAfterThrottleCutProducesAfterfireEvents) {
    const std::filesystem::path scriptPath = resolveScriptPath();
    ASSERT_TRUE(std::filesystem::exists(scriptPath))
        << "Engine script not found: '" << scriptPath.string()
        << "'. Set ATG_AFTERFIRE_SCRIPT, or check the AFTERFIRE_TEST_SCRIPT_PATH "
           "definition in CMakeLists.txt.";

    const std::filesystem::path assetBasePath = resolveAssetBasePath();
    ASSERT_TRUE(std::filesystem::exists(assetBasePath / "sound-library"))
        << "Impulse-response assets not found under: '" << assetBasePath.string()
        << "'. Set ATG_AFTERFIRE_ASSETS, or check the AFTERFIRE_TEST_ASSET_DIR "
           "definition in CMakeLists.txt.";

    ISimulatorConfig config;
    config.simulationFrequency = 0;  // 0 = use the engine script's own frequency

    std::unique_ptr<ISimulator> simulator = SimulatorFactory::create(
        SimulatorType::PistonEngine,
        scriptPath.string(),
        assetBasePath.string(),
        config,
        nullptr,
        nullptr);
    ASSERT_NE(simulator, nullptr) << "Factory returned null for " << scriptPath.string();
    ASSERT_TRUE(simulator->create(config, nullptr, nullptr))
        << "BridgeSimulator::create failed: " << simulator->getLastError();

    auto* bridge = dynamic_cast<BridgeSimulator*>(simulator.get());
    ASSERT_NE(bridge, nullptr) << "Factory did not produce a BridgeSimulator";

    SimulatorFactory::configureAfterfire(simulator.get(), acceptanceAfterfireConfig(), nullptr);

    // Guard the diagnostics channel itself: an empty vector would make the final
    // assertion a vacuous 0 > 0 and hide a wiring failure behind a plausible
    // behavioural RED.
    ASSERT_FALSE(bridge->getAfterfireDiagnostics().empty())
        << "getAfterfireDiagnostics() returned no chambers — afterfire is not wired "
           "through to the engine (is ATG_ENGINE_SIM_AFTERFIRE_SPIKE ON?)";

    // --- Start: ignition + starter, then settle at idle ---------------------
    bridge->setIgnition(true);
    bridge->setStarterMotor(true);
    driveFor(*simulator, 0.0, kCrankTicks);
    bridge->setStarterMotor(false);
    driveFor(*simulator, 0.0, kIdleSettleTicks);

    const int eventsBeforeOverrun = sumAfterfireEvents(*bridge);

    // --- Rev: full pedal ----------------------------------------------------
    driveFor(*simulator, 1.0, kRevTicks);
    const double revRpm = simulator->getStats().currentRPM;
    EXPECT_GT(revRpm, kMinRevRpm)
        << "Scenario guard: the engine did not rev on full pedal (1.0), so no overrun "
           "can follow. The pedal-to-throttle mapping is the likely cause. revRpm="
        << revRpm;

    // --- Cut: snap the pedal shut and coast on overrun ----------------------
    driveFor(*simulator, 0.0, kCoastTicks);
    const double coastRpm = simulator->getStats().currentRPM;
    EXPECT_LT(coastRpm, revRpm - kMinRpmDropRpm)
        << "Scenario guard: RPM did not fall meaningfully after the throttle cut, so "
           "the engine was never actually on overrun. revRpm=" << revRpm
        << " coastRpm=" << coastRpm;

    // --- The actual subject of this test ------------------------------------
    const int eventsAfterOverrun = sumAfterfireEvents(*bridge);
    std::cout << "eventsAfterOverrun=" << eventsAfterOverrun << " (before=" << eventsBeforeOverrun << ")" << std::endl;
    EXPECT_GT(eventsAfterOverrun, eventsBeforeOverrun)
        << "No afterfire events were produced by a throttle-cut overrun (before="
        << eventsBeforeOverrun << " after=" << eventsAfterOverrun << "). "
           "RED-phase expectation: the engine-sim firing decision is still a stub.";

    simulator->destroy();
}
