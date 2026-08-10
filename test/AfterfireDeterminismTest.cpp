// AfterfireDeterminismTest.cpp
//
// Deterministic, SILENT acceptance test for the afterfire ("pop on overrun")
// feature. It drives the real SimulatorFactory -> BridgeSimulator path — the same
// path the CLI uses — and proves that a throttle-cut overrun increases the
// per-chamber afterfire event counters.
//
// The afterfire model is physical: unburnt fuel in a runner that is above the
// auto-ignition temperature lights off once its Arrhenius induction period
// completes, and exhaust flow scavenging the pipe is what resets that clock.
// This test asserts the two halves of that contract that a user would notice —
// pops DO happen on a throttle-cut overrun, and pops do NOT happen while the
// engine is held at steady throttle (where scavenging always wins the race).
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
constexpr int    kSteadyThrottleTicks = 240;        // ~4.0 s held at constant pedal

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

// Afterfire tuning for the acceptance run: the PHYSICAL DEFAULTS, unmodified.
//
// There is deliberately nothing to open up here any more. The old config had to
// disable a probability roll, a cooldown, a pop-spacing timer and an event cap
// so that none of them could mask a working implementation; none of those exist
// now, because a pop is decided by the runner's own temperature, mixture and
// residence time. Asserting against the shipped defaults is therefore strictly
// stronger: it proves the effect works as delivered rather than only under
// test-only settings.
AfterfireConfig acceptanceAfterfireConfig() {
    AfterfireConfig config;
    config.enabled = true;
    config.diagnostics = true;
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

    // Report the physical state on failure: these say whether the runner ever
    // became reactive at all, and how close the induction integral came to
    // completing — the difference between "never hot enough" and "always
    // scavenged first". Without them a zero-event failure is unactionable.
    // By value, not by reference: getAfterfireDiagnostics() returns a fresh
    // vector, so a reference to .front() would dangle immediately.
    const AfterfireDiagnostics chamber0 = bridge->getAfterfireDiagnostics().front();
    EXPECT_GT(eventsAfterOverrun, eventsBeforeOverrun)
        << "No afterfire events were produced by a throttle-cut overrun (before="
        << eventsBeforeOverrun << " after=" << eventsAfterOverrun << ").\n"
        << "  chamber0 runner peaks: T=" << chamber0.maxRunnerTempK << "K"
        << " rawFuelFraction=" << chamber0.maxRawFuelFraction
        << " ignitionProgress=" << chamber0.maxIgnitionProgress << "\n"
        << "  not ignited: tooCold=" << chamber0.skippedTooCold
        << " noFuel=" << chamber0.skippedNoFuel
        << " noOxygen=" << chamber0.skippedNoOxygen
        << " inductionIncomplete=" << chamber0.skippedNotReady;

    simulator->destroy();
}

// ---------------------------------------------------------------------------
// The other half of the contract: a STEADY throttle must stay quiet.
//
// This is the regression that matters most in practice. The previous
// timer-driven implementation fired on an interval whenever its gates happened
// to agree, which produced a metronomic "knock" under cruise — an engine that
// pops while holding constant throttle sounds broken. Physically it must not
// happen: under power every exhaust stroke scavenges the runner, so the charge
// is swept out long before its induction period can complete.
//
// Held at a HIGH steady throttle deliberately, because that is the hostile case
// — it is where the runner is hottest (measured ~2400 K at WOT), so any model
// that keys off temperature alone rather than residence time fails here.
// ---------------------------------------------------------------------------
TEST(AfterfireDeterminismTest, SteadyThrottleProducesNoAfterfireEvents) {
    const std::filesystem::path scriptPath = resolveScriptPath();
    ASSERT_TRUE(std::filesystem::exists(scriptPath)) << "Engine script not found: " << scriptPath.string();
    const std::filesystem::path assetBasePath = resolveAssetBasePath();
    ASSERT_TRUE(std::filesystem::exists(assetBasePath / "sound-library"))
        << "Impulse-response assets not found under: " << assetBasePath.string();

    ISimulatorConfig config;
    config.simulationFrequency = 0;

    std::unique_ptr<ISimulator> simulator = SimulatorFactory::create(
        SimulatorType::PistonEngine, scriptPath.string(), assetBasePath.string(), config, nullptr, nullptr);
    ASSERT_NE(simulator, nullptr);
    ASSERT_TRUE(simulator->create(config, nullptr, nullptr)) << simulator->getLastError();

    auto* bridge = dynamic_cast<BridgeSimulator*>(simulator.get());
    ASSERT_NE(bridge, nullptr);

    SimulatorFactory::configureAfterfire(simulator.get(), acceptanceAfterfireConfig(), nullptr);
    ASSERT_FALSE(bridge->getAfterfireDiagnostics().empty())
        << "getAfterfireDiagnostics() returned no chambers — afterfire is not wired through";

    bridge->setIgnition(true);
    bridge->setStarterMotor(true);
    driveFor(*simulator, 0.0, kCrankTicks);
    bridge->setStarterMotor(false);
    driveFor(*simulator, 0.0, kIdleSettleTicks);

    // Spin up, then hold a constant pedal and count only what happens while it
    // is held — the ramp itself is excluded so this measures steady state.
    driveFor(*simulator, 1.0, kRevTicks);
    bridge->resetAfterfireDiagnostics();

    driveFor(*simulator, 1.0, kSteadyThrottleTicks);
    const double steadyRpm = simulator->getStats().currentRPM;
    EXPECT_GT(steadyRpm, kMinRevRpm)
        << "Scenario guard: the engine was not actually held under power, so this "
           "says nothing about steady-throttle behaviour. rpm=" << steadyRpm;

    const int steadyEvents = sumAfterfireEvents(*bridge);
    // By value, not by reference: getAfterfireDiagnostics() returns a fresh
    // vector, so a reference to .front() would dangle immediately.
    const AfterfireDiagnostics chamber0 = bridge->getAfterfireDiagnostics().front();
    EXPECT_EQ(steadyEvents, 0)
        << "Afterfire fired " << steadyEvents << " times at steady throttle. Under power the "
           "exhaust runner is scavenged every cycle, so the induction period cannot complete; "
           "firing here is the metronomic-knock regression.\n"
        << "  chamber0 runner peaks: T=" << chamber0.maxRunnerTempK << "K"
        << " rawFuelFraction=" << chamber0.maxRawFuelFraction
        << " ignitionProgress=" << chamber0.maxIgnitionProgress;

    simulator->destroy();
}
