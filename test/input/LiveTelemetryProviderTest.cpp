// LiveTelemetryProviderTest.cpp - Contract tests for LiveTelemetryProvider
//
// LiveTelemetryProvider.cpp is a dead-stripped file (120 LOC, 0% coverage) that
// wraps a VirtualIceInputProvider (the "twin") behind a thread-safe signal inbox.
// These tests assert the OBSERVABLE contract:
//   - Initialize() success makes the provider connected after a signal is submitted
//   - Initialize() called again returns false (already-initialized guard)
//   - OnUpdateSimulation before init returns empty input with a "not initialized" error
//   - submitSignal round-trips through getCurrentSignal
//   - submitSignal(timed) stamps the signal's timestamp
//   - Shutdown() disconnects the provider
//   - setGearSelector/setIgnition/provideFeedback delegate to the twin (no-crash
//     with a real twin, observed via IsConnected remaining true)
//
// The inner twin-failure branch (twin provider's Initialize fails) is NOT covered:
// VirtualIceInputProvider::Initialize only fails on an exception, which cannot be
// triggered from the public profile seam without hacking production code. Per the
// spec we accept the happy path + idempotency + not-init guard as the core.

#include "input/LiveTelemetryProvider.h"
#include "twin/IceVehicleProfile.h"
#include "simulator/EngineSimTypes.h"
#include "simulator/GearConventions.h"

#include <gtest/gtest.h>
#include <sstream>
#include <vector>

namespace {

class LiveTelemetryProviderTest : public ::testing::Test {
protected:
    void SetUp() override {
        profile_ = twin::IceVehicleProfile::zf8hp45();
        provider_ = std::make_unique<input::LiveTelemetryProvider>(profile_);
    }

    twin::IceVehicleProfile profile_;
    std::unique_ptr<input::LiveTelemetryProvider> provider_;
};

// Initialize() success path: provider becomes connected only after a signal is
// submitted (signalReceived_ gate).
TEST_F(LiveTelemetryProviderTest, InitializeConnectsAfterSignalSubmitted) {
    ASSERT_TRUE(provider_->Initialize());

    // Not connected until a signal arrives.
    EXPECT_FALSE(provider_->IsConnected());

    input::UpstreamSignal signal;
    signal.isValid = true;
    signal.throttleFraction = 0.5;
    provider_->submitSignal(signal);

    EXPECT_TRUE(provider_->IsConnected());
}

// Initialize() again returns false — already-initialized guard, with an error set.
TEST_F(LiveTelemetryProviderTest, InitializeTwiceReturnsFalse) {
    ASSERT_TRUE(provider_->Initialize());
    EXPECT_FALSE(provider_->Initialize());
    EXPECT_FALSE(provider_->GetLastError().empty());
}

// OnUpdateSimulation before init returns empty input and records a not-initialized
// error (intent, not exact message).
TEST_F(LiveTelemetryProviderTest, OnUpdateBeforeInitReturnsEmptyWithError) {
    EXPECT_FALSE(provider_->IsConnected());

    input::EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_EQ(input.throttle, 0.0);
    EXPECT_EQ(input.gearAbsolute, -1);

    // Intent: error communicates the provider is not initialized.
    EXPECT_FALSE(provider_->GetLastError().empty());
    EXPECT_NE(provider_->GetLastError().find("not initialized"), std::string::npos);
}

// submitSignal + getCurrentSignal round-trip the latest signal (atomic store/load).
TEST_F(LiveTelemetryProviderTest, SubmitAndGetCurrentSignalRoundTrip) {
    ASSERT_TRUE(provider_->Initialize());

    input::UpstreamSignal signal;
    signal.throttleFraction = 0.77;
    signal.speedKmh = 50.0;
    signal.isValid = true;
    provider_->submitSignal(signal);

    input::UpstreamSignal got = provider_->getCurrentSignal();
    EXPECT_DOUBLE_EQ(got.throttleFraction, 0.77);
    EXPECT_DOUBLE_EQ(got.speedKmh, 50.0);
    EXPECT_TRUE(got.isValid);
}

// submitSignal(timed) stamps the signal's timestampUtcMs.
TEST_F(LiveTelemetryProviderTest, SubmitWithTimestampSetsTimestamp) {
    ASSERT_TRUE(provider_->Initialize());

    input::UpstreamSignal signal;
    signal.isValid = true;
    provider_->submitSignal(signal, 1700000000123ULL);

    EXPECT_EQ(provider_->getCurrentSignal().timestampUtcMs, 1700000000123ULL);
}

// Shutdown() disconnects the provider.
TEST_F(LiveTelemetryProviderTest, ShutdownDisconnects) {
    ASSERT_TRUE(provider_->Initialize());
    provider_->submitSignal(input::UpstreamSignal{});

    ASSERT_TRUE(provider_->IsConnected());
    provider_->Shutdown();
    EXPECT_FALSE(provider_->IsConnected());
}

// setGearSelector / setIgnition / provideFeedback delegate to the twin without
// crashing (observed: provider stays connected after the delegated calls).
TEST_F(LiveTelemetryProviderTest, DelegatesForwardToTwin) {
    ASSERT_TRUE(provider_->Initialize());
    provider_->submitSignal(input::UpstreamSignal{});

    // These must not throw and must not break the connection.
    EXPECT_NO_THROW({
        provider_->setGearSelector(2);
        provider_->setIgnition(true);
        EngineSimStats stats;
        provider_->provideFeedback(stats);
    });

    EXPECT_TRUE(provider_->IsConnected());
}

}  // namespace

// ============================================================================
// Stream (CSV stdin) mode — the twin is mandatory.
//
// The stream ctor LiveTelemetryProvider(istream, autoStart) MUST route CSV
// telemetry THROUGH the VirtualIceTwin, which owns gearbox/clutch/throttle
// processing. The historical bypass mapped CSV columns straight to EngineInput
// (src/input/LiveTelemetryProvider.cpp Initialize/OnUpdateSimulation stream
// branches), skipping the twin entirely: gearAutoMode was never set, the box
// never auto-shifted, the engine never cranked. These tests prove the twin is
// in the loop.
// ============================================================================

namespace {

// Owns the istringstream for the provider's lifetime (the ctor takes istream&).
struct StreamHarness {
    std::istringstream stream;
    std::unique_ptr<input::LiveTelemetryProvider> provider;
    explicit StreamHarness(const std::string& csv, bool autoStart = true)
        : stream(csv) {
        provider = std::make_unique<input::LiveTelemetryProvider>(stream, autoStart);
    }
};

constexpr int kDrive = static_cast<int>(bridge::GearSelector::DRIVE);
constexpr int kReverse = static_cast<int>(bridge::GearSelector::REVERSE);

// Pump RPM feedback so the twin's engine catches quickly (CRANKING->IDLE), then
// run until gear advances past `target` or the budget is exhausted.
int runUntilGearAbove(input::LiveTelemetryProvider& p, int target, int maxTicks) {
    int gear = -1;
    for (int i = 0; i < maxTicks; ++i) {
        EngineSimStats stats;
        stats.currentRPM = 900.0;  // > CRANK_IDLE_RPM_THRESHOLD -> engine catches
        p.provideFeedback(stats);
        gear = p.OnUpdateSimulation(0.05).gearAbsolute;
        if (gear > target) break;
    }
    return gear;
}

// T1: routing through the twin sets gearAutoMode=true (the bypass left it false).
TEST(LiveTelemetryStreamTest, TwinInLoopSetsGearAutoMode) {
    StreamHarness h("time_s,throttle_pct,road_speed_kmh\n1.0,50,30\n");
    ASSERT_TRUE(h.provider->Initialize());

    input::EngineInput in = h.provider->OnUpdateSimulation(0.05);
    EXPECT_TRUE(in.gearAutoMode)
        << "Stream mode must route through the twin, which sets gearAutoMode=true";
}

// T2: the twin's gearbox upshifts at sustained high road speed (redline guard).
// 80 km/h in 1st implies an engine speed past redline -> upshift (see AC1/F3 in
// AutoGearboxShiftLogicTest). The bypass mapped gear straight from the CSV and
// never shifted.
TEST(LiveTelemetryStreamTest, GearboxUpshiftsAtSustainedHighSpeed) {
    StreamHarness h("time_s,throttle_pct,road_speed_kmh\n0.0,100,80\n");
    ASSERT_TRUE(h.provider->Initialize());
    h.provider->setGearSelector(kDrive);

    int gear = runUntilGearAbove(*h.provider, /*target*/ 1, /*ticks*/ 200);
    EXPECT_GT(gear, 1)
        << "At 80 km/h WOT the twin must upshift out of 1st (redline guard)";
}

// T3: provideFeedback is wired. RPM feedback (>500) catches the engine so the
// starter releases (CRANKING->IDLE) within a fraction of a second. With no twin
// feedback path the bypass (autoStart off) never cranks at all, and even a twin
// without feedback takes the 3s crank fallback — releasing inside 10 ticks
// proves the RPM feedback actually reached the twin.
TEST(LiveTelemetryStreamTest, FeedbackWiredReleasesStarterOnCatch) {
    StreamHarness h("time_s,throttle_pct,road_speed_kmh\n0.0,30,5\n", /*autoStart=*/false);
    ASSERT_TRUE(h.provider->Initialize());
    h.provider->setGearSelector(kDrive);

    bool starterFired = false;
    bool starterReleased = false;
    for (int i = 0; i < 10 && !starterReleased; ++i) {
        EngineSimStats stats;
        stats.currentRPM = 900.0;
        h.provider->provideFeedback(stats);
        bool starter = h.provider->OnUpdateSimulation(0.05).starterButton;
        if (starter) starterFired = true;
        else if (starterFired) starterReleased = true;
    }
    EXPECT_TRUE(starterFired) << "Twin must crank (starter on) when telemetry is valid";
    EXPECT_TRUE(starterReleased)
        << "RPM feedback must catch the engine (starter off -> IDLE) inside the 3s fallback";
}

// T4: with no gear_selector column and no explicit setGearSelector, the provider
// must default to DRIVE so the twin reaches RUNNING and upshifts.
TEST(LiveTelemetryStreamTest, DefaultSelectorIsDriveReachesRunning) {
    StreamHarness h("time_s,throttle_pct,road_speed_kmh\n0.0,100,80\n");
    ASSERT_TRUE(h.provider->Initialize());
    // Intentionally NO setGearSelector — the default selector is under test.

    int gear = runUntilGearAbove(*h.provider, /*target*/ 1, /*ticks*/ 200);
    EXPECT_GT(gear, 1)
        << "With no gear_selector column the twin must default to DRIVE and upshift";
}

// T5: validity/timestamp guard. A blank road_speed_kmh leaves speed at the -2
// sentinel (dyno off), but the row is still valid telemetry — the provider must
// mark the signal valid with a non-zero timestamp so the twin's telemetry-timeout
// guard never fires and the engine cranks. autoStart=false isolates the twin's
// own starter from the historical bypass hack.
TEST(LiveTelemetryStreamTest, ValidityTimestampGuardKeepsTwinAlive) {
    StreamHarness h("time_s,throttle_pct,road_speed_kmh\n0.0,50,\n", /*autoStart=*/false);
    ASSERT_TRUE(h.provider->Initialize());

    bool cranked = false;
    for (int i = 0; i < 5 && !cranked; ++i) {
        if (h.provider->OnUpdateSimulation(0.05).starterButton) cranked = true;
    }
    EXPECT_TRUE(cranked)
        << "Blank speed must still be valid telemetry (non-zero ts) so the twin cranks, not time out to OFF";
}

// T6: the gear_selector column is parsed and forwarded to the twin (observable
// as the selector echoed in EngineInput), and the delegation seams are safe.
TEST(LiveTelemetryStreamTest, GearSelectorColumnForwardedAndDelegatesSafe) {
    StreamHarness h("time_s,throttle_pct,road_speed_kmh,gear_selector\n0.0,40,20,R\n");
    ASSERT_TRUE(h.provider->Initialize());

    input::EngineInput in = h.provider->OnUpdateSimulation(0.05);
    EXPECT_EQ(in.gearSelector, kReverse)
        << "gear_selector=R must be parsed and forwarded to the twin";

    EXPECT_NO_THROW({
        h.provider->setGearSelector(static_cast<int>(bridge::GearSelector::PARK));
        h.provider->setIgnition(false);
        EngineSimStats stats;
        h.provider->provideFeedback(stats);
    });
    // Before EOF the live stream is still connected.
    EXPECT_TRUE(h.provider->IsConnected());
}

// T7: deterministic engine start from CSV when cranking RPM plateaus below the
// catch threshold — the failing bench condition. Real-time CSV pacing + the
// 1-tick feedback lag mean the cranking RPM fed back to the twin frequently
// stays below the 500 RPM fast-path threshold. The twin's 3s cranking fallback
// MUST still release the starter so the engine starts, deterministically.
TEST(LiveTelemetryStreamTest, EngineStartsFromCsvWhenRpmPlateausBelowThreshold) {
    StreamHarness h("time_s,throttle_pct,road_speed_kmh\n0.0,50,5\n");
    ASSERT_TRUE(h.provider->Initialize());
    h.provider->setGearSelector(kDrive);

    // Sub-threshold RPM feedback every tick — the closed loop never crosses the
    // 500 RPM fast-path catch. dt=0.05s, so the 3s fallback lands at tick 60.
    bool starterFired = false;
    bool starterReleased = false;
    for (int i = 0; i < 80 && !starterReleased; ++i) {
        EngineSimStats stats;
        stats.currentRPM = 250.0;  // plateau below the 500 RPM catch threshold
        h.provider->provideFeedback(stats);
        bool starter = h.provider->OnUpdateSimulation(0.05).starterButton;
        if (starter) starterFired = true;
        else if (starterFired) starterReleased = true;
    }

    EXPECT_TRUE(starterFired) << "Twin must crank when valid CSV telemetry arrives";
    EXPECT_TRUE(starterReleased)
        << "Engine must start from CSV via the 3s fallback even when cranking RPM < threshold";
}

// T8: EVERY CSV data row is consumed and surfaces — no row is dropped.
//
// Regression guard for a row-dropping bug in tryReadNextRow: the 2nd+ call
// getline'd the next data row but then ran it through parseHeader, which
// resets the header (header_ = {}) and returns false on a data row (it has no
// time-column name). So row #1 surfaced and EVERY later row was consumed-and-
// discarded, currentSample_ frozen at row #1 for the whole run.
//
// The gear_selector column is the clean row-varying observable (echoed in
// EngineInput.gearSelector, NOT masked by the CRANKING throttle). A 4-row CSV
// with selectors P,R,N,D must surface all four in order, one per pump —
// proving each row is consumed.
TEST(LiveTelemetryStreamTest, EveryCsvRowIsConsumedAndSurfaces) {
    StreamHarness h(
        "time_s,throttle_pct,road_speed_kmh,gear_selector\n"
        "0.0,10,5,P\n"
        "1.0,20,10,R\n"
        "2.0,30,15,N\n"
        "3.0,40,20,D\n");
    ASSERT_TRUE(h.provider->Initialize());

    const int kP = static_cast<int>(bridge::GearSelector::PARK);
    const int kR = static_cast<int>(bridge::GearSelector::REVERSE);
    const int kN = static_cast<int>(bridge::GearSelector::NEUTRAL);
    const int kD = static_cast<int>(bridge::GearSelector::DRIVE);

    std::vector<int> seen;
    seen.reserve(4);
    for (int i = 0; i < 4; ++i) {
        h.provider->provideFeedback(EngineSimStats{});
        seen.push_back(h.provider->OnUpdateSimulation(0.05).gearSelector);
    }

    EXPECT_EQ(seen, (std::vector<int>{kP, kR, kN, kD}))
        << "Each CSV row must surface in order (was: row #1 frozen, later rows dropped)";
}

}  // namespace
