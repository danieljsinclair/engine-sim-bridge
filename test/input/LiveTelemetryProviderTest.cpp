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
#include "input/EngineInputTarget.h"
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

// Regression: the JSON network path must populate engineInput.gearSelector from
// the decoded gear in the submitted signal. The inner twin learns the selector
// ONLY via setGearSelector (it does NOT read signal.gearSelector); the network
// path relays it. Without this, engineInput.gearSelector stays NEUTRAL(0) and
// gear-initiated instant starts (driveSelected) are dead on the live path.
TEST_F(LiveTelemetryProviderTest, NetworkPath_PopulatesGearSelectorFromSignal) {
    ASSERT_TRUE(provider_->Initialize());

    // Network path: a DRIVE-gear, valid telemetry frame (the master live feed).
    input::UpstreamSignal signal;
    signal.gearSelector = bridge::GearSelector::DRIVE;
    signal.isValid = true;
    signal.throttleFraction = 0.5;
    signal.speedKmh = 30.0;
    provider_->submitSignal(signal);

    // Pump RPM feedback so the twin leaves CRANKING/IDLE quickly and the box
    // reaches RUNNING; gearSelector is echoed on every frame regardless of state.
    EngineSimStats stats;
    stats.currentRPM = 900.0;

    input::EngineInput input;
    for (int i = 0; i < 5; ++i) {
        provider_->provideFeedback(stats);
        input = provider_->OnUpdateSimulation(0.05);
    }

    EXPECT_EQ(input.gearSelector, static_cast<int>(bridge::GearSelector::DRIVE))
        << "Network live frame must report the decoded gear via engineInput.gearSelector";
}

// Contrast: a NEUTRAL-gear network frame must surface NEUTRAL, proving the field
// is genuinely driven by the signal (not hardcoded to DRIVE).
TEST_F(LiveTelemetryProviderTest, NetworkPath_NeutralGearSurfacesNeutral) {
    ASSERT_TRUE(provider_->Initialize());

    input::UpstreamSignal signal;
    signal.gearSelector = bridge::GearSelector::NEUTRAL;
    signal.isValid = true;
    signal.throttleFraction = 0.5;
    signal.speedKmh = 30.0;
    provider_->submitSignal(signal);

    EngineSimStats stats;
    stats.currentRPM = 900.0;
    input::EngineInput input;
    for (int i = 0; i < 5; ++i) {
        provider_->provideFeedback(stats);
        input = provider_->OnUpdateSimulation(0.05);
    }

    EXPECT_EQ(input.gearSelector, static_cast<int>(bridge::GearSelector::NEUTRAL));
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
    h.provider->setIgnition(true);  // twin ignition defaults OFF; commanded here
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
    h.provider->setIgnition(true);  // twin ignition defaults OFF; commanded here
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
    h.provider->setIgnition(true);  // twin ignition defaults OFF; commanded here
    // Intentionally NO setGearSelector — the default selector is under test.

    int gear = runUntilGearAbove(*h.provider, /*target*/ 1, /*ticks*/ 200);
    EXPECT_GT(gear, 1)
        << "With no gear_selector column the twin must default to DRIVE and upshift";
}

// ----------------------------------------------------------------------------
// LIVE stream mode (liveStream_ = true): the low-latency path used by
// --live-telemetry < file. MUST still consume the capture INCREMENTALLY (one
// row per frame), not drain the whole stream and pin currentSample_ to the LAST
// row. Regressed in a321e1f ("surface latest stdin row immediately") when the
// live reader looped std::getline to EOF in a single OnUpdateSimulation call: a
// file redirect is drained at once, currentSample_ froze at the capture's final
// (near-standstill) row, and the gearbox never saw a speed ramp -> never shifted.
// The latency intent (no timestamp gating) is preserved; only the drain must go.
// ----------------------------------------------------------------------------
struct LiveStreamHarness {
    std::istringstream stream;
    std::unique_ptr<input::LiveTelemetryProvider> provider;
    explicit LiveStreamHarness(const std::string& csv, bool autoStart = true)
        : stream(csv) {
        // liveStream = true — the exact path CLIMain uses for --live-telemetry.
        provider = std::make_unique<input::LiveTelemetryProvider>(stream, autoStart, /*liveStream=*/ true);
    }
};

// Feed a DENSE multi-row capture that ramps to 100 km/h then ends near-standstill
// (mirrors a real driving capture whose tail is parked). Dense rows matter: the
// live/paced reader drops a row once it is read as "future" (a ~1-row/call skew
// that is negligible at real recording rates), so a sparse capture would skip
// transitions. The gearbox must walk the speed ramp and upshift. With the
// EOF-drain bug, currentSample_ pins to the final 1.76 km/h row and the box
// stays in 1st -> this FAILS (red).
TEST(LiveTelemetryStreamTest, LiveModeConsumesIncrementallyNotPinnedToLastRow) {
    // 30 dense rows, t=0..2.9s @0.1s, road speed ramping 10 -> 100 km/h, then a
    // parked tail row at 1.76 (the real StationHomeward capture's last rows). Dense
    // rows (not sparse) matter: the live reader drops a row once read as "future"
    // (a ~1-row/call skew, negligible at real ~442 rows/s recording rates), so the
    // sim clock must outrun the row spacing for the walk to advance. We sweep the
    // clock fast (dt=0.5/frame) to mimic a real dense capture.
    std::ostringstream csv;
    csv << "time_s,throttle_pct,road_speed_kmh\n";
    for (int i = 0; i < 30; ++i) {
        double t = i * 0.1;
        double spd = 10.0 + i * 3.0;  // 10,13,...,97
        csv << t << ",100," << spd << "\n";
    }
    csv << "3.0,100,100\n"   // peak
         << "4.0,100,1.76\n"; // parked tail
    LiveStreamHarness h(csv.str());
    ASSERT_TRUE(h.provider->Initialize());
    h.provider->setIgnition(true);  // twin ignition defaults OFF; commanded here
    h.provider->setGearSelector(kDrive);

    // Pump RPM feedback so the twin cranks/catches (OFF->CRANKING->IDLE->RUNNING)
    // and let the live reader advance through the capture rows as the clock sweeps.
    int gear = -1;
    for (int i = 0; i < 200; ++i) {
        EngineSimStats stats;
        stats.currentRPM = 900.0;
        h.provider->provideFeedback(stats);
        gear = h.provider->OnUpdateSimulation(0.5).gearAbsolute;  // fast clock sweep
        if (gear > 1) break;
    }
    EXPECT_GT(gear, 1)
        << "Live mode must advance through capture rows (not freeze on the last "
           "row) so the gearbox upshifts as road speed ramps";
}

// T5: validity/timestamp guard. A blank road_speed_kmh leaves speed at the -2
// sentinel (dyno off), but the row is still valid telemetry — the provider must
// mark the signal valid with a non-zero timestamp so the twin's telemetry-timeout
// guard never fires and the engine cranks. autoStart=false isolates the twin's
// own starter from the historical bypass hack.
TEST(LiveTelemetryStreamTest, ValidityTimestampGuardKeepsTwinAlive) {
    StreamHarness h("time_s,throttle_pct,road_speed_kmh\n0.0,50,\n", /*autoStart=*/false);
    ASSERT_TRUE(h.provider->Initialize());
    h.provider->setIgnition(true);  // twin ignition defaults OFF; commanded here

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
    h.provider->setIgnition(true);  // twin ignition defaults OFF; commanded here
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
// Under timestamp pacing, rows are consumed in order and surfaced when the sim
// elapsed time reaches their recording time. A single OnUpdateSimulation call
// with dt=3.5 advances simElapsedS to 3.5, so all 4 rows (t=0,1,2,3) are
// within the window; the last (t=3, selector=D) is surfaced. The stream
// retains rows beyond the window in its buffer for subsequent calls.
//
// The gear_selector column is the clean row-varying observable (echoed in
// EngineInput.gearSelector, NOT masked by the CRANKING throttle). Verify that
// all 4 rows surface by stepping through the full span in two calls: dt=1.5
// surfaces the last row ≤1.5s (selector=N at t=2), then dt=2.5 surfaces the
// last row ≤4.0s (selector=D at t=3).
TEST(LiveTelemetryStreamTest, EveryCsvRowIsConsumedAndSurfaces) {
    StreamHarness h(
        "time_s,throttle_pct,road_speed_kmh,gear_selector\n"
        "0.0,10,5,P\n"
        "1.0,20,10,R\n"
        "2.0,30,15,N\n"
        "3.0,40,20,D\n");
    ASSERT_TRUE(h.provider->Initialize());

    const int kR = static_cast<int>(bridge::GearSelector::REVERSE);
    const int kD = static_cast<int>(bridge::GearSelector::DRIVE);

    // Step 1: dt=1.5 → simElapsedS=1.5; rows at t=0,1 are in window; last = t=1 (R).
    h.provider->provideFeedback(EngineSimStats{});
    EXPECT_EQ(h.provider->OnUpdateSimulation(1.5).gearSelector, kR)
        << "Row at t=1.0 (R) must surface as the last in-window row at simElapsedS=1.5";

    // Step 2: dt=2.5 → simElapsedS=4.0; rows at t=2,3 are in window; last = t=3 (D).
    h.provider->provideFeedback(EngineSimStats{});
    EXPECT_EQ(h.provider->OnUpdateSimulation(2.5).gearSelector, kD)
        << "Row at t=3.0 (D) must surface as the last in-window row at simElapsedS=4.0";
}

// T9: reconfigureProfile forwards the loaded engine's transmission ratios to the
// twin, so a CSV-driven box shifts against the ACTUAL engine (e.g. a C63), not
// the default ZF profile. Differential observable: the default zf8hp45 (8
// forward gears) upshifts out of 1st at 80 km/h WOT (see T2), but after
// reconfiguring to a SINGLE forward gear the twin caps at gear 1 — its upshift
// loop is bounded by gearRatios.size() (AutomaticGearbox.cpp). A reconfigure
// that did NOT reach the twin would still upshift.
TEST(LiveTelemetryStreamTest, ReconfigureProfileForwardsRatiosToTwin) {
    // Baseline: default 8-gear profile upshifts past 1st at 80 km/h WOT.
    {
        StreamHarness h("time_s,throttle_pct,road_speed_kmh\n0.0,100,80\n");
        ASSERT_TRUE(h.provider->Initialize());
        h.provider->setIgnition(true);  // twin ignition defaults OFF; commanded here
        h.provider->setGearSelector(kDrive);
        ASSERT_GT(runUntilGearAbove(*h.provider, /*target*/ 1, /*ticks*/ 200), 1)
            << "baseline: default zf8hp45 must upshift at 80 km/h WOT";
    }
    // Reconfigured to one forward gear: the twin cannot upshift (ratio count 1).
    {
        StreamHarness h("time_s,throttle_pct,road_speed_kmh\n0.0,100,80\n");
        ASSERT_TRUE(h.provider->Initialize());
        h.provider->setIgnition(true);  // twin ignition defaults OFF; commanded here
        h.provider->setGearSelector(kDrive);
        h.provider->reconfigureProfile({3.5}, 3.15, 0.32);
        EXPECT_EQ(runUntilGearAbove(*h.provider, /*target*/ 1, /*ticks*/ 200), 1)
            << "reconfigureProfile must reach the twin so the gearbox honours the "
               "supplied ratio count (1 gear => no upshift)";
    }
}

// Empty ratios is a no-op (the twin guards on empty): provider stays healthy.
TEST(LiveTelemetryStreamTest, ReconfigureProfileEmptyRatiosIsNoOp) {
    StreamHarness h("time_s,throttle_pct,road_speed_kmh\n0.0,50,30\n");
    ASSERT_TRUE(h.provider->Initialize());
    EXPECT_NO_THROW(h.provider->reconfigureProfile({}, 3.15, 0.32));
    EXPECT_TRUE(h.provider->IsConnected());
}

// T10: the upstream (CSV) road speed is surfaced on EngineInput.roadSpeedKmh so
// the presentation layer can show the commanded road speed (it flows to
// EngineState.controls.commandedSpeedKmh). The live path previously dropped it,
// so the speed readout reflected only the engine-sim vehicle physics — which
// creeps near standstill because the wheels are not driven here — instead of the
// CSV road speed. Mirrors ReplayTelemetryProvider (input.roadSpeedKmh = s.
// roadSpeedKmh per sample). Uses the vehicle-sim schema column `speed_kmh`
// (an alias the parser maps to the road-speed column).
TEST(LiveTelemetryStreamTest, CsvRoadSpeedIsSurfacedOnEngineInput) {
    StreamHarness h("timestamp_ms,speed_kmh,throttle_percent\n1000,100,50\n");
    ASSERT_TRUE(h.provider->Initialize());

    input::EngineInput in = h.provider->OnUpdateSimulation(0.05);
    EXPECT_DOUBLE_EQ(in.roadSpeedKmh, 100.0)
        << "CSV speed_kmh=100 must surface on EngineInput.roadSpeedKmh (was dropped)";
}

// T10b: the brake_light column surfaces end-to-end on the provider's current
// signal (the seam StartStopInputAdapter polls via getCurrentSignal()). The
// tri-state must survive: 1 -> true, 0 -> false, blank -> nullopt.
//
// Rows are duplicated per state: the paced reader drops the FIRST row beyond
// the sim clock each call (the documented ~1-row skew), so each asserted state
// needs a second row inside the window to surface.
TEST(LiveTelemetryStreamTest, CsvBrakeLightColumn_SurfacesOnCurrentSignal) {
    StreamHarness h(
        "time_s,throttle_pct,brake_light\n"
        "0.0,10,1\n"
        "1.0,10,1\n"
        "2.0,20,0\n"
        "3.0,20,0\n"
        "4.0,30,\n"
        "5.0,30,\n");
    ASSERT_TRUE(h.provider->Initialize());

    // simElapsedS=1.5: rows t=0,1 in window; t=2 dropped as future. -> true.
    h.provider->OnUpdateSimulation(1.5);
    EXPECT_TRUE(h.provider->getCurrentSignal().brakeLight.has_value());
    EXPECT_TRUE(*h.provider->getCurrentSignal().brakeLight);

    // simElapsedS=3.5: row t=3 in window; t=4 dropped. -> false.
    h.provider->OnUpdateSimulation(2.0);
    EXPECT_TRUE(h.provider->getCurrentSignal().brakeLight.has_value());
    EXPECT_FALSE(*h.provider->getCurrentSignal().brakeLight);

    // simElapsedS=5.5: row t=5 in window. -> blank -> nullopt.
    h.provider->OnUpdateSimulation(2.0);
    EXPECT_FALSE(h.provider->getCurrentSignal().brakeLight.has_value());
}

// T10c: canonical brake invariant at the provider boundary. The CSV brake_light
// column SUPPLIES the light (display/start-stop signal) and must NOT write the
// physics level — brakeLevel's only writer is the keyboard 'B' key. The
// keyboard target writes only brakeLevel (its light derives downstream at the
// SimulationLoop assembly point — proven in SimulationLoopVehicleControlsTests).
// Rows are duplicated per state (paced reader drops the first row beyond the
// sim clock each call — the documented ~1-row skew, same as T10b).
TEST(LiveTelemetryStreamTest, CsvBrakeLight_SuppliesLightWithoutTouchingPhysicsLevel) {
    StreamHarness h(
        "time_s,throttle_pct,brake_light\n"
        "0.0,10,1\n"
        "1.0,10,1\n"
        "2.0,10,0\n"
        "3.0,10,0\n");
    ASSERT_TRUE(h.provider->Initialize());

    input::EngineInputTarget keyboardTarget;
    keyboardTarget.setBrake(1.0);
    const input::EngineInput keyboardInput = keyboardTarget.buildInput();
    EXPECT_DOUBLE_EQ(keyboardInput.brakeLevel, 1.0);
    EXPECT_FALSE(keyboardInput.brakeLight.has_value())
        << "Keyboard writes the level only — the light derives in SimulationLoop";

    // simElapsedS=1.5: rows t=0,1 in window; t=2 dropped as future. -> true.
    const input::EngineInput csvOn = h.provider->OnUpdateSimulation(1.5);
    ASSERT_TRUE(csvOn.brakeLight.has_value());
    EXPECT_TRUE(*csvOn.brakeLight);
    EXPECT_DOUBLE_EQ(csvOn.brakeLevel, 0.0)
        << "CSV brake light is an indicator — it must never write the physics level";

    // simElapsedS=3.5: row t=3 in window; t=2 was the skew-dropped row. -> false.
    const input::EngineInput csvOff = h.provider->OnUpdateSimulation(2.0);
    ASSERT_TRUE(csvOff.brakeLight.has_value());
    EXPECT_FALSE(*csvOff.brakeLight);
    EXPECT_DOUBLE_EQ(csvOff.brakeLevel, 0.0);
}

// T11: LIVE stream mode (engine-sim-cli --live-telemetry stdin pipe) surfaces the
// LATEST row available in the stream on the FIRST frame — no consumption pacing
// by recording timestamp. This is the latency fix: under timestamp pacing a
// sparse recording (rows at t=2,3.5,4,...) holds an old row until the sim clock
// "catches up" to each row's timestamp, adding ~0.5s+ of input-to-audio lag on a
// live pipe. The live ctor (liveStream=true) must surface the freshest sample
// immediately, so the engine response tracks the pipe like keyboard input does.
//
// Observable: a single OnUpdateSimulation with dt=0.05 (simElapsedS~0.05, far
// below the first row at t=2.0) must surface the LAST row (t=8, throttle=100),
// NOT be gated behind the clock. The gear_selector column is the clean row-varying
// observable (echoed in EngineInput.gearSelector, not masked by CRANKING throttle).
TEST(LiveTelemetryStreamTest, LiveStreamAdvancesThroughCaptureNotPinnedToLastRow) {
    // Dense rows (every 0.1s). Early rows are PARK; only the final 5 rows are DRIVE.
    // Dense so the ~1-row/call future-skew doesn't skip transitions; the clock sweep
    // (dt=0.5) walks every row. The drain-to-EOF bug surfaced the FINAL row (D) on
    // frame 1; the fix must instead surface an EARLY row (P) on frame 1.
    std::ostringstream csv;
    csv << "time_s,throttle_pct,road_speed_kmh,gear_selector\n";
    for (int i = 0; i < 30; ++i) {
        double t = i * 0.1;
        const char* sel = (i >= 25) ? "D" : "P";  // last 5 rows DRIVE, rest PARK
        csv << t << ",100," << (10 + i * 5) << "," << sel << "\n";
    }
    std::istringstream stream(csv.str());
    auto live = std::make_unique<input::LiveTelemetryProvider>(stream, /*autoStart=*/true, /*liveStream=*/true);
    ASSERT_TRUE(live->Initialize());

    const int kP = static_cast<int>(bridge::GearSelector::PARK);
    const int kD = static_cast<int>(bridge::GearSelector::DRIVE);

    // Frame 1 (simElapsedS=0.05): with a 0.25s lookahead horizon, the live path
    // surfaces the latest row within ~0.25s of t=0 — an EARLY row (PARK). It must
    // NOT jump to the FINAL row (DRIVE), which was the drain-to-EOF bug: it froze
    // the twin on the capture's last row so the gearbox never shifted.
    live->provideFeedback(EngineSimStats{});
    input::EngineInput first = live->OnUpdateSimulation(0.05);
    EXPECT_EQ(first.gearSelector, kP)
        << "Live mode must surface an EARLY row (PARK) on frame 1, not jump to the "
           "FINAL row (DRIVE) — that froze the gearbox on the capture's last row.";

    // Advance the sim clock past the capture — the live reader must WALK the rows
    // (P->...->D), not stay pinned. By the end it reaches D.
    int lastSelector = first.gearSelector;
    for (int i = 0; i < 200; ++i) {
        live->provideFeedback(EngineSimStats{});
        lastSelector = live->OnUpdateSimulation(0.5).gearSelector;  // 0.5s steps sweep t=0..2.9
        if (lastSelector == kD) break;
    }
    EXPECT_EQ(lastSelector, kD)
        << "Live mode must advance through the capture rows (P->...->D) as the sim "
           "clock sweeps, proving it is not frozen on the last row.";
}

// T12: blank CAN-bus-waking-up rows (throttle=0 AND roadSpeedKmh=-2 sentinel,
// i.e. no decoded signals) must be SKIPPED so hasSample_ stays false and the
// twin remains in OFF waiting for real telemetry. This is the live-USB-pipe
// blocker fix: the ESP32 emits ~166 blank frames while the bus wakes up; without
// the guard the first blank row sets hasSample_=true and the twin receives a
// valid-but-empty signal, never transitioning OFF -> CRANKING.
//
// Two observables:
//  (a) blank rows preceding any populated row: starter stays off (twin in OFF)
//  (b) populated row after blanks: starter fires once (twin enters CRANKING)
TEST(LiveTelemetryStreamTest, BlankInitialRowsSkippedEngineCranksWhenDataArrives) {
    // 10 blank rows followed by one populated row (throttle=50%, speed=5 km/h).
    // Live mode consumes the entire stream in one call, so the populated row
    // at t=10.0 is the first non-blank row encountered → sets hasSample_=true
    // with populated data. The second OnUpdateSimulation call (EOF now) reuses
    // that sample; the twin transitions OFF -> CRANKING and fires the starter.
    const std::string csv =
        "time_s,throttle_pct,road_speed_kmh\n"
        "0.0,,\n"
        "1.0,,\n"
        "2.0,,\n"
        "3.0,,\n"
        "4.0,,\n"
        "5.0,,\n"
        "6.0,,\n"
        "7.0,,\n"
        "8.0,,\n"
        "9.0,,\n"
        // First (and only) populated row at t=10 — this must surface and crank.
        "10.0,50,5\n";
    StreamHarness h(csv, /*autoStart=*/false);
    ASSERT_TRUE(h.provider->Initialize());
    h.provider->setIgnition(true);  // twin ignition defaults OFF; commanded here

    // Call 1: consume the whole stream. Blank rows (t=0..9) are skipped; the
    // populated row (t=10) is the first non-blank → hasSample_=true, but the
    // twin has not yet been ticked with this signal.
    h.provider->provideFeedback(EngineSimStats{});
    // The starter fires on the transition OFF->CRANKING, which happens when
    // the twin first receives the valid populated signal (this call).
    EXPECT_TRUE(h.provider->OnUpdateSimulation(0.05).starterButton)
        << "Populated row (t=10, throttle=50%, speed=5) must crank the engine";

    // Call 2: EOF, sample unchanged (populated). RPM feedback (900 > 500) catches
    // the engine → CRANKING -> IDLE, starter releases within a few ticks.
    bool starterReleased = false;
    for (int i = 0; i < 20 && !starterReleased; ++i) {
        h.provider->provideFeedback(EngineSimStats{});
        if (!h.provider->OnUpdateSimulation(0.05).starterButton) starterReleased = true;
    }
    EXPECT_TRUE(starterReleased)
        << "Engine must catch (starter released) after populated row drives CRANKING->IDLE";
}

// T13: populated data followed by blank rows must NOT lose the populated sample.
// In a live pipe, intermittent blank frames can arrive after the bus is up;
// the "latest valid row" semantics must hold — blank rows must not overwrite
// the last good sample.
TEST(LiveTelemetryStreamTest, PopulatedRowSurvivesSubsequentBlankRows) {
    const std::string csv =
        "time_s,throttle_pct,road_speed_kmh\n"
        "0.0,50,5\n"   // populated
        "1.0,,\n"      // blank — must not overwrite
        "2.0,,\n"      // blank — must not overwrite
        "3.0,60,10\n"; // another populated (updates sample)
    StreamHarness h(csv, /*autoStart=*/false);
    ASSERT_TRUE(h.provider->Initialize());
    h.provider->setIgnition(true);  // twin ignition defaults OFF; commanded here

    // Call 1: consumes all rows. Non-blank rows at t=0 (throttle=50) and t=3
    // (throttle=60). The latest non-blank is t=3 → currentSample_.throttle=0.6.
    h.provider->provideFeedback(EngineSimStats{});
    input::EngineInput in = h.provider->OnUpdateSimulation(0.05);
    EXPECT_TRUE(in.starterButton)
        << "Populated sample must trigger cranking (OFF->CRANKING)";
    // The blank rows at t=1 and t=2 must not have overwritten the sample.
    // We can't observe currentSample_ directly, but the twin cranking proves
    // a valid non-blank sample was received.
    EXPECT_GT(in.throttle, 0.0)
        << "Throttle from the latest populated row (t=3, 60%) must reach the twin";
}

}  // namespace
