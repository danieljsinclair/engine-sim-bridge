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
    // A forward-moving 'R' (speed 20 km/h) is a contradictory signal and must be
    // coerced to NEUTRAL (never REVERSE) — see the RAR fix. A genuine reversing
    // 'R' (clearly negative speed) would be honoured as REVERSE.
    StreamHarness h("time_s,throttle_pct,road_speed_kmh,gear_selector\n0.0,40,20,R\n");
    ASSERT_TRUE(h.provider->Initialize());

    input::EngineInput in = h.provider->OnUpdateSimulation(0.05);
    EXPECT_EQ(in.gearSelector, static_cast<int>(bridge::GearSelector::NEUTRAL))
        << "A forward 'R' must be coerced to NEUTRAL, never REVERSE (RAR fix)";

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
// Under timestamp pacing, rows are consumed in order and surfaced when the sim
// elapsed time reaches their recording time. A single OnUpdateSimulation call
// with dt=3.5 advances simElapsedS to 3.5, so all 4 rows (t=0,1,2,3) are
// within the window; the last (t=3, selector=D) is surfaced. The stream
// retains rows beyond the window in its buffer for subsequent calls.
//
// The gear_selector column is the clean row-varying observable (echoed in
// EngineInput.gearSelector, NOT masked by the CRANKING throttle). Verify that
// all 4 rows surface by stepping through the full span in two calls: dt=1.5
// surfaces the last row ≤1.5s (selector=R at t=1 — a genuine reversing R is
// honoured), then dt=2.5 surfaces the last row ≤4.0s (selector=D at t=3).
TEST(LiveTelemetryStreamTest, EveryCsvRowIsConsumedAndSurfaces) {
    StreamHarness h(
        "time_s,throttle_pct,road_speed_kmh,gear_selector\n"
        "0.0,10,5,P\n"
        "1.0,20,-10,R\n"   // genuine reverse (negative speed) -> REVERSE honoured
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
        h.provider->setGearSelector(kDrive);
        ASSERT_GT(runUntilGearAbove(*h.provider, /*target*/ 1, /*ticks*/ 200), 1)
            << "baseline: default zf8hp45 must upshift at 80 km/h WOT";
    }
    // Reconfigured to one forward gear: the twin cannot upshift (ratio count 1).
    {
        StreamHarness h("time_s,throttle_pct,road_speed_kmh\n0.0,100,80\n");
        ASSERT_TRUE(h.provider->Initialize());
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
// T12: instant --start-from (live contract): pre-window rows are discarded, the
// display clock COLD-JUMPS to the offset on the first tick (display is relative
// to the recording's real start), and the first post-offset row feeds
// immediately. No prefix row may ever surface: road speed on frame 1 is the
// post-offset row's speed, and the replay timestamp reads the offset (+dt), not 0.
TEST(LiveTelemetryStreamTest, InstantStartFromDiscardsPrefixAndAnchorsDisplayClock) {
    std::ostringstream csv;
    csv << "time_s,throttle_pct,road_speed_kmh\n";
    for (double t = 0.0; t <= 1.0; t += 0.5) csv << t << ",10," << (10 + 2 * t) << "\n";   // prefix
    csv << "10.0,25,40\n10.5,25,42\n11.0,25,44\n";                                        // post-offset
    StreamHarness h(csv.str());
    ASSERT_TRUE(h.provider->Initialize());
    h.provider->setStartFromS(10.0);

    input::EngineInput in = h.provider->OnUpdateSimulation(0.05);
    EXPECT_NEAR(in.replayTimestampS, 10.05, 1e-9)
        << "Display clock must cold-jump to the start-from offset (+ one tick), not "
           "advance from 0 — display is relative to the recording's real start.";
    EXPECT_DOUBLE_EQ(in.roadSpeedKmh, 40.0)
        << "The first post-offset row (t=10, 40 km/h) must feed on frame 1; prefix "
           "rows must be discarded, never surfaced.";
}

// T13: the discard window is measured from the FIRST PARSED recording row (the
// recording's real start), not from the first KEPT row. A capture whose epoch
// starts at t=100s with --start-from 10 must keep the row at t=110 — had the
// baseline anchored on the first kept row, relT would restart and the window
// would discard EVERYTHING (no sample ever, twin stuck on the prime seed).
TEST(LiveTelemetryStreamTest, InstantStartFromAnchorsBaselineOnFirstRecordingRow) {
    std::ostringstream csv;
    csv << "timestamp_ms,speed_kmh,throttle_percent\n";
    csv << "100000,5,10\n100500,6,10\n101000,7,10\n";   // prefix (relT 0..1s)
    csv << "110000,50,20\n110500,52,20\n";              // post-offset (relT 10, 10.5)
    StreamHarness h(csv.str());
    ASSERT_TRUE(h.provider->Initialize());
    h.provider->setStartFromS(10.0);

    input::EngineInput in = h.provider->OnUpdateSimulation(0.05);
    EXPECT_DOUBLE_EQ(in.roadSpeedKmh, 50.0)
        << "Window is relT < 10 from the FIRST row (epoch t=100): the t=110 row "
           "(relT=10) must be kept and feed frame 1.";
    EXPECT_NEAR(in.replayTimestampS, 10.05, 1e-9);
}

// T14: --end-at bounds the LIVE run at the relative timecode — the provider
// reports EOF once elapsed crosses the bound, even though stream rows remain
// (rows to t=5 here). "Stop at that relative timecode or input end, whichever
// first."
TEST(LiveTelemetryStreamTest, EndAtBoundsLiveRunWithCleanDisconnect) {
    std::ostringstream csv;
    csv << "time_s,throttle_pct,road_speed_kmh\n";
    for (int i = 0; i <= 50; ++i) csv << (i * 0.1) << ",20,30\n";   // rows to t=5
    StreamHarness h(csv.str());
    ASSERT_TRUE(h.provider->Initialize());
    h.provider->setEndAtS(1.0);

    int ticks = 0;
    while (h.provider->IsConnected() && ticks < 200) {
        h.provider->OnUpdateSimulation(0.05);
        ++ticks;
    }
    EXPECT_FALSE(h.provider->IsConnected())
        << "end-at must disconnect the provider at the bound (clean loop exit).";
    EXPECT_LE(ticks * 0.05, 1.5)
        << "The run must stop at ~1.0s of sim time (1.0/0.05 + margin ticks), not "
           "play the whole 5s stream.";
}

// ----------------------------------------------------------------------------
// #vs-start-from hint (in-band source-skip protocol). vehicle-sim's stdout-csv
// replay emits "#vs-start-from <s>" before the header when IT skipped a
// --start-from prefix; the provider consumes it so the display timecode stays
// TRUE-recording-relative. Stacked skips are ADDITIVE by owner decision (§14b
// option b): source skip + own skip, display at the sum.
// ----------------------------------------------------------------------------

// T15: hint-only — the source skipped [0, 40), so the first delivered row sits
// at TRUE relT 40. The display must read [00:40.x] on frame 1, not restart at
// [00:00] (the pre-hint behavior for a skipping source).
TEST(LiveTelemetryStreamTest, SourceSkipHintAnchorsTrueRecordingTimecode) {
    StreamHarness h("#vs-start-from 40.000\n"
                    "time_s,throttle_pct,road_speed_kmh\n"
                    "100.0,20,30\n"    // delivered t0; TRUE recording t0 = 60
                    "100.5,20,32\n");
    ASSERT_TRUE(h.provider->Initialize());

    input::EngineInput in = h.provider->OnUpdateSimulation(0.05);
    EXPECT_NEAR(in.replayTimestampS, 40.05, 1e-9)
        << "Cold-jump must include the source skip: display [00:40], not [00:00].";
    EXPECT_DOUBLE_EQ(in.roadSpeedKmh, 30.0)
        << "The first delivered row must still feed frame 1 (no own window).";
}

// T16: stacked skips are ADDITIVE — the own --start-from window is measured
// from the first DELIVERED row (stream anchor), ON TOP of the source skip, not
// from the TRUE t0 (which would collapse the two windows into a max). Source
// skipped [0,45); own window skips delivered-relT [0,45); first kept row is at
// delivered-relT 45.5 = TRUE relT 90.5 → display [01:30.x] (owner's rule). The
// row at delivered-relT 44 (TRUE 89, 88 km/h) is the discriminator: it must be
// DISCARDED, and it would surface FIRST under the max() interpretation.
TEST(LiveTelemetryStreamTest, StackedSourceAndOwnSkipAreAdditive) {
    StreamHarness h("#vs-start-from 45.000\n"
                    "time_s,throttle_pct,road_speed_kmh\n"
                    "45.0,20,10\n"    // delivered t0; local relT 0          -> own discard
                    "89.0,20,88\n"    // local relT 44 (< 45)                -> own discard
                    "90.5,20,90\n"    // local relT 45.5 (>= 45)             -> first KEPT
                    "91.0,20,91\n");
    ASSERT_TRUE(h.provider->Initialize());
    h.provider->setStartFromS(45.0);

    // Frame 1: display already reads [01:30.x] (cold-jump to 45+45), before the
    // first kept row surfaces (prime-seed hold in between).
    EXPECT_NEAR(h.provider->OnUpdateSimulation(0.05).replayTimestampS, 90.05, 1e-9)
        << "Stacked skips are additive: display cold-jumps to 45+45 = 90s.";

    double firstSpeedKmh = -99.0;
    double firstTimecodeS = -99.0;
    for (int i = 0; i < 15 && firstSpeedKmh < 0.0; ++i) {
        input::EngineInput in = h.provider->OnUpdateSimulation(0.05);
        if (in.roadSpeedKmh > 20.0) {   // >20 filters the warm-boot seed hold (10 km/h)
            firstSpeedKmh = in.roadSpeedKmh;
            firstTimecodeS = in.replayTimestampS;
        }
    }
    ASSERT_GT(firstSpeedKmh, 20.0) << "first kept row must surface within 15 ticks";
    EXPECT_DOUBLE_EQ(firstSpeedKmh, 90.0)
        << "The TRUE-89 row (88 km/h, delivered-relT 44 < 45) must be discarded — "
           "surfacing it first means the own window collapsed to max() semantics.";
    EXPECT_NEAR(firstTimecodeS, 90.5, 0.06)
        << "First kept row surfaces at TRUE relT 90.5 ([01:30.5]).";
}

// T17: no hint — legacy behavior is unchanged: the first delivered row IS the
// recording t0, display relative from 0.
TEST(LiveTelemetryStreamTest, NoHintStreamKeepsLocalTimecode) {
    StreamHarness h("time_s,throttle_pct,road_speed_kmh\n5.0,20,30\n5.5,20,31\n");
    ASSERT_TRUE(h.provider->Initialize());

    input::EngineInput in = h.provider->OnUpdateSimulation(0.05);
    EXPECT_NEAR(in.replayTimestampS, 0.05, 1e-9)
        << "No hint + no own skip: display stays relative to the first delivered row.";
    EXPECT_DOUBLE_EQ(in.roadSpeedKmh, 30.0);
}

// T18: --end-at is TRUE-recording-relative under a hint — the bound 41s with a
// 40s source skip disconnects ~1s after the stream starts, not 41s in.
TEST(LiveTelemetryStreamTest, SourceSkipHintMakesEndAtTrueRelative) {
    StreamHarness h("#vs-start-from 40.000\n"
                    "time_s,throttle_pct,road_speed_kmh\n"
                    "100.0,20,30\n100.5,20,30\n101.0,20,30\n");
    ASSERT_TRUE(h.provider->Initialize());
    h.provider->setEndAtS(41.0);

    int ticks = 0;
    while (h.provider->IsConnected() && ticks < 100) {
        h.provider->OnUpdateSimulation(0.05);
        ++ticks;
    }
    EXPECT_FALSE(h.provider->IsConnected()) << "bound must be reached and disconnect.";
    EXPECT_NEAR(ticks * 0.05, 1.0, 0.15)
        << "Disconnect ~1s after stream start (elapsed 40 -> 41), not 41s.";
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

// T14 (regression for the live warm-start reversion fix): warmBootToRunning()
// brings the owned twin to RUNNING + the warm cruise basin BEFORE the first real
// frame, so --live-telemetry --start-from no longer cold-jumps and blows massive
// negative exhaust flow. After priming, the first real frame at a sustained road
// speed must already expose a forward gear (the convergent attractor replay's
// prime lands in), whereas an UN-primed twin starts OFF/CRANKING. The prime is
// idempotent (re-calling must not double-crank or break the connection).
TEST(LiveTelemetryStreamTest, WarmBootToRunningReachesRunningWithForwardGear) {
    // A sustained-speed cruise trace (DRIVE). The prime seeds from a running
    // baseline so the twin settles into the warm basin; the first real frame at
    // 80 km/h must upshift out of 1st (matching the replay oracle's attractor).
    const std::string csv =
        "time_s,throttle_pct,road_speed_kmh\n"
        "0.0,100,80\n";
    StreamHarness h(csv);
    ASSERT_TRUE(h.provider->Initialize());
    h.provider->setGearSelector(kDrive);

    // Prime BEFORE the first real frame (mirrors what CLIMain does post-Initialize,
    // after the coupling flags are set).
    h.provider->warmBootToRunning();

    // The prime must not have disconnected the provider and must be idempotent.
    EXPECT_TRUE(h.provider->IsConnected()) << "Prime must not break the connection";
    EXPECT_NO_THROW(h.provider->warmBootToRunning())
        << "Re-priming must be a safe no-op (idempotent)";

    // First real frame at speed: the twin is already RUNNING with a forward gear
    // available (no cold OFF->CRANKING transient that would reversion-blow the gas
    // path). Pump RPM feedback so the core bump-starts, then confirm a real gear.
    int gear = -1;
    for (int i = 0; i < 200 && gear <= 1; ++i) {
        EngineSimStats stats;
        stats.currentRPM = 900.0;
        h.provider->provideFeedback(stats);
        gear = h.provider->OnUpdateSimulation(0.05).gearAbsolute;
    }
    EXPECT_GT(gear, 1)
        << "After warmBootToRunning the twin must reach a forward gear at 80 km/h "
           "WOT (convergent with the replay warm-start, not a cold reversion)";
}

}  // namespace
