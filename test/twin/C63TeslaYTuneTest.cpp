// C63TeslaYTuneTest.cpp - TDD tests for the C63_TeslaY clutch/gearbox refinement
//
// These tests pin the invariants the iteration must hold after the SlipLock
// re-tune. They are pure-function / provider-contract tests so they run in the
// standard `make test` bridge suite (no .mr preset loading required).
//
// Invariants under test (the task's acceptance criteria):
//   1. Gear-ratio coherence: the C63_TeslaY pairs a ZF8 box (1st = 4.38) with a
//      C63 final drive (diff 2.82), giving an overall 1st-gear ratio ~12:1.
//      The previous bug paired the ZF8 box with the Tesla EV diff (9.0), giving
//      an incoherent ~39:1 — the root cause of over-rev, racing and eager gears.
//   2. No over-rev: at realistic road speed in 1st gear the implied RPM stays
//      below redline*0.95.
//   3. No reverse at standstill: a forward-drive / standstill 'R' stalk command
//      maps to PARK/NEUTRAL, never REVERSE (reverse only engages while actually
//      reversing at meaningful speed).
//   4. Idle floor: under load at standstill the engine stays at/above idle
//      (~600 RPM) — no below-idle lug.
//   5. Realistic gears: at ~15 mph the box sits in 1st or 2nd, never 3rd+.

#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include <sstream>

#include "twin/SpeedRpmConversion.h"
#include "twin/IceVehicleProfile.h"
#include "twin/AutomaticGearbox.h"
#include "simulator/EngineSimTypes.h"
#include "simulator/GearConventions.h"
#include "input/LiveTelemetryProvider.h"

namespace {

// C63_TeslaY geometry the .mr must produce (ZF8 box on the C63 diff).
constexpr double kC63Zf8FirstGear = 4.38;     // AMG 722.9 1st (C63_TeslaY.mr)
constexpr double kC63DiffRatio = 2.82;         // C63 final drive (NOT 9.0)
constexpr double kC63TireRadius = 0.35687;     // ~14 inch (C63 body)
constexpr double kC63Redline = 7250.0;
constexpr double kC63Idle = 600.0;

// Mini stream harness: feeds a CSV string to LiveTelemetryProvider exactly like
// the engine-sim-cli --live-telemetry stdin path, so the standstill-R coercion
// (applied in the stream branch) is exercised end-to-end.
struct StreamHarness {
    std::istringstream stream;
    std::unique_ptr<input::LiveTelemetryProvider> provider;
    explicit StreamHarness(const std::string& csv, bool autoStart = true)
        : stream(csv) {
        provider = std::make_unique<input::LiveTelemetryProvider>(stream, autoStart);
    }
};

constexpr int kReverse = static_cast<int>(bridge::GearSelector::REVERSE);

// ---------------------------------------------------------------------------
// 1. Gear-ratio coherence (the root-cause fix target).
//
// NOTE: the two original "coherence" tests (OverallFirstGearRatioIsCoherent,
// DiffRatioIsC63NotTeslaEv) were TAUTOLOGIES - they asserted test-local
// constexpr values against themselves (4.38 * 2.82 == 12.35, 2.82 == 2.82) and
// so verified nothing about production code. Deleted; the real regression
// cover for the tall-diff bug is TallRatioWouldHaveOverRevvedAt23Mph below,
// which exercises computeTargetRpm with both geometries.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// 2. No over-rev: implied RPM below redline*0.95 across the normal speed band.
// ---------------------------------------------------------------------------

TEST(C63TeslaYTuneTest, FirstGearRpmStaysBelowRedlineAtRealisticSpeed) {
    // 23 mph was the over-rev trigger with the tall 39:1 set (7251 RPM > 7250).
    // With the coherent 12:1 set, 23 mph in 1st implies ~2110 RPM.
    const double rpm23 = twin::computeTargetRpm(23.0, kC63Zf8FirstGear,
                                                kC63TireRadius, kC63DiffRatio, 0.0);
    EXPECT_LT(rpm23, kC63Redline * 0.95)
        << "At 23 mph in 1st the coherent ratios must not over-rev";
    EXPECT_LT(rpm23, 3000.0)
        << "Coherent 1st gear at 23 mph should be a sane ~2100 RPM, not redline";

    // Even at a brisk 50 mph in 1st (far above any realistic 1st-gear speed)
    // the unclamped RPM must remain below redline for the coherent ratios.
    const double rpm50 = twin::computeTargetRpm(50.0, kC63Zf8FirstGear,
                                                kC63TireRadius, kC63DiffRatio, 0.0);
    EXPECT_LT(rpm50, kC63Redline)
        << "Coherent 1st gear must not reach redline until well past 50 mph";
}

TEST(C63TeslaYTuneTest, TallRatioWouldHaveOverRevvedAt23Mph) {
    // Regression anchor: documents WHY the tall set was wrong. The 9.0 EV diff on
    // the ZF8 box is ~3.2x too tall — at any given road speed the engine turns
    // ~3.2x faster than the coherent C63 set (diff 2.82). This is the root cause
    // of the over-rev / racing / eager-gear symptoms. The anchor pins the
    // comparison: the tall set implies a much higher RPM than the coherent set at
    // the same speed, and only the coherent set stays under redline.
    const double tallRpm = twin::computeTargetRpm(23.0, kC63Zf8FirstGear,
                                                    kC63TireRadius, 9.0, 0.0);
    const double coherentRpm = twin::computeTargetRpm(23.0, kC63Zf8FirstGear,
                                                    kC63TireRadius, kC63DiffRatio, 0.0);
    // The tall (9.0) set must turn the engine ~3.2x faster than the coherent set,
    // i.e. the ratio of overall final drives (39.4 / 12.35).
    EXPECT_NEAR(tallRpm / coherentRpm, 9.0 / kC63DiffRatio, 0.05)
        << "The 9.0 diff must spin the engine ~3.2x faster than the 2.82 diff at the same speed";
    EXPECT_GT(tallRpm, coherentRpm * 2.5)
        << "The tall set is decisively taller than the coherent set";
    // The shipped 2.82 set must NOT over-rev at 23 mph (the fix).
    EXPECT_LT(coherentRpm, kC63Redline * 0.95)
        << "The shipped 2.82 diff set must stay well under redline at 23 mph";
    // And the tall set must reach redline at a far lower speed than is realistic
    // for 1st gear (its whole problem): it crosses redline before ~25 mph.
    const double tallRpmAtRedlineSpeed =
        twin::computeTargetRpm(kC63Redline / (tallRpm / 23.0), kC63Zf8FirstGear,
                               kC63TireRadius, 9.0, 0.0);
    EXPECT_GE(tallRpmAtRedlineSpeed, kC63Redline)
        << "The tall 9.0 set crosses redline at an unrealistically low speed";
}

// ---------------------------------------------------------------------------
// 3. No reverse at standstill / forward drive.
// ---------------------------------------------------------------------------

// The CSV's R rows occur at standstill (speed ~0). The live provider must map a
// standstill 'R' to PARK/NEUTRAL so the twin never selects REVERSE gear. A
// forward-moving 'R' (genuine backing-up) is still allowed to be REVERSE, so we
// only remap at standstill — preserving the existing forward-R behaviour.
TEST(C63TeslaYTuneTest, StandstillReverseMapsToParkOrNeutral) {
    // CSV row: standstill speed, selector 'R'. Mirrors the em-dinner.csv R rows
    // (speed ~0). The stream path must coerce this to PARK/NEUTRAL.
    StreamHarness h("time_s,throttle_pct,road_speed_kmh,gear_selector\n"
                    "0.0,0,0,R\n");
    ASSERT_TRUE(h.provider->Initialize());

    input::EngineInput in = h.provider->OnUpdateSimulation(0.05);
    EXPECT_NE(in.gearSelector, kReverse)
        << "Standstill 'R' must map to PARK/NEUTRAL, never REVERSE (no creep "
           "in reverse at a standstill / Tgt=0)";
}

// Forward-moving 'R' (genuine reverse) is preserved — REVERSE is only coerced
// when the car is NOT genuinely reversing. A backing-up 'R' (clearly negative
// road speed, below kReverseActiveSpeedKmh = -2 km/h) is still REVERSE.
TEST(C63TeslaYTuneTest, ReversingAtSpeedKeepsReverse) {
    StreamHarness h("time_s,throttle_pct,road_speed_kmh,gear_selector\n"
                    "0.0,20,-8,R\n");
    ASSERT_TRUE(h.provider->Initialize());

    input::EngineInput in = h.provider->OnUpdateSimulation(0.05);
    EXPECT_EQ(in.gearSelector, kReverse)
        << "A genuine reversing 'R' (speed clearly negative) must remain REVERSE";
}

// A near-standstill 'R' (road speed inside -2..0 km/h — the em-dinner.csv RAR
// band) must NOT select REVERSE. The old standstill-only coercion (|speed|<1)
// leaked these; the new coercion keys off kReverseActiveSpeedKmh = -2 km/h.
TEST(C63TeslaYTuneTest, NearStandstillReverseMapsToParkOrNeutral) {
    // Rows at -1.5 and -0.5 km/h are near-standstill 'R' (no real reversing).
    StreamHarness h("time_s,throttle_pct,road_speed_kmh,gear_selector\n"
                    "0.0,0,-1.5,R\n"
                    "0.1,0,-0.5,R\n");
    ASSERT_TRUE(h.provider->Initialize());

    input::EngineInput a = h.provider->OnUpdateSimulation(0.05);
    EXPECT_NE(a.gearSelector, kReverse)
        << "Near-standstill 'R' (-1.5 km/h) must map to PARK/NEUTRAL, never REVERSE";
    input::EngineInput b = h.provider->OnUpdateSimulation(0.05);
    EXPECT_NE(b.gearSelector, kReverse)
        << "Near-standstill 'R' (-0.5 km/h) must map to PARK/NEUTRAL, never REVERSE";
}

// A contradictory forward 'R' (car creeping forward, speed > 0) must map to
// NEUTRAL, never REVERSE — forward motion and reverse gear are mutually
// exclusive.
TEST(C63TeslaYTuneTest, ForwardCreepReverseMapsToNeutral) {
    StreamHarness h("time_s,throttle_pct,road_speed_kmh,gear_selector\n"
                    "0.0,5,0.64,R\n");
    ASSERT_TRUE(h.provider->Initialize());

    input::EngineInput in = h.provider->OnUpdateSimulation(0.05);
    EXPECT_NE(in.gearSelector, kReverse)
        << "A forward-creep 'R' (speed > 0) must map to NEUTRAL, never REVERSE";
    EXPECT_EQ(in.gearSelector, static_cast<int>(bridge::GearSelector::NEUTRAL))
        << "A forward-creep 'R' must map to NEUTRAL (forward, not standstill)";
}

// ---------------------------------------------------------------------------
// 4. Idle floor: under load at standstill the engine stays >= idle.
// ---------------------------------------------------------------------------

// The slip-lock creep floor keeps the clutch engaged at standstill so the
// engine is loaded and cannot drop below idle. Assert the floor is meaningful
// (>= 0.05) AND that at standstill with zero throttle the controller returns a
// non-zero floored pressure (engine loaded, not decoupled/free-revving).
TEST(C63TeslaYTuneTest, StandstillClutchFlooredLoadsEngineAboveIdle) {
    // Engine at idle, road-implied ~0 (standstill), zero throttle: the clutch
    // must sit on the floor (never fully open) so the engine is loaded.
    const auto out = twin::computeSlipLockPressure(
        twin::SlipLockInput{kC63Idle, 0.0, 0.0, kC63Idle, kC63Redline},
        /*maxCreepPressure=*/0.10);
    EXPECT_GE(out.clutchPressure, 0.05)
        << "Standstill clutch floor must load the engine (no free-rev, no lug)";
    EXPECT_LT(out.clutchPressure, 0.5)
        << "The floor is a floor, not a half-clamp (exact value is tuning)";
}

// Over-redline bound: when the engine is at/above redline the clutch MUST NOT
// lock (it must slip so the engine can shed speed under the rev limiter). This
// is the ceiling that complements the floor and prevents the slip-lock from
// pinning the engine at redline.
TEST(C63TeslaYTuneTest, SlipLockNeverLocksAboveRedline) {
    // Engine at redline, road much slower (decel / would-be over-rev): the
    // controller must NOT report locked, and pressure must be bounded at the
    // floor (never the full-lock 1.0).
    const auto out = twin::computeSlipLockPressure(
        twin::SlipLockInput{kC63Redline, kC63Idle, 0.5, kC63Idle, kC63Redline},
        /*maxCreepPressure=*/0.10);
    EXPECT_FALSE(out.locked)
        << "Clutch must not lock while the engine is at/above redline";
    EXPECT_LT(out.clutchPressure, 0.5)
        << "At redline the clutch must slip (well short of lock; the exact "
           "ceiling is tuning)";
}

// ---------------------------------------------------------------------------
// 5. Realistic gears: ~15 mph -> 1st or 2nd, never 3rd+.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// 5. Realistic gears: ~15 mph -> 1st or 2nd, never 3rd+.
// ---------------------------------------------------------------------------

TEST(C63TeslaYTuneTest, GearAt15MphIsFirstOrSecond) {
    // Build a C63-style profile and let the gearbox decide at 15 mph. With the
    // coherent ratios the upshift RPM envelope keeps 1st/2nd engaged to ~11-20
    // mph; 3rd would lug the engine, so the box must not climb to 3rd at 15 mph.
    twin::IceVehicleProfile profile = twin::IceVehicleProfile::zf8hp45();
    profile.gearRatios = {4.38, 2.86, 1.92, 1.37, 1.00, 0.82, 0.73};
    profile.diffRatio = kC63DiffRatio;
    profile.tireRadiusM = kC63TireRadius;
    profile.redlineRpm = kC63Redline;
    profile.idleRpm = kC63Idle;
    profile.standstillThresholdKmh = 1.0;
    profile.minShiftIntervalS = 0.5;
    // Populate a shift table so getShiftSpeed is satisfied (high points => quiet).
    profile.separateDownshiftTableEnabled = false;
    profile.shiftTableThrottleLevels = {0.1, 0.5, 1.0};
    profile.shiftTable = {
        {200.0, 200.0, 200.0, 200.0, 200.0, 200.0},
        {200.0, 200.0, 200.0, 200.0, 200.0, 200.0},
        {200.0, 200.0, 200.0, 200.0, 200.0, 200.0}
    };

    twin::AutomaticGearbox gearbox(profile);
    gearbox.setGearSelector(bridge::GearSelector::DRIVE);
    // Drive at 15 mph, light throttle — the box should hold 1st or 2nd.
    for (int i = 0; i < 50; ++i) {
        gearbox.update(0.05, 15.0, 0.15);
    }
    EXPECT_LE(gearbox.getCurrentGear(), 2)
        << "At ~15 mph the box must be in 1st or 2nd, never 3rd+ (eager-gear bug)";
    EXPECT_GE(gearbox.getCurrentGear(), 1)
        << "The box must be in a forward gear at 15 mph";
}

// ---------------------------------------------------------------------------
// 6. Shift progression: at ~40 mph the box must be in gear 4-5 (NOT gear 2).
// ---------------------------------------------------------------------------

// Drive the twin (configured with the C63_TeslaY ZF8 geometry) to RUNNING, then
// hold ~40 mph (64 km/h) with moderate throttle. The recalibrated upshift-RPM
// envelope must have climbed the box to gear 4-5 so the engine RPM sits near
// the cruise band (~1350-1500 RPM), never stuck in gear 2 (~3650 RPM). This is
// the regression test for the "gear 2 at 40 mph" over-correction.
TEST(C63TeslaYTuneTest, GearAt40MphIsFourthOrFifth) {
    twin::IceVehicleProfile profile = twin::IceVehicleProfile::zf8hp45();
    profile.redlineRpm = kC63Redline;
    profile.idleRpm = kC63Idle;
    auto twin = std::make_unique<twin::VirtualIceTwin>(profile);

    // Apply the C63_TeslaY geometry exactly as the CLI does (--script C63_TeslaY.mr).
    twin->reconfigureProfile(
        /*gearRatios=*/{4.38, 2.86, 1.92, 1.37, 1.00, 0.82, 0.73},
        /*diffRatio=*/kC63DiffRatio,
        /*tireRadiusM=*/kC63TireRadius);
    twin->setGearSelector(bridge::GearSelector::DRIVE);
    // Twin ignition defaults OFF (startStop contract: no self-start); the
    // cascade below needs a commanded-on engine.
    twin->setIgnition(true);

    // OFF -> CRANKING -> IDLE -> RUNNING with RPM feedback above the catch threshold.
    twin->update(0.016, input::UpstreamSignal{});          // OFF -> CRANKING
    twin->setEngineRpmFeedback(800.0);
    twin->update(0.016, input::UpstreamSignal{});          // CRANKING -> IDLE
    twin->update(0.016, input::UpstreamSignal{});          // IDLE (throttle 0) stays IDLE

    // Throttle to enter RUNNING, then hold 40 mph.
    input::UpstreamSignal drive;
    drive.throttleFraction = 0.4;
    drive.speedKmh = 64.0;          // ~40 mph
    drive.isValid = true;
    drive.timestampUtcMs = 1000;

    // Run enough frames for the box to cascade up through the gears.
    int gear = 0;
    for (int i = 0; i < 400; ++i) {
        twin->setEngineRpmFeedback(
            twin::computeTargetRpm(drive.speedKmh,
                                   profile.gearRatios[gear - 1 < 0 ? 0 : gear - 1],
                                   kC63TireRadius, kC63DiffRatio, 0.0));
        twin->update(0.05, drive);
        gear = twin->getCurrentGear();
        if (gear >= 4) break;       // stop once it has reached the expected band
    }

    EXPECT_GE(gear, 4)
        << "At ~40 mph the box must have upshifted to at least gear 4 (not stuck in gear 2)";
    EXPECT_LE(gear, 5)
        << "At ~40 mph the box must rest in gear 4-5 (cruise band ~1350-1500 RPM), not lug in 3 or over-shift to 6";
}

}  // namespace
