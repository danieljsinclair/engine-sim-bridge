// SlipLockControllerTest.cpp
//
// Deterministic (input -> output) tests for the torque-converter slip-lock
// pressure function. Pure function — no engine-sim physics, no CLI.
//
// The creep-modified slip-lock algorithm: when road speed implies below-idle RPM,
// a small clutch pressure proportional to throttle is applied (TC fluid coupling
// creep). At zero throttle the clutch is fully open. Above idle the normal TC
// slip characteristic applies.
//
// Algorithm under test (see SlipLockController.h):
//   1. roadImplied < idle:
//        creep = throttle * maxCreepPressure
//        if creep > 0: return {creep, false}   (TC fluid coupling)
//        else:         return {0.0, false}     (true neutral, zero throttle)
//   2. slip      = max(0, engine - roadImplied)
//      stallBand = redline * (0.10 + 0.40 * throttle)
//      slipRatio = clamp(slip / stallBand, 0, 1)
//      pressure  = 1 - sqrt(slipRatio)
//      locked    = slipRatio < 0.1

#include <gtest/gtest.h>
#include <twin/SlipLockController.h>

#include <vector>

using namespace twin;

namespace {
// Common engine parameters (match zf8hp45 profile range).
constexpr double kIdleRpm = 700.0;
constexpr double kRedlineRpm = 6500.0;

SlipLockOutput compute(double engineRpm, double roadImplied, double throttle) {
    return computeSlipLockPressure(SlipLockInput{
        engineRpm, roadImplied, throttle, kIdleRpm, kRedlineRpm},
        /*maxCreepPressure=*/0.10);
}
}  // namespace

// ---------------------------------------------------------------------------
// Stall prevention: the floor from the circle.
// ---------------------------------------------------------------------------

TEST(SlipLockControllerTest, StandstillZeroThrottle_FlooredClutch_NoFreeRev) {
    // Idle: engine 800, road-implied 0, throttle 0. The clutch MUST NOT be fully
    // open (pressure 0): a fully-open clutch decouples the engine from the road
    // and lets it free-rev to the redline. The pressure floor keeps enough clamp
    // to transmit torque and load the engine. The "fully open at standstill"
    // behaviour was the original redline bug and is intentionally removed.
    const auto out = compute(800.0, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(out.clutchPressure, twin::kSlipLockPressureFloor)
        << "At standstill the clutch must sit on the floor (never fully open), "
           "so the engine is loaded and cannot free-rev";
    EXPECT_GE(out.clutchPressure, 0.05)
        << "Floor must be a meaningful minimum clutch pressure";
    EXPECT_FALSE(out.locked);
}

TEST(SlipLockControllerTest, RoadImpliedBelowIdle_CreepProportionalToThrottle) {
    // In-gear crawl at ~5 km/h: road-implied 300 RPM is below idle 700.
    // The creep model applies a small clutch pressure proportional to throttle,
    // mimicking TC fluid coupling. This loads the engine so it doesn't free-rev.
    // With default maxCreepPressure=0.10 and throttle=0.10: creep = 0.10 * 0.10 = 0.01.
    const auto out = compute(800.0, 300.0, 0.10);
    EXPECT_GT(out.clutchPressure, 0.0)
        << "Creep mode must apply some pressure to load the engine at standstill";
    EXPECT_LE(out.clutchPressure, 0.10)
        << "Creep pressure must not exceed maxCreepPressure";
    EXPECT_FALSE(out.locked);
}

TEST(SlipLockControllerTest, RoadImpliedBelowIdle_ZeroThrottle_FlooredNotOpen) {
    // At zero throttle the creep pressure is 0, but the clutch is still held on
    // the floor (never fully open) so the engine is loaded and cannot free-rev.
    const auto out = compute(800.0, 300.0, 0.0);
    EXPECT_DOUBLE_EQ(out.clutchPressure, twin::kSlipLockPressureFloor)
        << "At zero throttle the clutch must sit on the floor (no creep pressure, "
           "but never fully open)";
    EXPECT_GE(out.clutchPressure, 0.05);
    EXPECT_FALSE(out.locked);
}

TEST(SlipLockControllerTest, RoadImpliedExactlyAtIdle_FloorDoesNotForceZero) {
    // Boundary: at exactly idle the creep branch ends and the rolling band
    // begins, so the pressure is the floor (never fully open) and the clutch is
    // not yet locked. The floor is kSlipLockPressureFloor (0.05, lowered from
    // 0.10 in iter2 so standstill isn't over-loaded).
    const auto out = compute(800.0, kIdleRpm, 0.0);
    EXPECT_GT(out.clutchPressure, 0.0)
        << "Once road-implied >= idle the rolling band must produce non-zero pressure";
    EXPECT_LT(out.clutchPressure, 1.0)
        << "At idle the clutch must not yet be fully locked";
    EXPECT_GE(out.clutchPressure, twin::kSlipLockPressureFloor)
        << "Pressure is bounded by the floor (never fully open)";
    EXPECT_FALSE(out.locked);
}

// ---------------------------------------------------------------------------
// Cruise / lock-up: zero slip -> locked.
// ---------------------------------------------------------------------------

TEST(SlipLockControllerTest, CruiseZeroSlip_FullLock) {
    // Engine == road-implied: perfect sync, full lock.
    const auto out = compute(2500.0, 2500.0, 0.40);
    EXPECT_DOUBLE_EQ(out.clutchPressure, 1.0)
        << "At zero slip the clutch should be fully locked (direct coupling)";
    EXPECT_TRUE(out.locked);
}

TEST(SlipLockControllerTest, Decel_EngineSlowerThanRoad_LockedForEngineBraking) {
    // Engine braking: engine 1000 < road-implied 2000. Slip is clamped to 0
    // (no reverse coupling), so the clutch locks and the road drags the engine.
    const auto out = compute(1000.0, 2000.0, 0.0);
    EXPECT_DOUBLE_EQ(out.clutchPressure, 1.0)
        << "On decel (engine slower than road) the clutch must lock for engine braking";
    EXPECT_TRUE(out.locked);
}

// ---------------------------------------------------------------------------
// Launch / torque-converter slip: high slip under throttle -> partial pressure.
// ---------------------------------------------------------------------------

TEST(SlipLockControllerTest, LaunchWOT_HighSlip_PartialPressureInSlipBand) {
    // Launch: engine revs to 3500 (power band), road still slow but above idle
    // (road-implied 850) — the low-speed launch band, well below the cruise lock
    // threshold (0.18 * redline = 1170 at idle 700 / red 6500). The clutch must
    // slip with PARTIAL pressure (engine revs, not rigidly locked), but the
    // pressure is driven by ROAD SPEED so it stays partial only at low road speed.
    // At this low launch road speed pressure is ~0.35, comfortably below the lock.
    const auto out = compute(3500.0, 850.0, 1.0);
    EXPECT_GT(out.clutchPressure, 0.0)
        << "Under throttle at launch the clutch must apply some pressure";
    EXPECT_LT(out.clutchPressure, 0.5)
        << "But not so much that it kills the TC slip / drags the engine down";
    EXPECT_FALSE(out.locked)
        << "A low-speed launch should not yet be locked";
}

TEST(SlipLockControllerTest, WOTWidensStallBand_HigherPressureThanClosedThrottle) {
    // The slip-based "WOT widens the stall band" behaviour belonged to the old
    // K-factor model. The lock is now driven by ROAD SPEED, so at a fixed road
    // speed the rolling-band pressure is throttle-independent: both WOT and
    // closed-throttle give the same lock pressure (the engine is either coupled
    // to the road or not, regardless of pedal). What matters now: as the wheels
    // ROLL (road rises toward cruise) the pressure ramps to a full lock, and at
    // cruise it is locked for both throttle positions.
    const double engineRpm = 2300.0;
    const double roadImplied = 2000.0;  // above cruise threshold -> locked
    const auto closedThrottle = compute(engineRpm, roadImplied, 0.0);
    const auto wot = compute(engineRpm, roadImplied, 1.0);
    EXPECT_NEAR(wot.clutchPressure, closedThrottle.clutchPressure, 1e-9)
        << "At a fixed road speed the lock pressure is throttle-independent";
    EXPECT_NEAR(wot.clutchPressure, 1.0, 1e-6)
        << "At cruise road speed the clutch locks regardless of throttle";
    EXPECT_TRUE(wot.locked);
    EXPECT_TRUE(closedThrottle.locked);
}


// ---------------------------------------------------------------------------
// Monotonicity: as road-implied rises toward engine RPM, pressure only rises.
// ---------------------------------------------------------------------------

TEST(SlipLockControllerTest, PressureMonotonicAsRoadImpliedApproachesEngine) {
    const double engineRpm = 3500.0;
    const double throttle = 1.0;
    const std::vector<double> roadImpliedSteps = {
        kIdleRpm, 1000.0, 1500.0, 2000.0, 2500.0, 3000.0, 3500.0};

    double prev = -1.0;
    for (const double roadImplied : roadImpliedSteps) {
        const auto out = compute(engineRpm, roadImplied, throttle);
        EXPECT_GE(out.clutchPressure, prev)
            << "Pressure must be monotonically non-decreasing as road-implied rises toward engine RPM"
            << " (roadImplied=" << roadImplied << ", pressure=" << out.clutchPressure << ")";
        prev = out.clutchPressure;
    }
    EXPECT_DOUBLE_EQ(prev, 1.0) << "At full sync pressure must reach 1.0";
}

TEST(SlipLockControllerTest, LockedFlagTrueOnlyBelowLockThreshold) {
    // locked is defined as slipRatio < 0.1, i.e. very near sync.
    const auto lockedCase = compute(2500.0, 2500.0, 0.40);
    EXPECT_TRUE(lockedCase.locked);

    const auto slippingCase = compute(3500.0, 1000.0, 1.0);
    EXPECT_FALSE(slippingCase.locked);
}

// ---------------------------------------------------------------------------
// Clamping: pressure stays within [0, 1] even at extreme slip / throttle.
// ---------------------------------------------------------------------------

TEST(SlipLockControllerTest, ExtremeSlipClampsPressureToBand) {
    // Slip far exceeds the stall band -> slipRatio clamps to 1 -> pressure 0.
    // (Above idle floor so the TC path runs.)
    const auto out = compute(kRedlineRpm, kIdleRpm, 0.0);
    EXPECT_GE(out.clutchPressure, 0.0);
    EXPECT_LE(out.clutchPressure, 1.0);
    EXPECT_FALSE(out.locked);
}

TEST(SlipLockControllerTest, OutputAlwaysWithinUnitRange) {
    // Sweep a wide grid; pressure must always be in [0, 1].
    for (double road = 0.0; road <= 7000.0; road += 350.0) {
        for (double engine = 0.0; engine <= kRedlineRpm; engine += 650.0) {
            for (double throttle : {0.0, 0.25, 0.5, 0.75, 1.0}) {
                const auto out = compute(engine, road, throttle);
                EXPECT_GE(out.clutchPressure, 0.0)
                    << "engine=" << engine << " road=" << road << " thr=" << throttle;
                EXPECT_LE(out.clutchPressure, 1.0)
                    << "engine=" << engine << " road=" << road << " thr=" << throttle;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Cruise lock (iter2 + user feedback): the clutch must LOCK at cruise — pressure
// -> 1.0 once the wheels are rolling at road speed — and NOT slip-wide-open
// everywhere (the old free-rev symptom). The lock is driven by ROAD SPEED, not
// by (engine - road) slip, because a slip-driven pressure deadlocks: high engine
// RPM -> large slip -> low pressure -> engine revs higher -> larger slip, so at
// cruise the engine free-revved uncoupled with no engine braking.
// ---------------------------------------------------------------------------

TEST(SlipLockControllerTest, Cruise_LocksAtRoadSpeed_EngineNotFreeRevving) {
    // The reported failure: at 40 mph (C63_TeslaY, DA5 gear 1.00, diff 2.82,
    // tire ~0.356) the road-implied RPM is ~1350, but the engine was sitting at
    // 4250 with the clutch effectively open. The clutch must LOCK at cruise so
    // the engine is loaded and dragged down to the road (≈1350), not left to
    // free-rev. Road-implied 1350 is well above the cruise threshold
    // (0.18 * 6500 = 1170), so pressure must be ~1.0 and the clutch locked.
    const double roadImpliedCruise = 1350.0;
    const auto out = compute(4250.0, roadImpliedCruise, 0.3);
    EXPECT_NEAR(out.clutchPressure, 1.0, 1e-6)
        << "At cruise the clutch must lock (pressure ~1.0) so the engine is loaded, "
           "not free-revving uncoupled";
    EXPECT_TRUE(out.locked)
        << "At cruise (road-implied well above the lock threshold) the clutch must be locked";
}

TEST(SlipLockControllerTest, Cruise_PressureDrivenByRoadSpeed_NoDeadlock) {
    // The deadlock proof: holding engine RPM fixed at an over-revving 4250 while
    // the road rises from standstill to cruise, pressure must MONOTONICALLY
    // RISE and reach ~1.0 at cruise. The old slip model deadlocked the pressure
    // at the floor (0.05) for every road speed once the engine was spinning up,
    // so the engine never got loaded. With road-driven lock, pressure climbs as
    // the wheels roll.
    const double engineRpm = 4250.0;
    const double throttle = 0.3;
    double prev = -1.0;
    for (double road = kIdleRpm; road <= 2000.0; road += 100.0) {
        const auto out = compute(engineRpm, road, throttle);
        EXPECT_GE(out.clutchPressure, prev)
            << "Cruise pressure must rise with road speed (no deadlock): road=" << road;
        prev = out.clutchPressure;
    }
    // At a genuine cruise road speed the clutch is fully locked.
    const auto cruise = compute(engineRpm, 1350.0, throttle);
    EXPECT_NEAR(cruise.clutchPressure, 1.0, 1e-6);
    EXPECT_TRUE(cruise.locked);
}

TEST(SlipLockControllerTest, CruiseThreshold_JustAboveIdle_SlipBelowLaunch) {
    // The lock threshold is a fraction of redline (0.18) — comfortably above
    // idle so standstill/low-speed launch still slips, but at a real cruise road
    // speed it locks. Below the threshold the clutch must NOT be fully locked
    // (launch slip is allowed); at/above it the clutch locks.
    const double cruiseRpm = kRedlineRpm * twin::kCruiseLockRpmFraction;  // 1170
    ASSERT_GT(cruiseRpm, kIdleRpm)
        << "Cruise threshold must be above idle so launch still slips";

    // Just below threshold: partial pressure, not locked (launch slip allowed).
    const double below = cruiseRpm - 50.0;
    const auto lowSpeed = compute(3000.0, below, 1.0);
    EXPECT_LT(lowSpeed.clutchPressure, 1.0)
        << "Below the cruise threshold the clutch may still slip (launch)";
    EXPECT_FALSE(lowSpeed.locked);

    // At/above threshold: locked.
    const auto cruise = compute(3000.0, cruiseRpm + 50.0, 1.0);
    EXPECT_NEAR(cruise.clutchPressure, 1.0, 1e-6)
        << "At/above the cruise threshold the clutch must lock";
    EXPECT_TRUE(cruise.locked);
}

TEST(SlipLockControllerTest, CruiseLock_EngineBrakingRestoredOnDecel) {
    // On decel at cruise (engine slower than road, both above the threshold) the
    // clutch must lock so the road drags the engine (engine braking). This is
    // the other half of the free-rev bug: with the old slip model the clutch was
    // open at cruise so there was no engine braking either.
    const auto out = compute(1200.0, 1500.0, 0.0);
    EXPECT_NEAR(out.clutchPressure, 1.0, 1e-6)
        << "On cruise decel the clutch must lock for engine braking";
    EXPECT_TRUE(out.locked);
}

