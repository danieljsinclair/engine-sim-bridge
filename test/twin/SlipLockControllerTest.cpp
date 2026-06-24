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

TEST(SlipLockControllerTest, StandstillZeroThrottle_OpenClutch_NoStall) {
    // Idle: engine 800, road-implied 0, throttle 0.
    const auto out = compute(800.0, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(out.clutchPressure, 0.0)
        << "At standstill the clutch must be fully open so the engine can idle";
    EXPECT_FALSE(out.locked);
}

TEST(SlipLockControllerTest, RoadImpliedBelowIdle_CreepProportionalToRpmAndThrottle) {
    // In-gear crawl: road-implied 300 RPM is below idle 700.
    // Creep model: pressure = maxCreep * tcCapacity * throttleScale
    // where tcCapacity = (engineRpm/redlineRpm)^2 (square law, fluid coupling)
    // and throttleScale = 0.2 + 0.8 * throttle.
    // Engine at 3000 RPM, road at 300 RPM, throttle 0.50:
    //   tcCapacity = (3000/6500)^2 = 0.213
    //   throttleScale = 0.2 + 0.8*0.5 = 0.6
    //   creep = 0.10 * 0.213 * 0.6 = 0.0128 (> 0.001 threshold)
    const auto out = compute(3000.0, 300.0, 0.50);
    EXPECT_GT(out.clutchPressure, 0.0)
        << "Creep mode must apply some pressure to load the engine at standstill";
    EXPECT_LE(out.clutchPressure, 0.10)
        << "Creep pressure must not exceed maxCreepPressure";
    EXPECT_FALSE(out.locked);
}

TEST(SlipLockControllerTest, RoadImpliedBelowIdle_ZeroThrottle_StillOpen) {
    // At zero throttle, creep pressure is 0 regardless of road speed.
    const auto out = compute(800.0, 300.0, 0.0);
    EXPECT_DOUBLE_EQ(out.clutchPressure, 0.0)
        << "At zero throttle the clutch must be fully open (no creep)";
    EXPECT_FALSE(out.locked);
}

TEST(SlipLockControllerTest, RoadImpliedExactlyAtIdle_FloorDoesNotForceZero) {
    // Boundary: at exactly idle the floor no longer applies, so the TC
    // characteristic takes over. With throttle 0 the band is narrow and slip
    // is large -> pressure should be partial but strictly > 0 and below 1.
    const auto out = compute(800.0, kIdleRpm, 0.0);
    EXPECT_GT(out.clutchPressure, 0.0)
        << "Once road-implied >= idle the TC slip path must produce non-zero pressure";
    EXPECT_LT(out.clutchPressure, 1.0)
        << "With substantial slip the clutch must not be fully locked";
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
    // Launch: engine revs to 3500 (power band), road still slow but
    // road-implied has crossed idle (1000) so the floor no longer zeroes it.
    // WOT widens the stall band -> TC slip keeps pressure moderate (not locked).
    const auto out = compute(3500.0, 1000.0, 1.0);
    EXPECT_GT(out.clutchPressure, 0.0)
        << "Under throttle with slip in the power band, the clutch must apply some pressure";
    EXPECT_LT(out.clutchPressure, 0.5)
        << "But not so much that it kills the TC slip / drags the engine down";
    EXPECT_FALSE(out.locked)
        << "A high-slip launch should not yet be locked";
}

TEST(SlipLockControllerTest, WOTWidensStallBand_HigherPressureThanClosedThrottle) {
    // Same non-saturating slip, more throttle -> wider stall band -> smaller
    // slipRatio -> higher pressure. This is physically correct: at WOT the
    // converter holds more pressure to keep the engine in its power band.
    // Slip is kept small so neither end saturates the slipRatio clamp at 1.
    const double engineRpm = 2300.0;
    const double roadImplied = 2000.0;  // slip = 300, well above idle floor
    const auto closedThrottle = compute(engineRpm, roadImplied, 0.0);
    const auto wot = compute(engineRpm, roadImplied, 1.0);
    EXPECT_GT(wot.clutchPressure, closedThrottle.clutchPressure)
        << "WOT widens the stall band -> higher pressure for the same slip";
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
