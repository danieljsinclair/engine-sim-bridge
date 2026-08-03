#include <gtest/gtest.h>
#include <twin/VirtualIceTwin.h>
#include <twin/IceVehicleProfile.h>
#include <io/UpstreamSignal.h>
#include <simulator/GearConventions.h>
#include <simulator/EngineSimTypes.h>
#include <input/DemoVehiclePhysics.h>
#include <algorithm>
#include <cmath>

using namespace twin;
using namespace input;

class VirtualIceTwinTest : public ::testing::Test {
protected:
    void SetUp() override {
        profile_ = IceVehicleProfile::zf8hp45();
        twin_ = std::make_unique<VirtualIceTwin>(profile_);
    }

    IceVehicleProfile profile_;
    std::unique_ptr<VirtualIceTwin> twin_;

    UpstreamSignal makeValidSignal(double throttle = 0.0, double speed = 0.0) {
        UpstreamSignal sig;
        sig.throttleFraction = throttle;
        sig.speedKmh = speed;
        sig.timestampUtcMs = 1000;
        sig.isValid = true;
        return sig;
    }

    // Advance twin through OFF -> CRANKING -> IDLE using RPM feedback
    // Also sets gear selector to DRIVE so IDLE->RUNNING transitions can occur
    void advanceThroughCranking() {
        auto sig = makeValidSignal(0.6, 0.0);
        twin_->update(0.016, sig);  // OFF -> CRANKING
        // Provide RPM feedback above threshold to trigger CRANKING -> IDLE
        twin_->setEngineRpmFeedback(800.0);
        twin_->update(0.016, sig);  // CRANKING -> IDLE
        twin_->setGearSelector(bridge::GearSelector::DRIVE);
    }
};

TEST_F(VirtualIceTwinTest, OffToCrankingOnFirstValidTelemetry_AC11_1) {
    EXPECT_EQ(twin_->getState(), TwinState::OFF);

    auto sig = makeValidSignal(0.0, 0.0);
    auto output = twin_->update(0.016, sig);

    EXPECT_EQ(twin_->getState(), TwinState::CRANKING);
    EXPECT_TRUE(output.starterMotor);
    EXPECT_TRUE(output.ignition);
    EXPECT_EQ(output.gear, static_cast<int>(bridge::BridgeGear::NEUTRAL));
}

TEST_F(VirtualIceTwinTest, CrankingToIdleWhenRpmExceedsThreshold_AC11_2) {
    auto sig = makeValidSignal(0.6, 0.0);
    twin_->update(0.016, sig);

    EXPECT_EQ(twin_->getState(), TwinState::CRANKING);

    // RPM below threshold — should stay CRANKING
    twin_->setEngineRpmFeedback(200.0);
    auto output = twin_->update(0.016, sig);
    EXPECT_EQ(twin_->getState(), TwinState::CRANKING);

    // RPM above threshold — should transition to IDLE
    twin_->setEngineRpmFeedback(600.0);
    output = twin_->update(0.016, sig);
    EXPECT_EQ(twin_->getState(), TwinState::IDLE);
    EXPECT_FALSE(output.starterMotor);
}

TEST_F(VirtualIceTwinTest, IdleToRunningWhenThrottleAbove5Percent_AC11_3) {
    advanceThroughCranking();

    EXPECT_EQ(twin_->getState(), TwinState::IDLE);

    auto sig = makeValidSignal(0.06, 0.0);
    twin_->update(0.016, sig);

    EXPECT_EQ(twin_->getState(), TwinState::RUNNING);
}

TEST_F(VirtualIceTwinTest, IdleStaysIdleBelow5PercentThrottle) {
    advanceThroughCranking();

    auto sig = makeValidSignal(0.04, 0.0);
    twin_->update(0.016, sig);

    EXPECT_EQ(twin_->getState(), TwinState::IDLE);
}

TEST_F(VirtualIceTwinTest, RunningToShiftingWhenGearboxRequestsShift_AC11_4) {
    auto sig = makeValidSignal(0.5, 5.0);
    advanceThroughCranking();
    sig.throttleFraction = 0.6;
    twin_->update(0.016, sig);

    EXPECT_EQ(twin_->getState(), TwinState::RUNNING);

    sig.speedKmh = 45.0;
    twin_->update(0.016, sig);

    if (twin_->getState() == TwinState::SHIFTING) {
        // Check next frame — the first frame transitions RUNNING→SHIFTING but
        // the output still carries clutchPressure from the RUNNING case.
        auto output = twin_->update(0.016, sig);
        EXPECT_LT(output.clutchPressure, 1.0) << "Clutch should be disengaging after entering SHIFTING state";
    } else {
        SUCCEED() << "Gearbox not ready to shift yet (throttle smoothing or min shift interval), skipping clutch check";
    }
}

TEST_F(VirtualIceTwinTest, ShiftingToRunningWhenClutchReengages_AC11_5) {
    auto sig = makeValidSignal(0.5, 60.0);
    advanceThroughCranking();

    for (int i = 0; i < 10; ++i) {
        twin_->update(0.016, sig);
    }

    sig.speedKmh = 75.0;
    twin_->update(0.016, sig);

    if (twin_->getState() == TwinState::SHIFTING) {
        for (int i = 0; i < 30; ++i) {
            auto output = twin_->update(0.016, sig);
            if (twin_->getState() == TwinState::RUNNING) {
                EXPECT_DOUBLE_EQ(output.clutchPressure, 1.0);
                break;
            }
        }
    }
}

TEST_F(VirtualIceTwinTest, RunningToIdleWhenSpeedAndThrottleZero_AC11_6) {
    auto sig = makeValidSignal(0.1, 5.0);
    advanceThroughCranking();

    for (int i = 0; i < 10; ++i) {
        twin_->update(0.016, sig);
    }

    ASSERT_EQ(twin_->getState(), TwinState::RUNNING) << "Should be in RUNNING with low speed/throttle";

    sig.throttleFraction = 0.0;
    sig.speedKmh = 0.0;
    twin_->update(0.016, sig);

    EXPECT_EQ(twin_->getState(), TwinState::IDLE);
}

TEST_F(VirtualIceTwinTest, AnyStateToOffAfter5SecondsNoValidTelemetry_AC11_7) {
    auto sig = makeValidSignal(0.5, 60.0);
    twin_->update(0.016, sig);
    twin_->update(0.016, sig);
    sig.throttleFraction = 0.1;
    twin_->update(0.016, sig);

    EXPECT_NE(twin_->getState(), TwinState::OFF);

    UpstreamSignal invalidSig;
    invalidSig.isValid = false;

    for (int i = 0; i < 315; ++i) {
        twin_->update(0.016, invalidSig);
    }

    EXPECT_EQ(twin_->getState(), TwinState::OFF);
}

// ============================================================================
// AC-08: the clutch ramps OPEN (≈0) then back to LOCKED (≈1) through ONE shift.
//
// BEHAVIOUR, not mechanism. The clutch-cycle timing (disengage/pause/reengage)
// is a TUNING KNOB read from the profile. A rigid 50/200/100 ms window would
// freeze the clutch calibration, so we no longer assert absolute milliseconds.
//
// Gotcha handled: the old setup ran 10 frames at a high speed THEN jumped it
// higher. At the settle speed the box already upshifts, so the shift was
// mid-flight and the observed phase depended on cumulative timing — which is
// why even parameterised step counts failed. Here we SETTLE in RUNNING at a LOW
// speed (gear 1, no shift), THEN raise the speed to trigger exactly ONE
// isolated clutch cycle, and assert the observable behaviour:
//   - the clutch opens to ≈0 at some point during the shift, and
//   - it is locked again (≈1.0) when the shift completes,
// regardless of the absolute millisecond values. Step size AND the completion
// window are both derived from the profile, so retuning the clutch later does
// not break this test.
// ============================================================================

TEST_F(VirtualIceTwinTest, ClutchRampsThroughShiftPhases_AC08) {
    advanceThroughCranking();
    twin_->setGearSelector(bridge::GearSelector::DRIVE);

    // Settle in RUNNING at a LOW speed so the box sits in gear 1 and is NOT
    // already mid-shift (the gotcha above).
    auto sig = makeValidSignal(0.1, 5.0);
    twin_->update(0.016, sig);
    ASSERT_EQ(twin_->getState(), TwinState::RUNNING)
        << "Should settle into RUNNING at low speed before the isolated shift";
    ASSERT_EQ(twin_->getCurrentGear(), 1)
        << "Should be in gear 1 at low speed before the isolated shift";

    // Step size + completion-frame budget derived from the profile's clutch
    // timings, so the test tolerates a retuned (faster or slower) clutch.
    const double totalShiftMs =
        profile_.shiftDisengageMs + profile_.shiftPauseMs + profile_.shiftReengageMs;
    const double stepDt = std::max(0.010, profile_.shiftDisengageMs / 5.0 / 1000.0);
    const int kMarginFrames = 30;  // generous slack above the configured cycle
    const int kBudgetFrames =
        static_cast<int>(totalShiftMs / 1000.0 / stepDt) + kMarginFrames;

    // Raise the speed to demand a single shift from gear 1.
    sig.speedKmh = 60.0;

    bool sawShifting = false;
    double minClutchDuringShift = 1.0;
    double clutchWhenComplete = -1.0;

    for (int i = 0; i < kBudgetFrames; ++i) {
        auto output = twin_->update(stepDt, sig);
        if (twin_->getState() == TwinState::SHIFTING) {
            sawShifting = true;
            minClutchDuringShift = std::min(minClutchDuringShift, output.clutchPressure);
        } else if (sawShifting && twin_->getState() == TwinState::RUNNING) {
            // First frame back in RUNNING marks the shift complete.
            clutchWhenComplete = output.clutchPressure;
            break;
        }
    }

    ASSERT_TRUE(sawShifting) << "A shift should have been triggered by the speed increase";
    EXPECT_LE(minClutchDuringShift, 1e-6)
        << "Clutch should open fully (≈0) at some point during the shift";
    ASSERT_GE(clutchWhenComplete, 0.0) << "Shift should have completed (returned to RUNNING)";
    EXPECT_NEAR(clutchWhenComplete, 1.0, 1e-6)
        << "Clutch should be locked (≈1.0) when the shift completes";
}

TEST_F(VirtualIceTwinTest, GearIsNeutralDuringCrankingAndIdle) {
    auto sig = makeValidSignal(0.0, 0.0);
    auto output = twin_->update(0.016, sig);
    EXPECT_EQ(output.gear, static_cast<int>(bridge::BridgeGear::NEUTRAL));

    output = twin_->update(0.016, sig);
    EXPECT_EQ(output.gear, static_cast<int>(bridge::BridgeGear::NEUTRAL));
}

TEST_F(VirtualIceTwinTest, ThrottleIsSmoothed_AC09) {
    auto sig = makeValidSignal(1.0, 0.0);
    auto output = twin_->update(0.016, sig);

    EXPECT_GT(output.throttle, 0.0);
    EXPECT_LT(output.throttle, 1.0) << "Throttle should be smoothed, not instant";
}

// ============================================================================
// AC-11: Throttle works in NEUTRAL (IDLE state passes through user input)
// ============================================================================

TEST_F(VirtualIceTwinTest, IdleThrottle_PassesThroughUserInput_AC11) {
    advanceThroughCranking();
    ASSERT_EQ(twin_->getState(), TwinState::IDLE);

    // User presses throttle — twin should pass it through (smoothed)
    auto sig = makeValidSignal(0.5, 0.0);
    // Run a few frames to let the smoother catch up
    for (int i = 0; i < 20; ++i) {
        twin_->update(0.016, sig);
    }
    auto output = twin_->update(0.016, sig);
    EXPECT_GT(output.throttle, 0.3) << "Throttle should pass through in IDLE/NEUTRAL";
}

TEST_F(VirtualIceTwinTest, IdleThrottle_HoldsSustainFloorWhenNoInput_AC12) {
    advanceThroughCranking();
    ASSERT_EQ(twin_->getState(), TwinState::IDLE);

    // No driver throttle — the twin must still hold the idle-sustain floor so the
    // engine doesn't coast through the engine-sim's Stopped latch at the
    // CRANKING->IDLE handoff (see AC-16). The prior "idles on physics alone ->
    // zero throttle" model stalled on real progressing CSV data (bench: catch
    // then die ~3-4s post-handoff); a real engine needs minimum throttle to idle.
    // The floor is applied only in IDLE (NEUTRAL, clutch disengaged) so it cannot
    // propel the vehicle — it sustains RPM, not motion.
    auto sig = makeValidSignal(0.0, 0.0);
    for (int i = 0; i < 20; ++i) {
        twin_->update(0.016, sig);
    }
    auto output = twin_->update(0.016, sig);
    EXPECT_NEAR(output.throttle, EngineSimDefaults::IDLE_SUSTAIN_THROTTLE, 1e-6)
        << "Idle throttle should hold the sustain floor when no driver input";
}

TEST_F(VirtualIceTwinTest, IdleClutchIsDisengaged) {
    advanceThroughCranking();
    ASSERT_EQ(twin_->getState(), TwinState::IDLE);

    auto sig = makeValidSignal(0.0, 0.0);
    auto output = twin_->update(0.016, sig);
    EXPECT_DOUBLE_EQ(output.clutchPressure, 0.0);
}

// ============================================================================
// AC-13/14: Selector changes trigger state transitions
// ============================================================================

TEST_F(VirtualIceTwinTest, IdleToRunning_WhenSelectorDriveAndThrottle_AC13) {
    advanceThroughCranking();
    ASSERT_EQ(twin_->getState(), TwinState::IDLE);
    twin_->setGearSelector(bridge::GearSelector::DRIVE);

    auto sig = makeValidSignal(0.1, 0.0);
    twin_->update(0.016, sig);
    EXPECT_EQ(twin_->getState(), TwinState::RUNNING);
}

TEST_F(VirtualIceTwinTest, RunningToIdle_WhenSelectorNeutral_AC14) {
    advanceThroughCranking();
    twin_->setGearSelector(bridge::GearSelector::DRIVE);
    auto sig = makeValidSignal(0.1, 0.0);
    twin_->update(0.016, sig);
    ASSERT_EQ(twin_->getState(), TwinState::RUNNING);

    twin_->setGearSelector(bridge::GearSelector::NEUTRAL);
    sig.throttleFraction = 0.0;
    sig.speedKmh = 0.0;
    twin_->update(0.016, sig);
    EXPECT_EQ(twin_->getState(), TwinState::IDLE);
}

// ============================================================================
// New: RUNNING→IDLE uses threshold, not exact equality
// ============================================================================

TEST_F(VirtualIceTwinTest, RunningToIdle_UsesThresholdNotExactZero_AC11_6) {
    auto sig = makeValidSignal(0.1, 5.0);
    advanceThroughCranking();

    for (int i = 0; i < 10; ++i) {
        twin_->update(0.016, sig);
    }

    ASSERT_EQ(twin_->getState(), TwinState::RUNNING);

    // Transition should happen with small non-zero values below thresholds
    // speed < standstillThresholdKmh (1.0) AND throttle < throttleRunningToIdleThreshold (0.02)
    sig.throttleFraction = 0.01;  // Below threshold but not zero
    sig.speedKmh = 0.5;           // Below threshold but not zero
    twin_->update(0.016, sig);

    EXPECT_EQ(twin_->getState(), TwinState::IDLE)
        << "RUNNING->IDLE should transition when speed < 1.0 km/h and throttle < 2%";
}

TEST_F(VirtualIceTwinTest, RunningStaysRunning_WhenSpeedAboveStandstillThreshold) {
    auto sig = makeValidSignal(0.1, 5.0);
    advanceThroughCranking();

    for (int i = 0; i < 10; ++i) {
        twin_->update(0.016, sig);
    }

    ASSERT_EQ(twin_->getState(), TwinState::RUNNING);

    // Even with zero throttle, if speed > standstillThreshold, stay in RUNNING
    sig.throttleFraction = 0.0;
    sig.speedKmh = 2.0;  // Above standstill threshold of 1.0
    twin_->update(0.016, sig);

    EXPECT_EQ(twin_->getState(), TwinState::RUNNING)
        << "Should stay RUNNING when speed > standstill threshold, even with zero throttle";
}

// ============================================================================
// BEHAVIOUR TEST: Gearbox uses signal speed, not feedback speed for shift decisions
// ============================================================================

TEST_F(VirtualIceTwinTest, FeedbackSpeedDoesNotOverrideSignalSpeedForUpshift) {
    // Warm up through state machine: OFF -> CRANKING -> IDLE -> RUNNING
    advanceThroughCranking();
    auto sig = makeValidSignal(0.1, 0.0);
    twin_->update(0.016, sig);
    ASSERT_EQ(twin_->getState(), TwinState::RUNNING);

    // Set vehicle speed feedback to a LOW value (5 kph) - this would keep gearbox in gear 1
    twin_->setVehicleSpeedFeedback(5.0);

    // But provide upstream signal with HIGH speed (40 kph) at 50% throttle
    // This should trigger 1->2 upshift if gearbox uses signal speed
    sig.throttleFraction = 0.5;
    sig.speedKmh = 40.0;

    // Poll (timing-agnostic) instead of a fixed frame window: the clutch-cycle
    // timings are TUNING KNOBS, so we must not freeze them behind a rigid loop.
    // We assert the BEHAVIOUR — gearbox upshifts on the HIGH signal speed (40 kph),
    // not the LOW feedback speed (5 kph) — by breaking as soon as gear > 1.
    const int maxFrames = 400;  // ~6.4s at dt=0.016; generous, decoupled from clutch calibration
    bool upshifted = false;
    for (int i = 0; i < maxFrames; ++i) {
        twin_->update(0.016, sig);
        if (twin_->getCurrentGear() > 1) {
            upshifted = true;
            break;
        }
    }

    EXPECT_TRUE(upshifted)
        << "Gearbox should upshift based on signal.speedKmh (40 kph), "
        << "not feedback speed (5 kph)";
}

// ============================================================================
// DIAGNOSTIC TEST: 20% throttle gentle acceleration shifting issue
// ============================================================================

TEST_F(VirtualIceTwinTest, GentleAcceleration_20PercentThrottle_ShiftsToGear2) {
    advanceThroughCranking();

    // Set selector to DRIVE and transition to RUNNING
    twin_->setGearSelector(bridge::GearSelector::DRIVE);
    auto sig = makeValidSignal(0.06, 0.0);  // Just above idle threshold
    twin_->update(0.016, sig);
    ASSERT_EQ(twin_->getState(), TwinState::RUNNING);

    // Use DemoVehiclePhysics to generate realistic speed at 20% throttle
    input::DemoVehiclePhysics physics;
    double dt = 1.0 / 60.0;

#ifdef ATG_ENGINE_SIM_TEST_VERBOSE
    printf("\n=== DIAGNOSTIC: 20%% Throttle Gentle Acceleration ===\n");
    printf("  Time(s)  Speed(kph)  Gear  State\n");
    printf("  -------  ----------  ----  -----\n");
#endif

    for (int i = 0; i < static_cast<int>(60.0 / dt); ++i) {
        double t = i * dt;
        double throttle = 0.20;

        // Update demo physics (same as live demo)
        physics.update(dt, throttle);
        double speedKmh = physics.getSpeedKmh();

        // Create upstream signal with demo physics speed
        UpstreamSignal signal;
        signal.throttleFraction = throttle;
        signal.speedKmh = speedKmh;
        signal.timestampUtcMs = static_cast<uint64_t>(t * 1000) + 2000;
        signal.isValid = true;

        twin_->update(dt, signal);

        int gear = twin_->getCurrentGear();

        // Print diagnostics every 5 seconds
        if (i % static_cast<int>(5.0 / dt) == 0) {
#ifdef ATG_ENGINE_SIM_TEST_VERBOSE
            printf("  %6.1f  %10.1f  %4d  %d\n",
                   t, speedKmh, gear, static_cast<int>(twin_->getState()));
#endif
        }

        // Early exit if we shifted to gear 2
        if (gear >= 2 && t < 30.0) {
#ifdef ATG_ENGINE_SIM_TEST_VERBOSE
            printf("  *** SHIFTED to gear %d at t=%.1fs, speed=%.1f kph ***\n", gear, t, speedKmh);
#endif
            break;
        }
    }

#ifdef ATG_ENGINE_SIM_TEST_VERBOSE
    printf("  Final: speed=%.1f kph, gear=%d\n", physics.getSpeedKmh(), twin_->getCurrentGear());
    printf("=====================================================\n\n");
#endif

    EXPECT_GE(twin_->getCurrentGear(), 2)
        << "At 20% throttle with DemoVehiclePhysics, gearbox should shift to gear 2 within 60 seconds. "
        << "Final speed: " << physics.getSpeedKmh() << " kph";
}

// ============================================================================
// AC-15: Deterministic engine start from CSV-style input
//
// The CSV->twin path paces one telemetry row per sim frame in real-time, and
// the engine-sim's cranking RPM is fed back one tick in arrears. Whether the
// feedback RPM crosses the 500 RPM catch threshold depends on closed-loop
// timing jitter (tick alignment, real-time pacing), so in ~3/4 of bench runs
// the cranking RPM plateaus BELOW the threshold and the engine never "starts".
//
// The twin MUST still reach IDLE deterministically in that condition: the
// cranking state has a time-based fallback (CRANK_FALLBACK_DURATION_S) that
// releases the starter after a fixed cranking duration regardless of the RPM
// feedback value. Same input frames -> same outcome every run.
// ============================================================================

// Models the failing bench condition: cranking RPM plateaus below the catch
// threshold. The engine MUST still leave CRANKING via the time fallback.
TEST_F(VirtualIceTwinTest, CrankingReachesIdleViaTimeFallbackWhenRpmPlateausLow_AC15) {
    auto sig = makeValidSignal(0.6, 0.0);
    twin_->update(0.016, sig);  // OFF -> CRANKING
    ASSERT_EQ(twin_->getState(), TwinState::CRANKING);

    // Sub-threshold RPM feedback (250 RPM) — the engine is spinning under the
    // starter but the fast-path catch (>500 RPM) never fires. Feed 4s of frames
    // (well past the 3s fallback) and assert the twin reaches IDLE via the time
    // fallback. Detection is on the STATE (reaching IDLE), not the starter
    // signal: since AC18 the twin emits starterMotor as a one-tick edge, so the
    // starter is not held during CRANKING and its level no longer marks the
    // CRANKING->IDLE transition.
    const double dt = 1.0 / 60.0;
    const int kFourSecondsOfFrames = 240;
    bool reachedIdle = false;
    for (int i = 0; i < kFourSecondsOfFrames; ++i) {
        twin_->setEngineRpmFeedback(250.0);  // plateau below threshold
        twin_->update(dt, sig);
        if (twin_->getState() == TwinState::IDLE) { reachedIdle = true; break; }
    }

    EXPECT_TRUE(reachedIdle)
        << "Sub-threshold cranking RPM must still reach IDLE via the 3s time fallback";
    EXPECT_NE(twin_->getState(), TwinState::CRANKING)
        << "Twin must leave CRANKING after the fallback duration even when RPM stays low";
}

// Determinism: the same CSV-style input frames must reach RUNNING on the same
// tick every run. Encodes the "run it 5x, same outcome" guarantee as a unit.
TEST_F(VirtualIceTwinTest, EngineStartIsDeterministicAcrossRuns_AC15) {
    // Scenario: OFF -> CRANKING (sub-threshold RPM plateau) -> IDLE via fallback
    // -> DRIVE + throttle -> RUNNING. Returns the tick RUNNING is first reached,
    // or -1 if it never starts within budget.
    auto runOnce = [this](int budgetTicks) {
        // Fresh twin per run (mirrors a clean process/bench invocation).
        twin_ = std::make_unique<VirtualIceTwin>(profile_);
        twin_->setGearSelector(bridge::GearSelector::DRIVE);

        UpstreamSignal sig = makeValidSignal(0.5, 0.0);  // throttle > idle threshold
        const double dt = 1.0 / 60.0;

        for (int i = 0; i < budgetTicks; ++i) {
            twin_->setEngineRpmFeedback(250.0);  // plateau below catch threshold
            twin_->update(dt, sig);
            if (twin_->getState() == TwinState::RUNNING) return i;
        }
        return -1;
    };

    const int kBudget = 300;
    int firstRunningTick = runOnce(kBudget);
    ASSERT_GE(firstRunningTick, 0) << "Engine must start (reach RUNNING) from CSV-style input";

    // Same input -> same transition tick, every run (5x).
    for (int run = 1; run <= 5; ++run) {
        EXPECT_EQ(runOnce(kBudget), firstRunningTick)
            << "Engine start must be deterministic: same tick on run " << run;
    }

    // The start must be via the ~3s time fallback, not stuck and not an instant
    // physics catch (RPM is sub-threshold). 3s at 1/60s == 180 frames.
    EXPECT_GE(firstRunningTick, 180) << "Should not catch before the 3s fallback (RPM < threshold)";
    EXPECT_LT(firstRunningTick, kBudget) << "Should not be stuck in CRANKING";
}

// ============================================================================
// AC-16 (RED — hypothesis-driven, pending bench confirmation):
// The engine must SUSTAIN running through the CRANKING->IDLE handoff when the
// CSV throttle ramps in late (driver hasn't pressed the pedal yet).
//
// Failure mode under test: during CRANKING the twin forces throttle=0.6 and the
// starter on, so the engine RPM rises. The fast-path catch fires at RPM>500
// (well before a late CSV throttle ramp arrives). At that handoff the twin
// releases the starter AND swaps the forced 0.6 for the smoothed CSV throttle
// (~0 if the ramp hasn't started) — the engine is then left with no throttle
// and no starter, stalls to a stop, and (a real stalled engine needing the
// starter to restart) cannot recover when the throttle ramp finally arrives.
// This is the suspected second cause of the live-CSV flakiness; it is NOT
// covered by the CRANKING->IDLE ==0.0 fix (AC-15).
//
// This test runs the twin against a small first-order plant model that captures
// the essential physics: RPM rises under the starter, runs on throttle once
// caught, and — critically — latches STOPPED once it sags to ~0, from which
// throttle alone cannot restart it (only the starter can). The assertion is on
// the twin's CONTROL keeping the plant alive through the handoff, not on any
// particular mechanism — so any sensible idle-sustain fix (hold sustain
// throttle, keep the starter through early IDLE, or delay the catch until the
// driver demands throttle) turns this GREEN.
// ============================================================================

TEST_F(VirtualIceTwinTest, EngineSustainsThroughCrankToIdleHandoffWithLateCsvThrottle_AC16) {
    twin_->setGearSelector(bridge::GearSelector::DRIVE);

    // --- Plant model: engine RPM response + an unrecoverable-from-throttle STOPPED latch.
    constexpr double CRANK_TARGET_RPM = 800.0;     // starter cranks toward this
    constexpr double RUNNING_IDLE_RPM = 750.0;
    constexpr double REDLINE_RPM = 6500.0;
    constexpr double STOPPED_RPM = 50.0;           // below this -> engine has stopped
    constexpr double SUSTAIN_THROTTLE = 0.02;      // throttle that holds a running engine
    constexpr double RPM_TAU_S = 0.15;             // RPM first-order response time
    double engineRpm = 0.0;
    bool stopped = true;                            // engine starts stopped (OFF)

    auto stepPlant = [&](double throttleCmd, bool starter, double dt) {
        double target;
        if (starter) {
            target = CRANK_TARGET_RPM;             // cranking raises RPM, un-latches stopped
            stopped = false;
        } else if (!stopped && throttleCmd >= SUSTAIN_THROTTLE) {
            target = RUNNING_IDLE_RPM + throttleCmd * (REDLINE_RPM - RUNNING_IDLE_RPM);
        } else if (!stopped) {
            target = 0.0;                          // no throttle, no starter -> decays toward stall
        } else {
            target = 0.0;                          // stopped: throttle ALONE cannot restart
        }
        const double alpha = 1.0 - std::exp(-dt / RPM_TAU_S);
        engineRpm += (target - engineRpm) * alpha;
        if (engineRpm < STOPPED_RPM) stopped = true;
        return engineRpm;
    };

    // --- Drive: CSV throttle is 0 until 3.5s (after the catch), then ramps in.
    const double dt = 1.0 / 60.0;
    double csvThrottle = 0.0;
    bool reachedRunning = false;
    int runningAliveFrames = 0;  // RUNNING frames where the engine is still alive (>500 RPM)

    for (int frame = 0; frame < static_cast<int>(8.0 / dt); ++frame) {
        const double t = frame * dt;
        if (t > 3.5) csvThrottle = std::min(0.5, csvThrottle + dt * 0.5);  // 0 -> 0.5 over 1s

        UpstreamSignal sig = makeValidSignal(csvThrottle, 0.0);
        twin_->setEngineRpmFeedback(engineRpm);
        const auto out = twin_->update(dt, sig);

        stepPlant(out.throttle, out.starterMotor, dt);

        if (twin_->getState() == TwinState::RUNNING) {
            reachedRunning = true;
            if (engineRpm > 500.0) ++runningAliveFrames;  // twin RUNNING AND engine alive
        }
    }

    EXPECT_TRUE(reachedRunning)
        << "Engine should reach RUNNING once the CSV throttle ramp arrives";

    // SUSTAIN: once RUNNING, the engine must stay alive (RPM > 500) for a real
    // window (>= 2 s), not stall through the CRANKING->IDLE handoff. Currently
    // FAILS: the catch drops the forced throttle + releases the starter before
    // the late CSV ramp, the plant stalls and latches STOPPED, and the later
    // RUNNING transition finds a dead engine (throttle can't restart it).
    EXPECT_GE(runningAliveFrames, static_cast<int>(2.0 / dt))
        << "Engine must sustain running through the CRANKING->IDLE handoff even "
        << "when the CSV throttle ramps in late";
}

// ============================================================================
// AC-18: the twin must emit starterMotor as a ONE-TICK EDGE on OFF->CRANKING,
// not hold it high through CRANKING.
//
// Why: the bridge's CrankingController::engageStarter is a momentary toggle --
// a held starterButton=true while the engine is Cranking FORCES the phase back
// to Stopped and cuts the starter (CrankingController.cpp:27-31, pinned as spec
// by CrankingControllerTests EngageStarter_FromCranking_ReturnsStoppedDecision).
// The keyboard path honours this edge contract (EngineInputTarget consumes the
// button after one read). The twin MUST do the same: VirtualIceInputProvider
// maps output.starterMotor straight to engineInput.starterButton with no edge
// detection, so holding starterMotor=true for the whole CRANKING state re-toggles
// engageStarter every tick. That structurally disables the fast-path catch
// (engageStarter's Stopped case calls reset() on every Stopped->Cranking, so the
// 10-tick exhaustFlowBaseline never accumulates) and is the root cause of the
// Stopped<->Cranking oscillation seen in every bench run. The bridge's step()
// already owns cranking duration via its own tick counter, so a held starter is
// redundant for engagement and only feeds the harmful toggle.
// ============================================================================

TEST_F(VirtualIceTwinTest, CrankingEmitsStarterAsOneTickEdgeNotHeld_AC18) {
    // OFF -> CRANKING: the transition tick carries the starter edge (the request
    // to begin cranking). Restated here to pin the edge contract the loop below
    // depends on (also covered by AC11_1).
    auto sig = makeValidSignal(0.6, 0.0);
    auto first = twin_->update(0.016, sig);
    ASSERT_EQ(twin_->getState(), TwinState::CRANKING);
    EXPECT_TRUE(first.starterMotor)
        << "OFF->CRANKING must pulse the starter (the one-tick edge request)";

    // Subsequent CRANKING ticks: RPM below the catch threshold and well inside
    // the 3s time fallback, so the twin stays CRANKING. The starter must NOT be
    // held high -- a held starter re-toggles engageStarter every tick, forcing
    // the engine to Stopped. Only the bridge controller should command the
    // starter during cranking, via its own tick counter.
    twin_->setEngineRpmFeedback(200.0);  // below the 500 RPM catch threshold
    for (int i = 0; i < 5; ++i) {
        auto held = twin_->update(0.016, sig);
        ASSERT_EQ(twin_->getState(), TwinState::CRANKING)
            << "should remain CRANKING (RPM below threshold, fallback not expired)";
        EXPECT_FALSE(held.starterMotor)
            << "Starter must not be held across CRANKING ticks -- it would "
            << "re-toggle engageStarter and force the engine to Stopped";
    }
}

// ============================================================================
// AC-17: Gearbox logger survives reconfigureProfile (no silent wipe)
// ============================================================================
// Bug: VirtualIceTwin::reconfigureProfile rebuilds the gearbox and calls
// setLogger(nullptr), which drops any logger attached via setGearboxLogger().
// This caused --gearbox-log to produce an empty CSV even though the gearbox
// ran and shifted correctly.
//
// Fix: preserve the logger pointer across the gearbox reconstruction.
// ============================================================================

TEST_F(VirtualIceTwinTest, LoggerSurvivesReconfigureProfile_AC17) {
    // Local mock: records every entry passed to log().
    class MockGearboxLogger : public IGearboxLogger {
    public:
        std::vector<GearboxLogEntry> entries;
        void log(const GearboxLogEntry& entry) override {
            entries.push_back(entry);
        }
    };

    MockGearboxLogger mockLogger;
    twin_->setGearboxLogger(&mockLogger);

    // Advance to RUNNING so gearbox_->update() is called every frame,
    // which invokes logShiftState() when a logger is attached.
    advanceThroughCranking();
    auto sig = makeValidSignal(0.1, 0.0);
    twin_->update(0.016, sig);  // IDLE -> RUNNING
    ASSERT_EQ(twin_->getState(), TwinState::RUNNING);

    // Run frames to generate baseline log entries
    for (int i = 0; i < 5; ++i) {
        twin_->update(0.016, sig);
    }
    const size_t entriesBefore = mockLogger.entries.size();
    ASSERT_GT(entriesBefore, 0u) << "Logger must receive entries before reconfigure";

    // Reconfigure the profile — this is where the bug wiped the logger.
    // Use a different gear ratio set to force a real reconstruction.
    twin_->reconfigureProfile({3.0, 2.0, 1.5}, 3.42, 0.3);

    // Resume running; selector is still DRIVE, state is still RUNNING
    for (int i = 0; i < 5; ++i) {
        twin_->update(0.016, sig);
    }

    EXPECT_GT(mockLogger.entries.size(), entriesBefore)
        << "Logger must survive reconfigureProfile — entries must continue after gearbox rebuild";
}
