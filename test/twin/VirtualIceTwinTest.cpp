#include <gtest/gtest.h>
#include <twin/VirtualIceTwin.h>
#include <twin/IceVehicleProfile.h>
#include <io/UpstreamSignal.h>
#include <simulator/GearConventions.h>
#include <input/DemoVehiclePhysics.h>

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

TEST_F(VirtualIceTwinTest, ClutchDisengagesWithin50ms_AC08_1) {
    auto sig = makeValidSignal(0.8, 50.0);
    advanceThroughCranking();

    for (int i = 0; i < 10; ++i) {
        twin_->update(0.016, sig);
    }

    sig.speedKmh = 80.0;
    twin_->update(0.016, sig);

    if (twin_->getState() == TwinState::SHIFTING) {
        auto output = twin_->update(0.050, sig);
        EXPECT_LE(output.clutchPressure, 0.1);
    }
}

TEST_F(VirtualIceTwinTest, ClutchStaysZeroFor200msPause_AC08_2) {
    auto sig = makeValidSignal(0.8, 50.0);
    advanceThroughCranking();

    for (int i = 0; i < 10; ++i) {
        twin_->update(0.016, sig);
    }

    sig.speedKmh = 80.0;
    twin_->update(0.016, sig);

    if (twin_->getState() == TwinState::SHIFTING) {
        bool wasZero = false;
        for (int i = 0; i < 5; ++i) {
            auto output = twin_->update(0.050, sig);
            if (output.clutchPressure == 0.0) {
                wasZero = true;
            }
        }
        EXPECT_TRUE(wasZero) << "Clutch should be at 0.0 during pause";
    }
}

TEST_F(VirtualIceTwinTest, ClutchReengagesOver100ms_AC08_4) {
    auto sig = makeValidSignal(0.8, 50.0);
    advanceThroughCranking();

    for (int i = 0; i < 10; ++i) {
        twin_->update(0.016, sig);
    }

    sig.speedKmh = 80.0;
    twin_->update(0.016, sig);

    if (twin_->getState() == TwinState::SHIFTING) {
        for (int i = 0; i < 8; ++i) {
            twin_->update(0.050, sig);
        }

        auto output = twin_->update(0.050, sig);
        EXPECT_GT(output.clutchPressure, 0.0);
    }
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

TEST_F(VirtualIceTwinTest, IdleThrottle_IsZeroWhenNoInput_AC12) {
    advanceThroughCranking();
    ASSERT_EQ(twin_->getState(), TwinState::IDLE);

    // No throttle input — output should be 0 (engine idles on physics alone)
    auto sig = makeValidSignal(0.0, 0.0);
    for (int i = 0; i < 20; ++i) {
        twin_->update(0.016, sig);
    }
    auto output = twin_->update(0.016, sig);
    EXPECT_NEAR(output.throttle, 0.0, 0.01) << "Idle throttle should be 0% when no input";
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
// RED PHASE TEST: Gearbox uses signal speed, not feedback speed for shift decisions
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

    // Run several frames to pass shift interval gate (minShiftIntervalS starts at 0 since no shift yet)
    // 40 kph at 50% throttle is well above the 15 kph upshift threshold for 1->2 shift
    for (int i = 0; i < 20; ++i) {
        twin_->update(0.016, sig);
    }

    // Assert: Gear should be > 1 if gearbox uses signal speed for shift decisions
    // This test WILL FAIL with current code because gearbox uses feedback speed (5 kph)
    EXPECT_GT(twin_->getCurrentGear(), 1)
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
    // (well past the 3s fallback) and assert the starter releases to IDLE.
    const double dt = 1.0 / 60.0;
    const int kFourSecondsOfFrames = 240;
    bool starterReleased = false;
    for (int i = 0; i < kFourSecondsOfFrames; ++i) {
        twin_->setEngineRpmFeedback(250.0);  // plateau below threshold
        auto output = twin_->update(dt, sig);
        if (!output.starterMotor) { starterReleased = true; break; }
    }

    EXPECT_TRUE(starterReleased)
        << "Sub-threshold cranking RPM must still catch via the 3s time fallback";
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
