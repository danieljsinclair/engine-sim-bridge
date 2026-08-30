// PinTargetChaseTest - behavior contract for the PIN-coupling compliance filter.
//
// Context (the bug this guards): the road-speed signal the PIN coupling pins
// the sim vehicle speed to updates only ~5.5 Hz in ~0.9 km/h steps (CAN
// median-hold), so the rigid pin teleports wheel speed — and with it engine
// rpm / pitch — onto each held level ("piano keys"). The chase filter gives
// the pin a finite response instead.
//
// These tests assert OWNER-VISIBLE behavior, not the integrator internals:
//   - tau=0 is exactly today's rigid pin (the regression contract),
//   - a fresh engage snaps (no ramp from stale zero),
//   - the unpinned sentinel passes through and rearms,
//   - a step is chased smoothly (no teleport, no overshoot, error -> 0),
//   - the measured 5.5 Hz / 0.9 km/h staircase is smoothed.
#include <gtest/gtest.h>
#include <twin/PinTargetChase.h>
#include <cmath>
#include <vector>

using namespace twin;

namespace {
// The measured pathology from the road capture: road speed holds ~181 ms per
// level and steps ~0.9 km/h (5.5 Hz staircase).
constexpr double kFrameDt = 0.016;      // ~60 Hz twin frame
constexpr double kStepKmh = 0.9;        // held-level step size
constexpr double kHoldS = 0.181;        // median hold between steps
constexpr int kFramesPerHold = static_cast<int>(kHoldS / kFrameDt);  // 11
}  // namespace

TEST(PinTargetChaseTest, TauZero_IsExactRigidPassthrough) {
    PinTargetChase chase;
    chase.setTauMs(0.0);
    // A held-then-stepping signal must come back VERBATIM: tau=0 is today's
    // rigid pin, bit-identical (the --pin-tau-ms 0 regression contract).
    const double levels[] = {0.0, 0.9, 1.8, 4.7, 4.7, 3.1};
    for (int rep = 0; rep < 3; ++rep) {
        for (const double level : levels) {
            for (int f = 0; f < kFramesPerHold; ++f) {
                ASSERT_EQ(chase.update(kFrameDt, level), level)
                    << "tau=0 must return the raw target untouched";
            }
        }
    }
}

TEST(PinTargetChaseTest, FreshEngage_SnapsToTarget_NotFromStaleZero) {
    PinTargetChase chase;
    chase.setTauMs(150.0);
    // First pinned frame: return the target exactly. Ramping up from a
    // stale zero would brake a moving vehicle to a crawl on engage.
    EXPECT_EQ(chase.update(kFrameDt, 42.0), 42.0);
}

TEST(PinTargetChaseTest, SentinelUnpinned_PassesThroughAndRearms) {
    PinTargetChase chase;
    chase.setTauMs(150.0);
    ASSERT_EQ(chase.update(kFrameDt, 30.0), 30.0);  // engage at 30
    for (int i = 0; i < 5; ++i) chase.update(kFrameDt, 30.5);  // chasing

    // -1 is "no pin" (FREE/Torque/neutral frames): must surface verbatim —
    // NEVER chased toward as if it were a speed.
    for (int i = 0; i < 3; ++i) {
        EXPECT_EQ(chase.update(kFrameDt, -1.0), -1.0);
    }

    // After the unpinned spell, the next pinned target snaps (fresh engage),
    // rather than gliding from the pre-neutral 30.x toward 12.
    EXPECT_EQ(chase.update(kFrameDt, 12.0), 12.0);
}

TEST(PinTargetChaseTest, Step_IsChased_NotTeleported) {
    PinTargetChase chase;
    chase.setTauMs(150.0);
    ASSERT_EQ(chase.update(kFrameDt, 0.0), 0.0);  // engage at standstill

    // A 10 km/h step arrives. The rigid pin would put the full 10 on the
    // wheels in one frame; the chase must move a small fraction of it.
    const double first = chase.update(kFrameDt, 10.0);
    EXPECT_GT(first, 0.0) << "the chase must start moving immediately";
    EXPECT_LT(first, 1.0) << "the first frame must not teleport the target";
}

TEST(PinTargetChaseTest, Step_NoOvershoot_MonotonicApproach) {
    PinTargetChase chase;
    chase.setTauMs(150.0);
    ASSERT_EQ(chase.update(kFrameDt, 0.0), 0.0);

    double prev = 0.0;
    for (int i = 0; i < 200; ++i) {  // 3.2 s >> settling
        const double v = chase.update(kFrameDt, 10.0);
        EXPECT_LE(v, 10.0) << "critically damped: never overshoots the target";
        EXPECT_GE(v, prev) << "monotonic approach toward a held step-up target";
        prev = v;
    }
    EXPECT_NEAR(prev, 10.0, 0.05) << "zero steady-state error on a held target";
}

TEST(PinTargetChaseTest, StepDown_AlsoChased) {
    PinTargetChase chase;
    chase.setTauMs(150.0);
    ASSERT_EQ(chase.update(kFrameDt, 10.0), 10.0);

    double prev = 10.0;
    for (int i = 0; i < 100; ++i) {
        const double v = chase.update(kFrameDt, 0.0);
        EXPECT_GE(v, 0.0) << "never undershoots below a held step-down target";
        EXPECT_LE(v, prev) << "monotonic descent toward a held step-down target";
        prev = v;
    }
    EXPECT_NEAR(prev, 0.0, 0.05) << "zero steady-state error after step-down";
}

TEST(PinTargetChaseTest, StepResponse_SettlesToTheTarget) {
    PinTargetChase chase;
    chase.setTauMs(150.0);
    ASSERT_EQ(chase.update(kFrameDt, 0.0), 0.0);
    const double tau = 0.150;
    const double target = 10.0;

    // Critically-damped 2nd order: error (1+t/tau)e^(-t/tau). Convergence is
    // honest here — 95% by ~5 tau, ~1% by ~8 tau — a bit slower than a 1st-
    // order lowpass's 3 tau, in exchange for a continuous glide SLOPE.
    auto errorAfter = [&](int frames) {
        double v = 0.0;
        for (int i = 0; i < frames; ++i) v = chase.update(kFrameDt, target);
        return target - v;
    };
    EXPECT_NEAR(errorAfter(static_cast<int>(3 * tau / kFrameDt)), 2.1, 0.6)
        << "3 tau: most of the way there (~79%)";
    EXPECT_LT(errorAfter(static_cast<int>(5 * tau / kFrameDt)), 0.75)
        << "5 tau: ~95% settled";
    EXPECT_LT(errorAfter(static_cast<int>(8 * tau / kFrameDt)), 0.1)
        << "8 tau: steady-state error ~0";
}

TEST(PinTargetChaseTest, MeasuredStaircase_FrameToFrameJumpShrinks) {
    PinTargetChase chase;
    chase.setTauMs(150.0);
    ASSERT_EQ(chase.update(kFrameDt, 0.0), 0.0);

    // 20 held levels of +0.9 km/h each: the rigid pin moves the full 0.9 in
    // one frame every 181 ms (the audible staircase). The chase must spread
    // each step across the hold — max per-frame motion well under half a raw
    // step — while still gaining speed across the whole staircase.
    double maxRawJump = 0.0;
    double maxChasedJump = 0.0;
    double prevRaw = 0.0;
    double prevChased = 0.0;
    double finalChased = 0.0;
    double finalRaw = 0.0;
    for (int level = 1; level <= 20; ++level) {
        const double raw = level * kStepKmh;
        for (int f = 0; f < kFramesPerHold; ++f) {
            const double out = chase.update(kFrameDt, raw);
            maxRawJump = std::max(maxRawJump, std::abs(raw - prevRaw));
            maxChasedJump = std::max(maxChasedJump, std::abs(out - prevChased));
            prevRaw = raw;
            prevChased = out;
            finalChased = out;
        }
        finalRaw = raw;
    }
    EXPECT_NEAR(maxRawJump, kStepKmh, 1e-9) << "sanity: the raw signal steps in full levels";
    EXPECT_LT(maxChasedJump, 0.45 * kStepKmh)
        << "chased per-frame jump must be well under half a raw staircase step";
    // And the chase must actually TRACK the staircase: after 20 levels it is
    // within a small lag of the final level (not drifting behind).
    EXPECT_GT(finalChased, finalRaw - 1.5)
        << "chase keeps up with a sustained 0.9 km/h / 181 ms ramp";
}

TEST(PinTargetChaseTest, HeldLevel_ConvergesExactlyBetweenWidelySpacedSteps) {
    PinTargetChase chase;
    chase.setTauMs(150.0);
    ASSERT_EQ(chase.update(kFrameDt, 5.0), 5.0);

    // A step to 8 followed by a 2 s hold (>> 8 tau): the surfaced target must
    // settle ON the held level — no offset, no drift.
    double v = 5.0;
    for (int i = 0; i < static_cast<int>(2.0 / kFrameDt); ++i) {
        v = chase.update(kFrameDt, 8.0);
    }
    EXPECT_NEAR(v, 8.0, 0.02);
}

TEST(PinTargetChaseTest, SustainedRamp_BoundedTrailingLag) {
    // The mph trade-off, stated as a contract: during a SUSTAINED ramp (a
    // staircase IS a quantized ramp) a critically-damped chase trails by the
    // textbook 2*tau*v, never more (a slower/broken filter would trail
    // without bound) and never AHEAD (a lead - e.g. derivative feedforward -
    // overshoots isolated level changes). v = step/hold: ~5 km/h/s at the
    // measured cadence, ~20 km/h/s on a hard pull.
    const double tauS = 0.150;
    for (const double stepKmh : {0.9, 3.6}) {
        const double rampRate = stepKmh / kHoldS;
        PinTargetChase chase;
        chase.setTauMs(150.0);
        ASSERT_EQ(chase.update(kFrameDt, 0.0), 0.0);
        double errSum = 0.0;
        double maxErr = 0.0;
        double minErr = 0.0;
        int measured = 0;
        for (int level = 1; level <= 30; ++level) {
            const double raw = level * stepKmh;
            for (int f = 0; f < kFramesPerHold; ++f) {
                const double out = chase.update(kFrameDt, raw);
                if (level > 5) {  // skip the engage transient
                    const double err = out - raw;
                    errSum += err;
                    maxErr = std::max(maxErr, err);
                    minErr = std::min(minErr, err);
                    ++measured;
                }
            }
        }
        const double bound = 2.0 * tauS * rampRate;
        EXPECT_GT(errSum / measured, -(bound + stepKmh))
            << "step " << stepKmh << ": mean lag must stay within 2*tau*v + one level";
        EXPECT_LT(errSum / measured, 0.5 * stepKmh)
            << "step " << stepKmh << ": the chase must not run AHEAD of the ramp";
        EXPECT_LT(maxErr, stepKmh)
            << "step " << stepKmh << ": instantaneous error stays under one level";
        EXPECT_GT(minErr, -(bound + stepKmh))
            << "step " << stepKmh << ": instantaneous lag bounded by 2*tau*v + one level";
    }
}

TEST(PinTargetChaseTest, NegativeTau_IsRigidPassthrough) {
    PinTargetChase chase;
    chase.setTauMs(-5.0);  // malformed input treated as "off", mirroring ThrottleSmoother
    EXPECT_EQ(chase.update(kFrameDt, 7.0), 7.0);
    EXPECT_EQ(chase.update(kFrameDt, 7.9), 7.9);
}

TEST(PinTargetChaseTest, SmallerTau_MovesFaster) {
    PinTargetChase slow;
    PinTargetChase fast;
    slow.setTauMs(150.0);
    fast.setTauMs(60.0);
    ASSERT_EQ(slow.update(kFrameDt, 0.0), 0.0);
    ASSERT_EQ(fast.update(kFrameDt, 0.0), 0.0);
    double slowV = 0.0;
    double fastV = 0.0;
    for (int i = 0; i < 10; ++i) {
        slowV = slow.update(kFrameDt, 10.0);
        fastV = fast.update(kFrameDt, 10.0);
    }
    EXPECT_GT(fastV, slowV) << "a smaller tau must close on the target faster";
    EXPECT_GT(fast.getTauMs(), 0.0);  // tau is readable (CLI echo/diagnostics)
}
