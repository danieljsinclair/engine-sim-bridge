#include <gtest/gtest.h>
#include <twin/UpstreamTorqueHint.h>

using namespace twin;

namespace {

// Fixture torques are copied verbatim from the road-test evidence:
//   ReadingPickup_2026-08-30/AUTOPILOT-SIG-30s-95kmh.csv
//     AP cruise     +134 Nm (1,998 rows at 95.6 km/h, pedal 0.00)
//     AP light       +46 Nm (95.68 km/h, pedal 0.00)
//     AP hard pull  +598 Nm (641 rows, pedal 0.00 — the observed max)
//     AP regen      -262 Nm (95.12 km/h, pedal 0.00, brake light OFF)
//   ReadingPickup_2026-08-31_001408/drive.csv — AP brake events where the
//   pedal switch stays 0.00 with the brake light on:
//     event 1  ts 1788131660569: 98.88 km/h,  -986 Nm
//     event 2  ts 1788131668175: 105.12 km/h, -668 Nm
//     event 3  ts 1788131686372: 104.56 km/h, -910 Nm
constexpr double kCruiseTorqueNm = 134.0;
constexpr double kHardPullTorqueNm = 598.0;
constexpr double kDriveDenominatorNm = 600.0;
constexpr double kRegenDenominatorNm = 1000.0;

TorqueInformedGearboxConfig enabledConfig() {
    TorqueInformedGearboxConfig config;
    config.enabled = true;
    return config;
}

}  // namespace

// ---------------------------------------------------------------------------
// Config contract — pinned constants ARE the spec (see UpstreamTorqueHint.h).
// ---------------------------------------------------------------------------
TEST(TorqueInformedGearboxConfigTest, DefaultsAreOffAndPinnedToEvidence) {
    TorqueInformedGearboxConfig config;

    EXPECT_FALSE(config.enabled);
    EXPECT_DOUBLE_EQ(config.maxDriveTorqueNm, 600.0);
    EXPECT_DOUBLE_EQ(config.maxRegenTorqueNm, 1000.0);
    EXPECT_DOUBLE_EQ(config.maxShiftBias, 0.30);
    EXPECT_DOUBLE_EQ(config.engageTorqueNm, 20.0);
}

// ---------------------------------------------------------------------------
// OFF contract — disabled must be indistinguishable from NullTorqueHint.
// ---------------------------------------------------------------------------
TEST(UpstreamTorqueHintTest, DisabledHintIsAlwaysZeroBias) {
    const double torques[] = {kHardPullTorqueNm, kCruiseTorqueNm,
                              -986.0, 0.0, 15.0, -15.0};

    for (const double torque : torques) {
        UpstreamTorqueHint hint(TorqueInformedGearboxConfig{}, torque);
        EXPECT_DOUBLE_EQ(hint.shiftBias(), 0.0) << "torque=" << torque;
    }
}

// ---------------------------------------------------------------------------
// ON — AP pull (positive torque at pedal 0) reads as accelerating demand.
// ---------------------------------------------------------------------------
TEST(UpstreamTorqueHintTest, ApCruisePullProducesPositiveBias) {
    UpstreamTorqueHint hint(enabledConfig(), kCruiseTorqueNm);

    EXPECT_DOUBLE_EQ(hint.shiftBias(),
                     kCruiseTorqueNm / kDriveDenominatorNm * 0.30);
}

TEST(UpstreamTorqueHintTest, ApHardPullProducesPositiveBias) {
    UpstreamTorqueHint hint(enabledConfig(), kHardPullTorqueNm);

    EXPECT_DOUBLE_EQ(hint.shiftBias(),
                     kHardPullTorqueNm / kDriveDenominatorNm * 0.30);
}

TEST(UpstreamTorqueHintTest, BiasIsMonotonicInPullTorque) {
    const double torques[] = {kCruiseTorqueNm / 2.0, kCruiseTorqueNm, 300.0,
                              kHardPullTorqueNm};
    double previous = -1.0;

    for (const double torque : torques) {
        UpstreamTorqueHint hint(enabledConfig(), torque);
        const double bias = hint.shiftBias();
        EXPECT_GT(bias, previous) << "torque=" << torque;
        previous = bias;
    }
}

// ---------------------------------------------------------------------------
// ON — AP braking (negative torque, brake light on, pedal 0) reads as braking
// demand: POSITIVE bias (downshift-promoting), not the coast read.
// ---------------------------------------------------------------------------
TEST(UpstreamTorqueHintTest, ApBrakeEventsProducePositiveBias) {
    // All three recorded AP brake events.
    const double regenTorques[] = {-986.0, -668.0, -910.0};
    const double denominators[] = {kRegenDenominatorNm, kRegenDenominatorNm,
                                   kRegenDenominatorNm};

    for (size_t i = 0; i < 3; ++i) {
        UpstreamTorqueHint hint(enabledConfig(), regenTorques[i]);
        const double bias = hint.shiftBias();
        EXPECT_GT(bias, 0.0) << "braking must not read as coast";
        EXPECT_DOUBLE_EQ(bias, -regenTorques[i] / denominators[i] * 0.30);
    }
}

TEST(UpstreamTorqueHintTest, RegenWithoutBrakeLightStillBiases) {
    // -262 Nm lift-off regen (brake light OFF) is still deceleration demand;
    // the hint keys on torque, not the light — the light is corroborating
    // evidence in the captures, not an input.
    UpstreamTorqueHint hint(enabledConfig(), -262.0);

    EXPECT_DOUBLE_EQ(hint.shiftBias(), 262.0 / kRegenDenominatorNm * 0.30);
}

// ---------------------------------------------------------------------------
// ON — true coast stays exactly today's read.
// ---------------------------------------------------------------------------
TEST(UpstreamTorqueHintTest, TorqueBelowEngageIsTrueCoast) {
    const double coastTorques[] = {15.0, -15.0, 0.0};

    for (const double torque : coastTorques) {
        UpstreamTorqueHint hint(enabledConfig(), torque);
        EXPECT_DOUBLE_EQ(hint.shiftBias(), 0.0) << "torque=" << torque;
    }
}

// ---------------------------------------------------------------------------
// ON — saturation at the cap for both directions.
// ---------------------------------------------------------------------------
TEST(UpstreamTorqueHintTest, DriveTorqueSaturatesAtBiasCap) {
    UpstreamTorqueHint hint(enabledConfig(), 5000.0);

    EXPECT_DOUBLE_EQ(hint.shiftBias(), 0.30);
}

TEST(UpstreamTorqueHintTest, RegenTorqueSaturatesAtBiasCap) {
    UpstreamTorqueHint hint(enabledConfig(), -1500.0);

    EXPECT_DOUBLE_EQ(hint.shiftBias(), 0.30);
}

// ---------------------------------------------------------------------------
// The gearbox decision consumes the ITorqueHint interface — the hint must be
// usable polymorphically (no slicing, no concrete-type dependency).
// ---------------------------------------------------------------------------
TEST(UpstreamTorqueHintTest, UsableThroughITorqueHintInterface) {
    UpstreamTorqueHint concrete(enabledConfig(), kCruiseTorqueNm);
    const ITorqueHint& hint = concrete;

    EXPECT_DOUBLE_EQ(hint.shiftBias(),
                     kCruiseTorqueNm / kDriveDenominatorNm * 0.30);
}
