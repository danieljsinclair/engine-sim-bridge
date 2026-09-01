#include <gtest/gtest.h>
#include <twin/EffectiveThrottle.h>

#include <vector>

using namespace twin;

namespace {

// Fixture rows are copied verbatim (values only) from the road-test evidence:
//   ReadingPickup_2026-08-30/AUTOPILOT-SIG-30s-95kmh.csv
//     AP cruise    ts 1788130890505: 95.60 km/h, pedal 0.00, torque +134 Nm
//     AP light     ts 1788130917908: 95.68 km/h, pedal 0.00, torque  +46 Nm
//     AP hard pull ts 1788130908964: 101.28 km/h, pedal 0.00, torque +598 Nm
//     AP regen     ts 1788130911222: 95.12 km/h, pedal 0.00, torque -262 Nm (brake light OFF)
// They are embedded here so the tests never depend on live/transient files.
constexpr double kCruiseTorqueNm = 134.0;
constexpr double kLightTorqueNm = 46.0;
constexpr double kHardPullTorqueNm = 598.0;
constexpr double kRegenTorqueNm = -262.0;
constexpr double kDenominatorNm = 600.0;

EffectiveThrottleConfig enabledConfig() {
    EffectiveThrottleConfig config;
    config.enabled = true;
    return config;
}

}  // namespace

// ---------------------------------------------------------------------------
// Config contract — the pinned constants ARE the spec (see EffectiveThrottle.h
// for the evidence-based justification of each).
// ---------------------------------------------------------------------------
TEST(EffectiveThrottleConfigTest, DefaultsAreOffAndPinnedToEvidence) {
    EffectiveThrottleConfig config;

    EXPECT_FALSE(config.enabled);
    EXPECT_DOUBLE_EQ(config.maxAxleTorqueNm, 600.0);
    EXPECT_DOUBLE_EQ(config.torqueToThrottleGain, 1.0);
    EXPECT_DOUBLE_EQ(config.pedalDeadbandFraction, 0.02);
    EXPECT_DOUBLE_EQ(config.engageTorqueNm, 20.0);
    EXPECT_DOUBLE_EQ(config.releaseTorqueNm, 10.0);
}

// ---------------------------------------------------------------------------
// OFF contract — default must be a byte-identical no-op.
// ---------------------------------------------------------------------------
TEST(EffectiveThrottleTest, DisabledPassesPedalThroughUnchanged) {
    EffectiveThrottleDerivation derivation(EffectiveThrottleConfig{});

    // The exact inputs that make the engine silent today, plus pedal-active and
    // regen phases. With the feature off every pedal value must come back
    // bit-identical and the latch must never arm.
    const std::vector<std::pair<double, double>> frames = {
        {0.0, kHardPullTorqueNm},   // AP hard pull, foot off — the killer case
        {0.0, kCruiseTorqueNm},     // AP cruise, foot off
        {0.30, kCruiseTorqueNm},    // driver's foot down
        {0.0, kRegenTorqueNm},      // AP regen
        {0.5, 0.0},                 // plain pedal
        {0.0, 0.0},                 // plain coast
    };
    for (const auto& [pedal, torque] : frames) {
        EXPECT_DOUBLE_EQ(derivation.update(pedal, torque), pedal);
        EXPECT_FALSE(derivation.isTorqueDriving());
    }
}

// ---------------------------------------------------------------------------
// ON — happy path: Autopilot cruise/pull must not read as silent.
// ---------------------------------------------------------------------------
TEST(EffectiveThrottleTest, ApCruiseIsNotSilent) {
    EffectiveThrottleDerivation derivation(enabledConfig());

    const double effective = derivation.update(0.0, kCruiseTorqueNm);

    EXPECT_DOUBLE_EQ(effective, kCruiseTorqueNm / kDenominatorNm);
    EXPECT_TRUE(derivation.isTorqueDriving());
}

TEST(EffectiveThrottleTest, ApHardPullReadsNearWideOpenThrottle) {
    EffectiveThrottleDerivation derivation(enabledConfig());

    const double effective = derivation.update(0.0, kHardPullTorqueNm);

    EXPECT_DOUBLE_EQ(effective, kHardPullTorqueNm / kDenominatorNm);
}

TEST(EffectiveThrottleTest, LightApTorqueStillEngages) {
    EffectiveThrottleDerivation derivation(enabledConfig());

    const double effective = derivation.update(0.0, kLightTorqueNm);

    EXPECT_DOUBLE_EQ(effective, kLightTorqueNm / kDenominatorNm);
    EXPECT_TRUE(derivation.isTorqueDriving());
}

TEST(EffectiveThrottleTest, TorqueAboveDenominatorSaturates) {
    EffectiveThrottleDerivation derivation(enabledConfig());

    const double effective = derivation.update(0.0, 5000.0);

    EXPECT_DOUBLE_EQ(effective, 1.0);
}

// ---------------------------------------------------------------------------
// ON — regen and true coast contribute nothing (engine drive = pedal).
// ---------------------------------------------------------------------------
TEST(EffectiveThrottleTest, RegenTorqueContributesZeroThrottle) {
    EffectiveThrottleDerivation derivation(enabledConfig());

    EXPECT_DOUBLE_EQ(derivation.update(0.0, kRegenTorqueNm), 0.0);
    EXPECT_FALSE(derivation.isTorqueDriving());

    // AP brake event magnitudes (ReadingPickup_2026-08-31_001408/drive.csv)
    // must not raise the drive either — braking is the gearbox feature's job.
    EXPECT_DOUBLE_EQ(derivation.update(0.0, -986.0), 0.0);
}

TEST(EffectiveThrottleTest, TorqueBelowEngageThresholdIsTrueCoast) {
    EffectiveThrottleDerivation derivation(enabledConfig());

    EXPECT_DOUBLE_EQ(derivation.update(0.0, 15.0), 0.0);
    EXPECT_FALSE(derivation.isTorqueDriving());
}

// ---------------------------------------------------------------------------
// ON — hysteresis on AP takeover/release.
// ---------------------------------------------------------------------------
TEST(EffectiveThrottleTest, LatchHoldsThroughHysteresisBand) {
    EffectiveThrottleDerivation derivation(enabledConfig());

    derivation.update(0.0, kCruiseTorqueNm);  // engage at 134 Nm

    // 15 Nm sits in the release(10)..engage(20) band: latch HOLDS.
    EXPECT_DOUBLE_EQ(derivation.update(0.0, 15.0), 15.0 / kDenominatorNm);
    EXPECT_TRUE(derivation.isTorqueDriving());

    // Below release: latch opens, drive falls back to the pedal.
    EXPECT_DOUBLE_EQ(derivation.update(0.0, 5.0), 0.0);
    EXPECT_FALSE(derivation.isTorqueDriving());

    // Back inside the band: must NOT re-engage (re-engage needs >= engage).
    EXPECT_DOUBLE_EQ(derivation.update(0.0, 15.0), 0.0);
    EXPECT_FALSE(derivation.isTorqueDriving());

    // Above engage: re-engages.
    EXPECT_DOUBLE_EQ(derivation.update(0.0, 25.0), 25.0 / kDenominatorNm);
    EXPECT_TRUE(derivation.isTorqueDriving());
}

TEST(EffectiveThrottleTest, PedalOwnershipReleasesAndReengages) {
    EffectiveThrottleDerivation derivation(enabledConfig());

    derivation.update(0.0, kCruiseTorqueNm);  // engaged

    // Driver's foot returns: pedal owns the drive exactly, torque term off.
    EXPECT_DOUBLE_EQ(derivation.update(0.50, kCruiseTorqueNm), 0.50);
    EXPECT_FALSE(derivation.isTorqueDriving());

    // Foot off again while AP still pulls: re-engages.
    EXPECT_DOUBLE_EQ(derivation.update(0.0, kCruiseTorqueNm),
                     kCruiseTorqueNm / kDenominatorNm);
    EXPECT_TRUE(derivation.isTorqueDriving());
}

TEST(EffectiveThrottleTest, DeadbandPedalBlendsWithTorqueTerm) {
    EffectiveThrottleDerivation derivation(enabledConfig());

    derivation.update(0.0, kCruiseTorqueNm);  // engaged at 22.3%

    // Pedal inside the deadband: the larger of pedal vs torque term wins.
    EXPECT_DOUBLE_EQ(derivation.update(0.015, kCruiseTorqueNm),
                     kCruiseTorqueNm / kDenominatorNm);

    // A lighter torque term lets the deadband pedal show through.
    EXPECT_DOUBLE_EQ(derivation.update(0.015, 5.0), 0.015);
}

TEST(EffectiveThrottleTest, ResetClearsTheLatch) {
    EffectiveThrottleDerivation derivation(enabledConfig());

    derivation.update(0.0, kCruiseTorqueNm);  // engaged
    derivation.reset();

    EXPECT_FALSE(derivation.isTorqueDriving());
    // Band torque must not drive after a reset (latch is gone).
    EXPECT_DOUBLE_EQ(derivation.update(0.0, 15.0), 0.0);
}
