// TwinTorqueInputWiringTest.cpp — twin-level behaviour of the two CLI-toggled
// torque features against EMBEDDED road-test fixture rows.
//
// Fixture rows are copied verbatim (values only) from:
//   ReadingPickup_2026-08-30/AUTOPILOT-SIG-30s-95kmh.csv
//     AP cruise    ts 1788130890505: 95.60 km/h, pedal 0.00, +134 Nm
//     AP light     ts 1788130917908: 95.68 km/h, pedal 0.00,  +46 Nm
//     AP hard pull ts 1788130908964: 101.28 km/h, pedal 0.00, +598 Nm
//     AP regen     ts 1788130911222: 95.12 km/h, pedal 0.00, -262 Nm (brake light OFF)
//   ReadingPickup_2026-08-31_001408/drive.csv — AP brake events, pedal stays 0.00:
//     event 1  ts 1788131660569: 98.88 km/h, brake light ON, -986 Nm
//     event 2  ts 1788131668175: 105.12 km/h, brake light ON, -668 Nm
//     event 3  ts 1788131686372: 104.56 km/h, brake light ON, -910 Nm
// No test reads those files at runtime.
//
// Assertions are RELATIVE (feature ON vs feature OFF twins driven frame-by-
// frame identically) so they stay honest to behaviour, not to tuning knobs.

#include <gtest/gtest.h>
#include <twin/VirtualIceTwin.h>
#include <twin/IceVehicleProfile.h>
#include <io/UpstreamSignal.h>
#include <simulator/GearConventions.h>
#include <memory>
#include <vector>

using namespace twin;
using namespace input;

namespace {

constexpr double kDt = 0.016;
constexpr double kCruiseRpmFeedback = 1800.0;  // held above idle: floors stay inert

struct FixtureRow {
    double speedKmh;
    double pedalFraction;
    double motorTorqueNm;
    const char* provenance;
};

const FixtureRow kApCruise   = {95.60, 0.0, 134.0, "AUTOPILOT-SIG ts 1788130890505"};
const FixtureRow kApLight    = {95.68, 0.0, 46.0,  "AUTOPILOT-SIG ts 1788130917908"};
const FixtureRow kApHardPull = {101.28, 0.0, 598.0, "AUTOPILOT-SIG ts 1788130908964"};
const FixtureRow kApRegen    = {95.12, 0.0, -262.0, "AUTOPILOT-SIG ts 1788130911222"};

UpstreamSignal makeSignal(double throttleFraction, double speedKmh, double motorTorqueNm) {
    UpstreamSignal sig;
    sig.throttleFraction = throttleFraction;
    sig.speedKmh = speedKmh;
    sig.motorTorqueNm = motorTorqueNm;
    sig.timestampUtcMs = 1000;
    sig.isValid = true;
    return sig;
}

UpstreamSignal makeSignal(const FixtureRow& row) {
    return makeSignal(row.pedalFraction, row.speedKmh, row.motorTorqueNm);
}

// Field-by-field TwinOutput identity: the byte-identical-when-off contract at
// twin level. Every observable field, not just the ones the features touch.
void expectIdentical(const TwinOutput& a, const TwinOutput& b, int frame) {
    EXPECT_DOUBLE_EQ(a.throttle, b.throttle) << "frame " << frame;
    EXPECT_EQ(a.gear, b.gear) << "frame " << frame;
    EXPECT_DOUBLE_EQ(a.clutchPressure, b.clutchPressure) << "frame " << frame;
    EXPECT_EQ(a.starterMotor, b.starterMotor) << "frame " << frame;
    EXPECT_EQ(a.ignition, b.ignition) << "frame " << frame;
    EXPECT_EQ(a.gearSelector, b.gearSelector) << "frame " << frame;
    EXPECT_DOUBLE_EQ(a.pinVehicleSpeedTargetKmh, b.pinVehicleSpeedTargetKmh) << "frame " << frame;
    EXPECT_DOUBLE_EQ(a.dynoTorqueScale, b.dynoTorqueScale) << "frame " << frame;
    EXPECT_DOUBLE_EQ(a.drivetrainInputTorqueNm, b.drivetrainInputTorqueNm) << "frame " << frame;
    EXPECT_DOUBLE_EQ(a.roadImpliedRpm, b.roadImpliedRpm) << "frame " << frame;
    EXPECT_EQ(a.creepReliefFired, b.creepReliefFired) << "frame " << frame;
    EXPECT_EQ(a.couplingIsTorqueConverter, b.couplingIsTorqueConverter) << "frame " << frame;
}

}  // namespace

class TwinTorqueInputWiringTest : public ::testing::Test {
protected:
    std::unique_ptr<VirtualIceTwin> makeTwin() {
        auto t = std::make_unique<VirtualIceTwin>(IceVehicleProfile::zf8hp45());
        t->setIgnition(true);
        return t;
    }

    // OFF -> CRANKING -> IDLE with selector DRIVE (mirrors VirtualIceTwinTest).
    void advanceThroughCranking(VirtualIceTwin& twin) {
        auto sig = makeSignal(0.6, 0.0, 0.0);
        twin.update(kDt, sig);               // OFF -> CRANKING
        twin.setEngineRpmFeedback(800.0);
        twin.update(kDt, sig);               // CRANKING -> IDLE
        twin.setGearSelector(bridge::GearSelector::DRIVE);
        twin.setEngineRpmFeedback(kCruiseRpmFeedback);
    }

    // RUNNING, ramp 0 -> cruiseKmh at light pedal so the zf8hp45 climbs into
    // 8th (upshift interval 2 s x 7 shifts => a long, generous ramp), then
    // settle at cruise with pedal 0 / torque 0 until top gear (poll-based,
    // timing-agnostic).
    void driveToTopGearAtCruise(VirtualIceTwin& twin, double cruiseKmh) {
        auto sig = makeSignal(0.1, 0.0, 0.0);
        twin.update(kDt, sig);               // IDLE -> RUNNING

        const int rampFrames = 1100;         // ~17.6 s
        for (int i = 1; i <= rampFrames; ++i) {
            sig.speedKmh = cruiseKmh * static_cast<double>(i) / rampFrames;
            twin.setVehicleSpeedFeedback(sig.speedKmh);
            twin.update(kDt, sig);
        }
        sig.throttleFraction = 0.0;
        const int settleFrames = 1200;       // generous; breaks as soon as 8th
        for (int i = 0; i < settleFrames && twin.getCurrentGear() < 8; ++i) {
            twin.update(kDt, sig);
        }
    }

    EffectiveThrottleConfig effectiveThrottleOn() {
        EffectiveThrottleConfig config;
        config.enabled = true;
        return config;
    }

    TorqueInformedGearboxConfig torqueGearboxOn() {
        TorqueInformedGearboxConfig config;
        config.enabled = true;
        return config;
    }
};

// ---------------------------------------------------------------------------
// OFF contracts — DEFAULT OFF must be provably inert.
// ---------------------------------------------------------------------------

TEST_F(TwinTorqueInputWiringTest, DefaultConfigsAreByteIdenticalToUnconfiguredTwin) {
    auto configured = makeTwin();
    auto unconfigured = makeTwin();

    // Explicitly setting the DEFAULT (disabled) configs must change nothing:
    // the twin a user gets without ever touching the flags is bit-for-bit the
    // twin a user gets after passing no-op config. Both features, both sides.
    configured->setEffectiveThrottleConfig(EffectiveThrottleConfig{});
    configured->setTorqueInformedGearboxConfig(TorqueInformedGearboxConfig{});

    advanceThroughCranking(*configured);
    advanceThroughCranking(*unconfigured);

    // A mixed drive: foot-off cruise, AP pull, regen, AP brake, foot-down.
    const std::vector<FixtureRow> phases = {
        kApCruise, kApCruise, kApLight, kApHardPull, kApRegen,
        {98.88, 0.0, -986.0, "drive.csv event 1"},
        {105.12, 0.0, -668.0, "drive.csv event 2"},
        {95.60, 0.30, 0.0, "driver pedal returns"},
        {95.60, 0.0, 0.0, "plain coast"},
    };
    constexpr int kFramesPerPhase = 30;
    int frame = 0;
    for (const FixtureRow& row : phases) {
        const auto sig = makeSignal(row);
        for (int i = 0; i < kFramesPerPhase; ++i) {
            ++frame;
            const TwinOutput a = configured->update(kDt, sig);
            const TwinOutput b = unconfigured->update(kDt, sig);
            expectIdentical(a, b, frame);
        }
    }
}

TEST_F(TwinTorqueInputWiringTest, TorqueInSignalAloneChangesNothingToday) {
    // Today's contract: the twin ignores UpstreamSignal::motorTorqueNm except
    // in Torque/MATCH coupling. Feeding AP-pull torque to an UNCONFIGURED twin
    // must leave the engine silent (this is the bug being fixed, stated as the
    // current baseline so the feature's ON test below is provably a change).
    auto twin = makeTwin();
    advanceThroughCranking(*twin);
    driveToTopGearAtCruise(*twin, kApCruise.speedKmh);

    auto silent = makeSignal(0.0, kApCruise.speedKmh, 0.0);
    twin->update(kDt, silent);
    const double baselineThrottle = [&]() {
        for (int i = 0; i < 60; ++i) twin->update(kDt, silent);   // let smoother settle
        return twin->update(kDt, silent).throttle;
    }();

    const TwinOutput withTorque = [&]() {
        auto pull = makeSignal(kApCruise);
        for (int i = 0; i < 60; ++i) twin->update(kDt, pull);     // same settle time
        return twin->update(kDt, pull);
    }();

    EXPECT_NEAR(baselineThrottle, 0.0, 1e-9);
    EXPECT_NEAR(withTorque.throttle, baselineThrottle, 1e-9)
        << "unconfigured twin must not react to motor torque";
}

// ---------------------------------------------------------------------------
// --effective-throttle ON — the Autopilot silent-engine fix.
// ---------------------------------------------------------------------------

TEST_F(TwinTorqueInputWiringTest, EffectiveThrottleOnKeepsEngineAudibleAtApCruise) {
    auto featureOn = makeTwin();
    auto featureOff = makeTwin();
    featureOn->setEffectiveThrottleConfig(effectiveThrottleOn());

    for (auto* twin : {&featureOn, &featureOff}) {
        advanceThroughCranking(**twin);
        driveToTopGearAtCruise(**twin, kApCruise.speedKmh);
    }

    const auto sig = makeSignal(kApCruise);
    double onThrottle = 0.0;
    double offThrottle = 0.0;
    for (int i = 0; i < 90; ++i) {           // ~1.5 s: past the 50 ms smoother
        onThrottle = featureOn->update(kDt, sig).throttle;
        offThrottle = featureOff->update(kDt, sig).throttle;
    }

    // OFF stays silent (the bug); ON holds a light-cruise band around
    // 134/600 = 22% — audible, not WOT.
    EXPECT_LT(offThrottle, 0.02);
    EXPECT_GT(onThrottle, 0.15);
    EXPECT_LT(onThrottle, 0.35);
}

TEST_F(TwinTorqueInputWiringTest, EffectiveThrottleOnStaysSilentDuringApRegen) {
    auto featureOn = makeTwin();
    featureOn->setEffectiveThrottleConfig(effectiveThrottleOn());
    advanceThroughCranking(*featureOn);
    driveToTopGearAtCruise(*featureOn, kApCruise.speedKmh);

    // Engage on the cruise pull...
    const auto cruise = makeSignal(kApCruise);
    for (int i = 0; i < 60; ++i) featureOn->update(kDt, cruise);
    // ...then regen (lift-off with brake light OFF) must drop back to silence.
    const auto regen = makeSignal(kApRegen);
    double throttle = 1.0;
    for (int i = 0; i < 90; ++i) {
        throttle = featureOn->update(kDt, regen).throttle;
    }

    EXPECT_LT(throttle, 0.02) << "regen must contribute zero effective throttle";
}

TEST_F(TwinTorqueInputWiringTest, EffectiveThrottleOnLeavesGearboxDecisionsAlone) {
    // Orthogonality: feature 1 drives the ENGINE only. Gear decisions over the
    // same drive must be identical to an unconfigured twin — torque-informed
    // shifting is feature 2's job, off here.
    auto featureOn = makeTwin();
    auto featureOff = makeTwin();
    featureOn->setEffectiveThrottleConfig(effectiveThrottleOn());

    for (auto* twin : {&featureOn, &featureOff}) {
        advanceThroughCranking(**twin);
        driveToTopGearAtCruise(**twin, kApCruise.speedKmh);
    }

    const std::vector<FixtureRow> phases = {kApCruise, kApLight, kApHardPull,
                                            kApRegen};
    constexpr int kFramesPerPhase = 120;
    int frame = 0;
    for (const FixtureRow& row : phases) {
        const auto sig = makeSignal(row);
        for (int i = 0; i < kFramesPerPhase; ++i) {
            ++frame;
            const int onGear = featureOn->update(kDt, sig).gear;
            const int offGear = featureOff->update(kDt, sig).gear;
            EXPECT_EQ(onGear, offGear) << "frame " << frame;
        }
    }
}

// ---------------------------------------------------------------------------
// --torque-informed-gearbox ON — shift-DECISION input from commanded torque.
// ---------------------------------------------------------------------------

TEST_F(TwinTorqueInputWiringTest, TorqueInformedGearboxDownshiftsEarlierUnderApHardPull) {
    // The owner's scenario: pedal 0 at ~100 km/h while AP pulls 598 Nm. Today
    // the box reads COAST (0 throttle) and holds 8th; with the hint the pull
    // reads as firm demand and the box drops gear EARLIER (at a higher speed)
    // than the pedal-only box.
    //
    // SCENARIO SHAPE — deceleration sweep, NOT a fixed point. A downshift
    // fires when speed falls BELOW the (throttle-raised) threshold. With the
    // pinned bias cap 0.30 the zf8hp45 8->7 threshold sits at ~85 km/h, so a
    // fixed speed of 101.28 km/h can NEVER cross it inside the cap (crossing
    // 101.28 needs eff throttle > ~0.41 — above the cap by design, keeping
    // the cap below kickdownDelta 0.4 so a torque step can never fake a pedal
    // stab). The pull is therefore observed the same way the braking test
    // observes regen: sweep speed down through the raised threshold and
    // compare WHEN each box leaves 8th.
    auto featureOn = makeTwin();
    auto featureOff = makeTwin();
    featureOn->setTorqueInformedGearboxConfig(torqueGearboxOn());

    for (auto* twin : {&featureOn, &featureOff}) {
        advanceThroughCranking(**twin);
        driveToTopGearAtCruise(**twin, kApHardPull.speedKmh);
    }

    // Sweep 101.28 -> 40 km/h under the held 598 Nm pull, pedal 0. The floor
    // (40) sits below the pedal-only 8->7 threshold (~50 km/h) so BOTH boxes
    // eventually downshift and the comparison is meaningful.
    constexpr double kSweepEndKmh = 40.0;
    const int sweepFrames = 240;             // ~3.8 s
    double onFirstDownshiftKmh = -1.0;
    double offFirstDownshiftKmh = -1.0;
    for (int i = 1; i <= sweepFrames; ++i) {
        const double speed = kApHardPull.speedKmh
            - (kApHardPull.speedKmh - kSweepEndKmh) * static_cast<double>(i) / sweepFrames;
        const auto sig = makeSignal(kApHardPull.pedalFraction, speed,
                                    kApHardPull.motorTorqueNm);
        const int onGear = featureOn->update(kDt, sig).gear;
        const int offGear = featureOff->update(kDt, sig).gear;
        if (onFirstDownshiftKmh < 0.0 && onGear < 8) onFirstDownshiftKmh = speed;
        if (offFirstDownshiftKmh < 0.0 && offGear < 8) offFirstDownshiftKmh = speed;
    }

    EXPECT_GE(onFirstDownshiftKmh, 0.0) << "hinted box must leave 8th during the pull sweep";
    EXPECT_GE(offFirstDownshiftKmh, 0.0) << "baseline box must eventually follow (sweep floor)";
    EXPECT_GT(onFirstDownshiftKmh, offFirstDownshiftKmh)
        << "AP hard pull must be read as demand, not coasting";
}

TEST_F(TwinTorqueInputWiringTest, TorqueInformedGearboxDownshiftsEarlierUnderApBraking) {
    // AP brake event 3 magnitude (-910 Nm, brake light on, pedal 0): as speed
    // falls through the 8->7 band the hinted box downshifts EARLIER (at a
    // higher speed) than the pedal-only box, which holds 8th until ~50 km/h.
    auto featureOn = makeTwin();
    auto featureOff = makeTwin();
    featureOn->setTorqueInformedGearboxConfig(torqueGearboxOn());

    for (auto* twin : {&featureOn, &featureOff}) {
        advanceThroughCranking(**twin);
        driveToTopGearAtCruise(**twin, 105.0);
    }

    // Deceleration sweep 105 -> 70 km/h over ~3 s under AP braking.
    const int sweepFrames = 200;
    int onGear = 8;
    int offGear = 8;
    for (int i = 1; i <= sweepFrames; ++i) {
        const double speed = 105.0 - 35.0 * static_cast<double>(i) / sweepFrames;
        const auto sig = makeSignal(0.0, speed, -910.0);
        onGear = featureOn->update(kDt, sig).gear;
        offGear = featureOff->update(kDt, sig).gear;
    }

    EXPECT_EQ(offGear, 8) << "baseline: coasting pedal-only box holds 8th at 70 km/h";
    EXPECT_LT(onGear, offGear) << "AP braking must inform the downshift";
}

TEST_F(TwinTorqueInputWiringTest, PedalStillDrivesGearboxWhenBothFeaturesOn) {
    // Guard against over-reach: with both features ON but the DRIVER's foot
    // down (pedal 0.5) and no commanded torque, decisions must match an
    // unconfigured twin exactly — torque features must not distort foot-driven
    // behaviour.
    auto bothOn = makeTwin();
    auto unconfigured = makeTwin();
    bothOn->setEffectiveThrottleConfig(effectiveThrottleOn());
    bothOn->setTorqueInformedGearboxConfig(torqueGearboxOn());

    for (auto* twin : {&bothOn, &unconfigured}) {
        advanceThroughCranking(**twin);
        driveToTopGearAtCruise(**twin, kApCruise.speedKmh);
    }

    const auto sig = makeSignal(0.5, kApCruise.speedKmh, 0.0);
    int onGear = 0;
    int offGear = 0;
    for (int i = 0; i < 120; ++i) {
        onGear = bothOn->update(kDt, sig).gear;
        offGear = unconfigured->update(kDt, sig).gear;
    }

    EXPECT_EQ(onGear, offGear);
}

TEST_F(TwinTorqueInputWiringTest, FeaturesComposeUnderApPull) {
    // Both toggles together on the SAME frames: the engine is audible
    // (feature 1, near-WOT effective throttle from the 598 Nm pull) at the
    // exact moment the informed box downshifts (feature 2). Same sweep shape
    // as the hard-pull test — a fixed 101.28 km/h point cannot cross the
    // 8->7 threshold inside the pinned 0.30 bias cap (see that test's note).
    auto bothOn = makeTwin();
    auto featureOff = makeTwin();
    bothOn->setEffectiveThrottleConfig(effectiveThrottleOn());
    bothOn->setTorqueInformedGearboxConfig(torqueGearboxOn());

    for (auto* twin : {&bothOn, &featureOff}) {
        advanceThroughCranking(**twin);
        driveToTopGearAtCruise(**twin, kApHardPull.speedKmh);
    }

    constexpr double kSweepEndKmh = 40.0;
    const int sweepFrames = 240;             // ~3.8 s
    double onFirstDownshiftKmh = -1.0;
    double offFirstDownshiftKmh = -1.0;
    double throttleAtOnDownshift = 0.0;
    for (int i = 1; i <= sweepFrames; ++i) {
        const double speed = kApHardPull.speedKmh
            - (kApHardPull.speedKmh - kSweepEndKmh) * static_cast<double>(i) / sweepFrames;
        const auto sig = makeSignal(kApHardPull.pedalFraction, speed,
                                    kApHardPull.motorTorqueNm);
        const TwinOutput out = bothOn->update(kDt, sig);
        const int offGear = featureOff->update(kDt, sig).gear;
        if (onFirstDownshiftKmh < 0.0 && out.gear < 8) {
            onFirstDownshiftKmh = speed;
            throttleAtOnDownshift = out.throttle;
        }
        if (offFirstDownshiftKmh < 0.0 && offGear < 8) offFirstDownshiftKmh = speed;
    }

    EXPECT_GE(onFirstDownshiftKmh, 0.0) << "hinted box must leave 8th during the pull sweep";
    EXPECT_GE(offFirstDownshiftKmh, 0.0) << "baseline box must eventually follow (sweep floor)";
    EXPECT_GT(onFirstDownshiftKmh, offFirstDownshiftKmh)
        << "feature 2 must act while feature 1 drives";
    EXPECT_GT(throttleAtOnDownshift, 0.5)
        << "598 Nm pull must keep the engine near-WOT-audible at the downshift frame";
}
