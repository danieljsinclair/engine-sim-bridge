#include <gtest/gtest.h>
#include <twin/VirtualIceTwin.h>
#include <twin/IceVehicleProfile.h>
#include <twin/SlipLockController.h>
#include <io/UpstreamSignal.h>
#include <simulator/GearConventions.h>
#include <simulator/EngineSimTypes.h>
#include <algorithm>
#include <cmath>
#include <vector>

// ============================================================================
// CHARACTERIZATION NET for VirtualIceTwin::update (pre S3776 refactor).
//
// The refactor will decompose the ~CC-50 update() into single-responsibility
// helpers with behaviour preserved EXACTLY. These tests pin, through the PUBLIC
// twin API only, the behaviours that a mechanical split most easily breaks:
//   - the gate ORDER at the top of update (invalid telemetry -> ignition ->
//     derivation/smoother -> state machine) and the exact output shape each
//     early-return produces;
//   - per-state field ownership (which outputs each state touches and leaves
//     untouched);
//   - decision PRECEDENCE inside one tick (selector move vs shift request,
//     stall guard vs idle-hold vs transition);
//   - tick-exact boundary semantics (catch threshold, crank fallback, re-crank
//     cooldown cadence, clutch rate caps, shift floor);
//   - the raw-vs-smoothed throttle scoping of the gearbox shift decision.
//
// Characterization contract: every assertion here passes against CURRENT
// master. Where the current observable looks surprising (noted per test), it
// is pinned as-is and flagged in the refactor handover — the twin is the heart
// of the closed-loop vehicle twin and tonight's mandate is behaviour-preserving
// decomposition only.
// ============================================================================

using namespace twin;
using namespace input;

namespace {

// C63 M156 ZF8-style ratios (same as VirtualIceTwinTest's per-gear-map block):
// reconfigureProfile's banded shift map is derived from these, so the
// raw-throttle-vs-band test below uses true km/h band tops (24.14 * scale).
const std::vector<double> kC63Ratios = {4.38, 2.86, 1.92, 1.37, 1.00, 0.82, 0.73};
constexpr double kC63Diff = 2.82;
constexpr double kC63TireM = 0.356;

}  // namespace

class VirtualIceTwinCharacterizationTest : public ::testing::Test {
protected:
    void SetUp() override {
        profile_ = IceVehicleProfile::zf8hp45();
        twin_ = std::make_unique<VirtualIceTwin>(profile_);
        // Commanded-on twin (ignition defaults OFF — pinned by the existing
        // NoSelfStartWithoutIgnitionCommand; not re-pinned here).
        twin_->setIgnition(true);
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

    UpstreamSignal makeInvalidSignal() {
        UpstreamSignal sig;
        sig.isValid = false;
        return sig;
    }

    // OFF -> CRANKING -> IDLE via the RPM fast path; selector left NEUTRAL.
    void advanceToIdle() {
        auto sig = makeValidSignal(0.6, 0.0);
        twin_->update(1.0 / 60.0, sig);  // OFF -> CRANKING (starter edge)
        twin_->setEngineRpmFeedback(800.0);
        twin_->update(1.0 / 60.0, sig);  // CRANKING -> IDLE
        ASSERT_EQ(twin_->getState(), TwinState::IDLE);
    }

    // IDLE -> RUNNING via the DRIVE selector; engine held healthy at 2500 rpm.
    void enterRunningHealthy(double speedKmh = 0.0, double throttle = 0.1) {
        twin_->setGearSelector(bridge::GearSelector::DRIVE);
        twin_->setEngineRpmFeedback(2500.0);
        twin_->setVehicleSpeedFeedback(speedKmh);
        auto sig = makeValidSignal(throttle, speedKmh);
        twin_->update(1.0 / 60.0, sig);
        ASSERT_EQ(twin_->getState(), TwinState::RUNNING);
    }
};

// ---------------------------------------------------------------------------
// GATE 1: telemetry validity (top of update). Order matters: the invalid
// early-return runs BEFORE the ignition handling and the state machine, and
// produces a distinctive output shape.
// ---------------------------------------------------------------------------

// A signal that is well-formed in every field except timestampUtcMs == 0 is
// treated exactly like isValid == false: no state advance on the first frame
// (OFF -> nothing) and OFF after the 5 s telemetry timeout.
TEST_F(VirtualIceTwinCharacterizationTest, InvalidTelemetry_ZeroTimestampIsInvalid) {
    UpstreamSignal zeroStamp = makeValidSignal(0.6, 0.0);
    zeroStamp.timestampUtcMs = 0;

    // Not even the OFF -> CRANKING edge may fire on a zero-timestamp frame.
    twin_->update(1.0 / 60.0, zeroStamp);
    EXPECT_EQ(twin_->getState(), TwinState::OFF)
        << "timestampUtcMs == 0 must gate exactly like isValid == false";

    // ...and sustained zero-timestamp frames time out to OFF like any invalid
    // telemetry (state was OFF already; the timeout keeps it there).
    for (int i = 0; i < 320; ++i) twin_->update(1.0 / 60.0, zeroStamp);
    EXPECT_EQ(twin_->getState(), TwinState::OFF);
}

// Below the 5 s timeout an invalid frame must NOT force OFF: the state machine
// simply pauses (no transitions, no stall guard, no crank timer) and the
// gearbox's current gear is still surfaced.
TEST_F(VirtualIceTwinCharacterizationTest, InvalidTelemetry_BelowTimeout_PausesNotKills) {
    advanceToIdle();
    enterRunningHealthy(5.0);

    // 4 s of invalid frames (< TELEMETRY_TIMEOUT_S): still RUNNING afterwards.
    for (int i = 0; i < 240; ++i) {
        auto out = twin_->update(1.0 / 60.0, makeInvalidSignal());
        ASSERT_EQ(twin_->getState(), TwinState::RUNNING)
            << "invalid frame below the 5 s timeout must not leave RUNNING";
        // The one field the invalid path surfaces is the gearbox's gear.
        EXPECT_EQ(out.gear, twin_->getCurrentGear());
    }
}

// The invalid-telemetry early-return touches ONLY output.gear. Everything
// else keeps the TwinOutput struct defaults — including clutchPressure = 1.0
// (the struct default, NOT the twin's tracked pressure) and gearSelector =
// NEUTRAL (the default, NOT the live selector). Characterization: this
// asymmetry vs the ignition-off return (see IgnitionKill below) is pinned
// as-is.
TEST_F(VirtualIceTwinCharacterizationTest, InvalidTelemetry_FrameShape_OnlyGearPopulated) {
    advanceToIdle();
    enterRunningHealthy(5.0);  // selector DRIVE
    twin_->setGearSelector(bridge::GearSelector::DRIVE);

    auto out = twin_->update(1.0 / 60.0, makeInvalidSignal());

    EXPECT_EQ(out.gear, twin_->getCurrentGear()) << "gear is surfaced";
    EXPECT_FALSE(out.ignition);
    EXPECT_FALSE(out.starterMotor);
    EXPECT_DOUBLE_EQ(out.throttle, 0.0);
    EXPECT_DOUBLE_EQ(out.dynoTorqueScale, 0.0);
    EXPECT_DOUBLE_EQ(out.clutchPressure, 1.0)
        << "invalid frames surface the TwinOutput DEFAULT clutch (1.0), not the "
        << "tracked pressure — pinned as current observable";
    EXPECT_EQ(out.gearSelector, bridge::GearSelector::NEUTRAL)
        << "invalid frames surface the DEFAULT selector, not the live one — "
        << "pinned as current observable";
    EXPECT_DOUBLE_EQ(out.pinVehicleSpeedTargetKmh, -1.0);
    EXPECT_DOUBLE_EQ(out.drivetrainInputTorqueNm, 0.0);
    EXPECT_FALSE(out.creepReliefFired);
}

// The 5 s timeout accumulator resets when valid telemetry returns: 4 s
// invalid + valid frame + 4 s invalid must NOT produce OFF (without the reset
// it would be 8 s cumulative).
TEST_F(VirtualIceTwinCharacterizationTest, TelemetryTimeoutTimer_ResetsOnValidFrame) {
    advanceToIdle();
    enterRunningHealthy(5.0);

    for (int i = 0; i < 240; ++i) twin_->update(1.0 / 60.0, makeInvalidSignal());  // 4 s
    twin_->update(1.0 / 60.0, makeValidSignal(0.1, 5.0));                          // reset
    for (int i = 0; i < 240; ++i) twin_->update(1.0 / 60.0, makeInvalidSignal());  // 4 s
    EXPECT_EQ(twin_->getState(), TwinState::RUNNING)
        << "a valid frame in between must reset the timeout accumulator";

    // ...but 5+ s CONTINUOUS invalid frames still force OFF (existing behaviour).
    for (int i = 0; i < 70; ++i) twin_->update(1.0 / 60.0, makeInvalidSignal());
    EXPECT_EQ(twin_->getState(), TwinState::OFF);
}

// The invalid early-return precedes the state machine, so the CRANKING
// fallback timer only accumulates over VALID frames: an invalid window
// mid-crank pauses the 3 s fallback rather than counting toward it.
TEST_F(VirtualIceTwinCharacterizationTest, InvalidTelemetry_PausesCrankFallbackTimer) {
    auto sig = makeValidSignal(0.6, 0.0);
    twin_->update(1.0 / 60.0, sig);  // OFF -> CRANKING
    twin_->setEngineRpmFeedback(0.0);  // never fast-path catch

    // 2.5 s of VALID cranking frames.
    for (int i = 0; i < 150; ++i) twin_->update(1.0 / 60.0, sig);
    ASSERT_EQ(twin_->getState(), TwinState::CRANKING);

    // 1 s of INVALID frames: must not push the (valid-frame) timer past 3 s.
    for (int i = 0; i < 60; ++i) twin_->update(1.0 / 60.0, makeInvalidSignal());
    ASSERT_EQ(twin_->getState(), TwinState::CRANKING)
        << "invalid window under the timeout must not leave CRANKING";

    // 25 more VALID crank frames -> 2.92 s of valid crank time: still cranking.
    for (int i = 0; i < 25; ++i) twin_->update(1.0 / 60.0, sig);
    EXPECT_EQ(twin_->getState(), TwinState::CRANKING)
        << "the 3 s fallback must count VALID frames only (invalid window paused it)";

    // The 180th valid crank frame (3.0 s) catches.
    bool caught = false;
    for (int i = 0; i < 10; ++i) {
        twin_->update(1.0 / 60.0, sig);
        if (twin_->getState() == TwinState::IDLE) { caught = true; break; }
    }
    EXPECT_TRUE(caught) << "fallback fires once 3 s of VALID cranking elapses";
}

// ---------------------------------------------------------------------------
// GATE 2: ignition off (runs only on valid frames, after the validity gate).
// ---------------------------------------------------------------------------

// Ignition off mid-RUNNING forces OFF on the SAME frame. The ignition-off
// return surfaces gear + ignition + the tracked clutch pressure + the live
// selector (a DIFFERENT field set than the invalid-telemetry return above —
// both shapes pinned).
TEST_F(VirtualIceTwinCharacterizationTest, IgnitionKillMidRunning_ForcesOff_FrameShape) {
    advanceToIdle();
    enterRunningHealthy(5.0);
    const auto pre = twin_->update(1.0 / 60.0, makeValidSignal(0.3, 5.0));
    ASSERT_EQ(twin_->getState(), TwinState::RUNNING);

    twin_->setIgnition(false);
    const auto out = twin_->update(1.0 / 60.0, makeValidSignal(0.3, 5.0));

    EXPECT_EQ(twin_->getState(), TwinState::OFF) << "ignition off kills immediately";
    EXPECT_FALSE(out.ignition);
    EXPECT_FALSE(out.starterMotor);
    EXPECT_DOUBLE_EQ(out.throttle, 0.0);
    EXPECT_EQ(out.gear, twin_->getCurrentGear());
    EXPECT_EQ(out.gearSelector, bridge::GearSelector::DRIVE)
        << "ignition-off surfaces the LIVE selector (contrast: invalid frames "
        << "surface the default)";
    EXPECT_DOUBLE_EQ(out.clutchPressure, pre.clutchPressure)
        << "ignition-off surfaces the tracked clutch pressure unchanged "
        << "(contrast: invalid frames surface the 1.0 default)";
}

// The ignition kill also resets the cranking timer: a re-ignition starts a
// FRESH 3 s crank budget. If the timer survived the kill, the second crank
// would catch ~1.3 s in (3 s minus the first crank's 1.67 s); with the reset
// it must take the full 3 s.
TEST_F(VirtualIceTwinCharacterizationTest, IgnitionKillMidCranking_RestartsCrankBudget) {
    auto sig = makeValidSignal(0.6, 0.0);
    twin_->setEngineRpmFeedback(0.0);
    twin_->update(1.0 / 60.0, sig);  // OFF -> CRANKING
    for (int i = 0; i < 100; ++i) twin_->update(1.0 / 60.0, sig);  // 1.67 s crank
    ASSERT_EQ(twin_->getState(), TwinState::CRANKING);

    twin_->setIgnition(false);
    twin_->update(1.0 / 60.0, sig);  // kill -> OFF (timer reset here)
    ASSERT_EQ(twin_->getState(), TwinState::OFF);
    twin_->setIgnition(true);
    twin_->update(1.0 / 60.0, sig);  // fresh OFF -> CRANKING edge
    ASSERT_EQ(twin_->getState(), TwinState::CRANKING);

    // 1.67 s into the SECOND crank: a surviving timer would total 3.33 s and
    // have caught; the reset timer is only at 1.67 s.
    for (int i = 0; i < 100; ++i) twin_->update(1.0 / 60.0, sig);
    EXPECT_EQ(twin_->getState(), TwinState::CRANKING)
        << "re-ignition must restart the 3 s crank budget from zero";

    // It then catches at the full 3 s of the second crank.
    bool caught = false;
    for (int i = 0; i < 90; ++i) {
        twin_->update(1.0 / 60.0, sig);
        if (twin_->getState() == TwinState::IDLE) { caught = true; break; }
    }
    EXPECT_TRUE(caught);
}

// ---------------------------------------------------------------------------
// OFF -> CRANKING transition frame.
// ---------------------------------------------------------------------------

// The OFF case zeroes the tracked clutch pressure on the transition frame
// (the member inits at 1.0), so the first crank frame surfaces 0.0.
TEST_F(VirtualIceTwinCharacterizationTest, OffToCrankingFrame_ZeroesClutchPressure) {
    auto out = twin_->update(1.0 / 60.0, makeValidSignal(0.6, 0.0));
    ASSERT_EQ(twin_->getState(), TwinState::CRANKING);
    EXPECT_DOUBLE_EQ(out.clutchPressure, 0.0)
        << "the OFF -> CRANKING transition frame must surface clutch 0 (member "
        << "default is 1.0 — the transition resets it)";
}

// ---------------------------------------------------------------------------
// CRANKING: dyno load (FREE only), catch boundary, fallback boundary.
// ---------------------------------------------------------------------------

// In CRANKING the twin gives the starter a resistive dyno load in FREE
// coupling (0.15) but NOT in PIN (its vehicle-speed constraint is the load).
TEST_F(VirtualIceTwinCharacterizationTest, CrankingDynoLoad_FreeOnly) {
    // FREE (default): the second CRANKING frame (the first is the OFF->CRANKING
    // transition, which does not run the CRANKING case) carries the dyno load.
    auto sig = makeValidSignal(0.6, 0.0);
    twin_->setEngineRpmFeedback(0.0);
    twin_->update(1.0 / 60.0, sig);                            // OFF -> CRANKING
    const auto freeOut = twin_->update(1.0 / 60.0, sig);       // CRANKING frame
    ASSERT_EQ(twin_->getState(), TwinState::CRANKING);
    EXPECT_DOUBLE_EQ(freeOut.dynoTorqueScale, 0.15)
        << "FREE-mode cranking must load the starter with the dyno (0.15)";

    // PIN: same frame index, no dyno load.
    twin_ = std::make_unique<VirtualIceTwin>(profile_);
    twin_->setIgnition(true);
    twin_->setWheelCouplingMode(WheelCouplingMode::Pin);
    twin_->setEngineRpmFeedback(0.0);
    twin_->update(1.0 / 60.0, sig);                            // OFF -> CRANKING
    const auto pinOut = twin_->update(1.0 / 60.0, sig);        // CRANKING frame
    ASSERT_EQ(twin_->getState(), TwinState::CRANKING);
    EXPECT_DOUBLE_EQ(pinOut.dynoTorqueScale, 0.0)
        << "PIN-mode cranking has its own load path (no dyno)";
}

// The catch threshold is STRICTLY above 500 rpm: exactly 500 does not catch
// (falls through to the time fallback), 501 does.
TEST_F(VirtualIceTwinCharacterizationTest, CrankingCatchThreshold_StrictlyAbove500) {
    auto sig = makeValidSignal(0.6, 0.0);
    twin_->update(1.0 / 60.0, sig);  // OFF -> CRANKING

    twin_->setEngineRpmFeedback(500.0);
    twin_->update(1.0 / 60.0, sig);
    EXPECT_EQ(twin_->getState(), TwinState::CRANKING)
        << "exactly 500 rpm is NOT a catch (threshold is strictly greater)";

    twin_->setEngineRpmFeedback(501.0);
    twin_->update(1.0 / 60.0, sig);
    EXPECT_EQ(twin_->getState(), TwinState::IDLE)
        << "501 rpm IS a catch";
}

// The time fallback fires at EXACTLY 3.0 s of cranking (>= comparison): with
// dt = 0.5 s the timer reaches 3.0 on the 6th CRANKING frame and catches.
TEST_F(VirtualIceTwinCharacterizationTest, CrankingFallback_FiresAtExactly3s) {
    auto sig = makeValidSignal(0.6, 0.0);
    twin_->setEngineRpmFeedback(0.0);
    twin_->update(0.5, sig);  // OFF -> CRANKING (timer not yet advanced)

    for (int frame = 1; frame <= 5; ++frame) {  // timer: 0.5 .. 2.5 s
        twin_->update(0.5, sig);
        ASSERT_EQ(twin_->getState(), TwinState::CRANKING)
            << "frame " << frame << " (timer " << frame * 0.5 << " s) must stay CRANKING";
    }
    twin_->update(0.5, sig);  // timer: 3.0 s -> catches
    EXPECT_EQ(twin_->getState(), TwinState::IDLE)
        << "the fallback fires the frame the crank timer reaches exactly 3.0 s";
}

// The frame the engine catches, the dyno load is cleared (0.0) even in FREE.
TEST_F(VirtualIceTwinCharacterizationTest, CrankingCatch_ClearsDynoLoad) {
    auto sig = makeValidSignal(0.6, 0.0);
    twin_->setEngineRpmFeedback(0.0);
    twin_->update(1.0 / 60.0, sig);  // OFF -> CRANKING
    twin_->update(1.0 / 60.0, sig);  // CRANKING (dyno 0.15 in FREE)

    twin_->setEngineRpmFeedback(800.0);
    const auto out = twin_->update(1.0 / 60.0, sig);  // catch frame
    ASSERT_EQ(twin_->getState(), TwinState::IDLE);
    EXPECT_DOUBLE_EQ(out.dynoTorqueScale, 0.0)
        << "the catch frame must clear the cranking dyno load";
}

// ---------------------------------------------------------------------------
// IDLE: stall guard call-site + REVERSE engagement + untouched surfaces.
// ---------------------------------------------------------------------------

// The restart-on-stall guard is wired into IDLE too (the PARK-start path): a
// stalled engine in IDLE pulses the starter edge and floors throttle at the
// cranking level, and the re-crank cooldown means the NEXT frame does not
// pulse again.
TEST_F(VirtualIceTwinCharacterizationTest, IdleStallGuard_PulsesStarterInIdle) {
    advanceToIdle();  // selector NEUTRAL (P/N idle)
    twin_->setEngineRpmFeedback(0.0);

    const auto first = twin_->update(1.0 / 60.0, makeValidSignal(0.0, 0.0));
    EXPECT_EQ(twin_->getState(), TwinState::IDLE) << "N stays IDLE (no engagement)";
    EXPECT_TRUE(first.starterMotor) << "the IDLE stall guard pulses the starter edge";
    EXPECT_GE(first.throttle, EngineSimDefaults::CRANKING_THROTTLE - 1e-9)
        << "scripted stall-guard throttle floors at CRANKING_THROTTLE";

    const auto second = twin_->update(1.0 / 60.0, makeValidSignal(0.0, 0.0));
    EXPECT_FALSE(second.starterMotor)
        << "the cooldown suppresses an immediate second edge";
}

// REVERSE engages RUNNING from IDLE exactly like DRIVE (both drive positions).
TEST_F(VirtualIceTwinCharacterizationTest, IdleToRunning_OnReverseSelector) {
    advanceToIdle();
    twin_->setGearSelector(bridge::GearSelector::REVERSE);
    twin_->update(1.0 / 60.0, makeValidSignal(0.0, 0.0));
    EXPECT_EQ(twin_->getState(), TwinState::RUNNING)
        << "REVERSE is a drive position: IDLE -> RUNNING on selector alone";
}

// The IDLE case touches the throttle/gear/ignition/clutch surfaces and NOTHING
// else: no pin, no injected torque, no dyno, no road-implied rpm, no relief.
TEST_F(VirtualIceTwinCharacterizationTest, IdleFrame_TouchesNoCouplingSurfaces) {
    advanceToIdle();
    const auto out = twin_->update(1.0 / 60.0, makeValidSignal(0.0, 0.0));

    EXPECT_EQ(out.gear, static_cast<int>(bridge::BridgeGear::NEUTRAL));
    EXPECT_DOUBLE_EQ(out.clutchPressure, 0.0);
    EXPECT_DOUBLE_EQ(out.dynoTorqueScale, 0.0);
    EXPECT_DOUBLE_EQ(out.pinVehicleSpeedTargetKmh, -1.0);
    EXPECT_DOUBLE_EQ(out.drivetrainInputTorqueNm, 0.0);
    EXPECT_DOUBLE_EQ(out.roadImpliedRpm, 0.0);
    EXPECT_FALSE(out.creepReliefFired);
    EXPECT_FALSE(out.couplingIsTorqueConverter)
        << "default coupling model is ClutchMap, not the torque converter";
}

// ---------------------------------------------------------------------------
// RUNNING: idle-hold floor semantics (add-only, clamped, hysteresis).
// ---------------------------------------------------------------------------

// The idle-hold controller is a FLOOR: with the driver at ~60% and the engine
// sagging below idle, the output throttle stays at the driver's (smoothed)
// value — the controller may only ADD throttle, never reduce it.
TEST_F(VirtualIceTwinCharacterizationTest, IdleHoldFloor_AddOnly_NeverReducesDriverThrottle) {
    advanceToIdle();
    enterRunningHealthy(5.0, 0.6);

    auto sig = makeValidSignal(0.6, 5.0);
    for (int i = 0; i < 30; ++i) twin_->update(1.0 / 60.0, sig);  // smoother settled ~0.6
    twin_->setEngineRpmFeedback(profile_.idleRpm - 300.0);  // sag below idle -> hold engages

    const auto out = twin_->update(1.0 / 60.0, sig);
    EXPECT_GE(out.throttle, twin_->getSmoothedThrottle() - 1e-9)
        << "the idle-hold floor must never take throttle away from the driver";
    EXPECT_GE(out.throttle, 0.5)
        << "driver at ~60% keeps ~60% even while the engine sags (floor semantics)";
}

// The controller's authority is capped at 20% of full throttle: a deep sag
// (engine at 100 rpm, alive) commands at most 0.20, far below kickdown (95%)
// and the WOT gate (90%) so it cannot disturb shift logic.
TEST_F(VirtualIceTwinCharacterizationTest, IdleHoldFloor_ClampedAt20Percent_EvenOnDeepSag) {
    advanceToIdle();
    enterRunningHealthy(5.0, 0.0);

    twin_->setEngineRpmFeedback(100.0);  // alive (above STOPPED_RPM) but deep sag
    const auto out = twin_->update(1.0 / 60.0, makeValidSignal(0.0, 5.0));
    EXPECT_LE(out.throttle, 0.20 + 1e-9)
        << "idle-hold authority is capped at 20% (must not reach kickdown/WOT)";
    EXPECT_GE(out.throttle, EngineSimDefaults::IDLE_SUSTAIN_THROTTLE - 1e-9)
        << "an engaged hold floors at the idle-sustain minimum";
}

// Engage/release hysteresis: the hold engages below idle and releases only
// above idle + 150 rpm. Between idle and idle + 150 (here idle + 75) an
// already-engaged hold STAYS engaged; above idle + 150 it releases and the
// throttle falls back to the driver's (0).
TEST_F(VirtualIceTwinCharacterizationTest, IdleHoldRelease_HysteresisBand_HoldsUntilIdlePlus150) {
    advanceToIdle();
    enterRunningHealthy(5.0, 0.0);

    // Decay the smoother's cranking-throttle residue (~0.6) so the release
    // frame's throttle is attributable to the hold alone.
    auto zero = makeValidSignal(0.0, 5.0);
    for (int i = 0; i < 90; ++i) twin_->update(1.0 / 60.0, zero);
    ASSERT_LT(twin_->getSmoothedThrottle(), 1e-6)
        << "precondition: smoother residue decayed to ~0";

    // Engage below idle.
    twin_->setEngineRpmFeedback(profile_.idleRpm - 50.0);
    auto engaged = twin_->update(1.0 / 60.0, zero);
    EXPECT_GE(engaged.throttle, EngineSimDefaults::IDLE_SUSTAIN_THROTTLE - 1e-9)
        << "below idle the hold engages and floors the throttle";

    // Inside the hysteresis band (idle + 75 < idle + 150): still engaged.
    twin_->setEngineRpmFeedback(profile_.idleRpm + 75.0);
    engaged = twin_->update(1.0 / 60.0, zero);
    EXPECT_GE(engaged.throttle, EngineSimDefaults::IDLE_SUSTAIN_THROTTLE - 1e-9)
        << "inside the release hysteresis band the hold stays engaged";

    // Above idle + 150: released -> no floor at zero driver throttle.
    twin_->setEngineRpmFeedback(profile_.idleRpm + 250.0);
    engaged = twin_->update(1.0 / 60.0, zero);
    EXPECT_NEAR(engaged.throttle, 0.0, 1e-6)
        << "above idle + 150 the hold releases; throttle is the driver's own (0)";
}

// ---------------------------------------------------------------------------
// RUNNING: clutch rate limiter + road-implied rpm surfacing.
// ---------------------------------------------------------------------------

// Leaving IDLE (clutch tracked at 0) into a locked-cruise demand, the clutch
// pressure may rise by at most the ENGAGE rate (3/s * dt) on the first RUNNING
// frame — the anti-slam cap, applied to the default ClutchMap model path.
// (20 km/h at 0.5 throttle stays below the zf8 1->2 upshift point so the frame
// under test is genuinely RUNNING, not the first SHIFTING frame.)
TEST_F(VirtualIceTwinCharacterizationTest, FirstRunningFrame_RespectsEngageRateCap) {
    advanceToIdle();  // clutch tracked at 0 through IDLE
    twin_->setGearSelector(bridge::GearSelector::DRIVE);
    twin_->setEngineRpmFeedback(6500.0);
    twin_->setVehicleSpeedFeedback(20.0);

    const double dt = 1.0 / 60.0;
    auto sig = makeValidSignal(0.5, 20.0);
    (void)twin_->update(dt, sig);  // IDLE -> RUNNING transition frame (IDLE case
                                   // runs; the RUNNING case starts NEXT frame)
    const auto out = twin_->update(dt, sig);  // first RUNNING-case frame
    ASSERT_EQ(twin_->getState(), TwinState::RUNNING);
    const double maxRise = EngineSimDefaults::CLUTCH_ENGAGE_RATE_PER_SEC * dt;
    EXPECT_GT(out.clutchPressure, 0.0) << "a lock demand engages (pressure rises)";
    EXPECT_LE(out.clutchPressure, 0.0 + maxRise + 1e-9)
        << "engagement from 0 is rate-capped at CLUTCH_ENGAGE_RATE_PER_SEC*dt";

    // And the next frame is capped again, relative to the first.
    const auto out2 = twin_->update(dt, sig);
    EXPECT_LE(out2.clutchPressure, out.clutchPressure + maxRise + 1e-9)
        << "the engage cap applies every frame, not just the first";
}

// RUNNING surfaces the road-implied RPM (gear-ratio math on the coupling wheel
// speed; FREE uses the actual feedback speed) — the diagnostics feed for the
// driveability gate and the inline clutch readout.
TEST_F(VirtualIceTwinCharacterizationTest, RunningFrame_SurfacesRoadImpliedRpm) {
    advanceToIdle();
    enterRunningHealthy(5.0);
    ASSERT_EQ(twin_->getCurrentGear(), 1)
        << "precondition: box sits in 1st at crawl speed";

    const double wheelKmh = 5.0;  // FREE: slip-lock wheel speed == feedback
    const double expected = (wheelKmh / 3.6) / profile_.tireRadiusM
        * profile_.gearRatios[0] * profile_.diffRatio * 30.0 / 3.14159265358979;
    const auto out = twin_->update(1.0 / 60.0, makeValidSignal(0.1, 5.0));
    EXPECT_NEAR(out.roadImpliedRpm, expected, 0.5)
        << "road-implied RPM must match the gear-ratio formula on the current gear";
}

// ---------------------------------------------------------------------------
// RUNNING: the gearbox shift decision reads the RAW signal throttle, not the
// smoothed engine-drive throttle.
// ---------------------------------------------------------------------------

// Discriminator on the C63 banded map: at 23.5 km/h the light-throttle upshift
// top is ~23.0 km/h (shifts) but the WOT (0.95) top is ~25.2 km/h (holds 1st).
// Stepping the RAW throttle to 0.95 while the smoother is still converging:
// a raw-reading gearbox holds 1st; a smoothed-reading one would see light
// throttle for the first ~5 frames and upshift. Control arm: settled light
// throttle + the same speed step DOES upshift.
TEST_F(VirtualIceTwinCharacterizationTest, GearboxShiftDecision_ReadsRawSignalThrottle_NotSmoothed) {
    const auto settleLow = [this](double throttle) {
        advanceToIdle();
        enterRunningHealthy(15.0, throttle);
        twin_->reconfigureProfile(kC63Ratios, kC63Diff, kC63TireM);
        twin_->setWheelCouplingMode(WheelCouplingMode::Pin);
        twin_->setVehicleSpeedFeedback(15.0);
        twin_->setEngineRpmFeedback(2500.0);
        auto sig = makeValidSignal(throttle, 15.0);
        for (int i = 0; i < 120; ++i) twin_->update(1.0 / 60.0, sig);
        ASSERT_EQ(twin_->getCurrentGear(), 1) << "precondition: settle in 1st at 15 km/h";
    };

    // Arm A: RAW throttle steps 0.06 -> 0.95 together with the speed step to
    // 23.5 km/h. The gearbox must hold 1st (WOT band) from the very first
    // frame, through and past the smoother's convergence.
    settleLow(0.06);
    auto wotSig = makeValidSignal(0.95, 23.5);
    for (int i = 0; i < 120; ++i) {
        twin_->update(1.0 / 60.0, wotSig);
        ASSERT_EQ(twin_->getState(), TwinState::RUNNING)
            << "WOT at 23.5 km/h must not even enter SHIFTING (band top ~25.2)";
        ASSERT_EQ(twin_->getCurrentGear(), 1)
            << "the shift decision must see the RAW 0.95 throttle (WOT holds 1st)";
    }

    // Arm B (control): settled light throttle (0.10), speed-only step to
    // 23.5 km/h -> light band top ~23.0 is exceeded -> upshift to 2nd.
    twin_ = std::make_unique<VirtualIceTwin>(profile_);
    twin_->setIgnition(true);
    settleLow(0.10);
    auto lightSig = makeValidSignal(0.10, 23.5);
    bool upshifted = false;
    for (int i = 0; i < 90; ++i) {
        twin_->update(1.0 / 60.0, lightSig);
        if (twin_->getCurrentGear() >= 2) { upshifted = true; break; }
    }
    EXPECT_TRUE(upshifted)
        << "control: light throttle at 23.5 km/h must upshift (band top ~23.0)";
}

// ---------------------------------------------------------------------------
// RUNNING: transition precedence inside one tick.
// ---------------------------------------------------------------------------

// When a selector move to P/N arrives on the same frame as a speed step that
// would request a shift, the selector branch wins: the twin goes IDLE, not
// SHIFTING. (Control arm: the same speed step under DRIVE enters SHIFTING.)
TEST_F(VirtualIceTwinCharacterizationTest, SelectorMoveToN_WinsOverShiftRequest_SameFrame) {
    const auto settleInFirst = [this]() {
        advanceToIdle();
        enterRunningHealthy(5.0, 0.3);
        auto sig = makeValidSignal(0.3, 5.0);
        for (int i = 0; i < 30; ++i) twin_->update(1.0 / 60.0, sig);
        ASSERT_EQ(twin_->getCurrentGear(), 1) << "precondition: 1st gear at 5 km/h";
    };

    // Arm A: speed jump to 60 km/h AND selector to NEUTRAL in the same frame.
    settleInFirst();
    twin_->setGearSelector(bridge::GearSelector::NEUTRAL);
    twin_->update(1.0 / 60.0, makeValidSignal(0.3, 60.0));
    EXPECT_EQ(twin_->getState(), TwinState::IDLE)
        << "the selector branch is evaluated first: N wins over the shift request";

    // Arm B (control): identical speed jump under DRIVE enters SHIFTING.
    twin_ = std::make_unique<VirtualIceTwin>(profile_);
    twin_->setIgnition(true);
    settleInFirst();
    bool sawShifting = false;
    for (int i = 0; i < 5 && !sawShifting; ++i) {
        twin_->update(1.0 / 60.0, makeValidSignal(0.3, 60.0));
        sawShifting = (twin_->getState() == TwinState::SHIFTING);
    }
    EXPECT_TRUE(sawShifting)
        << "control: the same speed step under DRIVE requests the shift";
}

// A stall (feedback 0) and a selector move to N in the SAME frame: both
// effects land on that frame's output/state — the stall guard pulses the
// starter edge + cranking throttle floor, AND the selector branch still moves
// the state to IDLE.
TEST_F(VirtualIceTwinCharacterizationTest, StallAndSelectorLeave_SameFrame_BothEffectsLand) {
    advanceToIdle();
    enterRunningHealthy(5.0, 0.3);
    twin_->setGearSelector(bridge::GearSelector::NEUTRAL);
    twin_->setEngineRpmFeedback(0.0);

    const auto out = twin_->update(1.0 / 60.0, makeValidSignal(0.3, 5.0));
    EXPECT_TRUE(out.starterMotor)
        << "the stall guard runs before the transition check: edge lands";
    EXPECT_GE(out.throttle, EngineSimDefaults::CRANKING_THROTTLE - 1e-9)
        << "the stall guard's cranking throttle floor lands on the same frame";
    EXPECT_EQ(twin_->getState(), TwinState::IDLE)
        << "the selector branch still transitions to IDLE on the same frame";
}

// ---------------------------------------------------------------------------
// RUNNING: re-crank cooldown cadence (the stall guard's retry contract).
// ---------------------------------------------------------------------------

// Under a SUSTAINED stall the starter pulses as isolated one-tick edges ~3 s
// apart — never held, never machine-gunned: in a 6 s window exactly 2 edges
// fire (frame 0 and frame ~180), separated by the re-crank period.
TEST_F(VirtualIceTwinCharacterizationTest, SustainedStall_StarterEdgesArePeriodic_OnePer3s) {
    advanceToIdle();
    enterRunningHealthy(5.0, 0.3);

    auto sig = makeValidSignal(0.3, 5.0);
    std::vector<int> edgeFrames;
    const int frames = 360;  // 6.0 s at 60 Hz
    for (int i = 0; i < frames; ++i) {
        twin_->setEngineRpmFeedback(0.0);  // stall persists
        const auto out = twin_->update(1.0 / 60.0, sig);
        if (out.starterMotor) edgeFrames.push_back(i);
        // A held starter is the oscillation anti-pattern: at most one frame in
        // any 10-frame window.
        if (i >= 9) {
            const int edgesLast10 = static_cast<int>(edgeFrames.size())
                - static_cast<int>(std::count_if(edgeFrames.begin(), edgeFrames.end(),
                    [i](int f) { return f < i - 9; }));
            ASSERT_LE(edgesLast10, 1) << "starter edges must be isolated (frame " << i << ")";
        }
    }
    ASSERT_EQ(edgeFrames.size(), 2u)
        << "exactly two edges in 6 s of sustained stall (initial + one retry)";
    const int gap = edgeFrames[1] - edgeFrames[0];
    EXPECT_GE(gap, 170) << "retry only after the 3 s re-crank period";
    EXPECT_LE(gap, 190) << "retry fires promptly once the period elapses";
}

// armFreshCrankBudget bypasses the cooldown: mid-cooldown (edge fired ~1 s
// ago), arming a fresh budget makes the very next frame pulse again. This is
// the warm-boot seam for the first REAL frame after prime.
TEST_F(VirtualIceTwinCharacterizationTest, ArmFreshCrankBudget_BypassesCooldownNextFrame) {
    advanceToIdle();
    enterRunningHealthy(5.0, 0.3);

    auto sig = makeValidSignal(0.3, 5.0);
    twin_->setEngineRpmFeedback(0.0);
    const auto edge = twin_->update(1.0 / 60.0, sig);
    ASSERT_TRUE(edge.starterMotor) << "precondition: the stall edge fired";

    // 1 s into the 3 s cooldown: no edge.
    for (int i = 0; i < 60; ++i) {
        const auto out = twin_->update(1.0 / 60.0, sig);
        ASSERT_FALSE(out.starterMotor) << "cooldown suppresses edges mid-period";
    }

    twin_->armFreshCrankBudget();
    const auto out = twin_->update(1.0 / 60.0, sig);
    EXPECT_TRUE(out.starterMotor)
        << "armFreshCrankBudget makes the very next frame pulse the starter";
}

// ---------------------------------------------------------------------------
// SHIFTING: pin/torque not surfaced, TC capacity held, friction floor rule.
// ---------------------------------------------------------------------------

// During shift EXECUTION frames (every SHIFTING-tagged frame after the
// transition frame, whose output was computed by the RUNNING case) the twin
// surfaces NO vehicle-speed pin and NO injected torque — even in PIN mode,
// where the RUNNING frames pin every tick.
TEST_F(VirtualIceTwinCharacterizationTest, ShiftExecution_SurfacesNoPinAndNoTorque_EvenInPinMode) {
    twin_->setWheelCouplingMode(WheelCouplingMode::Pin);
    advanceToIdle();
    enterRunningHealthy(5.0, 0.3);
    auto sig = makeValidSignal(0.3, 5.0);
    for (int i = 0; i < 30; ++i) {
        auto out = twin_->update(1.0 / 60.0, sig);
        ASSERT_DOUBLE_EQ(out.pinVehicleSpeedTargetKmh, 5.0)
            << "precondition: RUNNING frames pin to the CSV speed";
    }

    sig.speedKmh = 60.0;  // demand the upshift
    int executionFrames = 0;
    bool firstShiftFrame = true;
    for (int i = 0; i < 400; ++i) {
        const auto out = twin_->update(1.0 / 60.0, sig);
        if (twin_->getState() == TwinState::SHIFTING) {
            if (!firstShiftFrame) {  // skip the transition frame (RUNNING-computed)
                ++executionFrames;
                EXPECT_DOUBLE_EQ(out.pinVehicleSpeedTargetKmh, -1.0)
                    << "shift-execution frames surface NO pin target";
                EXPECT_DOUBLE_EQ(out.drivetrainInputTorqueNm, 0.0)
                    << "shift-execution frames surface NO injected torque";
            }
            firstShiftFrame = false;
        } else if (executionFrames > 0) {
            break;  // shift completed
        }
    }
    EXPECT_GE(executionFrames, 1) << "precondition: shift execution frames were observed";
}

// Torque-converter shifts HOLD the converter capacity at 1.0 through the whole
// shift (the friction-clutch unload path must not run: it would free-rev the
// engine). The gear itself changes mid-shift (pause-half gearbox update).
TEST_F(VirtualIceTwinCharacterizationTest, TCShift_HoldsFullCapacity_GearChangesMidShift) {
    twin_->setCouplingModel(CouplingModelKind::TorqueConverter);
    twin_->setWheelCouplingMode(WheelCouplingMode::Pin);
    advanceToIdle();
    enterRunningHealthy(5.0, 0.3);
    auto sig = makeValidSignal(0.3, 5.0);
    for (int i = 0; i < 10; ++i) twin_->update(1.0 / 60.0, sig);
    ASSERT_EQ(twin_->getCurrentGear(), 1);

    sig.speedKmh = 60.0;
    const int gearBefore = twin_->getCurrentGear();
    bool sawGearAdvanceWhileShifting = false;
    bool sawShifting = false;
    for (int i = 0; i < 400; ++i) {
        const auto out = twin_->update(1.0 / 60.0, sig);
        if (twin_->getState() == TwinState::SHIFTING) {
            sawShifting = true;
            EXPECT_DOUBLE_EQ(out.clutchPressure, 1.0)
                << "TC shift execution must hold converter capacity at 1.0";
            EXPECT_TRUE(out.couplingIsTorqueConverter);
            if (twin_->getCurrentGear() > gearBefore) sawGearAdvanceWhileShifting = true;
        } else if (sawShifting) {
            break;  // back to RUNNING: shift complete
        }
    }
    ASSERT_TRUE(sawShifting) << "precondition: a shift was triggered";
    EXPECT_TRUE(sawGearAdvanceWhileShifting)
        << "the gear itself changes DURING the shift (pause-half update)";
    EXPECT_GT(twin_->getCurrentGear(), gearBefore)
        << "the shift completes into a HIGHER gear (the zf8 map at 60 km/h picks "
        << "the band's gear directly — the target gear value is the map's call)";
}

// Friction-clutch (default ClutchMap) shifts unload to the slip-lock floor
// (kSlipLockPressureFloor) but NEVER below it on execution frames: the clutch
// is not fully open through the gear change (free-rev guard), and the pause
// phase sits exactly at the floor.
TEST_F(VirtualIceTwinCharacterizationTest, FrictionShift_BottomsAtSlipLockFloor_NeverFullyOpen) {
    advanceToIdle();
    enterRunningHealthy(5.0, 0.3);
    auto sig = makeValidSignal(0.3, 5.0);
    for (int i = 0; i < 10; ++i) twin_->update(1.0 / 60.0, sig);
    ASSERT_EQ(twin_->getCurrentGear(), 1);

    sig.speedKmh = 60.0;
    bool firstShiftFrame = true;
    double minExecutionPressure = 1.0;
    bool sawShifting = false;
    for (int i = 0; i < 400; ++i) {
        const auto out = twin_->update(1.0 / 60.0, sig);
        if (twin_->getState() == TwinState::SHIFTING) {
            if (!firstShiftFrame) {  // execution frames only (transition frame
                                     // is RUNNING-computed and may sit lower)
                sawShifting = true;
                minExecutionPressure = std::min(minExecutionPressure, out.clutchPressure);
                EXPECT_GE(out.clutchPressure, twin::kSlipLockPressureFloor - 1e-9)
                    << "shift execution must never fully open the clutch (floor rule)";
            }
            firstShiftFrame = false;
        } else if (!firstShiftFrame) {
            break;  // shift complete
        }
    }
    ASSERT_TRUE(sawShifting) << "precondition: shift execution frames were observed";
    EXPECT_NEAR(minExecutionPressure, twin::kSlipLockPressureFloor, 1e-9)
        << "the pause phase bottoms out exactly at the slip-lock floor";
}
