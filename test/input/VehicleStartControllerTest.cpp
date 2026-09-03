// VehicleStartControllerTest.cpp - tests for the vehicle-driven start/stop
// decision layer. Time is advanced via explicit dt (no sleep, no ILoopClock).
// The controller is observed through a SpyActuator implementing IEngineActuator.

#include <gtest/gtest.h>

#include "input/VehicleStartController.h"
#include "input/IEngineActuator.h"
#include "input/ManualTwinProvider.h"
#include "input/IThrottleSource.h"
#include "simulator/ISimulator.h"
#include "simulator/EngineSimTypes.h"

#include <memory>

using namespace input;
using bridge::GearSelector;

namespace {

// Spy implementing IEngineActuator. Records last + history of ignition/starter.
class SpyActuator : public IEngineActuator {
public:
    bool ignition_ = false;
    bool starter_  = false;
    int ignitionCalls_ = 0;
    int starterCalls_  = 0;
    void setIgnition(bool on) override { ignition_ = on; ++ignitionCalls_; }
    void setStarterMotor(bool on) override { starter_ = on; ++starterCalls_; }
};

class MockThrottleSource : public IThrottleSource {
public:
    double pollThrottle() override { return throttle; }
    double throttle = 0.0;
};

class MockSimulator : public ISimulator {
public:
    bool create(const ISimulatorConfig&, ILogging*, telemetry::ITelemetryWriter*) override { return true; }
    void destroy() override {}
    std::string getLastError() const override { return ""; }
    const char* getName() const override { return "MockSimulator"; }
    void update(double) override {}
    EngineSimStats getStats() const override { return {}; }
    void setThrottle(double) override {}
    bool renderOnDemand(float*, int32_t, int32_t*) override { return false; }
    bool readAudioBuffer(float*, int32_t, int32_t*) override { return false; }
    bool start() override { return true; }
    void stop() override {}
    int getSimulationFrequency() const override { return 60; }
    double getEngineRpm() const override { return rpm; }
    double rpm = 0.0;
};

} // namespace

// 1. Brake press while off: starter engages at t0; ignition only after 0.5s.
TEST(VehicleStartControllerTest, BrakePressWhileOff_CranksThenIgnitesAfterDelay) {
    SpyActuator spy;
    VehicleStartController vsc(spy);

    vsc.update(0.2, /*brakePressed=*/true, GearSelector::PARK);
    EXPECT_TRUE(spy.starter_);     // starter engaged at t0
    EXPECT_FALSE(spy.ignition_);   // ignition still off
    EXPECT_TRUE(vsc.isEngineOn()); // engine considered "on" once cranking

    vsc.update(0.2, true, GearSelector::PARK); // accumulated 0.4
    EXPECT_FALSE(spy.ignition_);  // 0.4 < 0.5

    vsc.update(0.1, true, GearSelector::PARK); // accumulated 0.5
    EXPECT_TRUE(spy.ignition_);   // ignition on at t0.5
    EXPECT_TRUE(vsc.isEngineOn());
    EXPECT_FALSE(vsc.isStopLatched());
}

// 2. Gear D while off (no brake): a start demand that includes a drive gear
// fires starter AND ignition together on the first update (t=0, no crank
// delay — the delay applies only to brake-initiated cranks).
TEST(VehicleStartControllerTest, GearDWhileOff_StartsInstantly) {
    SpyActuator spy;
    VehicleStartController vsc(spy);

    vsc.update(0.2, /*brakePressed=*/false, GearSelector::DRIVE);
    EXPECT_TRUE(spy.starter_);     // starter engaged at t0
    EXPECT_TRUE(spy.ignition_);    // ignition fired on the SAME update
    EXPECT_TRUE(vsc.isEngineOn());
    EXPECT_FALSE(vsc.isStopLatched());
    EXPECT_GE(spy.ignitionCalls_, 1);
}

// 2b. Gear R while off: REVERSE is a drive gear too — the instant start must
// not be D-only logic.
TEST(VehicleStartControllerTest, GearRWhileOff_StartsInstantly) {
    SpyActuator spy;
    VehicleStartController vsc(spy);

    vsc.update(0.2, /*brakePressed=*/false, GearSelector::REVERSE);
    EXPECT_TRUE(spy.starter_);
    EXPECT_TRUE(spy.ignition_);
    EXPECT_TRUE(vsc.isEngineOn());
    EXPECT_FALSE(vsc.isStopLatched());
}

// 2c. Brake AND gear D together while off: the presence of a drive gear
// dominates the delay rule — ignition still fires on the first update.
TEST(VehicleStartControllerTest, BrakeAndGearDTogetherWhileOff_IgnitesImmediately) {
    SpyActuator spy;
    VehicleStartController vsc(spy);

    vsc.update(0.2, /*brakePressed=*/true, GearSelector::DRIVE);
    EXPECT_TRUE(spy.ignition_);
    EXPECT_TRUE(spy.starter_);
    EXPECT_TRUE(vsc.isEngineOn());
}

// 2d. A gear start must not arm the crank timer: after the instant start,
// continued DRIVE updates with no brake never re-fire ignition or drop it.
TEST(VehicleStartControllerTest, GearStartDoesNotArmCrankTimer) {
    SpyActuator spy;
    VehicleStartController vsc(spy);

    vsc.update(0.2, false, GearSelector::DRIVE);  // instant start
    ASSERT_TRUE(spy.ignition_);
    const int callsAtStart = spy.ignitionCalls_;

    // Accumulate well past the 0.5s crank delay in DRIVE with no brake.
    for (int i = 0; i < 10; ++i) {
        vsc.update(0.1, false, GearSelector::DRIVE);
    }

    EXPECT_TRUE(spy.ignition_);          // ignition never dropped
    EXPECT_EQ(spy.ignitionCalls_, callsAtStart);  // no re-fire artefact
    EXPECT_TRUE(vsc.isEngineOn());
    EXPECT_FALSE(vsc.isStopLatched());
}

// 3. Gear P while off, no brake: neither start nor stop triggers.
TEST(VehicleStartControllerTest, GearPWhileOff_NoStart) {
    SpyActuator spy;
    VehicleStartController vsc(spy);

    vsc.update(0.2, /*brake=*/false, GearSelector::PARK);
    EXPECT_FALSE(spy.starter_);
    EXPECT_FALSE(spy.ignition_);
    EXPECT_FALSE(vsc.isEngineOn());
    EXPECT_FALSE(vsc.isStopLatched());
}

// 4. (owner 2026-08-30) A brake press-and-hold in PARK must NOT cut ignition:
// the stop is edge-triggered on brake RELEASE in park, so the driver can put
// a foot on the brake to select first gear without the engine dying. The
// drive-since-start gate is armed first (brake-start in P, P->D drive-off,
// then back to P) — the sequence the real captures follow.
TEST(VehicleStartControllerTest, BrakePressAndHoldInPark_AfterDrive_DoesNotCutIgnition) {
    SpyActuator spy;
    VehicleStartController vsc(spy);

    // Start via brake first (ignition on, engineOn true).
    vsc.update(0.2, true, GearSelector::PARK);
    vsc.update(0.2, true, GearSelector::PARK);
    vsc.update(0.1, true, GearSelector::PARK);
    ASSERT_TRUE(vsc.isEngineOn());
    ASSERT_TRUE(spy.ignition_);

    // Drive gear selected in this run arms the stop gate.
    vsc.update(0.1, /*brake=*/false, GearSelector::DRIVE);
    ASSERT_TRUE(vsc.isEngineOn());

    // Back in PARK with the brake pressed and HELD — for a long time. Only the
    // later release edge may cut, so the press alone must keep the engine alive.
    for (int i = 0; i < 50; ++i) {
        vsc.update(0.1, /*brake=*/true, GearSelector::PARK);
        ASSERT_TRUE(vsc.isEngineOn())
            << "brake press-and-hold in PARK cut ignition at tick " << i;
        ASSERT_TRUE(spy.ignition_);
    }
    EXPECT_FALSE(vsc.isStopLatched());
}

// 4c. The cut fires on the brake ON->OFF transition while RUNNING in PARK with
// the drive gate armed — exactly once: the stop takes the engine off on the
// release tick and later PARK/no-brake ticks neither restart it nor re-fire
// the stop. The gate then re-arms per engine run: a fresh brake-start in P
// cannot be stopped by its own held brake until D/R is selected once more.
TEST(VehicleStartControllerTest, BrakeReleaseInPark_AfterDrive_CutsIgnitionOnce) {
    SpyActuator spy;
    VehicleStartController vsc(spy);

    vsc.update(0.2, true, GearSelector::PARK);
    vsc.update(0.2, true, GearSelector::PARK);
    vsc.update(0.1, true, GearSelector::PARK);          // running (brake-held start)
    vsc.update(0.1, false, GearSelector::DRIVE);        // drive-since-start armed
    vsc.update(0.1, false, GearSelector::REVERSE);      // R also counts as drive
    ASSERT_TRUE(vsc.isEngineOn());

    // Brake pressed and held in PARK: running continues (the press is not the cut).
    vsc.update(0.1, /*brake=*/true, GearSelector::PARK);
    ASSERT_TRUE(vsc.isEngineOn())
        << "brake press in PARK must not cut ignition";

    // Release edge in PARK: stopped ON THIS UPDATE, latched, exactly one cut.
    vsc.update(0.1, /*brake=*/false, GearSelector::PARK);
    EXPECT_FALSE(vsc.isEngineOn());
    EXPECT_FALSE(spy.ignition_);
    EXPECT_TRUE(vsc.isStopLatched());
    const int ignitionCallsAtStop = spy.ignitionCalls_;

    // Later PARK/no-brake ticks: the engine stays off (no restart demand, no
    // second stop artefact) and the latch releases on the brake-off tick.
    for (int i = 0; i < 5; ++i) {
        vsc.update(0.1, false, GearSelector::PARK);
        EXPECT_FALSE(vsc.isEngineOn());
        EXPECT_FALSE(spy.ignition_);
    }
    EXPECT_FALSE(vsc.isStopLatched());
    EXPECT_EQ(spy.ignitionCalls_, ignitionCallsAtStop)
        << "the stop must fire exactly once";

    // New engine run via brake-start in P: the drive gate must have reset with
    // the run, so the held brake must NOT stop this fresh start.
    vsc.update(0.1, true, GearSelector::PARK);
    vsc.update(0.1, true, GearSelector::PARK);
    vsc.update(0.1, true, GearSelector::PARK);
    vsc.update(0.1, true, GearSelector::PARK);  // past the 0.5s crank, brake held
    vsc.update(0.1, true, GearSelector::PARK);
    EXPECT_TRUE(vsc.isEngineOn())
        << "drive-since-start gate did not reset with the engine run";
    EXPECT_TRUE(spy.ignition_);
}

// 4d. Shifting out of PARK before the brake is released is a drive-off: no
// cut — neither on the shift tick (brake still held but no longer PARK) nor on
// the later release edge (released outside PARK). Includes the same-tick
// variant: the gear change and the release arriving together must not stop
// the engine either.
TEST(VehicleStartControllerTest, ShiftOutOfParkBeforeBrakeRelease_NoCut) {
    // Variant 1: shift while the brake is still held, release later in DRIVE.
    {
        SpyActuator spy;
        VehicleStartController vsc(spy);
        vsc.update(0.2, true, GearSelector::PARK);
        vsc.update(0.2, true, GearSelector::PARK);
        vsc.update(0.1, true, GearSelector::PARK);   // running
        vsc.update(0.1, false, GearSelector::DRIVE); // arm the gate, drive-off
        vsc.update(0.1, true, GearSelector::PARK);   // parked again, brake held
        ASSERT_TRUE(vsc.isEngineOn());

        vsc.update(0.1, /*brake=*/true, GearSelector::DRIVE);  // shift out first
        EXPECT_TRUE(vsc.isEngineOn())
            << "shifting out of PARK while braking must not cut ignition";
        vsc.update(0.1, /*brake=*/false, GearSelector::DRIVE); // release in DRIVE
        EXPECT_TRUE(vsc.isEngineOn())
            << "release edge outside PARK must not cut ignition";
        EXPECT_TRUE(spy.ignition_);
    }
    // Variant 2: release and shift out of PARK on the SAME tick.
    {
        SpyActuator spy;
        VehicleStartController vsc(spy);
        vsc.update(0.2, true, GearSelector::PARK);
        vsc.update(0.2, true, GearSelector::PARK);
        vsc.update(0.1, true, GearSelector::PARK);   // running
        vsc.update(0.1, false, GearSelector::DRIVE); // arm the gate
        vsc.update(0.1, true, GearSelector::PARK);   // brake held in PARK
        ASSERT_TRUE(vsc.isEngineOn());

        vsc.update(0.1, /*brake=*/false, GearSelector::DRIVE); // release + drive-off
        EXPECT_TRUE(vsc.isEngineOn())
            << "same-tick release+shift is a drive-off, not a stop";
        EXPECT_TRUE(spy.ignition_);
    }
}

// 4a. Gap 3, the old self-stop: a brake-initiated start in PARK must NOT be
// stopped by its own still-held brake. Before the drive-since-start gate, the
// brake+PARK stop fired on the very frame ignition completed (0.5s crank, both
// captures shift P->D ~0.73s later), so every "brake touch starts" ended
// latched-off. Now: RUNNING CONTINUES while brake+PARK is held.
TEST(VehicleStartControllerTest, BrakeStartInP_StaysRunningWhileBrakeHeld) {
    SpyActuator spy;
    VehicleStartController vsc(spy);

    // Brake-initiated start in PARK: crank 0.5s, brake held throughout.
    vsc.update(0.2, true, GearSelector::PARK);
    vsc.update(0.2, true, GearSelector::PARK);
    vsc.update(0.1, true, GearSelector::PARK);  // ignition fires here
    ASSERT_TRUE(spy.ignition_);
    ASSERT_TRUE(vsc.isEngineOn());

    // The brake stays held, still in PARK — for a long time. The engine that the
    // brake just started must keep running (the stop gate needs drive-since-start).
    for (int i = 0; i < 50; ++i) {
        vsc.update(0.1, /*brake=*/true, GearSelector::PARK);
        ASSERT_TRUE(vsc.isEngineOn())
            << "self-stopped at held-brake tick " << i;
        ASSERT_TRUE(spy.ignition_);
    }
    // Releasing the brake in PARK must not stop it either: no drive gear has
    // been selected in this run, so the release-edge stop is not armed yet.
    vsc.update(0.1, /*brake=*/false, GearSelector::PARK);
    EXPECT_TRUE(vsc.isEngineOn())
        << "release in PARK before any drive gear must not stop the engine";
    EXPECT_TRUE(spy.ignition_);
    EXPECT_FALSE(vsc.isStopLatched());
}

// 4b. A LATER brake press in PARK while running (not the starting brake) is
// not a stop while no drive gear has been selected in the current run.
TEST(VehicleStartControllerTest, BrakeAndPark_NoDriveSinceStart_DoesNotStop) {
    SpyActuator spy;
    VehicleStartController vsc(spy);

    // Start via a drive gear (instant), then immediately select PARK: the start
    // trigger itself was D/R, so the gate is armed (see 4c) — instead, build the
    // un-armed case cleanly: brake-start in P, release brake, re-press in P.
    vsc.update(0.2, true, GearSelector::PARK);
    vsc.update(0.2, true, GearSelector::PARK);
    vsc.update(0.1, true, GearSelector::PARK);  // running
    vsc.update(0.1, false, GearSelector::PARK); // brake released, still P
    ASSERT_TRUE(vsc.isEngineOn());

    // Fresh brake press in PARK, engine running, no D/R seen since the start.
    vsc.update(0.1, /*brake=*/true, GearSelector::PARK);
    EXPECT_TRUE(vsc.isEngineOn()) << "stopped without drive-since-start";
    EXPECT_TRUE(spy.ignition_);
    EXPECT_FALSE(vsc.isStopLatched());
}

// 5. Restart path (unchanged): after a release-triggered PARK stop the engine
// stays off until a new start demand — a later brake press cranks it again.
TEST(VehicleStartControllerTest, AfterParkReleaseStop_StaysOffUntilNewBrakeStart) {
    SpyActuator spy;
    VehicleStartController vsc(spy);

    // Reach stopped + latched (brake-start, D to arm the stop gate, brake press
    // in PARK — no cut — then the release edge in PARK).
    vsc.update(0.2, true, GearSelector::PARK);
    vsc.update(0.2, true, GearSelector::PARK);
    vsc.update(0.1, true, GearSelector::PARK);
    vsc.update(0.1, false, GearSelector::DRIVE);
    vsc.update(0.1, true, GearSelector::PARK);  // brake on in PARK: no cut yet
    vsc.update(0.1, false, GearSelector::PARK); // brake released in PARK: stop
    ASSERT_TRUE(vsc.isStopLatched());
    ASSERT_FALSE(vsc.isEngineOn());
    ASSERT_FALSE(spy.ignition_);

    // Brake-off frames in PARK: no restart demand, engine stays off.
    vsc.update(0.1, /*brake=*/false, GearSelector::PARK);
    EXPECT_FALSE(vsc.isEngineOn());
    EXPECT_FALSE(vsc.isStopLatched());  // latch released on the brake-off tick

    // Brake pressed again -> starts.
    vsc.update(0.1, /*brake=*/true, GearSelector::PARK);
    EXPECT_TRUE(vsc.isEngineOn());
    EXPECT_TRUE(spy.starter_);   // engaged at t0
    EXPECT_FALSE(vsc.isStopLatched());
}

// 6. Crank pending (brake-initiated), then D selected: ignites immediately.
TEST(VehicleStartControllerTest, CrankPendingThenGearD_IgnitesImmediatelyAndCancelsTimer) {
    SpyActuator spy;
    VehicleStartController vsc(spy);

    vsc.update(0.2, true, GearSelector::PARK);  // starter true, accum 0.2
    vsc.update(0.2, true, GearSelector::PARK);  // accum 0.4 (< 0.5 => ignition off)
    EXPECT_FALSE(spy.ignition_);

    vsc.update(0.05, true, GearSelector::DRIVE); // gear D before 0.5 reached
    EXPECT_TRUE(spy.ignition_);   // ON immediately, before 0.5
    EXPECT_TRUE(spy.starter_);
    EXPECT_TRUE(vsc.isEngineOn());
    EXPECT_FALSE(vsc.isStopLatched());
}

// 7. Restart path (unchanged, gap 4 lineage): the latch releases on brake
// release ALONE (plan: `stopLatch && !brake`) — not "brake released AND out of
// a drive gear". After a release-triggered PARK stop, selecting D with the
// brake already off (the walk-up restart) clears the latch and restarts the
// engine on the SAME update via the gear trigger: engaging a gear is a
// drive-off, not a stay-off.
TEST(VehicleStartControllerTest, AfterParkReleaseStop_GearSelectionRestartsInstantly) {
    SpyActuator spy;
    VehicleStartController vsc(spy);

    // Reach stopped + latched (brake-start, D to arm the stop gate, brake press
    // in PARK — no cut — then the release edge in PARK).
    vsc.update(0.2, true, GearSelector::PARK);
    vsc.update(0.2, true, GearSelector::PARK);
    vsc.update(0.1, true, GearSelector::PARK);
    vsc.update(0.1, false, GearSelector::DRIVE);
    vsc.update(0.1, true, GearSelector::PARK);  // brake on in PARK: no cut yet
    vsc.update(0.1, false, GearSelector::PARK); // release edge: stop + latch
    ASSERT_TRUE(vsc.isStopLatched());
    ASSERT_FALSE(vsc.isEngineOn());

    // Brake off + gear D on the SAME tick: latch clears AND the gear trigger
    // restarts the engine instantly (starter + ignition together).
    vsc.update(0.1, /*brake=*/false, GearSelector::DRIVE);
    EXPECT_FALSE(vsc.isStopLatched())
        << "latch must release on brake release alone, even in a drive gear";
    EXPECT_TRUE(vsc.isEngineOn())
        << "gear trigger must restart the engine the moment the latch clears";
    EXPECT_TRUE(spy.ignition_);
    EXPECT_TRUE(spy.starter_);
}

// 8. Manual path is fully independent of the vehicle start/stop layer.
TEST(VehicleStartControllerTest, ManualPathIndependence) {
    SpyActuator spy;
    VehicleStartController vsc(spy);

    // Set up the separate manual path.
    auto throttle = std::make_unique<MockThrottleSource>();
    MockSimulator sim;
    ManualTwinProvider mtp(std::move(throttle), sim);
    ASSERT_TRUE(mtp.Initialize());
    mtp.setIgnitionRequested(true);
    mtp.setStarterRequested(true);

    // Drive the vehicle controller through start -> stop -> restart.
    vsc.update(0.2, true, GearSelector::PARK);
    vsc.update(0.2, true, GearSelector::PARK);
    vsc.update(0.1, true, GearSelector::PARK); // vehicle engine on
    ASSERT_TRUE(vsc.isEngineOn());

    vsc.update(0.1, false, GearSelector::DRIVE); // arm the drive-since-start gate
    vsc.update(0.1, true, GearSelector::PARK);   // brake on in PARK: no cut yet
    vsc.update(0.1, false, GearSelector::PARK);  // release edge: stops + latches
    ASSERT_TRUE(vsc.isStopLatched());

    vsc.update(0.1, false, GearSelector::PARK); // latch clears
    vsc.update(0.1, true, GearSelector::PARK);  // vehicle engine restarts
    EXPECT_TRUE(vsc.isEngineOn());
    EXPECT_TRUE(spy.starter_);

    // The manual demanded state is untouched by the vehicle layer.
    EXPECT_TRUE(mtp.isIgnitionRequested());
    EXPECT_TRUE(mtp.isStarterRequested());
}

// ---- Composable primitives (owner spec 2026-09-02: single shared state
// machine, every frontend a thin caller). ----

// 9. requestCombinedStart fires starter+ignition together on the first call
// (t=0, no crank delay) — the CSV path's by-design operation and --start.
TEST(VehicleStartControllerTest, CombinedStart_FiresStarterAndIgnitionTogether) {
    SpyActuator spy;
    VehicleStartController vsc(spy);

    vsc.requestCombinedStart();
    EXPECT_TRUE(spy.starter_);
    EXPECT_TRUE(spy.ignition_);
    EXPECT_TRUE(vsc.isEngineOn());
    EXPECT_FALSE(vsc.isStopLatched());
}

// 10. requestStarter engages the starter WITHOUT firing ignition — the McLaren
// mod's starter-then-ignition sequencing (crank first, ignite a moment later).
TEST(VehicleStartControllerTest, RequestStarter_EngagesStarterWithoutIgnition) {
    SpyActuator spy;
    VehicleStartController vsc(spy);

    vsc.requestStarter();
    EXPECT_TRUE(spy.starter_);
    EXPECT_FALSE(spy.ignition_);  // ignition NOT fired by starter-only
    EXPECT_TRUE(vsc.isEngineOn());
}

// 11. requestStarter then requestIgnition(true) a moment later: the
// starter-then-ignition sequence completes the start (McLaren mod).
TEST(VehicleStartControllerTest, StarterThenIgnition_SequencesStart) {
    SpyActuator spy;
    VehicleStartController vsc(spy);

    vsc.requestStarter();
    EXPECT_TRUE(spy.starter_);
    EXPECT_FALSE(spy.ignition_);

    vsc.requestIgnition(true);  // ignite a moment later
    EXPECT_TRUE(spy.ignition_);
    EXPECT_TRUE(vsc.isEngineOn());
}

// 12. requestIgnition(false) turns ignition off (engine coasts down) without
// touching the starter — the separate ignition control the CLI/iOS need.
TEST(VehicleStartControllerTest, RequestIgnitionOff_CoastsDown) {
    SpyActuator spy;
    VehicleStartController vsc(spy);

    vsc.requestCombinedStart();
    ASSERT_TRUE(spy.ignition_);

    vsc.requestIgnition(false);
    EXPECT_FALSE(spy.ignition_);
    EXPECT_FALSE(vsc.isEngineOn());
}

// 13. requestIgnition(true) resolves a pending crank immediately — an explicit
// ignition command is authoritative and cancels the pending timer.
TEST(VehicleStartControllerTest, RequestIgnitionResolvesPendingCrank) {
    SpyActuator spy;
    VehicleStartController vsc(spy);

    vsc.requestStarter();  // starter engaged, crank pending
    EXPECT_TRUE(spy.starter_);
    EXPECT_FALSE(spy.ignition_);

    vsc.requestIgnition(true);  // resolves immediately, no delay wait
    EXPECT_TRUE(spy.ignition_);
    EXPECT_TRUE(vsc.isEngineOn());
}

// 14. A second requestCombinedStart while already running is a no-op (engine
// already on) — idempotent, no starter re-pulse artefact.
TEST(VehicleStartControllerTest, CombinedStart_IdempotentWhenRunning) {
    SpyActuator spy;
    VehicleStartController vsc(spy);

    vsc.requestCombinedStart();
    ASSERT_TRUE(spy.ignition_);
    const int ignitionCallsAtStart = spy.ignitionCalls_;

    vsc.requestCombinedStart();  // already running
    EXPECT_TRUE(vsc.isEngineOn());
    EXPECT_TRUE(spy.ignition_);
    EXPECT_EQ(spy.ignitionCalls_, ignitionCallsAtStart);  // no re-fire
}

// ===========================================================================
// Crank-delay scale locks (owner 2026-09-03: --starter-delay must behave as
// honest milliseconds at the controller seam — time-to-ignition scales
// linearly with the configured delay, zero fires immediately).
// ===========================================================================

namespace {
// Tick the controller with the brake held in PARK until ignition fires (or
// maxTicks); returns the tick count at the ignition edge (0 = never).
int ticksToIgnition(VehicleStartController& vsc, SpyActuator& spy,
                    double dt, int maxTicks = 10000) {
    spy.ignition_ = false;
    for (int i = 1; i <= maxTicks; ++i) {
        vsc.update(dt, /*brakePressed=*/true, GearSelector::PARK);
        if (spy.ignition_) return i;
    }
    return 0;
}
} // namespace

TEST(VehicleStartControllerTest, CrankDelayScalesLinearlyWithConfiguredDelay) {
    // The delay accumulates dt per tick and nothing caps or truncates it, so
    // ignition fires at ceil(delay/dt) ticks — within 1 tick either side (dt
    // is not exactly representable in binary, so the accumulated sum can land
    // a hair under/over the delay at the boundary; the exact tick is a float
    // artifact, not the contract).
    SpyActuator spyA;
    VehicleStartController vscA(spyA, 0.5);
    const int ticksA = ticksToIgnition(vscA, spyA, 0.05);
    EXPECT_NEAR(ticksA, 10, 1);

    SpyActuator spyB;
    VehicleStartController vscB(spyB, 1.0);
    const int ticksB = ticksToIgnition(vscB, spyB, 0.05);
    EXPECT_NEAR(ticksB, 20, 1);

    // Monotone: doubling the delay never SHORTENS the crank.
    EXPECT_GE(ticksB, ticksA);
}

TEST(VehicleStartControllerTest, CrankDelayZeroFiresIgnitionOnFirstTick) {
    // Explicit zero delay: the first accumulated dt (>= 0.0) satisfies the
    // timer immediately — no default fallback at the controller seam.
    SpyActuator spy;
    VehicleStartController vsc(spy, 0.0);
    const int ticks = ticksToIgnition(vsc, spy, 0.05);
    EXPECT_EQ(ticks, 1);
}
