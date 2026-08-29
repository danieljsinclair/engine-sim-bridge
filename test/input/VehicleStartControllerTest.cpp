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

// 4. Brake + PARK while running stops and latches — but only after a drive gear
// has been selected in the current run (drive-since-start gate, gap 3). Reaching
// RUNNING is not enough: the sequence below mirrors the real captures
// (brake-start in P, P->D drive-off, then back to P with the brake held).
TEST(VehicleStartControllerTest, BrakeAndParkWhileRunning_StopsAndLatches) {
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

    vsc.update(0.1, /*brake=*/true, GearSelector::PARK); // stop command
    EXPECT_FALSE(spy.ignition_);    // ignition off
    EXPECT_TRUE(vsc.isStopLatched());
    EXPECT_FALSE(vsc.isEngineOn());
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
    EXPECT_FALSE(vsc.isStopLatched());
}

// 4b. A LATER brake+PARK press while running (not the starting brake) is also
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

// 4c. The stop fires on the SAME frame brake+PARK arrives (once the gate is
// armed by a drive gear), latches, and the gate re-arms per engine run: after a
// restart, a held brake in PARK must not stop the engine again until D/R is
// selected once more.
TEST(VehicleStartControllerTest, BrakeAndPark_AfterDriveSelected_StopsSameFrameAndLatches) {
    SpyActuator spy;
    VehicleStartController vsc(spy);

    vsc.update(0.2, true, GearSelector::PARK);
    vsc.update(0.2, true, GearSelector::PARK);
    vsc.update(0.1, true, GearSelector::PARK);          // running (brake-held start)
    vsc.update(0.1, false, GearSelector::DRIVE);        // drive-since-start armed
    vsc.update(0.1, false, GearSelector::REVERSE);      // R also counts as drive
    ASSERT_TRUE(vsc.isEngineOn());

    // Brake+PARK: stopped ON THIS UPDATE, latched.
    vsc.update(0.1, /*brake=*/true, GearSelector::PARK);
    EXPECT_FALSE(vsc.isEngineOn());
    EXPECT_FALSE(spy.ignition_);
    EXPECT_TRUE(vsc.isStopLatched());

    // Latch clears on brake release (PARK, no start demand -> stays off).
    vsc.update(0.1, false, GearSelector::PARK);
    ASSERT_FALSE(vsc.isStopLatched());
    ASSERT_FALSE(vsc.isEngineOn());

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

// 5. Stop latch blocks restart while brake held; clears on release; then starts.
TEST(VehicleStartControllerTest, StopLatchBlocksRestart_UntilBrakeReleased) {
    SpyActuator spy;
    VehicleStartController vsc(spy);

    // Reach stopped + latched (brake-start, D to arm the stop gate, then P+brake).
    vsc.update(0.2, true, GearSelector::PARK);
    vsc.update(0.2, true, GearSelector::PARK);
    vsc.update(0.1, true, GearSelector::PARK);
    vsc.update(0.1, false, GearSelector::DRIVE);
    vsc.update(0.1, true, GearSelector::PARK); // brake + PARK while running
    ASSERT_TRUE(vsc.isStopLatched());

    // Brake still held -> blocked.
    vsc.update(0.1, /*brake=*/true, GearSelector::PARK);
    EXPECT_FALSE(vsc.isEngineOn());
    EXPECT_TRUE(vsc.isStopLatched());
    EXPECT_FALSE(spy.starter_); // starter not re-engaged

    // Brake released -> latch clears.
    vsc.update(0.1, /*brake=*/false, GearSelector::PARK);
    EXPECT_FALSE(vsc.isStopLatched());

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

// 7. Gap 4 (latch release): the latch releases on brake release ALONE (plan:
// `stopLatch && !brake`) — not "brake released AND out of a drive gear". While a
// drive gear is selected, the very tick that releases the latch also satisfies
// the START rule (gear trigger) and restarts the engine: releasing the brake in
// DRIVE is a drive-off. The old rule kept coldStart's engine dead for 135s
// (brake released while in D, latch never cleared).
TEST(VehicleStartControllerTest, LatchReleasesOnBrakeReleaseAlone_InDrive_RestartsViaGearTrigger) {
    SpyActuator spy;
    VehicleStartController vsc(spy);

    // Reach stopped + latched (brake-start, D to arm the stop gate, then P+brake).
    vsc.update(0.2, true, GearSelector::PARK);
    vsc.update(0.2, true, GearSelector::PARK);
    vsc.update(0.1, true, GearSelector::PARK);
    vsc.update(0.1, false, GearSelector::DRIVE);
    vsc.update(0.1, true, GearSelector::PARK); // brake + PARK while running
    ASSERT_TRUE(vsc.isStopLatched());

    // Gear D selected, brake STILL held -> latch stays, no start (the held brake
    // is exactly what the latch exists to block).
    vsc.update(0.1, /*brake=*/true, GearSelector::DRIVE);
    EXPECT_FALSE(vsc.isEngineOn());
    EXPECT_TRUE(vsc.isStopLatched());
    EXPECT_FALSE(spy.starter_);

    // Brake released while STILL in D -> latch clears and the gear trigger
    // restarts the engine on the SAME update (instant gear start).
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
    vsc.update(0.1, true, GearSelector::PARK);   // vehicle engine stops + latches
    ASSERT_TRUE(vsc.isStopLatched());

    vsc.update(0.1, false, GearSelector::PARK); // latch clears
    vsc.update(0.1, true, GearSelector::PARK);  // vehicle engine restarts
    EXPECT_TRUE(vsc.isEngineOn());
    EXPECT_TRUE(spy.starter_);

    // The manual demanded state is untouched by the vehicle layer.
    EXPECT_TRUE(mtp.isIgnitionRequested());
    EXPECT_TRUE(mtp.isStarterRequested());
}
