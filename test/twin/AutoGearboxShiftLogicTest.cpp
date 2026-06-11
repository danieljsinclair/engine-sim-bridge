// AutoGearboxShiftLogicTest.cpp
// Acceptance tests for the Virtual ICE Twin automatic shift logic spec:
//   "upshift before redline or when drivetrain torque is low; downshift on
//    kickdown (high throttle) or high drivetrain torque; throttle biases the
//    decision; hysteresis prevents hunting; neutral holds gear."
//
// Each AC is provable from inputs (speed/RPM/torque/throttle/selector) alone.
// These exercise the torque- and selector-aware update() overload.

#include <gtest/gtest.h>
#include <twin/AutomaticGearbox.h>
#include <twin/IceVehicleProfile.h>
#include <simulator/GearConventions.h>
#include <cmath>

using namespace twin;

namespace {
// Mirror of AutomaticGearbox's internal RPM calc, for asserting expected RPM.
double engineRpmAt(double speedKmh, int gear, const IceVehicleProfile& profile) {
    if (gear < 1 || gear > static_cast<int>(profile.gearRatios.size())) return 0.0;
    double speedMs = speedKmh / 3.6;
    double wheelRpm = speedMs / (2.0 * M_PI * profile.tireRadiusM) * 60.0;
    return wheelRpm * profile.gearRatios[gear - 1] * profile.diffRatio;
}

// Drive the gearbox for many small steps until either the predicate is true or
// the step budget is exhausted. Returns whether the predicate ever held.
template <typename Pred>
bool runUntil(AutomaticGearbox& gb, double dt, int maxSteps,
              double speedKmh, double throttle, double torqueNm,
              Pred pred) {
    for (int i = 0; i < maxSteps; ++i) {
        gb.update(dt, speedKmh, throttle, torqueNm);
        if (pred(gb)) return true;
    }
    return pred(gb);
}
} // namespace

class AutoGearboxShiftLogicTest : public ::testing::Test {
protected:
    IceVehicleProfile profile = IceVehicleProfile::zf8hp45();
};

// AC1: In DRIVE near redline -> upshifts (RPM held below redline).
TEST_F(AutoGearboxShiftLogicTest, AC1_UpshiftsBeforeRedlineInDrive) {
    AutomaticGearbox gb(profile);
    gb.setGearSelector(bridge::GearSelector::DRIVE);

    // Feed redline-band RPM feedback so the redline safety path is the driver.
    gb.setTwinContext(/*state*/ 3, /*clutch*/ 1.0, /*speedFb*/ 5.0,
                      /*rpmFb*/ profile.redlineRpm * 0.98);
    gb.update(0.1, 5.0, 0.9, /*torqueNm*/ 100.0);

    ASSERT_GT(gb.getCurrentGear(), 1)
        << "Near redline in DRIVE the box must upshift to keep RPM below redline";
    // In the new (higher) gear the same road speed yields lower engine RPM.
    double rpmAfter = engineRpmAt(5.0, gb.getCurrentGear(), profile);
    EXPECT_LT(rpmAfter, profile.redlineRpm)
        << "After upshift, engine RPM at this road speed must be below redline";
}

// AC2: Low load / light throttle (cruise) -> upshifts.
TEST_F(AutoGearboxShiftLogicTest, AC2_UpshiftsAtLowLoadLightThrottle) {
    AutomaticGearbox gb(profile);
    gb.setGearSelector(bridge::GearSelector::DRIVE);

    // Cruise: light throttle, low drivetrain torque. Sweep speed up.
    bool upshifted = runUntil(gb, /*dt*/ 0.5, /*steps*/ 200,
                              /*speed*/ 60.0, /*throttle*/ 0.10, /*torque*/ 40.0,
                              [](AutomaticGearbox& g) { return g.getCurrentGear() >= 3; });
    EXPECT_TRUE(upshifted)
        << "Cruise (light throttle, low torque) should climb into higher gears";
    EXPECT_LE(gb.getCurrentGear(), static_cast<int>(profile.gearRatios.size()));
}

// AC3: High throttle (kickdown) -> downshifts.
TEST_F(AutoGearboxShiftLogicTest, AC3_DownshiftsOnKickdownHighThrottle) {
    AutomaticGearbox gb(profile);
    gb.setGearSelector(bridge::GearSelector::DRIVE);

    // Get into a cruising gear first (gear >= 3) at moderate throttle.
    ASSERT_TRUE(runUntil(gb, 0.5, 200, 80.0, 0.30, 120.0,
        [](AutomaticGearbox& g) { return g.getCurrentGear() >= 3; }));
    int cruiseGear = gb.getCurrentGear();
    ASSERT_GE(cruiseGear, 3);

    // Clear shift interval timers.
    for (int i = 0; i < 40; ++i) gb.update(0.1, 80.0, 0.30, 120.0);

    // Kickdown: floor throttle. Torque feed is moderate-high (demand accel).
    gb.update(0.05, 80.0, 0.98, /*torqueNm*/ 250.0);

    EXPECT_LT(gb.getCurrentGear(), cruiseGear)
        << "High throttle (kickdown) must downshift to a lower gear";
}

// AC4: High drivetrain torque (load) -> downshifts even without a throttle spike.
TEST_F(AutoGearboxShiftLogicTest, AC4_DownshiftsOnHighDrivetrainTorque) {
    AutomaticGearbox gb(profile);
    gb.setGearSelector(bridge::GearSelector::DRIVE);

    // Cruise into a higher gear at light throttle + low torque.
    ASSERT_TRUE(runUntil(gb, 0.5, 200, 70.0, 0.15, 40.0,
        [](AutomaticGearbox& g) { return g.getCurrentGear() >= 3; }));
    int cruiseGear = gb.getCurrentGear();
    ASSERT_GE(cruiseGear, 3);

    // Clear interval timers.
    for (int i = 0; i < 40; ++i) gb.update(0.1, 70.0, 0.15, 40.0);

    // High load suddenly applied (e.g. towing/grade) at unchanged light throttle.
    // Sustained high torque should pull a downshift.
    bool downshifted = runUntil(gb, 0.5, 40, 70.0, 0.15, /*torqueNm*/ 400.0,
        [&](AutomaticGearbox& g) { return g.getCurrentGear() < cruiseGear; });
    EXPECT_TRUE(downshifted)
        << "Sustained high drivetrain torque (load) must downshift";
}

// AC5: Hysteresis — at the threshold the box does not oscillate on every step.
TEST_F(AutoGearboxShiftLogicTest, AC5_NoHuntingAtThreshold) {
    AutomaticGearbox gb(profile);
    gb.setGearSelector(bridge::GearSelector::DRIVE);

    // Climb to a stable cruising gear.
    ASSERT_TRUE(runUntil(gb, 0.5, 200, 60.0, 0.20, 80.0,
        [](AutomaticGearbox& g) { return g.getCurrentGear() >= 3; }));
    for (int i = 0; i < 40; ++i) gb.update(0.1, 60.0, 0.20, 80.0);

    // Hold speed/throttle/torque exactly at the current operating point and
    // count direction changes. A non-hunting box settles and stays put.
    int startGear = gb.getCurrentGear();
    int directionChanges = 0;
    int lastDir = 0;
    for (int i = 0; i < 200; ++i) {
        gb.update(0.1, 60.0, 0.20, 80.0);
        int g = gb.getCurrentGear();
        int dir = (g > startGear) - (g < startGear); // -1, 0, +1
        if (dir != 0 && lastDir != 0 && dir != lastDir) ++directionChanges;
        if (dir != 0) lastDir = dir;
    }
    EXPECT_EQ(directionChanges, 0)
        << "At a fixed operating point the box must not oscillate (hunt)";
}

// AC6: In NEUTRAL -> no shifting; gear holds.
TEST_F(AutoGearboxShiftLogicTest, AC6_NoShiftInNeutralGearHolds) {
    AutomaticGearbox gb(profile);
    gb.setGearSelector(bridge::GearSelector::NEUTRAL);

    // Establish a non-default gear so we can observe it holding.
    gb.setGearSelector(bridge::GearSelector::DRIVE);
    ASSERT_TRUE(runUntil(gb, 0.5, 200, 60.0, 0.30, 100.0,
        [](AutomaticGearbox& g) { return g.getCurrentGear() >= 3; }));
    int heldGear = gb.getCurrentGear();
    ASSERT_GE(heldGear, 3);

    // Switch to NEUTRAL and apply inputs that WOULD shift in DRIVE.
    gb.setGearSelector(bridge::GearSelector::NEUTRAL);
    for (int i = 0; i < 40; ++i) {
        gb.update(0.1, 60.0, /*throttle*/ 0.98, /*torque*/ 400.0);
        ASSERT_FALSE(gb.requestsShift())
            << "In NEUTRAL the gearbox must not request a shift";
    }
    EXPECT_EQ(gb.getCurrentGear(), heldGear)
        << "In NEUTRAL the current gear must hold";
}

// AC7: Shifts stay within the box's gear range; convention P=-2,R=-1,N=0,1..8.
TEST_F(AutoGearboxShiftLogicTest, AC7_GearStaysInRangeAndHoldsConvention) {
    // The gearbox currentGear_ is the bridge forward-gear index (1..N). The
    // selector/gear-convention mapping (P=-2, R=-1, N=0, forward 1..) is owned
    // by GearConventions; here we assert the forward range invariant under
    // aggressive operation, and that the convention enum values are correct.
    EXPECT_EQ(static_cast<int>(bridge::GearSelector::PARK),    -2);
    EXPECT_EQ(static_cast<int>(bridge::GearSelector::REVERSE), -1);
    EXPECT_EQ(static_cast<int>(bridge::GearSelector::NEUTRAL),  0);
    EXPECT_EQ(static_cast<int>(bridge::BridgeGear::NEUTRAL),    0);
    EXPECT_EQ(static_cast<int>(bridge::BridgeGear::FIRST),      1);

    AutomaticGearbox gb(profile);
    gb.setGearSelector(bridge::GearSelector::DRIVE);
    const int topGear = static_cast<int>(profile.gearRatios.size());

    // Push very fast + heavy throttle + heavy torque: must never exceed top gear
    // or drop below 1.
    for (int i = 0; i < 400; ++i) {
        gb.update(0.1, 250.0, 1.0, 500.0);
        ASSERT_GE(gb.getCurrentGear(), 1);
        ASSERT_LE(gb.getCurrentGear(), topGear);
    }

    // Push to a crawl: must never go below 1.
    for (int i = 0; i < 400; ++i) {
        gb.update(0.1, 1.5, 0.0, 0.0);
        ASSERT_GE(gb.getCurrentGear(), 1);
        ASSERT_LE(gb.getCurrentGear(), topGear);
    }
}
