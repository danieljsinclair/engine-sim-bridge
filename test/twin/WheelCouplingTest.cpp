// WheelCouplingTest.cpp
//
// Tests for the three wheel-coupling strategies (Free/Pin/Torque) behind the
// IWheelCoupling interface. Pins the MATCH-mode contract: Torque injects the
// recorded input torque and leaves the sim speed unpinned; Free/Pin inject none.

#include <gtest/gtest.h>
#include <twin/WheelCoupling.h>

using namespace twin;

namespace {
constexpr double kRecordedTorqueNm = 1234.5;
constexpr double kCsvSpeedKmh = 64.0;
constexpr double kActualSimSpeedKmh = 12.0;
}  // namespace

TEST(WheelCouplingTest, TorqueStrategy_InjectsRecordedTorqueAndDoesNotPinSpeed) {
    TorqueWheelCoupling coupling;
    EXPECT_EQ(coupling.injectedInputTorqueNm(kRecordedTorqueNm), kRecordedTorqueNm);
    EXPECT_EQ(coupling.vehicleSpeedTargetKmh(kCsvSpeedKmh), -1.0);
    // Slip-lock/gearbox respond to the ACTUAL (emergent) speed, not the CSV
    // target, so the drivetrain tracks the torque-driven physics.
    EXPECT_EQ(coupling.slipLockWheelSpeedKmh(kActualSimSpeedKmh, kCsvSpeedKmh), kActualSimSpeedKmh);
    EXPECT_TRUE(coupling.launchAssistAtStandstill());
    EXPECT_EQ(coupling.getMode(), WheelCouplingMode::Torque);
}

TEST(WheelCouplingTest, TorqueStrategy_NegativeRegenTorqueIsPreserved) {
    // Regen/braking torque is negative and physically valid; the strategy must
    // pass it through verbatim (no clamping to 0), so MATCH mode can brake too.
    TorqueWheelCoupling coupling;
    EXPECT_EQ(coupling.injectedInputTorqueNm(-800.0), -800.0);
}

TEST(WheelCouplingTest, FreeStrategy_InjectsNoTorqueAndDoesNotPinSpeed) {
    FreeWheelCoupling coupling;
    EXPECT_EQ(coupling.injectedInputTorqueNm(kRecordedTorqueNm), 0.0);
    EXPECT_EQ(coupling.vehicleSpeedTargetKmh(kCsvSpeedKmh), -1.0);
    EXPECT_EQ(coupling.slipLockWheelSpeedKmh(kActualSimSpeedKmh, kCsvSpeedKmh), kActualSimSpeedKmh);
    EXPECT_TRUE(coupling.launchAssistAtStandstill());
    EXPECT_EQ(coupling.getMode(), WheelCouplingMode::Free);
}

TEST(WheelCouplingTest, PinStrategy_InjectsNoTorqueAndPinsSpeed) {
    PinWheelCoupling coupling;
    EXPECT_EQ(coupling.injectedInputTorqueNm(kRecordedTorqueNm), 0.0);
    EXPECT_EQ(coupling.vehicleSpeedTargetKmh(kCsvSpeedKmh), kCsvSpeedKmh);
    EXPECT_EQ(coupling.slipLockWheelSpeedKmh(kActualSimSpeedKmh, kCsvSpeedKmh), kCsvSpeedKmh);
    EXPECT_FALSE(coupling.launchAssistAtStandstill());
    EXPECT_EQ(coupling.getMode(), WheelCouplingMode::Pin);
}

TEST(WheelCouplingTest, Factory_MapsEachModeToItsStrategy) {
    EXPECT_EQ(makeWheelCoupling(WheelCouplingMode::Free)->getMode(), WheelCouplingMode::Free);
    EXPECT_EQ(makeWheelCoupling(WheelCouplingMode::Pin)->getMode(), WheelCouplingMode::Pin);
    EXPECT_EQ(makeWheelCoupling(WheelCouplingMode::Torque)->getMode(), WheelCouplingMode::Torque);
}

// ============================================================
// clutchLockOverride — PIN returns the BOUNDED slip-lock clutch pressure
// (engine revs on launch via partial pressure, locks at cruise, never fully
// open). FREE/TORQUE always defer to slip-lock (-1.0).
// ============================================================

TEST(WheelCouplingTest, Pin_ReturnsBoundedSlipLock_NotRigidLock) {
    PinWheelCoupling coupling;
    // At cruise (engine ~ road, slip ~ 0) the bounded slip-lock LOCKS (== 1.0).
    EXPECT_DOUBLE_EQ(coupling.clutchLockOverride(2500.0, 2500.0, 0.4, 750.0, 6500.0), 1.0);
    // At a boundary where road == idle (just crossed idle) the slip-lock still
    // produces a valid, non-open clutch pressure (>= the floor, never fully open).
    const double boundary = coupling.clutchLockOverride(750.0, 750.0, 0.4, 750.0, 6500.0);
    EXPECT_GE(boundary, twin::kSlipLockPressureFloor)
        << "PIN clutch pressure must never be fully open (floor)";
    EXPECT_LE(boundary, 1.0);
}

TEST(WheelCouplingTest, Pin_SlipOnLaunch_NotRigidlyLocked) {
    PinWheelCoupling coupling;
    // Launch: engine revs (3500) well above road-implied (1000) under WOT -> the
    // bounded slip-lock slips with PARTIAL pressure (engine revs, not locked).
    const double launch = coupling.clutchLockOverride(3500.0, 1000.0, 1.0, 750.0, 6500.0);
    EXPECT_GT(launch, 0.1) << "Launch must apply some clutch pressure (floor)";
    EXPECT_LT(launch, 1.0) << "Launch must NOT be rigidly locked (engine must rev)";
}

TEST(WheelCouplingTest, Free_AlwaysDefersToSlipLock) {
    FreeWheelCoupling coupling;
    EXPECT_EQ(coupling.clutchLockOverride(5000.0, 2500.0, 0.4, 750.0, 6500.0), -1.0);
    EXPECT_EQ(coupling.clutchLockOverride(0.0, 0.0, 0.0, 750.0, 6500.0), -1.0);
}

TEST(WheelCouplingTest, Torque_AlwaysDefersToSlipLock) {
    TorqueWheelCoupling coupling;
    EXPECT_EQ(coupling.clutchLockOverride(5000.0, 2500.0, 0.4, 750.0, 6500.0), -1.0);
    EXPECT_EQ(coupling.clutchLockOverride(0.0, 0.0, 0.0, 750.0, 6500.0), -1.0);
}
