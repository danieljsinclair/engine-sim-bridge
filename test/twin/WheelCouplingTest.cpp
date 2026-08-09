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
// clutchLockOverride — PIN locks clutch when road-implied RPM >= idle;
// FREE/TORQUE always defer to slip-lock (-1.0).
// ============================================================

TEST(WheelCouplingTest, Pin_LocksWhenRoadRpmAboveIdle) {
    PinWheelCoupling coupling;
    EXPECT_EQ(coupling.clutchLockOverride(2000.0, 750.0), 1.0);
    EXPECT_EQ(coupling.clutchLockOverride(750.0, 750.0), 1.0);   // boundary: equal
}

TEST(WheelCouplingTest, Pin_DefersBelowIdle) {
    PinWheelCoupling coupling;
    EXPECT_EQ(coupling.clutchLockOverride(700.0, 750.0), -1.0);
    EXPECT_EQ(coupling.clutchLockOverride(0.0, 750.0), -1.0);
}

TEST(WheelCouplingTest, Free_AlwaysDefersToSlipLock) {
    FreeWheelCoupling coupling;
    EXPECT_EQ(coupling.clutchLockOverride(5000.0, 750.0), -1.0);
    EXPECT_EQ(coupling.clutchLockOverride(0.0, 750.0), -1.0);
}

TEST(WheelCouplingTest, Torque_AlwaysDefersToSlipLock) {
    TorqueWheelCoupling coupling;
    EXPECT_EQ(coupling.clutchLockOverride(5000.0, 750.0), -1.0);
    EXPECT_EQ(coupling.clutchLockOverride(0.0, 750.0), -1.0);
}
