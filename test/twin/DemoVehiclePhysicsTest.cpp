#include <gtest/gtest.h>
#include <input/DemoVehiclePhysics.h>

using namespace input;

class DemoVehiclePhysicsTest : public ::testing::Test {
protected:
    DemoVehiclePhysicsConfig defaultConfig;
    DemoVehiclePhysics physics{defaultConfig};
};

TEST_F(DemoVehiclePhysicsTest, ZeroThrottle_Stationary_StaysAtZero) {
    physics.update(0.016, 0.0);
    EXPECT_NEAR(physics.getSpeedKmh(), 0.0, 0.01);
}

TEST_F(DemoVehiclePhysicsTest, FullThrottleFromStandstill_Accelerates) {
    double prevSpeed = physics.getSpeedKmh();
    for (int i = 0; i < 10; ++i) {
        physics.update(0.016, 1.0);
    }
    EXPECT_GT(physics.getSpeedKmh(), prevSpeed);
}

TEST_F(DemoVehiclePhysicsTest, SpeedConvergesToTerminalVelocity) {
    for (int i = 0; i < 50000; ++i) {
        physics.update(0.016, 1.0);
    }
    double terminalKmh = physics.getSpeedKmh();

    // Terminal: F_engine = F_drag + F_rolling
    // maxForce = dragCoeff * v^2 + rolling
    // 4000 = 1.2 * v^2 + 150 → v = sqrt(3850/1.2) = 56.6 m/s = 203.6 km/h
    EXPECT_NEAR(terminalKmh, 203.6, 5.0);

    // One more tick should barely change speed
    physics.update(0.016, 1.0);
    EXPECT_NEAR(physics.getSpeedKmh(), terminalKmh, 0.1);
}

TEST_F(DemoVehiclePhysicsTest, ZeroThrottleAtSpeed_DeceleratesFromDrag) {
    // Accelerate to ~100 km/h
    for (int i = 0; i < 3000; ++i) {
        physics.update(0.016, 1.0);
    }
    double speedAt100 = physics.getSpeedKmh();
    EXPECT_GT(speedAt100, 90.0);

    // Coast (zero throttle)
    for (int i = 0; i < 1000; ++i) {
        physics.update(0.016, 0.0);
    }
    EXPECT_LT(physics.getSpeedKmh(), speedAt100);
}

TEST_F(DemoVehiclePhysicsTest, BrakeDeceleratesFasterThanCoast) {
    // Accelerate both instances to same speed
    DemoVehiclePhysicsConfig config = defaultConfig;
    DemoVehiclePhysics coasting{config};
    DemoVehiclePhysics braking{config};

    for (int i = 0; i < 3000; ++i) {
        coasting.update(0.016, 1.0);
        braking.update(0.016, 1.0);
    }
    EXPECT_NEAR(coasting.getSpeedKmh(), braking.getSpeedKmh(), 1.0);

    // Coast vs brake for 500 ticks
    for (int i = 0; i < 500; ++i) {
        coasting.update(0.016, 0.0);
        braking.update(0.016, 0.0, 1.0);
    }
    EXPECT_LT(braking.getSpeedKmh(), coasting.getSpeedKmh());
}

TEST_F(DemoVehiclePhysicsTest, Reset_ReturnsToZero) {
    for (int i = 0; i < 1000; ++i) {
        physics.update(0.016, 1.0);
    }
    EXPECT_GT(physics.getSpeedKmh(), 0.0);

    physics.reset();
    EXPECT_NEAR(physics.getSpeedKmh(), 0.0, 0.001);
    EXPECT_NEAR(physics.getAccelerationG(), 0.0, 0.001);
}

TEST_F(DemoVehiclePhysicsTest, AccelerationG_CorrectConversion) {
    // At standstill, full throttle: a = (4000 - 150) / 1800 = 2.139 m/s^2 = 0.218 G
    physics.update(0.016, 1.0);
    double expectedG = ((4000.0 - 0.0 - 150.0) / 1800.0) / 9.81;
    EXPECT_NEAR(physics.getAccelerationG(), expectedG, 0.01);
}

TEST_F(DemoVehiclePhysicsTest, ThrottleZeroAndBrakeZero_Stationary_StaysZero) {
    for (int i = 0; i < 100; ++i) {
        physics.update(0.016, 0.0, 0.0);
    }
    EXPECT_NEAR(physics.getSpeedKmh(), 0.0, 0.01);
    EXPECT_NEAR(physics.getAccelerationG(), 0.0, 0.01);
}

TEST_F(DemoVehiclePhysicsTest, CustomConfig_UsesProvidedValues) {
    DemoVehiclePhysicsConfig custom;
    custom.vehicleMassKg = 1000.0;
    custom.maxEngineForceN = 8000.0;
    custom.dragCoeffA = 0.3;
    custom.rollingResistanceN = 80.0;

    DemoVehiclePhysics customPhysics{custom};

    // Terminal: 8000 = 0.3*v^2 + 80 → v = sqrt(7920/0.3) = 162.5 m/s = 585 km/h
    for (int i = 0; i < 100000; ++i) {
        customPhysics.update(0.016, 1.0);
    }
    EXPECT_NEAR(customPhysics.getSpeedKmh(), 585.0, 20.0);
}
