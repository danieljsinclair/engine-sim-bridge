// BrakeConstraintTest.cpp - TDD: prove brake constraint applies retarding force

#include <gtest/gtest.h>
#include "simulator/BrakeConstraint.h"
#include "vehicle.h"
#include "rigid_body.h"
#include "rigid_body_system.h"

class BrakeConstraintTest : public ::testing::Test {
protected:
    atg_scs::RigidBodySystem system;
    atg_scs::RigidBody vehicleMass;
    Vehicle vehicle;
    BrakeConstraint brake;

    void SetUp() override {
        vehicleMass.reset();
        vehicleMass.m = 1.0;
        vehicleMass.I = 1.0;

        Vehicle::Parameters params;
        params.mass = 1500.0;
        params.dragCoefficient = 0.3;
        params.crossSectionArea = 2.2;
        params.diffRatio = 3.42;
        params.tireRadius = 0.32;
        params.rollingResistance = 100.0;
        vehicle.initialize(params);

        // Vehicle needs m_rotatingMass set (normally done by addToSystem)
        vehicle.addToSystem(&system, &vehicleMass);

        brake.initialize(&vehicleMass, &vehicle);
    }

    atg_scs::Constraint::Output calculateOutput() {
        atg_scs::Constraint::Output output;
        atg_scs::SystemState state;
        brake.calculate(&output, &state);
        return output;
    }
};

TEST_F(BrakeConstraintTest, ZeroBrake_NoForce) {
    brake.setBrakeLevel(0.0);
    auto out = calculateOutput();
    EXPECT_DOUBLE_EQ(out.limits[0][0], 0.0);
    EXPECT_DOUBLE_EQ(out.limits[0][1], 0.0);
}

TEST_F(BrakeConstraintTest, FullBrake_NegativeRetardingTorque) {
    brake.setBrakeLevel(1.0);
    auto out = calculateOutput();
    EXPECT_LT(out.limits[0][0], 0.0)
        << "Full brake should apply negative (retarding) torque";
    EXPECT_DOUBLE_EQ(out.limits[0][1], 0.0)
        << "Brake should never apply propulsive force";
}

TEST_F(BrakeConstraintTest, HalfBrake_HalfForce) {
    brake.setBrakeLevel(1.0);
    auto fullOut = calculateOutput();

    brake.setBrakeLevel(0.5);
    auto halfOut = calculateOutput();

    EXPECT_NEAR(halfOut.limits[0][0], fullOut.limits[0][0] * 0.5, 0.001)
        << "Half brake should be exactly half the retarding torque";
}

TEST_F(BrakeConstraintTest, JacobianCorrect) {
    brake.setBrakeLevel(1.0);
    auto out = calculateOutput();
    EXPECT_DOUBLE_EQ(out.J[0][2], -1.0)
        << "J[0][2] should be -1.0 (angular velocity constraint)";
}

TEST_F(BrakeConstraintTest, StiffnessAndDamping) {
    auto out = calculateOutput();
    EXPECT_DOUBLE_EQ(out.ks[0], 10.0);
    EXPECT_DOUBLE_EQ(out.kd[0], 1.0);
}
