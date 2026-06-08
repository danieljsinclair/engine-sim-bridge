#include "simulator/BrakeConstraint.h"
#include "vehicle.h"

BrakeConstraint::BrakeConstraint()
    : Constraint(1, 1)
    , m_ks(10.0)
    , m_kd(1.0)
    , m_vehicle(nullptr)
    , m_brakeLevel(0.0)
    , m_maxBrakeForceN(10000.0)
{}

void BrakeConstraint::initialize(atg_scs::RigidBody *vehicleMass, Vehicle *vehicle) {
    m_bodies[0] = vehicleMass;
    m_vehicle = vehicle;
}

void BrakeConstraint::setBrakeLevel(double level) {
    m_brakeLevel = level;
}

void BrakeConstraint::calculate(Output *output, atg_scs::SystemState *state) {
    output->C[0] = 0;

    output->J[0][0] = 0.0;
    output->J[0][1] = 0.0;
    output->J[0][2] = -1.0;

    output->J[0][3] = 0.0;
    output->J[0][4] = 0.0;
    output->J[0][5] = 1.0;

    output->J_dot[0][0] = 0;
    output->J_dot[0][1] = 0;
    output->J_dot[0][2] = 0;

    output->J_dot[0][3] = 0;
    output->J_dot[0][4] = 0;
    output->J_dot[0][5] = 0;

    output->kd[0] = m_kd;
    output->ks[0] = m_ks;

    output->v_bias[0] = 0;

    if (m_vehicle != nullptr && m_brakeLevel > 0.0) {
        output->limits[0][0] =
            -m_vehicle->linearForceToVirtualTorque(m_maxBrakeForceN * m_brakeLevel);
    } else {
        output->limits[0][0] = 0;
    }

    output->limits[0][1] = 0;
}
