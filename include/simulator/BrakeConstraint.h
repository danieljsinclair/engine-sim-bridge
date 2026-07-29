#ifndef BRAKE_CONSTRAINT_H
#define BRAKE_CONSTRAINT_H

#include "scs.h"

class Vehicle;

class BrakeConstraint : public atg_scs::Constraint {
public:
    BrakeConstraint();

    void initialize(atg_scs::RigidBody *vehicleMass, Vehicle *vehicle);
    void setBrakeLevel(double level);
    void calculate(Output *output, atg_scs::SystemState *state) override;

    double m_ks;
    double m_kd;

private:
    Vehicle *m_vehicle;
    double m_brakeLevel;
    double m_maxBrakeForceN;
};

#endif // BRAKE_CONSTRAINT_H
