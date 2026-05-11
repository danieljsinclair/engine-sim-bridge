#ifndef DEMO_VEHICLE_PHYSICS_H
#define DEMO_VEHICLE_PHYSICS_H

namespace input {

struct DemoVehiclePhysicsConfig {
    double vehicleMassKg = 1800.0;
    double maxEngineForceN = 4000.0;
    double dragCoeffA = 1.2;
    double rollingResistanceN = 150.0;
    double brakingDecelerationMs2 = 8.0;
};

class DemoVehiclePhysics {
public:
    explicit DemoVehiclePhysics(const DemoVehiclePhysicsConfig& config = {})
        : config_(config) {}

    double update(double dt, double throttle, double brake = 0.0) {
        double engineForce = throttle * config_.maxEngineForceN;
        double dragForce = config_.dragCoeffA * speedMs_ * speedMs_;
        double rollingForce = (speedMs_ > 0.001) ? config_.rollingResistanceN : 0.0;
        double brakeForce = brake * config_.brakingDecelerationMs2 * config_.vehicleMassKg;

        double netForce = engineForce - dragForce - rollingForce - brakeForce;
        accelerationMs2_ = netForce / config_.vehicleMassKg;

        speedMs_ += accelerationMs2_ * dt;
        if (speedMs_ < 0.0) {
            speedMs_ = 0.0;
            accelerationMs2_ = 0.0;
        }

        return speedMs_;
    }

    double getSpeedKmh() const { return speedMs_ * 3.6; }
    double getAccelerationG() const { return accelerationMs2_ / 9.81; }

    void reset() {
        speedMs_ = 0.0;
        accelerationMs2_ = 0.0;
    }

private:
    DemoVehiclePhysicsConfig config_;
    double speedMs_ = 0.0;
    double accelerationMs2_ = 0.0;
};

}

#endif
