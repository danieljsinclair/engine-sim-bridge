#ifndef TWIN_TORQUE_CONVERTER_PHYSICS_H
#define TWIN_TORQUE_CONVERTER_PHYSICS_H

namespace twin {

struct TorqueConverterInput {
    double engineRpm;
    double turbineRpm;
    double stallTorqueRatio;
    double capacityFactor;
    double maxInputTorque;
};

struct TorqueConverterOutput {
    double torqueRatio;
    double speedRatio;
    double slip;
    double maxInputTorque;
    double turbineTorque;
};

TorqueConverterOutput computeTorqueConverter(const TorqueConverterInput& input);

} // namespace twin

#endif
