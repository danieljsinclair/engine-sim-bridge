#ifndef TWIN_OUTPUT_H
#define TWIN_OUTPUT_H

#include <simulator/GearConventions.h>

namespace twin {

struct TwinOutput {
    double throttle = 0.0;
    int gear = 0;
    double clutchPressure = 1.0;
    bool starterMotor = false;
    bool ignition = false;
    bridge::GearSelector gearSelector = bridge::GearSelector::NEUTRAL;
};

}

#endif
