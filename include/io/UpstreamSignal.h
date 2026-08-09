#ifndef UPSTREAM_SIGNAL_H
#define UPSTREAM_SIGNAL_H

#include <cstdint>

namespace input {

struct UpstreamSignal {
    double throttleFraction = 0.0;
    double speedKmh = 0.0;
    double accelerationG = 0.0;
    double brakeFraction = 0.0;
    double motorTorqueNm = 0.0;       // Recorded motor/engine torque (Nm) for MATCH mode
    uint64_t timestampUtcMs = 0;
    bool isValid = false;
};

}

#endif
