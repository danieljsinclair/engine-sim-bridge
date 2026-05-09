#ifndef UPSTREAM_SIGNAL_H
#define UPSTREAM_SIGNAL_H

#include <cstdint>

namespace input {

struct UpstreamSignal {
    double throttleFraction = 0.0;
    double speedKmh = 0.0;
    double accelerationG = 0.0;
    double brakeFraction = 0.0;
    uint64_t timestampUtcMs = 0;
    bool isValid = false;
};

}

#endif
