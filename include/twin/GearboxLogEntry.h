#ifndef TWIN_GEARBOX_LOG_ENTRY_H
#define TWIN_GEARBOX_LOG_ENTRY_H

#include <cstdint>

namespace twin {

struct GearboxLogEntry {
    // Inputs
    double dt = 0.0;
    double speedKmh = 0.0;
    double throttleRaw = 0.0;
    double throttleSmoothed = 0.0;
    double vehicleSpeedFeedbackKmh = 0.0;
    double engineRpmFeedback = 0.0;

    // Internal state
    int currentGear = 0;
    int targetGear = 0;
    bool requestsShift = false;
    int lastShiftDirection = 0;
    double timeSinceLastShiftS = 0.0;
    bool kickdownActive = false;
    double throttleDeltaHistory = 0.0;

    // Computed
    double engineRpm = 0.0;
    double upshiftSpeed = 0.0;
    double downshiftSpeed = 0.0;

    // Twin state
    int twinState = 0;
    double clutchPressure = 0.0;

    // Meta
    uint64_t frame = 0;
};

}

#endif
