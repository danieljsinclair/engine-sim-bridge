#include "twin/GearboxCsvLogger.h"
#include <cstdio>
#include <cstring>

namespace twin {

GearboxCsvLogger::GearboxCsvLogger(const std::string& filePath) {
    file_ = std::fopen(filePath.c_str(), "w");
}

GearboxCsvLogger::~GearboxCsvLogger() {
    if (file_) {
        std::fclose(file_);
    }
}

void GearboxCsvLogger::log(const GearboxLogEntry& e) {
    if (!file_) return;

    if (!wroteHeader_) {
        std::fprintf(file_,
            "frame,dt,speedKmh,throttleRaw,throttleSmoothed,vehicleSpeedFeedbackKmh,"
            "engineRpmFeedback,currentGear,targetGear,requestsShift,lastShiftDirection,"
            "timeSinceLastShiftS,kickdownActive,throttleDeltaHistory,engineRpm,"
            "upshiftSpeed,downshiftSpeed,twinState,clutchPressure\n");
        wroteHeader_ = true;
    }

    std::fprintf(file_,
        "%llu,%.6f,%.3f,%.4f,%.4f,%.3f,"
        "%.1f,%d,%d,%d,%d,"
        "%.4f,%d,%.4f,%.1f,"
        "%.3f,%.3f,%d,%.4f\n",
        (unsigned long long)e.frame, e.dt, e.speedKmh, e.throttleRaw, e.throttleSmoothed, e.vehicleSpeedFeedbackKmh,
        e.engineRpmFeedback, e.currentGear, e.targetGear, e.requestsShift ? 1 : 0, e.lastShiftDirection,
        e.timeSinceLastShiftS, e.kickdownActive ? 1 : 0, e.throttleDeltaHistory, e.engineRpm,
        e.upshiftSpeed, e.downshiftSpeed, e.twinState, e.clutchPressure);
}

}
