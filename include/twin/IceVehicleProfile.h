#ifndef ICE_VEHICLE_PROFILE_H
#define ICE_VEHICLE_PROFILE_H

#include <vector>

namespace twin {

struct IceVehicleProfile {
    std::vector<double> gearRatios;
    double diffRatio = 3.15;
    double tireRadiusM = 0.32;
    double vehicleMassKg = 1800.0;
    std::vector<std::vector<double>> shiftTable;
    double hysteresisFactor = 0.85;
    double kickdownThrottleThreshold = 0.95;
    double kickdownDelta = 0.4;
    double kickdownWindowMs = 100.0;
    double shiftDisengageMs = 50.0;
    double shiftPauseMs = 200.0;
    double shiftReengageMs = 100.0;
    double throttleSmoothingTauMs = 50.0;
    double minShiftIntervalS = 3.0;
    double redlineRpm = 6500.0;
    double idleRpm = 750.0;
    double throttleIdleThreshold = 0.05;     // IDLE → RUNNING transition (5%)
    double idleThrottle = 0.0;               // No throttle injection — engine idles on physics alone
    double standstillThresholdKmh = 1.0;     // Below this speed = standstill

    IceVehicleProfile() = default;

    IceVehicleProfile(
        const std::vector<double>& gearRatios,
        double diffRatio,
        double tireRadiusM,
        double vehicleMassKg,
        const std::vector<std::vector<double>>& shiftTable,
        double hysteresisFactor,
        double kickdownThrottleThreshold,
        double kickdownDelta,
        double kickdownWindowMs,
        double shiftDisengageMs,
        double shiftPauseMs,
        double shiftReengageMs,
        double throttleSmoothingTauMs,
        double minShiftIntervalS,
        double redlineRpm,
        double idleRpm
    ) : gearRatios(gearRatios),
        diffRatio(diffRatio),
        tireRadiusM(tireRadiusM),
        vehicleMassKg(vehicleMassKg),
        shiftTable(shiftTable),
        hysteresisFactor(hysteresisFactor),
        kickdownThrottleThreshold(kickdownThrottleThreshold),
        kickdownDelta(kickdownDelta),
        kickdownWindowMs(kickdownWindowMs),
        shiftDisengageMs(shiftDisengageMs),
        shiftPauseMs(shiftPauseMs),
        shiftReengageMs(shiftReengageMs),
        throttleSmoothingTauMs(throttleSmoothingTauMs),
        minShiftIntervalS(minShiftIntervalS),
        redlineRpm(redlineRpm),
        idleRpm(idleRpm) {}

    static IceVehicleProfile zf8hp45() {
        return IceVehicleProfile{
            {4.714, 3.143, 2.106, 1.667, 1.285, 1.000, 0.839, 0.667},
            3.15,
            0.32,
            1800.0,
            {
                {20.0, 35.0, 50.0, 65.0, 80.0, 95.0, 110.0},
                {30.0, 50.0, 70.0, 90.0, 110.0, 130.0, 155.0},
                {40.0, 65.0, 90.0, 115.0, 140.0, 170.0, 200.0},
                {55.0, 85.0, 115.0, 145.0, 180.0, 215.0, 255.0},
                {70.0, 105.0, 140.0, 180.0, 220.0, 265.0, 315.0}
            },
            0.85,
            0.95,
            0.4,
            100.0,
            50.0,
            200.0,
            100.0,
            50.0,
            3.0,
            6500.0,
            750.0
        };
    }
};

}

#endif
