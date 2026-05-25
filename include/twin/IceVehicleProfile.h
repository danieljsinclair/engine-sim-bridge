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
    std::vector<double> shiftTableThrottleLevels;
    bool separateDownshiftTableEnabled = false;
    std::vector<std::vector<double>> downshiftTable;
    std::vector<double> downshiftTableThrottleLevels;
    double hysteresisFactor = 0.85;
    double kickdownThrottleThreshold = 0.95;
    double kickdownDelta = 0.4;
    double kickdownWindowMs = 100.0;
    double shiftDisengageMs = 50.0;
    double shiftPauseMs = 200.0;
    double shiftReengageMs = 100.0;
    double throttleSmoothingTauMs = 50.0;
    double minShiftIntervalS = 3.0;
    double upshiftMinIntervalS = 0.0;      // 0 = use minShiftIntervalS
    double downshiftMinIntervalS = 0.0;    // 0 = use minShiftIntervalS
    // Engine braking inhibitor
    bool engineBrakingInhibitorEnabled = false;
    double engineBrakingMaxThrottle = 0.01;
    double engineBrakingMinSpeedKmh = 50.0;
    // Tip-in/tip-out correction
    bool tipCorrectionEnabled = false;
    double tipInGradientThreshold = 10.0;    // %/s positive gradient blocks upshifts
    double tipOutGradientThreshold = -10.0;  // %/s negative gradient blocks upshifts
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
        IceVehicleProfile p;
        p.gearRatios = {4.714, 3.143, 2.106, 1.667, 1.285, 1.000, 0.839, 0.667};
        p.diffRatio = 3.15;
        p.tireRadiusM = 0.32;
        p.vehicleMassKg = 1800.0;
        p.hysteresisFactor = 0.85;

        // 10 throttle breakpoints
        p.shiftTableThrottleLevels = {0.05, 0.15, 0.25, 0.40, 0.55, 0.70, 0.80, 0.90, 0.95, 1.00};

        // Upshift table (10 throttle levels x 7 shift columns)
        p.shiftTable = {
            {11, 17, 26, 32, 42, 54, 64},    // 5%
            {15, 22, 33, 41, 54, 69, 82},    // 15%
            {19, 28, 42, 53, 69, 88, 105},   // 25%
            {24, 37, 55, 69, 90, 115, 137},  // 40%
            {30, 45, 67, 85, 110, 142, 169}, // 55%
            {36, 54, 80, 101, 131, 169, 201},// 70%
            {41, 61, 91, 115, 149, 192, 229},// 80%
            {45, 67, 100, 127, 164, 211, 251},// 90%
            {47, 70, 105, 132, 171, 220, 262},// 95%
            {49, 73, 109, 138, 179, 230, 274} // 100%
        };

        // Separate downshift table
        p.separateDownshiftTableEnabled = true;
        p.downshiftTableThrottleLevels = {0.05, 0.15, 0.25, 0.40, 0.55, 0.70, 0.80, 0.90, 0.95, 1.00};
        p.downshiftTable = {
            {9, 13, 20, 25, 33, 42, 50},    // 5%
            {11, 16, 24, 30, 39, 50, 59},   // 15%
            {14, 21, 31, 39, 51, 65, 78},   // 25%
            {18, 27, 40, 51, 66, 84, 100},  // 40%
            {22, 33, 49, 62, 81, 104, 123}, // 55%
            {26, 39, 58, 74, 95, 123, 146}, // 70%
            {29, 44, 66, 83, 107, 138, 165},// 80%
            {33, 49, 73, 92, 119, 153, 183},// 90%
            {34, 51, 76, 96, 125, 161, 192},// 95%
            {34, 51, 76, 97, 125, 161, 192} // 100%
        };

        // Kickdown
        p.kickdownThrottleThreshold = 0.95;
        p.kickdownDelta = 0.4;
        p.kickdownWindowMs = 100.0;

        // Shift timing
        p.shiftDisengageMs = 50.0;
        p.shiftPauseMs = 200.0;
        p.shiftReengageMs = 100.0;
        p.throttleSmoothingTauMs = 50.0;
        p.minShiftIntervalS = 3.0;

        // Asymmetric shift intervals (ZF 8HP45)
        p.upshiftMinIntervalS = 2.0;
        p.downshiftMinIntervalS = 1.5;

        // Engine braking inhibitor
        p.engineBrakingInhibitorEnabled = true;
        p.engineBrakingMaxThrottle = 0.01;
        p.engineBrakingMinSpeedKmh = 10.0;

        // Tip-in/tip-out correction
        p.tipCorrectionEnabled = true;
        p.tipInGradientThreshold = 10.0;    // %/s
        p.tipOutGradientThreshold = -10.0;  // %/s

        // Engine parameters
        p.redlineRpm = 6500.0;
        p.idleRpm = 750.0;
        p.throttleIdleThreshold = 0.05;
        p.idleThrottle = 0.0;
        p.standstillThresholdKmh = 1.0;

        return p;
    }
};

}

#endif
