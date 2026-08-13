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
    double kickdownDownshiftGears = 2.0;   // 1–2 gears on kickdown (findSafeGear bounds by redline)
    double shiftDisengageMs = 50.0;
    double shiftPauseMs = 60.0;
    double shiftReengageMs = 60.0;
    double throttleSmoothingTauMs = 50.0;
    double minShiftIntervalS = 3.0;
    double upshiftMinIntervalS = 0.0;      // 0 = use minShiftIntervalS
    double downshiftMinIntervalS = 0.0;    // 0 = use minShiftIntervalS
    // Declarative gearbox: short lockout after ANY shift to suppress sub-frame
    // oscillation. Anti-hunting is provided by the separate downshift table
    // (hysteresis by construction), so this need only cover a couple of frames.
    double shiftDwellS = 0.4;
    // Engine braking inhibitor
    bool engineBrakingInhibitorEnabled = false;
    double engineBrakingMaxThrottle = 0.01;
    double engineBrakingMinSpeedKmh = 50.0;
    // Tip-in/tip-out correction
    bool tipCorrectionEnabled = false;
    double tipInGradientThreshold = 10.0;    // %/s positive gradient blocks upshifts
    double tipOutGradientThreshold = -10.0;  // %/s negative gradient blocks upshifts
    double tipInhibitWindowS = 0.4;          // transient upshift-inhibit after a tip event
    double redlineRpm = 6500.0;
    double idleRpm = 950.0;   // C63 M156 V3 emergent idle (measured ~930-990, NOT the stale .mr "400-600")
    double throttleIdleThreshold = 0.05;     // IDLE → RUNNING transition (5%)
    double idleThrottle = 0.0;               // No throttle injection — engine idles on physics alone
    double standstillThresholdKmh = 1.0;     // Below this speed = standstill
    // Creep-drag relief ceiling (vehicle-speed gate). Below this road speed a
    // coupling that pins the wheels to the road (PIN) opens the clutch so the
    // engine idles decoupled instead of being lugged by 1st-gear road-implied
    // RPM (the creep-lug bug). Covers the FULL creep regime — raised from the
    // old standstill-only 1.0 km/h, which left 1-3 mph lugged. reconfigureProfile
    // sets this to the road speed at which 1st-gear road-implied RPM reaches
    // idle (so it is always coherent with the actual ratios); the default keeps
    // the prior near-standstill behaviour for non-reconfigured profiles.
    double creepReliefThresholdKmh = 5.0;

    // ---- Declarative shift-decision thresholds (no magic constants in logic) ----
    // Lug guard: the box downshifts whenever the ACTUAL engine RPM in the current
    // gear drops below max(idleRpm + lugFloorMarginRpm, downshiftRpmFloor). The
    // idle+margin term is the always-on hard floor (keeps the engine above idle
    // under load); downshiftRpmFloor is an optional higher override (tests/profiling).
    // 0 = no override → floor is just idle + margin.
    double downshiftRpmFloor = 0.0;          // optional override floor (0 = idle+margin only)
    double lugFloorMarginRpm = 150.0;        // keep engine >= idle + this margin
    // RPM at/below which the engine is considered STOPPED (or feedback unwired).
    // A stopped engine (0 rpm) must NOT be treated as lugging — that pinned the
    // box in 1st every frame. The stall/restart path owns the stopped case.
    double engineStoppedRpm = 30.0;
    // Redline safety nets (fractions of redlineRpm) — declarative, not hardcoded.
    double redlineUpshiftFraction = 0.95;    // upshift when speed-implied RPM > 0.95×redline
    double redlineKickdownFraction = 0.90;   // kickdown never drops into a gear > 0.90×redline
    // Torque-hint bounds (consumed by SimTorqueHint). The bias is a signed
    // throttle-equivalent nudge capped to ±torqueHintMaxBias, saturated at
    // torqueHintMaxNm. Zero torque → zero bias (NullTorqueHint).
    double torqueHintMaxNm = 3000.0;
    double torqueHintMaxBias = 0.15;

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
        // Calibrated to maintain ~85% redline (~5500 RPM) at WOT
        p.shiftTable = {
            {11, 17, 26, 32, 42, 54, 64},    // 5%
            {15, 22, 33, 41, 54, 69, 82},    // 15%
            {19, 28, 42, 53, 69, 88, 105},   // 25%
            {23, 35, 52, 65, 85, 110, 130},  // 40%
            {29, 43, 64, 81, 105, 136, 161}, // 55%
            {35, 52, 77, 97, 126, 163, 194},// 70%
            {40, 59, 88, 110, 143, 185, 220},// 80%
            {44, 65, 97, 122, 159, 205, 244},// 90%
            {46, 68, 102, 128, 167, 215, 256},// 95%
            {48, 71, 106, 134, 174, 225, 268} // 100% (~85% redline)
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
        p.shiftPauseMs = 60.0;
        p.shiftReengageMs = 60.0;
        p.throttleSmoothingTauMs = 50.0;
        p.minShiftIntervalS = 3.0;

        // Asymmetric shift intervals (ZF 8HP45 per x-engineer ch6 s4.2)
        p.upshiftMinIntervalS = 2.0;
        p.downshiftMinIntervalS = 1.0;

        // Declarative minimal dwell (sub-frame oscillation guard only).
        p.shiftDwellS = 0.4;

        // Engine braking inhibitor
        p.engineBrakingInhibitorEnabled = true;
        p.engineBrakingMaxThrottle = 0.01;
        p.engineBrakingMinSpeedKmh = 10.0;

        // Tip-in/tip-out correction: transient inhibit on the SMOOTHED throttle
        // gradient. Threshold sits above gentle ramps (~20 %/s) and the residual
        // of sub-percent CAN jitter after smoothing, but below a genuine pedal
        // stab (~hundreds of %/s); the inhibit decays over tipInhibitWindowS.
        p.tipCorrectionEnabled = true;
        p.tipInGradientThreshold = 50.0;    // %/s
        p.tipOutGradientThreshold = -50.0;  // %/s
        p.tipInhibitWindowS = 0.4;          // s

        // Engine parameters
        p.redlineRpm = 6500.0;
        p.idleRpm = 950.0;
        p.throttleIdleThreshold = 0.05;
        p.idleThrottle = 0.0;
        p.standstillThresholdKmh = 1.0;

        return p;
    }
};

}

#endif
