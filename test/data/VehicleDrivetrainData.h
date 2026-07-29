// Auto-generated drivetrain data for all engine presets.
// RPM formula: engineRPM = (speedKmh / 3.6) / (2 * PI * tireRadius) * 60 * gearRatio * diffRatio

#pragma once

#include <cstddef>

namespace drivetrain {

struct GearRange {
    int gear;
    double ratio;
    double minSpeedKmh;   // at idle RPM (700)
    double maxSpeedKmh;   // at redline RPM
    double optimalLowKmh; // at 2000 RPM
    double optimalHighKmh;// at 6000 RPM
};

struct VehicleDrivetrain {
    const char* name;
    const char* sourceFile;
    int gearCount;
    const double* gearRatios;
    double diffRatio;
    double tireRadiusM;
    double vehicleMassKg;
    double dragCoefficient;
    double crossSectionAreaM2;
    double redlineRpm;
    int rangeCount;
    const GearRange* ranges;
};

// Subaru EJ25 (06) (06_subaru_ej25.json)
static constexpr double kGearRatios_06_subaru_ej25_json[] = {3.636, 2.375, 1.761, 1.346, 0.971, 0.756};

static constexpr GearRange kRanges_06_subaru_ej25_json[] = {
    {1, 3.636, 4.7, 43.9, 13.5, 40.5},
    {2, 2.375, 7.2, 67.2, 20.7, 62.0},
    {3, 1.761, 9.8, 90.6, 27.9, 83.7},
    {4, 1.346, 12.8, 118.6, 36.5, 109.4},
    {5, 0.971, 17.7, 164.4, 50.6, 151.7},
    {6, 0.756, 22.7, 211.1, 65.0, 194.9}
};

static constexpr VehicleDrivetrain kVehicle_06_subaru_ej25_json = {
    "Subaru EJ25 (06)",
    "06_subaru_ej25.json",
    6,
    kGearRatios_06_subaru_ej25_json,
    3.9,
    0.254,
    1224.7,
    0.2,
    2.384511,
    6500,
    6,
    kRanges_06_subaru_ej25_json
};

// Merlin V-1650-9 V12 (11_merlin_v12.json)
static constexpr double kGearRatios_11_merlin_v12_json[] = {1.3, 1.3, 1.14, 0.98, 0.81, 0.65};

static constexpr GearRange kRanges_11_merlin_v12_json[] = {
    {1, 1.3, 22.4, 96.1, 64.1, 192.2},
    {2, 1.3, 22.4, 96.1, 64.1, 192.2},
    {3, 1.14, 25.6, 109.6, 73.0, 219.1},
    {4, 0.98, 29.7, 127.4, 85.0, 254.9},
    {5, 0.81, 36.0, 154.2, 102.8, 308.4},
    {6, 0.65, 44.8, 192.2, 128.1, 384.3}
};

static constexpr VehicleDrivetrain kVehicle_11_merlin_v12_json = {
    "Merlin V-1650-9 V12",
    "11_merlin_v12.json",
    6,
    kGearRatios_11_merlin_v12_json,
    2.3,
    0.254,
    1224.7,
    0.3,
    2.601285,
    3000,
    6,
    kRanges_11_merlin_v12_json
};

// Toyota 2JZ I6 (2jz.json)
static constexpr double kGearRatios_2jz_json[] = {5.25, 3.36, 2.17, 1.72, 1.32, 1};

static constexpr GearRange kRanges_2jz_json[] = {
    {1, 5.25, 4.1, 34.7, 11.6, 34.7},
    {2, 3.36, 6.3, 54.3, 18.1, 54.3},
    {3, 2.17, 9.8, 84.1, 28.0, 84.1},
    {4, 1.72, 12.4, 106.0, 35.3, 106.0},
    {5, 1.32, 16.1, 138.2, 46.1, 138.2},
    {6, 1, 21.3, 182.4, 60.8, 182.4}
};

static constexpr VehicleDrivetrain kVehicle_2jz_json = {
    "Toyota 2JZ I6",
    "2jz.json",
    6,
    kGearRatios_2jz_json,
    3.15,
    0.254,
    1542.2,
    0.4,
    2.129028,
    6000,
    6,
    kRanges_2jz_json
};

// Mercedes-AMG C63 M156 (C63_M156_V2.json)
static constexpr double kGearRatios_C63_M156_V2_json[] = {4.34, 2.89, 1.92, 1.37, 1, 0.82, 0.73};

static constexpr GearRange kRanges_C63_M156_V2_json[] = {
    {1, 4.34, 7.7, 79.7, 22.0, 66.0},
    {2, 2.89, 11.6, 119.7, 33.0, 99.0},
    {3, 1.92, 17.4, 180.1, 49.7, 149.1},
    {4, 1.37, 24.4, 252.5, 69.6, 208.9},
    {5, 1, 33.4, 345.9, 95.4, 286.2},
    {6, 0.82, 40.7, 421.8, 116.4, 349.1},
    {7, 0.73, 45.7, 473.8, 130.7, 392.1}
};

static constexpr VehicleDrivetrain kVehicle_C63_M156_V2_json = {
    "Mercedes-AMG C63 M156",
    "C63_M156_V2.json",
    7,
    kGearRatios_C63_M156_V2_json,
    2.82,
    0.35687,
    1730.0,
    0.34,
    2.738340,
    7250,
    7,
    kRanges_C63_M156_V2_json
};

// Ferrari F136 (ferrari_f136.json)
static constexpr double kGearRatios_ferrari_f136_json[] = {3.23, 2.19, 1.61, 1.23, 0.97, 0.8};

static constexpr GearRange kRanges_ferrari_f136_json[] = {
    {1, 3.23, 6.1, 78.0, 17.3, 52.0},
    {2, 2.19, 8.9, 115.1, 25.6, 76.7},
    {3, 1.61, 12.2, 156.5, 34.8, 104.3},
    {4, 1.23, 15.9, 204.9, 45.5, 136.6},
    {5, 0.97, 20.2, 259.8, 57.7, 173.2},
    {6, 0.8, 24.5, 315.0, 70.0, 210.0}
};

static constexpr VehicleDrivetrain kVehicle_ferrari_f136_json = {
    "Ferrari F136",
    "ferrari_f136.json",
    6,
    kGearRatios_ferrari_f136_json,
    3.42,
    0.254,
    1614.0,
    0.3,
    2.322576,
    9000,
    6,
    kRanges_ferrari_f136_json
};

// Lexus LFA 1LR-GUE V10 (lfa_v10.json)
static constexpr double kGearRatios_lfa_v10_json[] = {3.23, 2.19, 1.61, 1.23, 0.97, 0.8};

static constexpr GearRange kRanges_lfa_v10_json[] = {
    {1, 3.23, 6.1, 78.0, 17.3, 52.0},
    {2, 2.19, 8.9, 115.1, 25.6, 76.7},
    {3, 1.61, 12.2, 156.5, 34.8, 104.3},
    {4, 1.23, 15.9, 204.9, 45.5, 136.6},
    {5, 0.97, 20.2, 259.8, 57.7, 173.2},
    {6, 0.8, 24.5, 315.0, 70.0, 210.0}
};

static constexpr VehicleDrivetrain kVehicle_lfa_v10_json = {
    "Lexus LFA 1LR-GUE V10",
    "lfa_v10.json",
    6,
    kGearRatios_lfa_v10_json,
    3.42,
    0.254,
    1614.0,
    0.3,
    2.322576,
    9000,
    6,
    kRanges_lfa_v10_json
};

// Subaru EJ25 (subaru_ej25.json)
static constexpr double kGearRatios_subaru_ej25_json[] = {3.636, 2.375, 1.761, 1.346, 0.971, 0.756};

static constexpr GearRange kRanges_subaru_ej25_json[] = {
    {1, 3.636, 4.7, 43.9, 13.5, 40.5},
    {2, 2.375, 7.2, 67.2, 20.7, 62.0},
    {3, 1.761, 9.8, 90.6, 27.9, 83.7},
    {4, 1.346, 12.8, 118.6, 36.5, 109.4},
    {5, 0.971, 17.7, 164.4, 50.6, 151.7},
    {6, 0.756, 22.7, 211.1, 65.0, 194.9}
};

static constexpr VehicleDrivetrain kVehicle_subaru_ej25_json = {
    "Subaru EJ25",
    "subaru_ej25.json",
    6,
    kGearRatios_subaru_ej25_json,
    3.9,
    0.254,
    1224.7,
    0.3,
    2.601285,
    6500,
    6,
    kRanges_subaru_ej25_json
};

// GM LS V8 (v8_gm_ls.json)
static constexpr double kGearRatios_v8_gm_ls_json[] = {2.97, 2.07, 1.43, 1, 0.71, 0.57};

static constexpr GearRange kRanges_v8_gm_ls_json[] = {
    {1, 2.97, 6.6, 61.3, 18.9, 56.6},
    {2, 2.07, 9.5, 87.9, 27.1, 81.2},
    {3, 1.43, 13.7, 127.3, 39.2, 117.5},
    {4, 1, 19.6, 182.0, 56.0, 168.0},
    {5, 0.71, 27.6, 256.3, 78.9, 236.6},
    {6, 0.57, 34.4, 319.3, 98.2, 294.7}
};

static constexpr VehicleDrivetrain kVehicle_v8_gm_ls_json = {
    "GM LS V8",
    "v8_gm_ls.json",
    6,
    kGearRatios_v8_gm_ls_json,
    3.42,
    0.254,
    1614.0,
    0.3,
    2.322576,
    6500,
    6,
    kRanges_v8_gm_ls_json
};

static constexpr size_t kVehicleCount = 8;

static constexpr const VehicleDrivetrain* kAllVehicles[] = {
    &kVehicle_06_subaru_ej25_json,
    &kVehicle_11_merlin_v12_json,
    &kVehicle_2jz_json,
    &kVehicle_C63_M156_V2_json,
    &kVehicle_ferrari_f136_json,
    &kVehicle_lfa_v10_json,
    &kVehicle_subaru_ej25_json,
    &kVehicle_v8_gm_ls_json,
};

inline double rpmAtSpeed(double speedKmh, double gearRatio, double diffRatio, double tireRadiusM) {
    return (speedKmh / 3.6) / (2.0 * 3.14159265358979323846 * tireRadiusM) * 60.0 * gearRatio * diffRatio;
}

inline double speedAtRpm(double rpm, double gearRatio, double diffRatio, double tireRadiusM) {
    return rpm / (60.0 * gearRatio * diffRatio) * (2.0 * 3.14159265358979323846 * tireRadiusM) * 3.6;
}

} // namespace drivetrain
