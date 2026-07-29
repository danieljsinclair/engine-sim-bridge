#pragma once

// Speed-to-RPM lookup tables for unit testing the automatic gearbox.
// Formula: engineRPM = (speedKmh / 3.6) / (2 * PI * tireRadius) * 60 * gearRatio * diffRatio
// VALID if RPM == 0 (standstill) or idle <= RPM <= redline.
// recommendedGear picks the gear where RPM is closest to the midrange (idle+redline)/2.

#include <cstddef>
#include <array>

namespace test_data {

struct SpeedRpmEntry {
    double speedKmh;
    int gear;
    double expectedRpm;
    bool isValid;
};

struct GearLimit {
    int gear;
    double minSpeedKmh;
    double maxSpeedKmh;
};

struct RecommendedGearEntry {
    double speedKmh;
    int recommendedGear; // 0 = none valid
};

// ============================================================
// AMG C63 M156 V2 (7-speed AMG Speedshift MCT)
// ============================================================
namespace c63 {

constexpr double kGearRatios[] = {4.34, 2.89, 1.92, 1.37, 1.00, 0.82, 0.73};
constexpr double kDiffRatio = 2.82;
constexpr double kTireRadius = 0.35687;
constexpr double kRedline = 7200.0;
constexpr double kIdle = 700.0;
constexpr int kGearCount = 7;

// Speed points used in the table
constexpr double kSpeedPoints[] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 120, 150, 200};
constexpr int kSpeedPointCount = 14;

// Full speed x gear RPM table (14 speeds x 7 gears = 98 entries)
constexpr SpeedRpmEntry kSpeedRpmTable[] = {
    // 0 km/h - standstill, all gears valid at RPM 0
    {0, 1,     0.0, true},
    {0, 2,     0.0, true},
    {0, 3,     0.0, true},
    {0, 4,     0.0, true},
    {0, 5,     0.0, true},
    {0, 6,     0.0, true},
    {0, 7,     0.0, true},
    // 10 km/h
    {10, 1,   909.7, true},
    {10, 2,   605.8, false},
    {10, 3,   402.4, false},
    {10, 4,   287.2, false},
    {10, 5,   209.6, false},
    {10, 6,   171.9, false},
    {10, 7,   153.0, false},
    // 20 km/h
    {20, 1,  1819.4, true},
    {20, 2,  1211.5, true},
    {20, 3,   804.9, true},
    {20, 4,   574.3, false},
    {20, 5,   419.2, false},
    {20, 6,   343.8, false},
    {20, 7,   306.0, false},
    // 30 km/h
    {30, 1,  2729.1, true},
    {30, 2,  1817.3, true},
    {30, 3,  1207.3, true},
    {30, 4,   861.5, true},
    {30, 5,   628.8, false},
    {30, 6,   515.6, false},
    {30, 7,   459.0, false},
    // 40 km/h
    {40, 1,  3638.8, true},
    {40, 2,  2423.1, true},
    {40, 3,  1609.8, true},
    {40, 4,  1148.7, true},
    {40, 5,   838.4, true},
    {40, 6,   687.5, false},
    {40, 7,   612.1, false},
    // 50 km/h
    {50, 1,  4548.5, true},
    {50, 2,  3028.8, true},
    {50, 3,  2012.2, true},
    {50, 4,  1435.8, true},
    {50, 5,  1048.0, true},
    {50, 6,   859.4, true},
    {50, 7,   765.1, true},
    // 60 km/h
    {60, 1,  5458.2, true},
    {60, 2,  3634.6, true},
    {60, 3,  2414.7, true},
    {60, 4,  1723.0, true},
    {60, 5,  1257.6, true},
    {60, 6,  1031.3, true},
    {60, 7,   918.1, true},
    // 70 km/h
    {70, 1,  6367.9, true},
    {70, 2,  4240.4, true},
    {70, 3,  2817.1, true},
    {70, 4,  2010.1, true},
    {70, 5,  1467.3, true},
    {70, 6,  1203.2, true},
    {70, 7,  1071.1, true},
    // 80 km/h
    {80, 1,  7277.6, false},
    {80, 2,  4846.1, true},
    {80, 3,  3219.6, true},
    {80, 4,  2297.3, true},
    {80, 5,  1676.9, true},
    {80, 6,  1375.0, true},
    {80, 7,  1224.1, true},
    // 90 km/h
    {90, 1,  8187.3, false},
    {90, 2,  5451.9, true},
    {90, 3,  3622.0, true},
    {90, 4,  2584.5, true},
    {90, 5,  1886.5, true},
    {90, 6,  1546.9, true},
    {90, 7,  1377.1, true},
    // 100 km/h
    {100, 1,  9097.0, false},
    {100, 2,  6057.7, true},
    {100, 3,  4024.5, true},
    {100, 4,  2871.6, true},
    {100, 5,  2096.1, true},
    {100, 6,  1718.8, true},
    {100, 7,  1530.1, true},
    // 120 km/h
    {120, 1, 10916.4, false},
    {120, 2,  7269.2, false},
    {120, 3,  4829.4, true},
    {120, 4,  3446.0, true},
    {120, 5,  2515.3, true},
    {120, 6,  2062.5, true},
    {120, 7,  1836.2, true},
    // 150 km/h
    {150, 1, 13645.5, false},
    {150, 2,  9086.5, false},
    {150, 3,  6036.7, true},
    {150, 4,  4307.4, true},
    {150, 5,  3144.1, true},
    {150, 6,  2578.2, true},
    {150, 7,  2295.2, true},
    // 200 km/h
    {200, 1, 18194.0, false},
    {200, 2, 12115.3, false},
    {200, 3,  8048.9, false},
    {200, 4,  5743.3, true},
    {200, 5,  4192.2, true},
    {200, 6,  3437.6, true},
    {200, 7,  3060.3, true},
};

constexpr int kSpeedRpmTableSize = 98;

// Gear limits: minimum speed (at idle) and maximum speed (at redline)
constexpr GearLimit kGearLimits[] = {
    {1,   7.7,   79.1},
    {2,  11.6,  118.9},
    {3,  17.4,  178.9},
    {4,  24.4,  250.7},
    {5,  33.4,  343.5},
    {6,  40.7,  418.9},
    {7,  45.7,  470.5},
};

constexpr int kGearLimitsSize = 7;

// Recommended gear at each speed (gear closest to midrange RPM)
constexpr RecommendedGearEntry kRecommendedGear[] = {
    {  0, 0},  // standstill, no valid gear
    { 10, 1},
    { 20, 1},
    { 30, 1},
    { 40, 1},
    { 50, 1},
    { 60, 2},
    { 70, 2},
    { 80, 3},
    { 90, 3},
    {100, 3},
    {120, 4},
    {150, 4},
    {200, 5},
};

constexpr int kRecommendedGearSize = 14;

// Key overlap zones where multiple gears are valid (adjacent gears only)
// These are the TCU shift decision zones.
constexpr struct { int gearLow; int gearHigh; double overlapStart; double overlapEnd; } kOverlapZones[] = {
    {1, 2,  11.6,   79.1},
    {2, 3,  17.4,  118.9},
    {3, 4,  24.4,  178.9},
    {4, 5,  33.4,  250.7},
    {5, 6,  40.7,  343.5},
    {6, 7,  45.7,  418.9},
};

constexpr int kOverlapZonesSize = 6;

} // namespace c63

// ============================================================
// Ferrari F136 V8 (6-speed, e.g. F430/458)
// ============================================================
namespace ferrari_f136 {

constexpr double kGearRatios[] = {3.23, 2.19, 1.61, 1.23, 0.97, 0.80};
constexpr double kDiffRatio = 3.42;
constexpr double kTireRadius = 0.254;
constexpr double kRedline = 8200.0;
constexpr double kIdle = 750.0;
constexpr int kGearCount = 6;

constexpr SpeedRpmEntry kSpeedRpmTable[] = {
    // 0 km/h
    {0, 1,     0.0, true},
    {0, 2,     0.0, true},
    {0, 3,     0.0, true},
    {0, 4,     0.0, true},
    {0, 5,     0.0, true},
    {0, 6,     0.0, true},
    // 10 km/h
    {10, 1,  1153.6, true},
    {10, 2,   782.2, true},
    {10, 3,   575.0, false},
    {10, 4,   439.3, false},
    {10, 5,   346.4, false},
    {10, 6,   285.7, false},
    // 20 km/h
    {20, 1,  2307.2, true},
    {20, 2,  1564.4, true},
    {20, 3,  1150.1, true},
    {20, 4,   878.6, true},
    {20, 5,   692.9, false},
    {20, 6,   571.5, false},
    // 30 km/h
    {30, 1,  3460.9, true},
    {30, 2,  2346.5, true},
    {30, 3,  1725.1, true},
    {30, 4,  1317.9, true},
    {30, 5,  1039.3, true},
    {30, 6,   857.2, true},
    // 40 km/h
    {40, 1,  4614.5, true},
    {40, 2,  3128.7, true},
    {40, 3,  2300.1, true},
    {40, 4,  1757.2, true},
    {40, 5,  1385.8, true},
    {40, 6,  1142.9, true},
    // 50 km/h
    {50, 1,  5768.1, true},
    {50, 2,  3910.9, true},
    {50, 3,  2875.1, true},
    {50, 4,  2196.5, true},
    {50, 5,  1732.2, true},
    {50, 6,  1428.6, true},
    // 60 km/h
    {60, 1,  6921.7, true},
    {60, 2,  4693.1, true},
    {60, 3,  3450.2, true},
    {60, 4,  2635.8, true},
    {60, 5,  2078.7, true},
    {60, 6,  1714.4, true},
    // 70 km/h
    {70, 1,  8075.4, true},
    {70, 2,  5475.2, true},
    {70, 3,  4025.2, true},
    {70, 4,  3075.1, true},
    {70, 5,  2425.1, true},
    {70, 6,  2000.1, true},
    // 80 km/h
    {80, 1,  9229.0, false},
    {80, 2,  6257.4, true},
    {80, 3,  4600.2, true},
    {80, 4,  3514.4, true},
    {80, 5,  2771.6, true},
    {80, 6,  2285.8, true},
    // 90 km/h
    {90, 1, 10382.6, false},
    {90, 2,  7039.6, true},
    {90, 3,  5175.2, true},
    {90, 4,  3953.7, true},
    {90, 5,  3118.0, true},
    {90, 6,  2571.5, true},
    // 100 km/h
    {100, 1, 11536.2, false},
    {100, 2,  7821.8, true},
    {100, 3,  5750.3, true},
    {100, 4,  4393.1, true},
    {100, 5,  3464.4, true},
    {100, 6,  2857.3, true},
    // 120 km/h
    {120, 1, 13843.5, false},
    {120, 2,  9386.1, false},
    {120, 3,  6900.3, true},
    {120, 4,  5271.7, true},
    {120, 5,  4157.3, true},
    {120, 6,  3428.7, true},
    // 150 km/h
    {150, 1, 17304.3, false},
    {150, 2, 11732.7, false},
    {150, 3,  8625.4, false},
    {150, 4,  6589.6, true},
    {150, 5,  5196.7, true},
    {150, 6,  4285.9, true},
    // 200 km/h
    {200, 1, 23072.5, false},
    {200, 2, 15643.6, false},
    {200, 3, 11500.5, false},
    {200, 4,  8786.1, false},
    {200, 5,  6928.9, true},
    {200, 6,  5714.5, true},
};

constexpr int kSpeedRpmTableSize = 84;

constexpr GearLimit kGearLimits[] = {
    {1,   6.5,   71.1},
    {2,   9.6,  104.8},
    {3,  13.0,  142.6},
    {4,  17.1,  186.7},
    {5,  21.6,  236.7},
    {6,  26.2,  287.0},
};

constexpr int kGearLimitsSize = 6;

constexpr RecommendedGearEntry kRecommendedGear[] = {
    {  0, 0},
    { 10, 1},
    { 20, 1},
    { 30, 1},
    { 40, 1},
    { 50, 2},
    { 60, 2},
    { 70, 3},
    { 80, 3},
    { 90, 4},
    {100, 4},
    {120, 5},
    {150, 6},
    {200, 6},
};

constexpr int kRecommendedGearSize = 14;

} // namespace ferrari_f136

// ============================================================
// GM LS V8 (6-speed, e.g. Corvette/Tremec T56)
// ============================================================
namespace gm_ls {

constexpr double kGearRatios[] = {2.97, 2.07, 1.43, 1.00, 0.71, 0.57};
constexpr double kDiffRatio = 3.42;
constexpr double kTireRadius = 0.254;
constexpr double kRedline = 6600.0;
constexpr double kIdle = 600.0;
constexpr int kGearCount = 6;

constexpr SpeedRpmEntry kSpeedRpmTable[] = {
    // 0 km/h
    {0, 1,     0.0, true},
    {0, 2,     0.0, true},
    {0, 3,     0.0, true},
    {0, 4,     0.0, true},
    {0, 5,     0.0, true},
    {0, 6,     0.0, true},
    // 10 km/h
    {10, 1,  1060.8, true},
    {10, 2,   739.3, true},
    {10, 3,   510.7, false},
    {10, 4,   357.2, false},
    {10, 5,   253.6, false},
    {10, 6,   203.6, false},
    // 20 km/h
    {20, 1,  2121.5, true},
    {20, 2,  1478.6, true},
    {20, 3,  1021.5, true},
    {20, 4,   714.3, true},
    {20, 5,   507.2, false},
    {20, 6,   407.2, false},
    // 30 km/h
    {30, 1,  3182.3, true},
    {30, 2,  2218.0, true},
    {30, 3,  1532.2, true},
    {30, 4,  1071.5, true},
    {30, 5,   760.7, true},
    {30, 6,   610.7, true},
    // 40 km/h
    {40, 1,  4243.0, true},
    {40, 2,  2957.3, true},
    {40, 3,  2042.9, true},
    {40, 4,  1428.6, true},
    {40, 5,  1014.3, true},
    {40, 6,   814.3, true},
    // 50 km/h
    {50, 1,  5303.8, true},
    {50, 2,  3696.6, true},
    {50, 3,  2553.7, true},
    {50, 4,  1785.8, true},
    {50, 5,  1267.9, true},
    {50, 6,  1017.9, true},
    // 60 km/h
    {60, 1,  6364.6, true},
    {60, 2,  4435.9, true},
    {60, 3,  3064.4, true},
    {60, 4,  2143.0, true},
    {60, 5,  1521.5, true},
    {60, 6,  1221.5, true},
    // 70 km/h
    {70, 1,  7425.3, false},
    {70, 2,  5175.2, true},
    {70, 3,  3575.2, true},
    {70, 4,  2500.1, true},
    {70, 5,  1775.1, true},
    {70, 6,  1425.1, true},
    // 80 km/h
    {80, 1,  8486.1, false},
    {80, 2,  5914.5, true},
    {80, 3,  4085.9, true},
    {80, 4,  2857.3, true},
    {80, 5,  2028.7, true},
    {80, 6,  1628.6, true},
    // 90 km/h
    {90, 1,  9546.9, false},
    {90, 2,  6653.9, false},
    {90, 3,  4596.6, true},
    {90, 4,  3214.4, true},
    {90, 5,  2282.2, true},
    {90, 6,  1832.2, true},
    // 100 km/h
    {100, 1, 10607.6, false},
    {100, 2,  7393.2, false},
    {100, 3,  5107.4, true},
    {100, 4,  3571.6, true},
    {100, 5,  2535.8, true},
    {100, 6,  2035.8, true},
    // 120 km/h
    {120, 1, 12729.1, false},
    {120, 2,  8871.8, false},
    {120, 3,  6128.8, true},
    {120, 4,  4285.9, true},
    {120, 5,  3043.0, true},
    {120, 6,  2443.0, true},
    // 150 km/h
    {150, 1, 15911.4, false},
    {150, 2, 11089.8, false},
    {150, 3,  7661.1, false},
    {150, 4,  5357.4, true},
    {150, 5,  3803.7, true},
    {150, 6,  3053.7, true},
    // 200 km/h
    {200, 1, 21215.2, false},
    {200, 2, 14786.4, false},
    {200, 3, 10214.7, false},
    {200, 4,  7143.2, false},
    {200, 5,  5071.7, true},
    {200, 6,  4071.6, true},
};

constexpr int kSpeedRpmTableSize = 84;

constexpr GearLimit kGearLimits[] = {
    {1,   5.7,   62.2},
    {2,   8.1,   89.3},
    {3,  11.7,  129.2},
    {4,  16.8,  184.8},
    {5,  23.7,  260.3},
    {6,  29.5,  324.2},
};

constexpr int kGearLimitsSize = 6;

constexpr RecommendedGearEntry kRecommendedGear[] = {
    {  0, 0},
    { 10, 1},
    { 20, 1},
    { 30, 1},
    { 40, 2},
    { 50, 2},
    { 60, 3},
    { 70, 3},
    { 80, 3},
    { 90, 4},
    {100, 4},
    {120, 5},
    {150, 5},
    {200, 6},
};

constexpr int kRecommendedGearSize = 14;

} // namespace gm_ls

} // namespace test_data
