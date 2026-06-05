// GearboxDiagnosticSpike.cpp
//
// Standalone diagnostic that exercises the ZF 8HP45 automatic gearbox
// through a full-throttle acceleration run across all 8 gears.
//
// Logs every gearbox frame to CSV for post-run analysis of:
//   - Gear progression (1 through 8)
//   - Shift hunting (rapid up/down oscillation)
//   - Shift point speeds vs the ZF 8HP45 shift table
//   - Gear hold times (3-second minimum enforcement)
//   - Throttle smoothing behaviour
//   - Kickdown activation
//   - Speed source fidelity (demo physics vs feedback)

#include <gtest/gtest.h>

#include "input/DemoInputProvider.h"
#include "input/DemoThrottleSource.h"
#include "input/GearSelectorInput.h"
#include "input/IgnitionInput.h"
#include "twin/GearboxCsvLogger.h"
#include "twin/IceVehicleProfile.h"
#include "simulator/EngineSimTypes.h"
#include "simulator/GearConventions.h"

#include <cstdio>
#include <cmath>
#include <string>
#include <iostream>

namespace {

constexpr int TOTAL_FRAMES = 6000;
constexpr double DT = 0.016;  // ~62.5 Hz (close to 60 Hz update loop)
constexpr double CRANKING_RPM_RAMP_FRAMES = 60;  // ~1 second of cranking
constexpr double CRANKING_RAMP_START = 0.0;
constexpr double CRANKING_RAMP_END = 800.0;
constexpr double RUNNING_RPM_BASE = 800.0;

} // anonymous namespace

// ---------------------------------------------------------------------------
// TEST(GearboxDiagnostic, FullThrottleAcceleration)
//
// State machine walkthrough:
//   OFF -> CRANKING:   Automatic on first valid signal with ignition on
//   CRANKING -> IDLE:  engineRpmFeedback > 500 RPM (we ramp from 0 to 800 over ~1s)
//   IDLE -> RUNNING:   selector == DRIVE && throttle > 0.05
//
// Gear selector order (PRNDL): PARK(-2) -> REVERSE(-1) -> NEUTRAL(0) -> DRIVE(99)
// Starting at PARK (default GearSelectorInput constructor), we need 3x shiftUp()
// to reach DRIVE.
// ---------------------------------------------------------------------------
TEST(GearboxDiagnostic, FullThrottleAcceleration) {
    // --- 1. Build the provider stack ---
    auto rawThrottle = std::make_unique<input::DemoThrottleSource>();
    // Save a raw pointer before moving into the provider
    input::DemoThrottleSource* throttlePtr = rawThrottle.get();

    auto gearSelector = std::make_unique<input::GearSelectorInput>();
    auto ignition = std::make_unique<input::IgnitionInput>();

    twin::IceVehicleProfile profile = twin::IceVehicleProfile::zf8hp45();

    auto provider = std::make_unique<input::DemoInputProvider>(
        std::move(rawThrottle),
        std::move(gearSelector),
        std::move(ignition),
        profile
    );

    // --- 2. Attach CSV logger ---
    const std::string csvPath = "gearbox_accel_diagnostic.csv";
    twin::GearboxCsvLogger logger(csvPath);
    ASSERT_TRUE(logger.isOpen()) << "Failed to open CSV log file: " << csvPath;
    provider->setGearboxLogger(&logger);

    // --- 3. Initialize ---
    ASSERT_TRUE(provider->Initialize());
    ASSERT_TRUE(provider->IsConnected());

    // --- 4. Shift to DRIVE: PARK -> REVERSE -> NEUTRAL -> DRIVE (3x shiftUp) ---
    // Cast to IDemoControls to access shiftUp
    input::IDemoControls* controls = provider.get();
    controls->shiftUp();   // PARK -> REVERSE
    controls->shiftUp();   // REVERSE -> NEUTRAL
    controls->shiftUp();   // NEUTRAL -> DRIVE
    EXPECT_EQ(controls->getGearSelectorState(), static_cast<int>(bridge::GearSelector::DRIVE));

    // Ensure ignition is on
    controls->setIgnition(true);
    EXPECT_TRUE(controls->isIgnitionOn());

    // --- 5. Set full throttle ---
    throttlePtr->setThrottleLevel(1.0);
    controls->setThrottle(1.0);

    // --- 6. Run simulation loop ---
    for (int frame = 0; frame < TOTAL_FRAMES; ++frame) {
        // Maintain full throttle every frame
        throttlePtr->setThrottleLevel(1.0);
        controls->setThrottle(1.0);

        // Simulate engine RPM feedback for the twin state machine
        EngineSimStats fakeStats{};
        if (frame < CRANKING_RPM_RAMP_FRAMES) {
            // Cranking phase: ramp RPM from 0 to 800
            double t = static_cast<double>(frame) / CRANKING_RPM_RAMP_FRAMES;
            fakeStats.currentRPM = CRANKING_RAMP_START + t * (CRANKING_RAMP_END - CRANKING_RAMP_START);
        } else {
            // Running phase: simulate RPM proportional to speed + base idle
            // Use the demo physics road speed to compute a plausible RPM
            double speedKmh = provider->getDemoRoadSpeedKmh();
            // Simple model: RPM = 800 + speed-based component
            // At 100 km/h in top gear, ~2500 RPM is reasonable
            fakeStats.currentRPM = RUNNING_RPM_BASE + speedKmh * 20.0;
            // Cap at redline
            if (fakeStats.currentRPM > profile.redlineRpm) {
                fakeStats.currentRPM = profile.redlineRpm;
            }
        }
        fakeStats.vehicleSpeedKmh = provider->getDemoRoadSpeedKmh();

        // Feed RPM back to the twin
        provider->provideFeedback(fakeStats);

        // Tick the simulation
        input::EngineInput input = provider->OnUpdateSimulation(DT);
    }

    // --- 7. Report summary ---
    std::cout << "\n=== Gearbox Diagnostic Summary ===" << std::endl;
    std::cout << "CSV written to: " << csvPath << std::endl;
    std::cout << "Final speed: " << provider->getDemoRoadSpeedKmh() << " km/h" << std::endl;
    std::cout << "Final gear: " << provider->getDemoGear() << std::endl;
    std::cout << "===================================\n" << std::endl;

    // The CSV file is the primary output for analysis.
    // Assertions below are minimal -- the real analysis is post-run CSV inspection.
    EXPECT_GT(provider->getDemoRoadSpeedKmh(), 0.0)
        << "Vehicle should have moved during full-throttle acceleration";
}
