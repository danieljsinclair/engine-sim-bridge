// WarmBoot.h - Shared warm-boot seam for replay & live providers.
//
// WHY THIS EXISTS (DRY / OCP):
//   Both ReplayTelemetryProvider and LiveTelemetryProvider must bring their
//   owned VirtualIceInputProvider twin to RUNNING before the first real frame
//   (replay reproduces a running engine, not the cranking transient; live must
//   NOT cold-jump or the gas path reversion-blows). The OFF->CRANKING->IDLE->
//   RUNNING stepping plus the warm-up prime were duplicated in
//   ReplayTelemetryProvider::primeTwinToRunning. This free function is the
//   single copy both call, so the two layers stay in agreement (no twin/core
//   phase mismatch at frame 1).
//
// CONTRACT:
//   - Advances the twin's state machine only (the provider does not own the
//     engine-sim core), so the core starts Stopped and is bump-started on the
//     first real frames through its own physics. This mirrors replay's.
//   - seedThrottle/seedSpeedKmh/seedSelector initialise the synthetic prime
//     signal: replay seeds from the FIRST parsed sample; live (no parsed
//     samples at Initialize time) falls back to a running-baseline seed.
//   - Idempotent per twin instance via VirtualIceInputProvider::primeWarmUp's
//     warmedUp_ guard; the RUNNING-prime loop is cheap and safe to re-run.

#ifndef INPUT_WARM_BOOT_H
#define INPUT_WARM_BOOT_H

#include "input/VirtualIceInputProvider.h"

#include <cstdint>

namespace input {

/// Bring the owned twin provider to RUNNING (OFF->CRANKING->IDLE->RUNNING) then
/// settle it into the WARM cruise basin via primeWarmUp(), mirroring a running
/// engine so the first real frame already exposes a forward gear. Shared by
/// replay and live so the two prime paths never diverge.
///
/// @param twin            Owned twin provider (must be Initialize()'d).
/// @param seedThrottle    Throttle used for the synthetic prime frames.
/// @param seedSpeedKmh    Road speed (km/h) used for the synthetic prime frames.
/// @param seedSelector    Gear selector (bridge::GearSelector as int) for the
///                        synthetic prime frames; default DRIVE.
inline void warmBootTwinToRunning(VirtualIceInputProvider* twin,
                                  double seedThrottle,
                                  double seedSpeedKmh,
                                  int seedSelector = 99 /*bridge::GearSelector::DRIVE*/) {
    if (!twin) return;

    // Drive the twin through OFF->CRANKING->IDLE->RUNNING once. Past the 3s
    // CRANK_FALLBACK_DURATION_S crank budget (CRANKING -> IDLE), then a DRIVE
    // selector promotes IDLE -> RUNNING. 100 frames @ 0.05s = 5s sim, with
    // margin over the 3s fallback so RUNNING is guaranteed before frame 1.
    input::UpstreamSignal signal;
    signal.throttleFraction = seedThrottle;
    signal.speedKmh = seedSpeedKmh;
    signal.isValid = true;
    signal.timestampUtcMs = 1;  // non-zero -> twin treats as valid telemetry
    twin->setGearSelector(seedSelector);
    // The prime IS a start decision: command ignition ON. The twin defaults
    // ignition OFF and never self-starts (startStop contract — the controller
    // owns every start once engaged), but before the first driver input the
    // PROVIDER owns start authority, and a warm boot to RUNNING is exactly
    // such a decision (replay reproduces a running engine; live warm-boots to
    // avoid the cold-start flow reversion).
    twin->setIgnition(true);
    twin->setUpstreamSignal(signal);

    constexpr double kPrimeDt = 0.05;
    constexpr int kPrimeFrames = 100;
    for (int i = 0; i < kPrimeFrames; ++i) {
        twin->OnUpdateSimulation(kPrimeDt);
    }

    // Warm-up prime: settle the twin into the warm cruise basin (g4/g5, clutch
    // ~0.75) before the first real frame. One-shot per twin-provider instance.
    twin->primeWarmUp();
}

} // namespace input

#endif // INPUT_WARM_BOOT_H
