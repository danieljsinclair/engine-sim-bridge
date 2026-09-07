// GearboxReconfiguration.h - reconfigure gearbox-bearing input providers to
// match the simulator's actual transmission ratios, plus the Bug-C3
// fail-fast policy for the live path.
//
// Moved from the CLI's CLIMain.cpp (consolidation wave B). The CLI keeps a
// thin forwarder with the same CLI-side signature that maps
// GearboxReconfigurationRefusal to its CliException with the message
// unchanged; other hosts can apply the policy directly.
//
// Localizes the BridgeSimulator/provider casts into one cohesive unit (SRP)
// so the host's run loop stays flat. Open/Closed note: the casts here are
// the seam - providers expose reconfigureProfile() but it is not yet on the
// shared IInputProvider interface. When it is promoted there, this helper
// collapses to a single polymorphic call.

#ifndef INPUT_GEARBOX_RECONFIGURATION_H
#define INPUT_GEARBOX_RECONFIGURATION_H

#include <stdexcept>
#include <vector>

#include "io/IInputProvider.h"
#include "input/DemoInputProvider.h"
#include "input/LiveTelemetryProvider.h"
#include "input/ReplayTelemetryProvider.h"
#include "simulator/BridgeSimulator.h"

namespace input {

// Raised when the LIVE path would silently drive the engine on the twin's
// hardcoded zf8hp45 default profile because the loaded script supplied no
// transmission/vehicle geometry (Bug C3). Fail fast rather than hiding the
// geometry mismatch - determinism over silent fallback.
class GearboxReconfigurationRefusal : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// Reconfigure gearbox-bearing input providers to match the simulator's
// actual transmission ratios. telemetryProvider is the live/replay provider
// (nullable); demoProvider is the keyboard/--auto demo provider (nullable).
// Both are skipped when null or when the concrete cast misses. A
// non-BridgeSimulator (or null) simulator is skipped entirely - the cast
// guard precedes even the C3 check.
inline void reconfigureGearboxProviders(ISimulator* simulator,
                                        IInputProvider* telemetryProvider,
                                        IInputProvider* demoProvider) {
    auto* bridgeSim = dynamic_cast<BridgeSimulator*>(simulator);
    if (!bridgeSim) return;

    const auto* rawSim = bridgeSim->getInternalSimulator();
    const auto* trans = rawSim ? rawSim->getTransmission() : nullptr;
    const auto* vehicle = rawSim ? rawSim->getVehicle() : nullptr;

    // The LIVE path (--live-telemetry) builds its twin with a hardcoded
    // zf8hp45 default profile. If the named .mr did NOT supply a transmission +
    // vehicle, that default would silently drive the engine (Bug C3). Fail fast
    // rather than hiding the geometry mismatch - determinism over silent C63.
    if (dynamic_cast<LiveTelemetryProvider*>(telemetryProvider) != nullptr) {
        if (!trans || !vehicle || trans->getGearCount() <= 0) {
            throw GearboxReconfigurationRefusal(
                "Live telemetry requested but the loaded script supplies no transmission/"
                "vehicle geometry. The auto-gearbox twin has no ratios to match against. "
                "Add a `vehicle` + `transmission` node (or `import` a shared block such as "
                "tesla_y_performance.mr) to the .mr. Refusing to silently fall back to zf8hp45.");
        }
    }

    // Replay / demo paths may legitimately run on the default profile when no
    // geometry is present (legacy scripts), so leave the provider's default.
    if (!trans || !vehicle || trans->getGearCount() <= 0) return;

    std::vector<double> ratios;
    ratios.reserve(static_cast<size_t>(trans->getGearCount()));
    for (int g = 0; g < trans->getGearCount(); ++g) {
        ratios.push_back(trans->getGearRatio(g));
    }

    // Replay path
    if (auto* replay = dynamic_cast<ReplayTelemetryProvider*>(telemetryProvider)) {
        replay->reconfigureProfile(ratios, vehicle->getDiffRatio(), vehicle->getTireRadius());
    }
    // Live --live-telemetry path (CSV stdin drives the twin). The named engine
    // loaded via --script may have different ratios than the twin's default ZF
    // profile, so reconfigure the box to match (e.g. a C63 M156).
    if (auto* live = dynamic_cast<LiveTelemetryProvider*>(telemetryProvider)) {
        live->reconfigureProfile(ratios, vehicle->getDiffRatio(), vehicle->getTireRadius());
    }
    // Keyboard --auto path (via DemoInputProvider)
    if (auto* demo = dynamic_cast<DemoInputProvider*>(demoProvider)) {
        demo->reconfigureProfile(ratios, vehicle->getDiffRatio(), vehicle->getTireRadius());
    }
}

}  // namespace input

#endif  // INPUT_GEARBOX_RECONFIGURATION_H
