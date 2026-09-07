// TwinCouplingRecipe.h - shared twin coupling flag parse + apply recipe.
//
// The recipe every telemetry input path (live stdin CSV, replay CSV, demo)
// uses to translate host-side coupling flag strings onto a coupling-bearing
// provider's setter surface, in the historical order (coupling mode, coupling
// model, tau, torque toggles). Extracted from the CLI's
// TelemetryProviderFactory (consolidation wave B) so the CLI and any other
// bridge host apply the SAME recipe by construction instead of keeping
// parallel copies that can drift (the twin reversion bug was downstream of
// exactly such a silent divergence).
//
// String->enum parsing delegates to twin::resolveWheelCouplingMode /
// twin::resolveCouplingModel (twin/CouplingConfig.h) - the single source of
// truth for that translation, shared with the iOS app. A typo'd value throws
// std::invalid_argument naming the flag, the bad value and the legal set;
// hosts that surface typed errors map it at their seam (the CLI rethrows
// CliException with the identical message).
//
// Ordering contract (why the setters are called BEFORE the caller runs
// Initialize): the providers store the coupling/torque configs and re-apply
// them when Initialize() creates the twin, so a pre-Initialize set is
// equivalent to the historical post-Initialize set (see the setters'
// store + re-apply contract on LiveTelemetryProvider.h). The re-apply
// defaults are the TWIN defaults (Free / ClutchMap / 0.0) - the
// 2026-09-06 factory-ordering regression territory; do not reorder.
//
// Header-only by design: pure wiring over existing bridge types, no .cpp
// needed (keeps the bridge's tested surface unchanged).

#ifndef INPUT_TWIN_COUPLING_RECIPE_H
#define INPUT_TWIN_COUPLING_RECIPE_H

#include <string>

#include "twin/CouplingConfig.h"
#include "twin/CouplingModelSelector.h"
#include "twin/EffectiveThrottle.h"
#include "twin/UpstreamTorqueHint.h"
#include "twin/WheelCoupling.h"

namespace input {

// The coupling/torque knob set carried from a host's parsed arguments into
// the shared apply recipe. Field defaults mirror the CLI's TwinArgs defaults
// (pin / torque-converter / 150 ms / toggles off - the owner-tuned road
// values); hosts fill every field from their own argument struct.
struct TwinCouplingRecipe {
    std::string wheelCoupling = "pin";
    std::string couplingModel = "torque-converter";
    double pinTauMs = 150.0;
    bool effectiveThrottle = false;
    bool torqueInformedGearbox = false;
};

// Apply the shared twin coupling flags to a coupling-bearing provider, in the
// historical order (coupling mode, coupling model, tau, torque toggles).
// Template over the concrete provider: live, replay and demo expose the same
// setter surface. The torque toggles are forwarded unconditionally: the
// disabled configs are inert no-ops on the twin (set-disabled is provably
// identical to never-set), so the default path stays byte-identical.
// pinTauMs is passed through untouched - tau is warn-only by owner directive
// (the warning thresholds + text live in twin/PinTargetChase.h), and tau <= 0
// is rigid by PinTargetChase construction.
template <typename Provider>
void applyTwinCouplingFlags(Provider& provider, const TwinCouplingRecipe& recipe) {
    provider.setWheelCouplingMode(twin::resolveWheelCouplingMode(recipe.wheelCoupling));
    provider.setCouplingModel(twin::resolveCouplingModel(recipe.couplingModel));
    provider.setPinTauMs(recipe.pinTauMs);
    twin::EffectiveThrottleConfig effectiveThrottle;
    effectiveThrottle.enabled = recipe.effectiveThrottle;
    provider.setEffectiveThrottleConfig(effectiveThrottle);
    twin::TorqueInformedGearboxConfig torqueInformedGearbox;
    torqueInformedGearbox.enabled = recipe.torqueInformedGearbox;
    provider.setTorqueInformedGearboxConfig(torqueInformedGearbox);
}

}  // namespace input

#endif  // INPUT_TWIN_COUPLING_RECIPE_H
