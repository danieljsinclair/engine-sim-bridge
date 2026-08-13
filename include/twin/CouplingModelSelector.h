// CouplingModelSelector.h - Selects the active ICouplingModel and owns the
// `legacy` DEFER adapter.
//
// This header is the SELECTION seam for the --coupling-model CLI toggle. It is
// deliberately separate from ICouplingModel.h (the contract) so adding a model
// does not touch the interface header (which the torque-converter worker owns).
//
// THE DEFER IDIOM (legacy):
// The legacy live clutch path is ~60 lines INLINE in VirtualIceTwin (slip-lock +
// launch + the binary creep-drag relief). Rather than duplicate it into a class,
// the LegacyCouplingModel adapter makes compute() return a sentinel pressure
// (kCouplingDeferToLegacy < 0). The twin detects the sentinel and runs its
// existing inline legacy code unchanged. This reuses the -1.0 "decline"
// convention already used by IWheelCoupling::clutchLockOverride() and
// LAUNCH_PRESSURE_DEFER, and keeps legacy behaviour byte-for-byte (so the A/B
// comparison --coupling-model legacy reproduces the historical oscillation).
//
// A real coupling model (ClutchMapModel, TorqueConverter) ALWAYS returns a
// pressure in [0,1], so pressure < 0 unambiguously means "defer to legacy".

#ifndef TWIN_COUPLING_MODEL_SELECTOR_H
#define TWIN_COUPLING_MODEL_SELECTOR_H

#include <memory>

#include <twin/ClutchMapModel.h>
#include <twin/ICouplingModel.h>
#include <twin/TorqueConverter.h>

namespace twin {

// Selector mirroring the --coupling-model CLI flag.
enum class CouplingModelKind {
    ClutchMap,        // declarative smooth governor curve (DEFAULT — no binary relief)
    TorqueConverter,  // fluid-coupling model (stub: defers until the TC worker wires task #33)
    Legacy            // historical bang-bang relief path (twin inline — A/B comparison only)
};

// Sentinel: the model declines to decide the pressure; the twin runs its legacy
// inline clutch logic. See file header.
constexpr double kCouplingDeferToLegacy = -1.0;

// True when a CouplingOutput carries a real pressure the twin should use
// directly (i.e. the model did not defer to legacy).
inline bool modelOwnsPressure(const CouplingOutput& out) {
    return out.clutchPressure >= 0.0;
}

// Legacy adapter: defers to the twin's inline historical clutch path. compute()
// returns the sentinel; the twin runs slip-lock + launch + binary relief inline.
// This keeps the A/B `legacy` path identical to the pre-toggle behaviour.
class LegacyCouplingModel : public ICouplingModel {
public:
    CouplingOutput compute(const CouplingInput& /*input*/) override {
        return CouplingOutput{kCouplingDeferToLegacy, false};
    }
};

// Factory mapping kind -> strategy (Open/Closed: add a model by adding a branch
// here and a class). Mirrors makeWheelCoupling. The torque-converter branch is a
// DEFER stub until the TC worker wires the real TorqueConverter (task #33): they
// swap the body for `std::make_unique<TorqueConverter>()` and add the include.
inline std::unique_ptr<ICouplingModel> makeCouplingModel(CouplingModelKind kind) {
    switch (kind) {
        case CouplingModelKind::ClutchMap:
            return std::make_unique<ClutchMapModel>();
        case CouplingModelKind::TorqueConverter:
            // Fluid-coupling model (pump/turbine + TR/K curves). Ported from the
            // rescue branch; implements ICouplingModel. Default params suit the
            // M156 / C63 clutch; configure() can re-tune per profile.
            return std::make_unique<TorqueConverter>();
        case CouplingModelKind::Legacy:
            return std::make_unique<LegacyCouplingModel>();
    }
    return std::make_unique<ClutchMapModel>();  // defensive: unreachable enum
}

}  // namespace twin

#endif  // TWIN_COUPLING_MODEL_SELECTOR_H
