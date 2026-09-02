// CouplingConfig.h - Shared string->enum resolver for twin coupling config.
//
// The CLI (engine-sim-cli/src/config/CLIMain.cpp) and the iOS app both need to
// translate the user-facing coupling strings ("free"/"pin"/"torque" and
// "clutch-map"/"torque-converter"/"legacy") onto the bridge's enum/setter API.
// Previously each side kept its own copy of that translation, which is a
// maintenance hazard: a typo'd value is accepted by one and rejected by the
// other, and the two can drift.
//
// This header is the SINGLE source of truth for that translation. It is pure,
// header-only, dependency-free C++ (no twin logic is reimplemented here — it
// only maps a string onto the bridge's EXISTING enums), so it is trivially
// includable from both the CLI (a C++ host) and the app's .mm unit.
//
// Fail-fast contract (mirrors CLIMain's existing "--wheel-coupling must be
// 'free', 'pin' or 'torque'"): an unrecognised value is a programmer/user typo,
// not a valid fallback, so it throws std::invalid_argument naming the value.

#ifndef TWIN_COUPLING_CONFIG_H
#define TWIN_COUPLING_CONFIG_H

#include <stdexcept>
#include <string>
#include <string_view>

#include "twin/WheelCoupling.h"
#include "twin/CouplingModelSelector.h"  // canonical CouplingModelKind definition

namespace twin {

/// Map a coupling model to the wheel-coupling strategy it engages. Inverse of
/// the string resolver: given a resolved model, which mode does it imply?
inline WheelCouplingMode couplingModelToWheelCoupling(CouplingModelKind kind) {
    switch (kind) {
        case CouplingModelKind::ClutchMap:      return WheelCouplingMode::Pin;
        case CouplingModelKind::TorqueConverter: return WheelCouplingMode::Torque;
        case CouplingModelKind::Legacy:         return WheelCouplingMode::Free;
    }
    return WheelCouplingMode::Free;  // defensive: unreachable enum
}

/// Resolve a wheel-coupling strategy string to the bridge enum.
/// 'free' -> Free, 'pin' -> Pin, 'torque' -> Torque.
/// Throws std::invalid_argument naming the value on anything else.
inline WheelCouplingMode resolveWheelCouplingMode(std::string_view s) {
    if (s == "free") return WheelCouplingMode::Free;
    if (s == "pin") return WheelCouplingMode::Pin;
    if (s == "torque") return WheelCouplingMode::Torque;
    throw std::invalid_argument(
        "--wheel-coupling must be 'free', 'pin' or 'torque', got: " +
        std::string(s));
}

/// Resolve a coupling-model string to the twin coupling model.
/// 'clutch-map' -> ClutchMap, 'torque-converter' -> TorqueConverter,
/// 'legacy' -> Legacy.
/// Throws std::invalid_argument naming the value on anything else.
inline CouplingModelKind resolveCouplingModel(std::string_view s) {
    if (s == "clutch-map") return CouplingModelKind::ClutchMap;
    if (s == "torque-converter") return CouplingModelKind::TorqueConverter;
    if (s == "legacy") return CouplingModelKind::Legacy;
    throw std::invalid_argument(
        "--coupling-model must be 'clutch-map', 'torque-converter' or 'legacy', got: " +
        std::string(s));
}

/// Derive the SimulatorFactory::create() torque-converter flag from the
/// coupling model: the torque-converter model is the only one that engages the
/// torque converter; clutch-map and legacy do not.
inline bool useTorqueConverterFromModel(std::string_view couplingModel) {
    return couplingModel == "torque-converter";
}

}  // namespace twin

#endif  // TWIN_COUPLING_CONFIG_H