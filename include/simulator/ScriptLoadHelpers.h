// ScriptLoadHelpers.h - Shared helpers for Piranha script loading
// DRY: Shared helpers for engine simulation setup
// SRP: Each function has a single responsibility

#ifndef SCRIPT_LOAD_HELPERS_H
#define SCRIPT_LOAD_HELPERS_H

#include "simulator.h"
#include "engine.h"
#include "vehicle.h"
#include "transmission.h"
#include "units.h"
#include "common/ILogging.h"

#include <string>
#include <filesystem>

// Forward declarations (avoids pulling in WavLoader in every TU)
class ILogging;

namespace ScriptLoadHelpers {

/**
 * Normalize script path to absolute path.
 */
inline std::string normalizeScriptPath(const std::string& scriptPath) {
    if (scriptPath.empty()) {
        return "";
    }

    return std::filesystem::absolute(scriptPath).lexically_normal().string();
}

/**
 * Resolve asset base path from script path or explicit override.
 * If assetBasePath is empty or looks like a sentinel label (e.g., "(default engine)"),
 * derives the path from the script path.
 *
 * A valid asset base path must be a directory that contains both es/ and assets/
 * subdirectories, or contains es/ with a child engine-sim/ directory that has both.
 * If the explicit override doesn't meet this criteria, falls through to upward search.
 */
inline std::string resolveAssetBasePath(const std::string& scriptPath, const std::string& assetBasePath) {
    // Helper: check if a path is a valid engine-sim root (has es/ and assets/).
    auto isValidEngineSimRoot = [](const std::filesystem::path& p) -> bool {
        return std::filesystem::exists(p / "es") &&
               std::filesystem::exists(p / "assets");
    };

    // Helper: check if a path has es/ locally and assets/ in a child engine-sim/ directory.
    auto isValidSplitRoot = [](const std::filesystem::path& p) -> bool {
        if (!std::filesystem::exists(p / "es")) return false;
        std::filesystem::path subModule = p / "engine-sim";
        return std::filesystem::exists(subModule) &&
               std::filesystem::exists(subModule / "es") &&
               std::filesystem::exists(subModule / "assets");
    };

    // If an explicit assetBasePath is provided, validate it before accepting.
    // It must be a directory that is actually a valid engine-sim root.
    if (!assetBasePath.empty() && (assetBasePath[0] == '/' || assetBasePath[0] == '.' ||
        assetBasePath.find('/') != std::string::npos)) {
        std::filesystem::path candidate(assetBasePath);
        if (isValidEngineSimRoot(candidate)) {
            return assetBasePath;
        }
        // Also accept if the override itself is a parent of a valid engine-sim root
        // (e.g., the bridge root that has es/ but assets/ is in engine-sim/ child)
        if (isValidSplitRoot(candidate)) {
            return (candidate / "engine-sim").string();
        }
        // The override is a real path but not a valid engine-sim root.
        // Fall through to upward search from the script path.
    }

    size_t assetsPos = scriptPath.find("/assets/");
    if (assetsPos != std::string::npos) {
        return scriptPath.substr(0, assetsPos);
    }

    // For JSON presets (no /assets/ in path), search for the engine-sim root.
    // The engine-sim root is the directory containing es/ and assets/ subdirectories.
    // Search upward from the preset file, and also check common child directories
    // like "engine-sim/" since the submodule may be nested.
    {
        std::filesystem::path search = std::filesystem::path(scriptPath).parent_path();
        for (int i = 0; i < 10; i++) {
            // Check this directory directly (both es/ and assets/ present)
            if (isValidEngineSimRoot(search)) {
                return search.string();
            }
            // Check if this dir has es/ and a child engine-sim/ with both es/ and assets/
            if (isValidSplitRoot(search)) {
                return (search / "engine-sim").string();
            }
            if (search == search.parent_path()) break;  // filesystem root
            search = search.parent_path();
        }
    }

    size_t lastSlash = scriptPath.find_last_of('/');
    if (lastSlash != std::string::npos) {
        return scriptPath.substr(0, lastSlash);
    }

    return ".";
}

/**
 * Create default vehicle with sensible parameters.
 */
inline Vehicle* createDefaultVehicle() {
    Vehicle::Parameters vehParams;
    vehParams.mass = units::mass(1597, units::kg);
    vehParams.diffRatio = 3.42;
    vehParams.tireRadius = units::distance(10, units::inch);
    vehParams.dragCoefficient = 0.25;
    vehParams.crossSectionArea = units::distance(6.0, units::foot) * units::distance(6.0, units::foot);
    vehParams.rollingResistance = 2000.0;

    Vehicle* vehicle = new Vehicle;
    vehicle->initialize(vehParams);
    return vehicle;
}

/**
 * Create default transmission with sensible gear ratios.
 */
inline Transmission* createDefaultTransmission() {
    const double gearRatios[] = { 2.97, 2.07, 1.43, 1.00, 0.84, 0.56 };

    Transmission::Parameters tParams;
    tParams.GearCount = 6;
    tParams.GearRatios = gearRatios;
    tParams.MaxClutchTorque = units::torque(1000.0, units::ft_lb);

    Transmission* transmission = new Transmission;
    transmission->initialize(tParams);
    return transmission;
}

/**
 * Load impulse responses for all exhaust systems in the engine.
 * Implementation in ScriptLoadHelpers.cpp (avoids DR_WAV_IMPLEMENTATION in header).
 */
bool loadImpulseResponses(
    Simulator* simulator,
    Engine* engine,
    const std::string& assetBasePath,
    ILogging* logger);

} // namespace ScriptLoadHelpers

#endif // SCRIPT_LOAD_HELPERS_H
