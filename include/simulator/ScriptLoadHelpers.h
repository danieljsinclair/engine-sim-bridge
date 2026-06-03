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
 * Returns the directory that directly contains "sound-library/".
 *
 * The deserializer normalizes impulse response filenames to "sound-library/..."
 * (stripping "../" and "es/" platform-specific prefixes). This function returns
 * the platform-specific base directory such that base + "sound-library/..." resolves
 * to the actual WAV file:
 *
 *   macOS dev: base = <engine-sim-root>/es/  (es/sound-library/... lives here)
 *   iOS bundle: base = <bundle>/              (sound-library/... lives here directly)
 *
 * If assetBasePath is empty or looks like a sentinel label (e.g., "(default engine)"),
 * derives the path from the script path.
 */
inline std::string resolveAssetBasePath(const std::string& scriptPath, const std::string& assetBasePath) {
    // Helper: check if a path directly contains sound-library/
    auto hasSoundLibrary = [](const std::filesystem::path& p) -> bool {
        return std::filesystem::exists(p / "sound-library");
    };

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
    if (!assetBasePath.empty() && (assetBasePath[0] == '/' || assetBasePath[0] == '.' ||
        assetBasePath.find('/') != std::string::npos)) {
        std::filesystem::path candidate(assetBasePath);

        // Accept if candidate directly contains sound-library/ (e.g., the es/ directory)
        if (hasSoundLibrary(candidate)) {
            return assetBasePath;
        }
        // Accept if candidate is a valid engine-sim root: return <root>/es/
        if (isValidEngineSimRoot(candidate)) {
            return (candidate / "es").string();
        }
        // Accept if candidate is a parent of a valid engine-sim root (split layout)
        if (isValidSplitRoot(candidate)) {
            return (candidate / "engine-sim" / "es").string();
        }
        // The override is a real path but not a valid sound base.
        // Fall through to upward search from the script path.
    }

    size_t assetsPos = scriptPath.find("/assets/");
    if (assetsPos != std::string::npos) {
        std::filesystem::path root(scriptPath.substr(0, assetsPos));
        if (hasSoundLibrary(root / "es")) {
            return (root / "es").string();
        }
        return root.string();
    }

    // For JSON presets (no /assets/ in path), search upward for the sound base.
    {
        std::filesystem::path search = std::filesystem::path(scriptPath).parent_path();
        for (int i = 0; i < 10; i++) {
            // Direct root: has es/ and assets/ -> return <root>/es/
            if (isValidEngineSimRoot(search)) {
                return (search / "es").string();
            }
            // Split root: has es/ and engine-sim/ child -> return <parent>/engine-sim/es/
            if (isValidSplitRoot(search)) {
                return (search / "engine-sim" / "es").string();
            }
            // iOS flat bundle: directory directly contains sound-library/
            if (hasSoundLibrary(search)) {
                return search.string();
            }
            if (search == search.parent_path()) break;  // filesystem root
            search = search.parent_path();
        }
    }

    // Fallback: return parent of the script file
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
