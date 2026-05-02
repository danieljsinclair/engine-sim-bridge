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
 */
inline std::string resolveAssetBasePath(const std::string& scriptPath, const std::string& assetBasePath) {
    // Treat as valid only if it looks like a real path (starts with /, ./, or has a path separator)
    if (!assetBasePath.empty() && (assetBasePath[0] == '/' || assetBasePath[0] == '.' ||
        assetBasePath.find('/') != std::string::npos)) {
        return assetBasePath;
    }

    size_t assetsPos = scriptPath.find("/assets/");
    if (assetsPos != std::string::npos) {
        return scriptPath.substr(0, assetsPos);
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
