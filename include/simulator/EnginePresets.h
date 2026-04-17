// EnginePresets.h - Hardcoded C++ engine preset factories
// Translates .mr script engine definitions into direct C++ API calls
// No Piranha/Boost dependency -- works on all platforms including iOS
//
// SRP: Each factory builds one specific engine configuration
// OCP: New engines added by extending the registry, not modifying existing code
// DIP: Depends on engine-sim C++ API abstractions, not scripting layer

#ifndef ENGINE_PRESETS_H
#define ENGINE_PRESETS_H

#include <string>
#include <vector>
#include <memory>

class Simulator;
class ILogging;

// Describes an available engine preset
struct EnginePresetInfo {
    std::string id;           // Unique identifier (e.g., "honda_trx520")
    std::string displayName;  // Human-readable name (e.g., "Honda TRX520 (ATV)")
    std::string description;  // Short description (e.g., "Single cylinder ATV engine")
    int cylinderCount;        // Number of cylinders
    double displacementL;     // Approximate displacement in liters
};

// Factory for creating Simulator instances from hardcoded C++ engine presets.
// Each preset builds Engine + Vehicle + Transmission using direct API calls,
// mirroring what the .mr scripts do through the Piranha interpreter.
class EnginePresets {
public:
    // Get list of all available engine presets
    static const std::vector<EnginePresetInfo>& getAvailablePresets();

    // Create a Simulator for the given preset ID.
    // Returns nullptr if preset ID is not found.
    // Caller owns the returned Simulator and must call destroy() when done.
    static Simulator* createPreset(const std::string& presetId, ILogging* logger = nullptr);

private:
    // Individual engine factory functions
    static Simulator* createHondaTrx520(ILogging* logger);
    static Simulator* createSubaruEj25(ILogging* logger);
    static Simulator* createGmLs(ILogging* logger);
};

#endif // ENGINE_PRESETS_H
