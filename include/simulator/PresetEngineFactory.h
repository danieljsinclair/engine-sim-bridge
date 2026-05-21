// PresetEngineFactory.h - Loads JSON presets into Engine/Vehicle/Transmission objects
// No Piranha/Boost dependency -- works on all platforms

#ifndef PRESET_ENGINE_FACTORY_H
#define PRESET_ENGINE_FACTORY_H

#include <string>
#include <cstdint>

// Forward declarations
class Engine;
class Vehicle;
class Transmission;

// Result of preset loading
struct PresetLoadResult {
    Engine* engine;
    Vehicle* vehicle;
    Transmission* transmission;
    std::string presetName;
    std::string error;

    // Transmission state from JSON -- applied after wiring (vehicle must be set first)
    int initialGear;          // -1 = neutral (default if absent)
    double initialClutchPressure;  // 0.0 = disengaged (default if absent)

    PresetLoadResult()
        : engine(nullptr), vehicle(nullptr), transmission(nullptr),
          initialGear(-1), initialClutchPressure(0.0) {}
    bool success() const { return engine != nullptr && error.empty(); }
};

// PresetEngineFactory - Reads JSON presets and constructs engine object graphs
class PresetEngineFactory {
public:
    static PresetLoadResult loadFromFile(const std::string& jsonPath);
    static PresetLoadResult loadFromString(const std::string& jsonContent, const std::string& sourceName = "<string>");
    static PresetLoadResult loadFromJson(const char* jsonContent, size_t jsonSize);
};

#endif // PRESET_ENGINE_FACTORY_H
