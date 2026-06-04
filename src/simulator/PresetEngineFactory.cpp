// PresetEngineFactory.cpp - Delegates to PresetDeserializer for JSON loading

#include "simulator/PresetEngineFactory.h"
#include "preset/PresetDeserializer.h"
#include "common/JsonParser.h"

#include "engine.h"
#include "exhaust_system.h"
#include "impulse_response.h"

#include <fstream>
#include <sstream>
#include <filesystem>

using json::JsonValue;

// Resolve relative impulse response filenames to absolute paths using the
// engine-sim root as the base directory. Paths like "../../es/sound-library/..."
// are CWD-relative from the engine-sim root (the CWD when the preset compiler
// ran). weakly_canonical resolves symlinks and normalizes the path.
static void resolveImpulseResponsePaths(Engine* engine, const std::filesystem::path& assetBase) {
    if (!engine || assetBase.empty()) return;

    for (int i = 0; i < engine->getExhaustSystemCount(); i++) {
        ExhaustSystem* es = engine->getExhaustSystem(i);
        if (!es) continue;

        ImpulseResponse* ir = es->getImpulseResponse();
        if (!ir) continue;

        const std::string& filename = ir->getFilename();
        if (filename.empty()) continue;

        std::filesystem::path p(filename);
        if (!p.is_absolute()) {
            std::filesystem::path resolved =
                std::filesystem::weakly_canonical(assetBase / p);
            ir->initialize(resolved.string(), ir->getVolume());
        }
    }
}

PresetLoadResult PresetEngineFactory::loadFromFile(const std::string& jsonPath,
                                                    const std::string& assetBasePath) {
    std::ifstream file(jsonPath);
    if (!file.is_open()) {
        PresetLoadResult result;
        result.error = "Cannot open preset file: " + jsonPath;
        return result;
    }

    std::ostringstream ss;
    ss << file.rdbuf();
    return loadFromString(ss.str(), jsonPath, assetBasePath);
}

PresetLoadResult PresetEngineFactory::loadFromString(const std::string& jsonContent,
                                                     const std::string& sourceName,
                                                     const std::string& assetBasePath) {
    PresetLoadResult result;

    try {
        JsonValue root = json::parse(jsonContent);
        result = PresetDeserializer::deserialize(root, sourceName);
    } catch (const std::exception& e) {
        result.error = std::string("JSON parse error: ") + e.what();
        return result;
    }

    // Resolve relative impulse response paths to absolute using the asset base.
    if (result.success() && !assetBasePath.empty()) {
        resolveImpulseResponsePaths(result.engine, std::filesystem::path(assetBasePath));
    }

    return result;
}

PresetLoadResult PresetEngineFactory::loadFromJson(const char* jsonContent, size_t jsonSize) {
    if (!jsonContent || jsonSize == 0) {
        PresetLoadResult result;
        result.error = "Null or empty JSON content";
        return result;
    }
    std::string content(jsonContent, jsonSize);
    return loadFromString(content, "<json>");
}
