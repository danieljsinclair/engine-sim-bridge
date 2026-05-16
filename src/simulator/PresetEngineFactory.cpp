// PresetEngineFactory.cpp - Delegates to PresetDeserializer for JSON loading

#include "simulator/PresetEngineFactory.h"
#include "preset/PresetDeserializer.h"
#include "common/JsonParser.h"

#include <fstream>
#include <sstream>

using json::JsonValue;

PresetLoadResult PresetEngineFactory::loadFromFile(const std::string& jsonPath) {
    std::ifstream file(jsonPath);
    if (!file.is_open()) {
        PresetLoadResult result;
        result.error = "Cannot open preset file: " + jsonPath;
        return result;
    }

    std::ostringstream ss;
    ss << file.rdbuf();
    return loadFromString(ss.str(), jsonPath);
}

PresetLoadResult PresetEngineFactory::loadFromString(const std::string& jsonContent, const std::string& sourceName) {
    PresetLoadResult result;

    try {
        JsonValue root = json::parse(jsonContent);
        return PresetDeserializer::deserialize(root, sourceName);
    } catch (const std::exception& e) {
        result.error = std::string("JSON parse error: ") + e.what();
        return result;
    }
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
