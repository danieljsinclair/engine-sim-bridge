#ifndef PRESET_DESERIALIZER_H
#define PRESET_DESERIALIZER_H

#include "common/JsonParser.h"
#include "simulator/PresetEngineFactory.h"

#include <string>
#include <string_view>

class PresetDeserializer {
public:
    // Top-level orchestrator: parses JSON and returns fully constructed Engine/Vehicle/Transmission
    static PresetLoadResult deserialize(const json::JsonValue& root, std::string_view sourceName = "<string>");
};

#endif // PRESET_DESERIALIZER_H
