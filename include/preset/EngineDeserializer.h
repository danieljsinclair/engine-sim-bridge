#ifndef ENGINE_DESERIALIZER_H
#define ENGINE_DESERIALIZER_H

#include "common/JsonParser.h"
#include "engine.h"

class CylinderBank;
class Crankshaft;

/// Parameters for cylinder deserialization — groups related arguments.
struct CylinderDeserializationParams {
    const json::JsonValue* bankJson = nullptr;
    CylinderBank* bank = nullptr;
    const Engine* engine = nullptr;
    Crankshaft* mainCrank = nullptr;
    int bankIdx = 0;
    int globalCylIdx = 0;
    int cylCount = 0;
    const std::string* ctx = nullptr;
    size_t bankIndex = 0;
};

class EngineDeserializer {
public:
    static Engine* deserialize(const json::JsonValue& json, const std::string& context = "");

private:
    static Engine::Parameters readParams(const json::JsonValue& json, const std::string& ctx);
    static void validateRequiredSections(const json::JsonValue& json, const Engine::Parameters& params, const std::string& ctx);
    static void deserializeCrankshafts(const json::JsonValue& json, const Engine* engine, const std::string& ctx);
    static void deserializeExhaustSystems(const json::JsonValue& json, const Engine* engine, const std::string& ctx);
    static void deserializeIntakes(const json::JsonValue& json, const Engine* engine, const std::string& ctx);
    static void deserializeCylinderBanks(const json::JsonValue& json, Engine* engine, const std::string& ctx);
    static void deserializeCylinders(const CylinderDeserializationParams& params);
    static void initializeCombustionChambers(const json::JsonValue& json, Engine* engine, const std::string& ctx);
};

#endif // ENGINE_DESERIALIZER_H
