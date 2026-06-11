#ifndef ENGINE_DESERIALIZER_H
#define ENGINE_DESERIALIZER_H

#include "common/JsonParser.h"
#include "engine.h"

class CylinderBank;
class Crankshaft;

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
    static void deserializeCylinders(const json::JsonValue& bankJson, CylinderBank* bank,
            const Engine* engine, Crankshaft* mainCrank, int /*bankIdx*/,
            int globalCylIdx, int cylCount, const std::string& ctx, size_t bankIndex);
    static void initializeCombustionChambers(const json::JsonValue& json, Engine* engine, const std::string& ctx);
};

#endif // ENGINE_DESERIALIZER_H
