#include "preset/ExhaustSystemDeserializer.h"

#include "exhaust_system.h"
#include "impulse_response.h"

#include <stdexcept>

using json::JsonValue;

void ExhaustSystemDeserializer::deserialize(const JsonValue& json, ExhaustSystem* es,
                                             const std::string& context) {
    const std::string ctx = context.empty() ? "exhaustSystem" : context;

    if (!json.has("audioVolume")) {
        throw std::runtime_error("Missing required field 'audioVolume' in " + ctx);
    }

    ExhaustSystem::Parameters params;
    params.length = json["length"].numberOr(0);
    params.collectorCrossSectionArea = json["collectorCrossSectionArea"].numberOr(0);
    params.outletFlowRate = json["outletFlowRate"].numberOr(0);
    params.primaryTubeLength = json["primaryTubeLength"].numberOr(0);
    params.primaryFlowRate = json["primaryFlowRate"].numberOr(0);
    params.velocityDecay = json["velocityDecay"].numberOr(0);
    params.audioVolume = json["audioVolume"].asNumber();
    params.impulseResponse = nullptr;

    if (json.has("impulseResponseFilename") && json["impulseResponseFilename"].isString()) {
        std::string irFilename = json["impulseResponseFilename"].asString();

        // Normalize paths like "../../es/sound-library/..." to "sound-library/..."
        size_t esPos = irFilename.find("es/");
        if (esPos != std::string::npos) {
            irFilename = irFilename.substr(esPos + 3);
        }

        if (!json.has("impulseResponseVolume")) {
            throw std::runtime_error("Missing required field 'impulseResponseVolume' when 'impulseResponseFilename' present in " + ctx);
        }
        double irVolume = json["impulseResponseVolume"].asNumber();

        ImpulseResponse* ir = new ImpulseResponse();
        ir->initialize(irFilename, irVolume);
        params.impulseResponse = ir;
    }

    es->initialize(params);
}
