#include "preset/ExhaustSystemDeserializer.h"

#include "exhaust_system.h"
#include "impulse_response.h"
#include "common/PathNormalizer.h"

#include <stdexcept>
#include <memory>

using json::JsonValue;

void ExhaustSystemDeserializer::deserialize(const JsonValue& json, ExhaustSystem* es,
                                             const std::string& context) {
    const std::string ctx = context.empty() ? "exhaustSystem" : context;

    if (!json.has("audioVolume")) {
        throw std::runtime_error("Missing required field 'audioVolume' in " + ctx);
    }

    ExhaustSystem::Parameters params;

    if (!json.has("length")) {
        throw std::runtime_error("Missing required field 'length' in " + ctx);
    }
    params.length = json["length"].asNumber();

    if (!json.has("collectorCrossSectionArea")) {
        throw std::runtime_error("Missing required field 'collectorCrossSectionArea' in " + ctx);
    }
    params.collectorCrossSectionArea = json["collectorCrossSectionArea"].asNumber();

    if (!json.has("outletFlowRate")) {
        throw std::runtime_error("Missing required field 'outletFlowRate' in " + ctx);
    }
    params.outletFlowRate = json["outletFlowRate"].asNumber();

    if (!json.has("primaryTubeLength")) {
        throw std::runtime_error("Missing required field 'primaryTubeLength' in " + ctx);
    }
    params.primaryTubeLength = json["primaryTubeLength"].asNumber();

    if (!json.has("primaryFlowRate")) {
        throw std::runtime_error("Missing required field 'primaryFlowRate' in " + ctx);
    }
    params.primaryFlowRate = json["primaryFlowRate"].asNumber();

    if (!json.has("velocityDecay")) {
        throw std::runtime_error("Missing required field 'velocityDecay' in " + ctx);
    }
    params.velocityDecay = json["velocityDecay"].asNumber();

    params.audioVolume = json["audioVolume"].asNumber();
    params.impulseResponse = nullptr;

    if (json.has("impulseResponseFilename") && json["impulseResponseFilename"].isString()) {
        std::string irFilename = PathNormalizer::normalizeImpulseResponsePath(
            json["impulseResponseFilename"].asString());

        if (!json.has("impulseResponseVolume")) {
            throw std::runtime_error("Missing required field 'impulseResponseVolume' when 'impulseResponseFilename' present in " + ctx);
        }
        auto irVolume = json["impulseResponseVolume"].asNumber();

        auto ir = std::make_unique<ImpulseResponse>();
        ir->initialize(irFilename, irVolume);
        params.impulseResponse = ir.release();
    }

    es->initialize(params);
}
