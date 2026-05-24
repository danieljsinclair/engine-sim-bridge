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
        std::string irFilename = json["impulseResponseFilename"].asString();

        // Normalize paths like "../../es/sound-library/..." to "sound-library/..."
        // The preset_compiler stores Piranha-resolved paths relative to the .mr script
        // location. These contain "../" prefixes and the "es/" platform-specific prefix.
        // Strip both so the path is platform-independent: "sound-library/...".
        // resolveAssetBasePath returns the directory containing "sound-library/" on
        // both macOS (<root>/es/) and iOS (<bundle>/).
        while (irFilename.size() >= 3 && irFilename.substr(0, 3) == "../") {
            irFilename = irFilename.substr(3);
        }
        // Strip the "es/" platform-specific prefix left after removing ../
        if (irFilename.size() > 3 && irFilename.substr(0, 3) == "es/") {
            irFilename = irFilename.substr(3);
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
