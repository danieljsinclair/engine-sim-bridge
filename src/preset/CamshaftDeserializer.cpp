#include "preset/CamshaftDeserializer.h"

#include "camshaft.h"
#include "preset/FunctionDeserializer.h"
#include "common/PresetExceptions.h"

#include <memory>

using json::JsonValue;

Camshaft* CamshaftDeserializer::deserialize(const JsonValue& json, Crankshaft* crankshaft,
                                             const std::string& context) {
    const std::string ctx = context.empty() ? "camshaft" : context;

    const JsonValue& lobes = json["lobeCenterlines"];
    int lobeCount = lobes.isArray() ? static_cast<int>(lobes.size()) : 0;

    if (lobeCount == 0) {
        throw PresetDeserializationException("Missing or empty 'lobeCenterlines' in " + ctx);
    }

    if (!json.has("lobeProfile")) {
        throw PresetDeserializationException("Missing required field 'lobeProfile' in " + ctx);
    }
    Function* lobeProfile = FunctionDeserializer::deserialize(json["lobeProfile"], ctx + ".lobeProfile");

    Camshaft::Parameters params;
    params.lobes = lobeCount;

    if (!json.has("advance")) {
        throw PresetDeserializationException("Missing required field 'advance' in " + ctx);
    }
    params.advance = json["advance"].asNumber();

    if (!json.has("baseRadius")) {
        throw PresetDeserializationException("Missing required field 'baseRadius' in " + ctx);
    }
    params.baseRadius = json["baseRadius"].asNumber();

    params.crankshaft = crankshaft;
    params.lobeProfile = lobeProfile;

    auto cam = std::make_unique<Camshaft>();
    cam->initialize(params);

    // JSON stores CRANK angles. setLobeCenterline() divides by 2 internally.
    // Pass the raw JSON value directly.
    for (auto i = 0u; i < lobes.size(); i++) {
        cam->setLobeCenterline(static_cast<int>(i), lobes[i].asNumber());
    }

    return cam.release();
}
