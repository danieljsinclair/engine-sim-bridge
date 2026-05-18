#include "preset/CamshaftDeserializer.h"

#include "camshaft.h"
#include "preset/FunctionDeserializer.h"

#include <stdexcept>

using json::JsonValue;

Camshaft* CamshaftDeserializer::deserialize(const JsonValue& json, Crankshaft* crankshaft,
                                             const std::string& context) {
    const std::string ctx = context.empty() ? "camshaft" : context;

    const JsonValue& lobes = json["lobeCenterlines"];
    int lobeCount = lobes.isArray() ? static_cast<int>(lobes.size()) : 0;

    if (lobeCount == 0) {
        throw std::runtime_error("Missing or empty 'lobeCenterlines' in " + ctx);
    }

    if (!json.has("lobeProfile")) {
        throw std::runtime_error("Missing required field 'lobeProfile' in " + ctx);
    }
    Function* lobeProfile = FunctionDeserializer::deserialize(json["lobeProfile"], ctx + ".lobeProfile");

    Camshaft::Parameters params;
    params.lobes = lobeCount;
    params.advance = json["advance"].numberOr(0);
    params.baseRadius = json["baseRadius"].numberOr(0);
    params.crankshaft = crankshaft;
    params.lobeProfile = lobeProfile;

    Camshaft* cam = new Camshaft();
    cam->initialize(params);

    // JSON stores CRANK angles. setLobeCenterline() divides by 2 internally.
    // Pass the raw JSON value directly.
    for (size_t i = 0; i < lobes.size(); i++) {
        cam->setLobeCenterline(static_cast<int>(i), lobes[i].asNumber());
    }

    return cam;
}
