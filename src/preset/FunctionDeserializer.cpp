#include "preset/FunctionDeserializer.h"

#include <memory>

using json::JsonValue;

std::string FunctionDeserializer::fieldError(const std::string& field, const std::string& context) {
    return "Missing required field '" + field + "'" +
           (context.empty() ? "" : " in " + context);
}

Function* FunctionDeserializer::deserialize(const JsonValue& fnJson, const std::string& context) {
    if (!fnJson.isObject()) {
        throw std::runtime_error("Expected function object" +
            (context.empty() ? "" : " in " + context));
    }

    if (!fnJson.has("filterRadius")) {
        throw std::runtime_error(fieldError("filterRadius", context));
    }
    if (!fnJson.has("samples")) {
        throw std::runtime_error(fieldError("samples", context));
    }

    const JsonValue& samplesJson = fnJson["samples"];
    if (!samplesJson.isArray() || samplesJson.size() == 0) {
        throw std::runtime_error("Function 'samples' must be a non-empty array" +
            (context.empty() ? "" : " in " + context));
    }

    double filterRadius = fnJson["filterRadius"].asNumber();
    int n = static_cast<int>(samplesJson.size());

    auto fn = std::make_unique<Function>();
    fn->initialize(n + 2, filterRadius);

    if (fnJson.has("inputScale")) {
        fn->setInputScale(fnJson["inputScale"].asNumber());
    }
    if (fnJson.has("outputScale")) {
        fn->setOutputScale(fnJson["outputScale"].asNumber());
    }

    for (size_t i = 0; i < samplesJson.size(); i++) {
        const JsonValue& point = samplesJson[i];
        if (point.isArray() && point.size() >= 2) {
            fn->addSample(point[static_cast<size_t>(0)].asNumber(),
                          point[static_cast<size_t>(1)].asNumber());
        }
    }

    return fn.release();
}
