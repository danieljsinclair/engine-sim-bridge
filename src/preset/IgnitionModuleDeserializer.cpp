#include "preset/IgnitionModuleDeserializer.h"

#include "ignition_module.h"
#include "preset/FunctionDeserializer.h"
#include "common/PresetExceptions.h"

using json::JsonValue;

void IgnitionModuleDeserializer::deserialize(const JsonValue& json, IgnitionModule* ignition,
                                              Crankshaft* crankshaft, int cylinderCount,
                                              const std::string& context) {
    const std::string ctx = context.empty() ? "ignitionModule" : context;

    IgnitionModule::Parameters params;
    params.cylinderCount = cylinderCount;
    params.crankshaft = crankshaft;

    // Rev limit: read from JSON, no computation
    if (json.has("revLimit")) {
        params.revLimit = json["revLimit"].asNumber();
    } else {
        throw PresetDeserializationException("Missing required field 'revLimit' in " + ctx);
    }

    // Limiter duration: read from JSON, no hardcoded default
    if (json.has("limiterDuration")) {
        params.limiterDuration = json["limiterDuration"].asNumber();
    } else {
        throw PresetDeserializationException("Missing required field 'limiterDuration' in " + ctx);
    }

    // Timing curve function
    if (!json.has("timingCurve")) {
        throw PresetDeserializationException("Missing required field 'timingCurve' in " + ctx);
    }
    params.timingCurve = FunctionDeserializer::deserialize(
        json["timingCurve"], ctx + ".timingCurve");

    ignition->initialize(params);

    // Firing order
    if (json.has("firingOrder") && json["firingOrder"].isArray()) {
        const JsonValue& fo = json["firingOrder"];
        for (size_t i = 0; i < fo.size() && static_cast<int>(i) < cylinderCount; i++) {
            ignition->setFiringOrder(static_cast<int>(i), fo[i].asNumber());
        }
    } else {
        throw PresetDeserializationException("Missing required field 'firingOrder' in " + ctx);
    }
}
