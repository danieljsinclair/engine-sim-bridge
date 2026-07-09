#include "preset/FuelDeserializer.h"

#include "fuel.h"
#include "preset/FunctionDeserializer.h"
#include "common/PresetExceptions.h"

using json::JsonValue;

void FuelDeserializer::deserialize(const JsonValue& json, Fuel* fuel, const std::string& context) {
    const std::string ctx = context.empty() ? "fuel" : context;

    if (!json.has("turbulenceToFlameSpeedRatio")) {
        throw PresetDeserializationException("Missing required field 'turbulenceToFlameSpeedRatio' in " + ctx);
    }
    Function* turbulenceFn = FunctionDeserializer::deserialize(
        json["turbulenceToFlameSpeedRatio"], ctx + ".turbulenceToFlameSpeedRatio");

    auto require = [&](const char* field) {
        if (!json.has(field)) {
            throw PresetDeserializationException(std::string("Missing required field '") + field + "' in " + ctx);
        }
    };

    require("maxBurningEfficiency");
    require("burningEfficiencyRandomness");
    require("lowEfficiencyAttenuation");
    require("maxTurbulenceEffect");
    require("maxDilutionEffect");
    require("molecularAfr");
    require("molecularMass");
    require("energyDensity");
    require("density");

    Fuel::Parameters params;
    params.maxBurningEfficiency = json["maxBurningEfficiency"].asNumber();
    params.burningEfficiencyRandomness = json["burningEfficiencyRandomness"].asNumber();
    params.lowEfficiencyAttenuation = json["lowEfficiencyAttenuation"].asNumber();
    params.maxTurbulenceEffect = json["maxTurbulenceEffect"].asNumber();
    params.maxDilutionEffect = json["maxDilutionEffect"].asNumber();
    params.molecularAfr = json["molecularAfr"].asNumber();
    params.molecularMass = json["molecularMass"].asNumber();
    params.energyDensity = json["energyDensity"].asNumber();
    params.density = json["density"].asNumber();
    params.turbulenceToFlameSpeedRatio = turbulenceFn;

    fuel->initialize(params);
}
