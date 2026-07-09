#include "preset/TransmissionDeserializer.h"

#include "transmission.h"
#include "common/PresetExceptions.h"

#include <vector>
#include <memory>

using json::JsonValue;

Transmission* TransmissionDeserializer::deserialize(const JsonValue& json, const std::string& context) {
    const std::string ctx = context.empty() ? "transmission" : context;

    if (!json.has("gearCount")) {
        throw PresetDeserializationException("Missing required field 'gearCount' in " + ctx);
    }

    auto gearCount = json["gearCount"].asInt();
    if (gearCount <= 0) {
        throw PresetDeserializationException("Invalid 'gearCount' in " + ctx);
    }

    // Read gear ratios array
    std::vector<double> gearRatios;
    if (json.has("gearRatios") && json["gearRatios"].isArray()) {
        const JsonValue& ratiosJson = json["gearRatios"];
        for (auto i = 0u; i < ratiosJson.size(); i++) {
            gearRatios.push_back(ratiosJson[i].asNumber());
        }
    } else {
        throw PresetDeserializationException("Missing required field 'gearRatios' in " + ctx);
    }

    if (static_cast<int>(gearRatios.size()) != gearCount) {
        throw PresetDeserializationException("gearRatios count does not match gearCount in " + ctx);
    }

    if (!json.has("maxClutchTorque")) {
        throw PresetDeserializationException("Missing required field 'maxClutchTorque' in " + ctx);
    }
    auto maxClutchTorque = json["maxClutchTorque"].asNumber();

    Transmission::Parameters params;
    params.GearCount = gearCount;
    params.GearRatios = gearRatios.data();
    params.MaxClutchTorque = maxClutchTorque;

    auto trans = std::make_unique<Transmission>();
    trans->initialize(params);

    // clutchPressure is safe to set here -- it's a simple scalar with no dependencies.
    if (json.has("clutchPressure")) {
        trans->setClutchPressure(json["clutchPressure"].asNumber());
    }

    // NOTE: currentGear is also in the JSON but cannot be applied here.
    // changeGear() accesses m_vehicle->getMass() which is null at this point.
    // PresetDeserializer reads it into PresetLoadResult::initialGear, and
    // SimulatorFactory applies it after PistonEngineSimulator::loadSimulation()
    // wires the vehicle via addToSystem().

    return trans.release();
}
