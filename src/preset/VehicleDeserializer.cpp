#include "preset/VehicleDeserializer.h"

#include "vehicle.h"
#include "common/PresetExceptions.h"

#include <memory>

using json::JsonValue;

Vehicle* VehicleDeserializer::deserialize(const JsonValue& json, const std::string& context) {
    const std::string ctx = context.empty() ? "vehicle" : context;

    Vehicle::Parameters params;

    if (!json.has("mass")) {
        throw PresetDeserializationException("Missing required field 'mass' in " + ctx);
    }
    params.mass = json["mass"].asNumber();

    if (!json.has("dragCoefficient")) {
        throw PresetDeserializationException("Missing required field 'dragCoefficient' in " + ctx);
    }
    params.dragCoefficient = json["dragCoefficient"].asNumber();

    if (!json.has("crossSectionArea")) {
        throw PresetDeserializationException("Missing required field 'crossSectionArea' in " + ctx);
    }
    params.crossSectionArea = json["crossSectionArea"].asNumber();

    if (!json.has("diffRatio")) {
        throw PresetDeserializationException("Missing required field 'diffRatio' in " + ctx);
    }
    params.diffRatio = json["diffRatio"].asNumber();

    if (!json.has("tireRadius")) {
        throw PresetDeserializationException("Missing required field 'tireRadius' in " + ctx);
    }
    params.tireRadius = json["tireRadius"].asNumber();

    if (!json.has("rollingResistance")) {
        throw PresetDeserializationException("Missing required field 'rollingResistance' in " + ctx);
    }
    params.rollingResistance = json["rollingResistance"].asNumber();

    auto vehicle = std::make_unique<Vehicle>();
    vehicle->initialize(params);
    return vehicle.release();
}
