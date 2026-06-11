#include "preset/VehicleDeserializer.h"

#include "vehicle.h"

#include <stdexcept>
#include <memory>

using json::JsonValue;

Vehicle* VehicleDeserializer::deserialize(const JsonValue& json, const std::string& context) {
    const std::string ctx = context.empty() ? "vehicle" : context;

    Vehicle::Parameters params;

    if (!json.has("mass")) {
        throw std::runtime_error("Missing required field 'mass' in " + ctx);
    }
    params.mass = json["mass"].asNumber();

    if (!json.has("dragCoefficient")) {
        throw std::runtime_error("Missing required field 'dragCoefficient' in " + ctx);
    }
    params.dragCoefficient = json["dragCoefficient"].asNumber();

    if (!json.has("crossSectionArea")) {
        throw std::runtime_error("Missing required field 'crossSectionArea' in " + ctx);
    }
    params.crossSectionArea = json["crossSectionArea"].asNumber();

    if (!json.has("diffRatio")) {
        throw std::runtime_error("Missing required field 'diffRatio' in " + ctx);
    }
    params.diffRatio = json["diffRatio"].asNumber();

    if (!json.has("tireRadius")) {
        throw std::runtime_error("Missing required field 'tireRadius' in " + ctx);
    }
    params.tireRadius = json["tireRadius"].asNumber();

    if (!json.has("rollingResistance")) {
        throw std::runtime_error("Missing required field 'rollingResistance' in " + ctx);
    }
    params.rollingResistance = json["rollingResistance"].asNumber();

    auto vehicle = std::make_unique<Vehicle>();
    vehicle->initialize(params);
    return vehicle.release();
}
