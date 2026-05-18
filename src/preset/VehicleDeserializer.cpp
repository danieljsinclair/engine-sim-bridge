#include "preset/VehicleDeserializer.h"

#include "vehicle.h"

#include <stdexcept>

using json::JsonValue;

Vehicle* VehicleDeserializer::deserialize(const JsonValue& json, const std::string& context) {
    const std::string ctx = context.empty() ? "vehicle" : context;

    Vehicle::Parameters params;
    params.mass = json["mass"].numberOr(0);
    params.dragCoefficient = json["dragCoefficient"].numberOr(0);
    params.crossSectionArea = json["crossSectionArea"].numberOr(0);
    params.diffRatio = json["diffRatio"].numberOr(0);
    params.tireRadius = json["tireRadius"].numberOr(0);
    params.rollingResistance = json["rollingResistance"].numberOr(0);

    Vehicle* vehicle = new Vehicle();
    vehicle->initialize(params);
    return vehicle;
}
