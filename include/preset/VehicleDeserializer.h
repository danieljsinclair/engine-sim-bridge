#ifndef VEHICLE_DESERIALIZER_H
#define VEHICLE_DESERIALIZER_H

#include "common/JsonParser.h"

class Vehicle;

class VehicleDeserializer {
public:
    static Vehicle* deserialize(const json::JsonValue& json, const std::string& context = "");
};

#endif // VEHICLE_DESERIALIZER_H
