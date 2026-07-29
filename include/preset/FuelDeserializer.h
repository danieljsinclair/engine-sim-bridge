#ifndef FUEL_DESERIALIZER_H
#define FUEL_DESERIALIZER_H

#include "common/JsonParser.h"

class Fuel;

class FuelDeserializer {
public:
    static void deserialize(const json::JsonValue& json, Fuel* fuel,
                             const std::string& context = "");
};

#endif // FUEL_DESERIALIZER_H
