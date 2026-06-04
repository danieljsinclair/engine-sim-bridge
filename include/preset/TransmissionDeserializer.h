#ifndef TRANSMISSION_DESERIALIZER_H
#define TRANSMISSION_DESERIALIZER_H

#include "common/JsonParser.h"

class Transmission;

class TransmissionDeserializer {
public:
    static Transmission* deserialize(const json::JsonValue& json, const std::string& context = "");
};

#endif // TRANSMISSION_DESERIALIZER_H
