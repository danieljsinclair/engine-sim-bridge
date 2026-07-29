#ifndef INTAKE_DESERIALIZER_H
#define INTAKE_DESERIALIZER_H

#include "common/JsonParser.h"

class Intake;

class IntakeDeserializer {
public:
    static void deserialize(const json::JsonValue& json, Intake* intake,
                             const std::string& context = "");
};

#endif // INTAKE_DESERIALIZER_H
