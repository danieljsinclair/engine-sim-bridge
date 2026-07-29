#ifndef EXHAUST_SYSTEM_DESERIALIZER_H
#define EXHAUST_SYSTEM_DESERIALIZER_H

#include "common/JsonParser.h"

class ExhaustSystem;

class ExhaustSystemDeserializer {
public:
    static void deserialize(const json::JsonValue& json, ExhaustSystem* es,
                             const std::string& context = "");
};

#endif // EXHAUST_SYSTEM_DESERIALIZER_H
