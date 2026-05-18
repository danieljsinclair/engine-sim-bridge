#ifndef CRANKSHAFT_DESERIALIZER_H
#define CRANKSHAFT_DESERIALIZER_H

#include "common/JsonParser.h"

class Crankshaft;
class Engine;

class CrankshaftDeserializer {
public:
    static void deserialize(const json::JsonValue& json, Crankshaft* cs, const std::string& context = "");
};

#endif // CRANKSHAFT_DESERIALIZER_H
