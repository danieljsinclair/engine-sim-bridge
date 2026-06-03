#ifndef IGNITION_MODULE_DESERIALIZER_H
#define IGNITION_MODULE_DESERIALIZER_H

#include "common/JsonParser.h"

class IgnitionModule;
class Crankshaft;
class Engine;

class IgnitionModuleDeserializer {
public:
    static void deserialize(const json::JsonValue& json, IgnitionModule* ignition,
                             Crankshaft* crankshaft, int cylinderCount,
                             const std::string& context = "");
};

#endif // IGNITION_MODULE_DESERIALIZER_H
