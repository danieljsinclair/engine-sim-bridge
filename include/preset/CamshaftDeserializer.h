#ifndef CAMSHAFT_DESERIALIZER_H
#define CAMSHAFT_DESERIALIZER_H

#include "common/JsonParser.h"

class Camshaft;
class Crankshaft;

class CamshaftDeserializer {
public:
    static Camshaft* deserialize(const json::JsonValue& json, Crankshaft* crankshaft,
                                  const std::string& context = "");
};

#endif // CAMSHAFT_DESERIALIZER_H
