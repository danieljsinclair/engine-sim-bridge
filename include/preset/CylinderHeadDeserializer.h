#ifndef CYLINDER_HEAD_DESERIALIZER_H
#define CYLINDER_HEAD_DESERIALIZER_H

#include "common/JsonParser.h"

class CylinderHead;
class CylinderBank;
class Engine;

class CylinderHeadDeserializer {
public:
    static void deserialize(const json::JsonValue& json, CylinderHead* head,
                             CylinderBank* bank, Engine* engine, int bankIndex,
                             const std::string& context = "");
};

#endif // CYLINDER_HEAD_DESERIALIZER_H
