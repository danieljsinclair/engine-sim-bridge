#ifndef CYLINDER_BANK_DESERIALIZER_H
#define CYLINDER_BANK_DESERIALIZER_H

#include "common/JsonParser.h"

class CylinderBank;
class Crankshaft;

class CylinderBankDeserializer {
public:
    static void deserialize(const json::JsonValue& json, CylinderBank* bank,
                             Crankshaft* crankshaft, int /*bankIndex*/,
                             const std::string& context = "");
};

#endif // CYLINDER_BANK_DESERIALIZER_H
