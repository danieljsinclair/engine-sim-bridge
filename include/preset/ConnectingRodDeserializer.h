#ifndef CONNECTING_ROD_DESERIALIZER_H
#define CONNECTING_ROD_DESERIALIZER_H

#include "common/JsonParser.h"

class ConnectingRod;
class Crankshaft;
class Piston;

class ConnectingRodDeserializer {
public:
    static void deserialize(const json::JsonValue& json, ConnectingRod* rod,
                            Crankshaft* crankshaft, Piston* piston, int /*defaultJournal*/,
                            const std::string& context = "");
};

#endif // CONNECTING_ROD_DESERIALIZER_H
