#include "preset/ConnectingRodDeserializer.h"

#include "connecting_rod.h"
#include "piston.h"

#include <stdexcept>

using json::JsonValue;

void ConnectingRodDeserializer::deserialize(const JsonValue& json, ConnectingRod* rod,
                                             Crankshaft* crankshaft, Piston* piston,
                                             int defaultJournal, const std::string& context) {
    const std::string ctx = context.empty() ? "connectingRod" : context;

    ConnectingRod::Parameters params;
    params.mass = json["mass"].numberOr(0);
    params.momentOfInertia = json["momentOfInertia"].numberOr(0);
    params.centerOfMass = json["centerOfMass"].numberOr(0);
    params.length = json["length"].numberOr(0);
    params.crankshaft = crankshaft;
    params.piston = piston;
    params.journal = json.has("journal")
        ? json["journal"].asInt() : defaultJournal;
    params.slaveThrow = json["slaveThrow"].numberOr(0);

    rod->initialize(params);
}
