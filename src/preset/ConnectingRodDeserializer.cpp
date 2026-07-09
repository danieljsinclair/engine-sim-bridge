#include "preset/ConnectingRodDeserializer.h"

#include "connecting_rod.h"
#include "piston.h"
#include "common/PresetExceptions.h"

using json::JsonValue;

void ConnectingRodDeserializer::deserialize(const JsonValue& json, ConnectingRod* rod,
                                             Crankshaft* crankshaft, Piston* piston,
                                             int /*defaultJournal*/, const std::string& context) {
    const std::string ctx = context.empty() ? "connectingRod" : context;

    ConnectingRod::Parameters params;

    if (!json.has("mass")) {
        throw PresetDeserializationException("Missing required field 'mass' in " + ctx);
    }
    params.mass = json["mass"].asNumber();

    if (!json.has("momentOfInertia")) {
        throw PresetDeserializationException("Missing required field 'momentOfInertia' in " + ctx);
    }
    params.momentOfInertia = json["momentOfInertia"].asNumber();

    if (!json.has("centerOfMass")) {
        throw PresetDeserializationException("Missing required field 'centerOfMass' in " + ctx);
    }
    params.centerOfMass = json["centerOfMass"].asNumber();

    if (!json.has("length")) {
        throw PresetDeserializationException("Missing required field 'length' in " + ctx);
    }
    params.length = json["length"].asNumber();

    params.crankshaft = crankshaft;
    params.piston = piston;

    if (!json.has("journal")) {
        throw PresetDeserializationException("Missing required field 'journal' in " + ctx);
    }
    params.journal = json["journal"].asInt();

    if (!json.has("slaveThrow")) {
        throw PresetDeserializationException("Missing required field 'slaveThrow' in " + ctx);
    }
    params.slaveThrow = json["slaveThrow"].asNumber();

    rod->initialize(params);
}
