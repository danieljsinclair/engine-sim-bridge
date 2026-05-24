#include "preset/CrankshaftDeserializer.h"

#include "crankshaft.h"

#include <stdexcept>

using json::JsonValue;

void CrankshaftDeserializer::deserialize(const JsonValue& json, Crankshaft* cs, const std::string& context) {
    const std::string ctx = context.empty() ? "crankshaft" : context;

    auto require = [&](const char* field) {
        if (!json.has(field)) {
            throw std::runtime_error(std::string("Missing required field '") + field + "' in " + ctx);
        }
    };

    require("mass");
    require("momentOfInertia");
    require("crankThrow");

    Crankshaft::Parameters params;
    params.mass = json["mass"].asNumber();

    if (!json.has("flywheelMass")) {
        throw std::runtime_error("Missing required field 'flywheelMass' in " + ctx);
    }
    params.flywheelMass = json["flywheelMass"].asNumber();

    params.momentOfInertia = json["momentOfInertia"].asNumber();
    params.crankThrow = json["crankThrow"].asNumber();

    if (!json.has("posX")) {
        throw std::runtime_error("Missing required field 'posX' in " + ctx);
    }
    params.pos_x = json["posX"].asNumber();

    if (!json.has("posY")) {
        throw std::runtime_error("Missing required field 'posY' in " + ctx);
    }
    params.pos_y = json["posY"].asNumber();

    if (!json.has("tdc")) {
        throw std::runtime_error("Missing required field 'tdc' in " + ctx);
    }
    params.tdc = json["tdc"].asNumber();

    if (!json.has("frictionTorque")) {
        throw std::runtime_error("Missing required field 'frictionTorque' in " + ctx);
    }
    params.frictionTorque = json["frictionTorque"].asNumber();

    const JsonValue& journals = json["rodJournals"];
    params.rodJournals = journals.isArray() ? static_cast<int>(journals.size()) : 0;

    cs->initialize(params);

    if (journals.isArray()) {
        for (size_t j = 0; j < journals.size(); j++) {
            double angle = journals[j].has("angle")
                ? journals[j]["angle"].asNumber() : 0.0;
            cs->setRodJournalAngle(static_cast<int>(j), angle);
        }
    }
}
