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
    params.flywheelMass = json["flywheelMass"].numberOr(0);
    params.momentOfInertia = json["momentOfInertia"].asNumber();
    params.crankThrow = json["crankThrow"].asNumber();
    params.pos_x = json["posX"].numberOr(0);
    params.pos_y = json["posY"].numberOr(0);
    params.tdc = json["tdc"].numberOr(0);
    params.frictionTorque = json["frictionTorque"].numberOr(0);

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
