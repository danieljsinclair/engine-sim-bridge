#include "preset/IntakeDeserializer.h"

#include "intake.h"
#include "gas_system.h"
#include "units.h"

#include <stdexcept>

using json::JsonValue;

void IntakeDeserializer::deserialize(const JsonValue& json, Intake* intake,
                                      const std::string& context) {
    const std::string ctx = context.empty() ? "intake" : context;

    Intake::Parameters params;
    params.CrossSectionArea = json["plenumCrossSectionArea"].numberOr(0);
    params.RunnerFlowRate = json["runnerFlowRate"].numberOr(0);
    params.RunnerLength = json["runnerLength"].numberOr(0);
    if (!json.has("velocityDecay")) {
        throw std::runtime_error("Missing required field 'velocityDecay' in " + ctx);
    }
    params.VelocityDecay = json["velocityDecay"].asNumber();

    if (!json.has("idleThrottlePlatePosition")) {
        throw std::runtime_error("Missing required field 'idleThrottlePlatePosition' in " + ctx);
    }
    params.IdleThrottlePlatePosition = json["idleThrottlePlatePosition"].asNumber();

    if (!json.has("idleFlowK") && !json.has("idleFlowRate")) {
        throw std::runtime_error("Missing required field 'idleFlowK' (or 'idleFlowRate') in " + ctx);
    }
    params.IdleFlowK = json.has("idleFlowK")
        ? json["idleFlowK"].asNumber()
        : json["idleFlowRate"].asNumber();

    // InputFlowK: required field for engine to breathe
    if (json.has("inputFlowK")) {
        params.InputFlowK = json["inputFlowK"].asNumber();
    } else if (json.has("intakeFlowRate")) {
        params.InputFlowK = json["intakeFlowRate"].asNumber();
    } else {
        throw std::runtime_error(
            "Missing required field 'inputFlowK' (or 'intakeFlowRate') in " + ctx);
    }

    // Plenum volume
    if (json.has("plenumVolume")) {
        params.volume = json["plenumVolume"].numberOr(0);
    } else {
        throw std::runtime_error("Missing required field 'plenumVolume' in " + ctx);
    }

    intake->initialize(params);

    // Initialize manifold with air mixture (21% O2, 79% N2).
    GasSystem::Mix airMix;
    airMix.p_fuel = 0.0;
    airMix.p_inert = 0.79;
    airMix.p_o2 = 0.21;
    intake->m_system.reset(units::pressure(1.0, units::atm),
                           units::celcius(25.0), airMix);
}
