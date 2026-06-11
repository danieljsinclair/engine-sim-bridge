#include "preset/IntakeDeserializer.h"

#include "intake.h"
#include "gas_system.h"
#include "units.h"
#include "common/PresetExceptions.h"

using json::JsonValue;

void IntakeDeserializer::deserialize(const JsonValue& json, Intake* intake,
                                      const std::string& context) {
    const std::string ctx = context.empty() ? "intake" : context;

    Intake::Parameters params;

    if (!json.has("plenumCrossSectionArea")) {
        throw PresetDeserializationException("Missing required field 'plenumCrossSectionArea' in " + ctx);
    }
    params.CrossSectionArea = json["plenumCrossSectionArea"].asNumber();

    if (!json.has("runnerFlowRate")) {
        throw PresetDeserializationException("Missing required field 'runnerFlowRate' in " + ctx);
    }
    params.RunnerFlowRate = json["runnerFlowRate"].asNumber();

    if (!json.has("runnerLength")) {
        throw PresetDeserializationException("Missing required field 'runnerLength' in " + ctx);
    }
    params.RunnerLength = json["runnerLength"].asNumber();
    if (!json.has("velocityDecay")) {
        throw PresetDeserializationException("Missing required field 'velocityDecay' in " + ctx);
    }
    params.VelocityDecay = json["velocityDecay"].asNumber();

    if (!json.has("idleThrottlePlatePosition")) {
        throw PresetDeserializationException("Missing required field 'idleThrottlePlatePosition' in " + ctx);
    }
    params.IdleThrottlePlatePosition = json["idleThrottlePlatePosition"].asNumber();

    if (!json.has("idleFlowK") && !json.has("idleFlowRate")) {
        throw PresetDeserializationException("Missing required field 'idleFlowK' (or 'idleFlowRate') in " + ctx);
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
        throw PresetDeserializationException(
            "Missing required field 'inputFlowK' (or 'intakeFlowRate') in " + ctx);
    }

    // Plenum volume
    if (!json.has("plenumVolume")) {
        throw PresetDeserializationException("Missing required field 'plenumVolume' in " + ctx);
    }
    params.volume = json["plenumVolume"].asNumber();

    // molecularAfr: required field
    if (!json.has("molecularAfr")) {
        throw PresetDeserializationException("Missing required field 'molecularAfr' in " + ctx);
    }
    params.MolecularAfr = json["molecularAfr"].asNumber();

    intake->initialize(params);

    // Initialize manifold with air mixture (21% O2, 79% N2).
    GasSystem::Mix airMix;
    airMix.p_fuel = 0.0;
    airMix.p_inert = 0.79;
    airMix.p_o2 = 0.21;
    intake->m_system.reset(units::pressure(1.0, units::atm),
                           units::celcius(25.0), airMix);
}
