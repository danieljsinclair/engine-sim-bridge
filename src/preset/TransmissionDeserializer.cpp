#include "preset/TransmissionDeserializer.h"

#include "transmission.h"
#include "torque_converter.h"

#include <stdexcept>
#include <vector>

using json::JsonValue;

Transmission* TransmissionDeserializer::deserialize(const JsonValue& json, const std::string& context) {
    const std::string ctx = context.empty() ? "transmission" : context;

    if (!json.has("gearCount")) {
        throw std::runtime_error("Missing required field 'gearCount' in " + ctx);
    }

    int gearCount = json["gearCount"].asInt();
    if (gearCount <= 0) {
        throw std::runtime_error("Invalid 'gearCount' in " + ctx);
    }

    // Read gear ratios array
    std::vector<double> gearRatios;
    if (json.has("gearRatios") && json["gearRatios"].isArray()) {
        const JsonValue& ratiosJson = json["gearRatios"];
        for (size_t i = 0; i < ratiosJson.size(); i++) {
            gearRatios.push_back(ratiosJson[i].asNumber());
        }
    } else {
        throw std::runtime_error("Missing required field 'gearRatios' in " + ctx);
    }

    if (static_cast<int>(gearRatios.size()) != gearCount) {
        throw std::runtime_error("gearRatios count does not match gearCount in " + ctx);
    }

    if (!json.has("maxClutchTorque")) {
        throw std::runtime_error("Missing required field 'maxClutchTorque' in " + ctx);
    }
    double maxClutchTorque = json["maxClutchTorque"].asNumber();

    // Read optional torque converter parameters
    atg_scs::TorqueConverter::Parameters tcParams;
    bool hasTC = false;
    if (json.has("torqueConverter") && json["torqueConverter"].isObject()) {
        const JsonValue& tcJson = json["torqueConverter"];
        tcParams.stallTorqueRatio = tcJson.has("stallTorqueRatio")
            ? tcJson["stallTorqueRatio"].asNumber() : 2.0;
        tcParams.capacityFactor = tcJson.has("capacityFactor")
            ? tcJson["capacityFactor"].asNumber() : 1.8e-5;
        tcParams.lockupRpm = tcJson.has("lockupRpm")
            ? tcJson["lockupRpm"].asNumber() : 1500.0;
        tcParams.maxInputTorque = tcJson.has("maxInputTorque")
            ? tcJson["maxInputTorque"].asNumber() : maxClutchTorque;
        hasTC = true;
    }

    Transmission::Parameters params;
    params.GearCount = gearCount;
    params.GearRatios = gearRatios.data();
    params.MaxClutchTorque = maxClutchTorque;
    params.TorqueConverterParams = hasTC ? &tcParams : nullptr;

    Transmission* trans = new Transmission();
    trans->initialize(params);

    // clutchPressure is safe to set here -- it's a simple scalar with no dependencies.
    if (json.has("clutchPressure")) {
        trans->setClutchPressure(json["clutchPressure"].asNumber());
    }

    // NOTE: currentGear is also in the JSON but cannot be applied here.
    // changeGear() accesses m_vehicle->getMass() which is null at this point.
    // PresetDeserializer reads it into PresetLoadResult::initialGear, and
    // SimulatorFactory applies it after PistonEngineSimulator::loadSimulation()
    // wires the vehicle via addToSystem().

    return trans;
}
