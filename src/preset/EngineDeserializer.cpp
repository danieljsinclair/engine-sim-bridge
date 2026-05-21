#include "preset/EngineDeserializer.h"

#include "engine.h"
#include "crankshaft.h"
#include "cylinder_bank.h"
#include "piston.h"
#include "connecting_rod.h"
#include "combustion_chamber.h"
#include "fuel.h"
#include "units.h"
#include "direct_throttle_linkage.h"

#include "preset/CrankshaftDeserializer.h"
#include "preset/CylinderBankDeserializer.h"
#include "preset/ExhaustSystemDeserializer.h"
#include "preset/IntakeDeserializer.h"
#include "preset/FuelDeserializer.h"
#include "preset/IgnitionModuleDeserializer.h"
#include "preset/CylinderHeadDeserializer.h"
#include "preset/ConnectingRodDeserializer.h"
#include "simulator/EnginePresetsHelper.h"

#include <stdexcept>
#include <memory>

using json::JsonValue;

Engine::Parameters EngineDeserializer::readParams(const JsonValue& json, const std::string& ctx) {
    Engine::Parameters params;
    params.name = json["name"].stringOr("Unnamed Preset");
    params.cylinderBanks = json["cylinderBankCount"].intOr(0);
    params.cylinderCount = json["cylinderCount"].intOr(0);
    params.crankshaftCount = json["crankshaftCount"].intOr(0);
    params.exhaustSystemCount = json["exhaustSystemCount"].intOr(0);
    params.intakeCount = json["intakeCount"].intOr(0);
    params.starterTorque = json["starterTorque"].numberOr(0);
    params.starterSpeed = json["starterSpeed"].numberOr(0);
    params.redline = json["redline"].numberOr(0);
    params.dynoMinSpeed = json["dynoMinSpeed"].numberOr(0);
    params.dynoMaxSpeed = json["dynoMaxSpeed"].numberOr(0);
    params.dynoHoldStep = json["dynoHoldStep"].numberOr(0);

    auto requireField = [&](const char* field) {
        if (!json.has(field)) {
            throw std::runtime_error(std::string("Missing required field '") + field + "' in " + ctx);
        }
    };
    requireField("simulationFrequency");
    params.initialSimulationFrequency = json["simulationFrequency"].asNumber();
    requireField("initialHighFrequencyGain");
    params.initialHighFrequencyGain = json["initialHighFrequencyGain"].asNumber();
    requireField("initialNoise");
    params.initialNoise = json["initialNoise"].asNumber();
    requireField("initialJitter");
    params.initialJitter = json["initialJitter"].asNumber();

    return params;
}

void EngineDeserializer::validateRequiredSections(const JsonValue& json, const Engine::Parameters& params, const std::string& ctx) {
    if (params.cylinderCount <= 0 || params.cylinderBanks <= 0 || params.crankshaftCount <= 0) {
        throw std::runtime_error("Invalid engine configuration in " + ctx);
    }
    if (!json.has("crankshafts") || !json["crankshafts"].isArray()) {
        throw std::runtime_error("Missing required field 'crankshafts' in " + ctx);
    }
    if (!json.has("cylinderBanks") || !json["cylinderBanks"].isArray()) {
        throw std::runtime_error("Missing required field 'cylinderBanks' in " + ctx);
    }
    if (params.exhaustSystemCount > 0 &&
        (!json.has("exhaustSystems") || !json["exhaustSystems"].isArray())) {
        throw std::runtime_error("Missing required field 'exhaustSystems' in " + ctx);
    }
    if (params.intakeCount > 0 &&
        (!json.has("intakes") || !json["intakes"].isArray())) {
        throw std::runtime_error("Missing required field 'intakes' in " + ctx);
    }
}

void EngineDeserializer::deserializeCrankshafts(const JsonValue& json, Engine* engine, const std::string& ctx) {
    const JsonValue& arr = json["crankshafts"];
    for (size_t i = 0; i < arr.size() &&
         i < static_cast<size_t>(engine->getCrankshaftCount()); i++) {
        CrankshaftDeserializer::deserialize(
            arr[i], engine->getCrankshaft(static_cast<int>(i)),
            ctx + ".crankshafts[" + std::to_string(i) + "]");
    }
}

void EngineDeserializer::deserializeExhaustSystems(const JsonValue& json, Engine* engine, const std::string& ctx) {
    const JsonValue& arr = json["exhaustSystems"];
    if (!arr.isArray()) return;
    for (size_t i = 0; i < arr.size() &&
         i < static_cast<size_t>(engine->getExhaustSystemCount()); i++) {
        ExhaustSystemDeserializer::deserialize(
            arr[i], engine->getExhaustSystem(static_cast<int>(i)),
            ctx + ".exhaustSystems[" + std::to_string(i) + "]");
    }
}

void EngineDeserializer::deserializeIntakes(const JsonValue& json, Engine* engine, const std::string& ctx) {
    const JsonValue& arr = json["intakes"];
    if (!arr.isArray()) return;
    for (size_t i = 0; i < arr.size() &&
         i < static_cast<size_t>(engine->getIntakeCount()); i++) {
        IntakeDeserializer::deserialize(
            arr[i], engine->getIntake(static_cast<int>(i)),
            ctx + ".intakes[" + std::to_string(i) + "]");
    }
}

void EngineDeserializer::deserializeCylinders(const JsonValue& bankJson, CylinderBank* bank,
        Engine* engine, Crankshaft* mainCrank, int bankIdx,
        int globalCylIdx, int cylCount, const std::string& ctx, size_t bankIndex) {
    const JsonValue& cylinders = bankJson["cylinders"];
    if (!cylinders.isArray()) return;

    for (size_t ci = 0; ci < cylinders.size() &&
         ci < static_cast<size_t>(cylCount); ci++) {
        const JsonValue& cylJson = cylinders[ci];
        int pidx = globalCylIdx + static_cast<int>(ci);
        if (pidx >= engine->getCylinderCount()) break;

        ConnectingRod* rod = engine->getConnectingRod(pidx);
        int journalIdx = cylJson.has("rodJournalIndex")
            ? cylJson["rodJournalIndex"].asInt() : pidx;

        if (cylJson.has("connectingRod")) {
            ConnectingRodDeserializer::deserialize(
                cylJson["connectingRod"], rod, mainCrank,
                engine->getPiston(pidx), journalIdx,
                ctx + ".cylinderBanks[" + std::to_string(bankIndex) +
                "].cylinders[" + std::to_string(ci) + "].connectingRod");
        }

        Piston* piston = engine->getPiston(pidx);
        Piston::Parameters pParams;
        pParams.Bank = bank;
        pParams.Rod = rod;
        pParams.CylinderIndex = static_cast<int>(ci);
        pParams.BlowbyFlowCoefficient = cylJson["blowbyK"].numberOr(0);
        pParams.CompressionHeight = cylJson["compressionHeight"].numberOr(0);
        pParams.WristPinPosition = cylJson["wristPinPosition"].numberOr(0);
        pParams.Displacement = cylJson["displacement"].numberOr(0);
        pParams.mass = cylJson["pistonMass"].numberOr(0);
        piston->initialize(pParams);
    }
}

void EngineDeserializer::deserializeCylinderBanks(const JsonValue& json, Engine* engine, const std::string& ctx) {
    Crankshaft* mainCrank = engine->getCrankshaft(0);
    int globalCylIdx = 0;

    const JsonValue& banksJson = json["cylinderBanks"];
    for (size_t bi = 0; bi < banksJson.size() &&
         bi < static_cast<size_t>(engine->getCylinderBankCount()); bi++) {
        const JsonValue& bankJson = banksJson[bi];
        CylinderBank* bank = engine->getCylinderBank(static_cast<int>(bi));
        int bankIdx = static_cast<int>(bi);

        CylinderBankDeserializer::deserialize(
            bankJson, bank, mainCrank, bankIdx,
            ctx + ".cylinderBanks[" + std::to_string(bi) + "]");

        int cylCount = bank->getCylinderCount();
        deserializeCylinders(bankJson, bank, engine, mainCrank, bankIdx, globalCylIdx, cylCount, ctx, bi);

        if (bankJson.has("cylinderHead")) {
            CylinderHeadDeserializer::deserialize(
                bankJson["cylinderHead"], engine->getHead(bankIdx),
                bank, engine, bankIdx,
                ctx + ".cylinderBanks[" + std::to_string(bi) + "].cylinderHead");
        }

        globalCylIdx += cylCount;
    }
}

void EngineDeserializer::initializeCombustionChambers(Engine* engine) {
    Function* turbFn = EnginePresetsHelper::createMeanPistonSpeedToTurbulence();

    CombustionChamber::Parameters ccParams;
    ccParams.CrankcasePressure = units::pressure(1.0, units::atm);
    ccParams.Fuel = engine->getFuel();
    ccParams.StartingPressure = units::pressure(1.0, units::atm);
    ccParams.StartingTemperature = units::celcius(25.0);
    ccParams.MeanPistonSpeedToTurbulence = turbFn;

    for (int i = 0; i < engine->getCylinderCount(); i++) {
        ccParams.Piston = engine->getPiston(i);
        ccParams.Head = engine->getHead(
            ccParams.Piston->getCylinderBank()->getIndex());
        engine->getChamber(i)->initialize(ccParams);
        engine->getChamber(i)->setEngine(engine);
    }

    engine->calculateDisplacement();
}

Engine* EngineDeserializer::deserialize(const JsonValue& json, const std::string& context) {
    const std::string ctx = context.empty() ? "engine" : context;

    auto throttle = std::make_unique<DirectThrottleLinkage>();
    DirectThrottleLinkage::Parameters throttleParams;
    throttleParams.gamma = json["throttleGamma"].numberOr(1.0);
    throttle->initialize(throttleParams);

    Engine::Parameters params = readParams(json, ctx);
    params.throttle = throttle.get();
    validateRequiredSections(json, params, ctx);

    Engine* engine = new Engine();
    engine->initialize(params);
    throttle.release(); // Engine owns it now

    try {
        deserializeCrankshafts(json, engine, ctx);
        deserializeExhaustSystems(json, engine, ctx);
        deserializeIntakes(json, engine, ctx);
        deserializeCylinderBanks(json, engine, ctx);

        if (json.has("fuel")) {
            FuelDeserializer::deserialize(json["fuel"], engine->getFuel(), ctx + ".fuel");
        }

        if (json.has("ignitionModule")) {
            IgnitionModuleDeserializer::deserialize(
                json["ignitionModule"], engine->getIgnitionModule(),
                engine->getCrankshaft(0), engine->getCylinderCount(), ctx + ".ignitionModule");
        }

        initializeCombustionChambers(engine);
    } catch (...) {
        delete engine;
        throw;
    }

    return engine;
}
