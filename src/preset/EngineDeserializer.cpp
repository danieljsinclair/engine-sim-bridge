#include "preset/EngineDeserializer.h"

#include "engine.h"
#include "crankshaft.h"
#include "cylinder_bank.h"
#include "piston.h"
#include "connecting_rod.h"
#include "combustion_chamber.h"
#include "direct_throttle_linkage.h"

#include "preset/CrankshaftDeserializer.h"
#include "preset/CylinderBankDeserializer.h"
#include "preset/ExhaustSystemDeserializer.h"
#include "preset/IntakeDeserializer.h"
#include "preset/FuelDeserializer.h"
#include "preset/IgnitionModuleDeserializer.h"
#include "preset/CylinderHeadDeserializer.h"
#include "preset/ConnectingRodDeserializer.h"
#include "preset/FunctionDeserializer.h"

#include <stdexcept>
#include <memory>

using json::JsonValue;

Engine::Parameters EngineDeserializer::readParams(const JsonValue& json, const std::string& ctx) {
    Engine::Parameters params;

    if (!json.has("name")) {
        throw std::runtime_error("Missing required field 'name' in " + ctx);
    }
    params.name = json["name"].asString();

    if (!json.has("cylinderBankCount")) {
        throw std::runtime_error("Missing required field 'cylinderBankCount' in " + ctx);
    }
    params.cylinderBanks = json["cylinderBankCount"].asInt();

    if (!json.has("cylinderCount")) {
        throw std::runtime_error("Missing required field 'cylinderCount' in " + ctx);
    }
    params.cylinderCount = json["cylinderCount"].asInt();

    if (!json.has("crankshaftCount")) {
        throw std::runtime_error("Missing required field 'crankshaftCount' in " + ctx);
    }
    params.crankshaftCount = json["crankshaftCount"].asInt();

    if (!json.has("exhaustSystemCount")) {
        throw std::runtime_error("Missing required field 'exhaustSystemCount' in " + ctx);
    }
    params.exhaustSystemCount = json["exhaustSystemCount"].asInt();

    if (!json.has("intakeCount")) {
        throw std::runtime_error("Missing required field 'intakeCount' in " + ctx);
    }
    params.intakeCount = json["intakeCount"].asInt();

    auto requireField = [&](const char* field) {
        if (!json.has(field)) {
            throw std::runtime_error(std::string("Missing required field '") + field + "' in " + ctx);
        }
    };

    requireField("starterTorque");
    params.starterTorque = json["starterTorque"].asNumber();
    requireField("starterSpeed");
    params.starterSpeed = json["starterSpeed"].asNumber();
    requireField("redline");
    params.redline = json["redline"].asNumber();
    requireField("dynoMinSpeed");
    params.dynoMinSpeed = json["dynoMinSpeed"].asNumber();
    requireField("dynoMaxSpeed");
    params.dynoMaxSpeed = json["dynoMaxSpeed"].asNumber();
    requireField("dynoHoldStep");
    params.dynoHoldStep = json["dynoHoldStep"].asNumber();
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

        const std::string cylCtx = ctx + ".cylinderBanks[" + std::to_string(bankIndex) +
                                   "].cylinders[" + std::to_string(ci) + "]";

        if (!cylJson.has("blowbyK")) {
            throw std::runtime_error("Missing required field 'blowbyK' in " + cylCtx);
        }
        pParams.BlowbyFlowCoefficient = cylJson["blowbyK"].asNumber();

        if (!cylJson.has("compressionHeight")) {
            throw std::runtime_error("Missing required field 'compressionHeight' in " + cylCtx);
        }
        pParams.CompressionHeight = cylJson["compressionHeight"].asNumber();

        if (!cylJson.has("wristPinPosition")) {
            throw std::runtime_error("Missing required field 'wristPinPosition' in " + cylCtx);
        }
        pParams.WristPinPosition = cylJson["wristPinPosition"].asNumber();

        if (!cylJson.has("displacement")) {
            throw std::runtime_error("Missing required field 'displacement' in " + cylCtx);
        }
        pParams.Displacement = cylJson["displacement"].asNumber();

        if (!cylJson.has("pistonMass")) {
            throw std::runtime_error("Missing required field 'pistonMass' in " + cylCtx);
        }
        pParams.mass = cylJson["pistonMass"].asNumber();

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
        auto bankIdx = static_cast<int>(bi);

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

void EngineDeserializer::initializeCombustionChambers(const JsonValue& json, Engine* engine, const std::string& ctx) {
    if (!json.has("combustionChambers") || !json["combustionChambers"].isArray()) {
        throw std::runtime_error("Missing required field 'combustionChambers' in " + ctx);
    }
    const JsonValue& chambersArr = json["combustionChambers"];

    for (int i = 0; i < engine->getCylinderCount() &&
         i < static_cast<int>(chambersArr.size()); i++) {
        const JsonValue& ccJson = chambersArr[static_cast<size_t>(i)];
        const std::string ccCtx = ctx + ".combustionChambers[" + std::to_string(i) + "]";

        // Deserialize turbulence function
        if (!ccJson.has("meanPistonSpeedToTurbulence")) {
            throw std::runtime_error("Missing required field 'meanPistonSpeedToTurbulence' in " + ccCtx);
        }
        Function* turbFn = FunctionDeserializer::deserialize(
            ccJson["meanPistonSpeedToTurbulence"], ccCtx + ".meanPistonSpeedToTurbulence");

        CombustionChamber::Parameters ccParams;
        ccParams.Fuel = engine->getFuel();
        ccParams.Piston = engine->getPiston(i);
        ccParams.Head = engine->getHead(
            ccParams.Piston->getCylinderBank()->getIndex());
        ccParams.MeanPistonSpeedToTurbulence = turbFn;

        if (!ccJson.has("crankcasePressure")) {
            throw std::runtime_error("Missing required field 'crankcasePressure' in " + ccCtx);
        }
        ccParams.CrankcasePressure = ccJson["crankcasePressure"].asNumber();

        if (!ccJson.has("startingPressure")) {
            throw std::runtime_error("Missing required field 'startingPressure' in " + ccCtx);
        }
        ccParams.StartingPressure = ccJson["startingPressure"].asNumber();

        if (!ccJson.has("startingTemperature")) {
            throw std::runtime_error("Missing required field 'startingTemperature' in " + ccCtx);
        }
        ccParams.StartingTemperature = ccJson["startingTemperature"].asNumber();

        engine->getChamber(i)->initialize(ccParams);
        engine->getChamber(i)->setEngine(engine);

        // Friction model (flat fields in the chamber object, applied after init)
        if (!ccJson.has("frictionCoeff")) {
            throw std::runtime_error("Missing required field 'frictionCoeff' in " + ccCtx);
        }
        engine->getChamber(i)->m_frictionModel.frictionCoeff = ccJson["frictionCoeff"].asNumber();

        if (!ccJson.has("breakawayFriction")) {
            throw std::runtime_error("Missing required field 'breakawayFriction' in " + ccCtx);
        }
        engine->getChamber(i)->m_frictionModel.breakawayFriction = ccJson["breakawayFriction"].asNumber();

        if (!ccJson.has("breakawayFrictionVelocity")) {
            throw std::runtime_error("Missing required field 'breakawayFrictionVelocity' in " + ccCtx);
        }
        engine->getChamber(i)->m_frictionModel.breakawayFrictionVelocity = ccJson["breakawayFrictionVelocity"].asNumber();

        if (!ccJson.has("viscousFrictionCoefficient")) {
            throw std::runtime_error("Missing required field 'viscousFrictionCoefficient' in " + ccCtx);
        }
        engine->getChamber(i)->m_frictionModel.viscousFrictionCoefficient = ccJson["viscousFrictionCoefficient"].asNumber();
    }

    engine->calculateDisplacement();
}

Engine* EngineDeserializer::deserialize(const JsonValue& json, const std::string& context) {
    const std::string ctx = context.empty() ? "engine" : context;

    auto throttle = std::make_unique<DirectThrottleLinkage>();
    DirectThrottleLinkage::Parameters throttleParams;
    if (!json.has("throttleGamma")) {
        throw std::runtime_error("Missing required field 'throttleGamma' in " + ctx);
    }
    throttleParams.gamma = json["throttleGamma"].asNumber();
    throttle->initialize(throttleParams);

    Engine::Parameters params = readParams(json, ctx);
    params.throttle = throttle.get();
    validateRequiredSections(json, params, ctx);

    auto engine = std::make_unique<Engine>();
    engine->initialize(params);
    throttle.release(); // Engine owns it now

    try {
        deserializeCrankshafts(json, engine.get(), ctx);
        deserializeExhaustSystems(json, engine.get(), ctx);
        deserializeIntakes(json, engine.get(), ctx);
        deserializeCylinderBanks(json, engine.get(), ctx);

        if (json.has("fuel")) {
            FuelDeserializer::deserialize(json["fuel"], engine->getFuel(), ctx + ".fuel");
        }

        if (json.has("ignitionModule")) {
            IgnitionModuleDeserializer::deserialize(
                json["ignitionModule"], engine->getIgnitionModule(),
                engine->getCrankshaft(0), engine->getCylinderCount(), ctx + ".ignitionModule");
        }

        initializeCombustionChambers(json, engine.get(), ctx);
    } catch (...) {
        throw;
    }

    return engine.release();
}
