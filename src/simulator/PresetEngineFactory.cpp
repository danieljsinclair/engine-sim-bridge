// PresetEngineFactory.cpp - Constructs Engine/Vehicle/Transmission from JSON presets
// Reads JSON files produced by the macOS preset compiler
// No Piranha dependency -- works on iOS and all platforms

#include "simulator/PresetEngineFactory.h"
#include "common/JsonParser.h"

// Engine-sim headers
#include "engine.h"
#include "vehicle.h"
#include "transmission.h"
#include "crankshaft.h"
#include "cylinder_bank.h"
#include "piston.h"
#include "connecting_rod.h"
#include "exhaust_system.h"
#include "intake.h"
#include "cylinder_head.h"
#include "camshaft.h"
#include "ignition_module.h"
#include "combustion_chamber.h"
#include "fuel.h"
#include "function.h"
#include "units.h"
#include "throttle.h"
#include "direct_throttle_linkage.h"
#include "standard_valvetrain.h"

#include <fstream>
#include <sstream>
#include <cstdio>

using json::JsonValue;

// ============================================================================
// Helper: Reconstruct a Function from JSON array of [x, y] samples
// ============================================================================
static Function* reconstructFunction(const JsonValue& samples) {
    if (!samples.isArray() || samples.size() == 0) return nullptr;

    Function* fn = new Function;
    int n = static_cast<int>(samples.size());
    fn->initialize(n + 2, 1);

    for (size_t i = 0; i < samples.size(); i++) {
        const JsonValue& point = samples[i];
        if (point.isArray() && point.size() >= 2) {
            fn->addSample(point[static_cast<size_t>(0)].asNumber(),
                          point[static_cast<size_t>(1)].asNumber());
        }
    }

    return fn;
}

// ============================================================================
// PresetEngineFactory implementation
// ============================================================================

PresetLoadResult PresetEngineFactory::loadFromFile(const std::string& jsonPath) {
    std::ifstream file(jsonPath);
    if (!file.is_open()) {
        PresetLoadResult result;
        result.error = "Cannot open preset file: " + jsonPath;
        return result;
    }

    std::ostringstream ss;
    ss << file.rdbuf();
    return loadFromString(ss.str(), jsonPath);
}

PresetLoadResult PresetEngineFactory::loadFromString(const std::string& jsonContent, const std::string& sourceName) {
    PresetLoadResult result;

    try {
        JsonValue root = json::parse(jsonContent);

        if (root.has("presetName")) {
            result.presetName = root["presetName"].asString();
        } else {
            // Extract filename stem
            size_t lastSlash = sourceName.find_last_of('/');
            size_t lastDot = sourceName.find_last_of('.');
            size_t start = (lastSlash != std::string::npos) ? lastSlash + 1 : 0;
            size_t end = (lastDot > start) ? lastDot : std::string::npos;
            result.presetName = sourceName.substr(start, end - start);
        }

        if (!root.has("engine") || !root["engine"].isObject()) {
            result.error = "Missing or invalid 'engine' section in preset";
            return result;
        }

        const JsonValue& engineJson = root["engine"];

        // --- Construct Engine ---
        Engine::Parameters params;
        params.name = engineJson["name"].stringOr("Unnamed Preset");
        params.cylinderBanks = engineJson["cylinderBankCount"].intOr(0);
        params.cylinderCount = engineJson["cylinderCount"].intOr(0);
        params.crankshaftCount = engineJson["crankshaftCount"].intOr(0);
        params.exhaustSystemCount = engineJson["exhaustSystemCount"].intOr(0);
        params.intakeCount = engineJson["intakeCount"].intOr(0);
        params.starterTorque = engineJson["starterTorque"].numberOr(0);
        params.starterSpeed = engineJson["starterSpeed"].numberOr(0);
        params.redline = engineJson["redline"].numberOr(0);
        params.dynoMinSpeed = engineJson["dynoMinSpeed"].numberOr(0);
        params.dynoMaxSpeed = engineJson["dynoMaxSpeed"].numberOr(0);
        params.dynoHoldStep = engineJson["dynoHoldStep"].numberOr(0);
        params.initialSimulationFrequency = engineJson["simulationFrequency"].numberOr(10000);
        params.initialHighFrequencyGain = engineJson["initialHighFrequencyGain"].numberOr(0.01);
        params.initialNoise = engineJson["initialNoise"].numberOr(1.0);
        params.initialJitter = engineJson["initialJitter"].numberOr(0.5);

        Throttle* throttle = new DirectThrottleLinkage();
        params.throttle = throttle;

        if (params.cylinderCount <= 0 || params.cylinderBanks <= 0 || params.crankshaftCount <= 0) {
            result.error = "Invalid engine configuration";
            delete throttle;
            return result;
        }

        Engine* engine = new Engine();
        engine->initialize(params);

        // --- Crankshafts ---
        const JsonValue& crankshaftsJson = engineJson["crankshafts"];
        if (crankshaftsJson.isArray()) {
            for (size_t i = 0; i < crankshaftsJson.size() && i < static_cast<size_t>(engine->getCrankshaftCount()); i++) {
                const JsonValue& csJson = crankshaftsJson[i];
                Crankshaft* cs = engine->getCrankshaft(static_cast<int>(i));

                const JsonValue& journals = csJson["rodJournals"];
                int journalCount = journals.isArray() ? static_cast<int>(journals.size()) : 0;

                Crankshaft::Parameters csParams;
                csParams.mass = csJson["mass"].numberOr(0);
                csParams.flywheelMass = csJson["flywheelMass"].numberOr(0);
                csParams.momentOfInertia = csJson["momentOfInertia"].numberOr(0);
                csParams.crankThrow = csJson["crankThrow"].numberOr(0);
                csParams.pos_x = csJson["posX"].numberOr(0);
                csParams.pos_y = csJson["posY"].numberOr(0);
                csParams.tdc = csJson["tdc"].numberOr(0);
                csParams.frictionTorque = csJson["frictionTorque"].numberOr(0);
                csParams.rodJournals = journalCount;

                cs->initialize(csParams);

                // Set rod journal angles
                if (journals.isArray()) {
                    for (size_t j = 0; j < journals.size(); j++) {
                        cs->setRodJournalAngle(static_cast<int>(j),
                            journals[j].has("angle") ? journals[j]["angle"].asNumber() : 0.0);
                    }
                }
            }
        }

        // --- Exhaust systems ---
        const JsonValue& exhaustsJson = engineJson["exhaustSystems"];
        if (exhaustsJson.isArray()) {
            for (size_t i = 0; i < exhaustsJson.size() && i < static_cast<size_t>(engine->getExhaustSystemCount()); i++) {
                const JsonValue& exJson = exhaustsJson[i];
                ExhaustSystem* ex = engine->getExhaustSystem(static_cast<int>(i));

                ExhaustSystem::Parameters exParams;
                exParams.length = exJson["length"].numberOr(0);
                exParams.collectorCrossSectionArea = exJson["collectorCrossSectionArea"].numberOr(0);
                exParams.outletFlowRate = exJson["outletFlowRate"].numberOr(0);
                exParams.primaryTubeLength = exJson["primaryTubeLength"].numberOr(0);
                exParams.primaryFlowRate = exJson["primaryFlowRate"].numberOr(0);
                exParams.velocityDecay = exJson["velocityDecay"].numberOr(0);
                exParams.audioVolume = exJson["audioVolume"].numberOr(1.0);
                exParams.impulseResponse = nullptr;
                ex->initialize(exParams);
            }
        }

        // --- Intakes ---
        const JsonValue& intakesJson = engineJson["intakes"];
        if (intakesJson.isArray()) {
            for (size_t i = 0; i < intakesJson.size() && i < static_cast<size_t>(engine->getIntakeCount()); i++) {
                const JsonValue& inJson = intakesJson[i];
                Intake* intake = engine->getIntake(static_cast<int>(i));

                Intake::Parameters inParams;
                inParams.volume = 0;  // Volume derived from CrossSectionArea * runner length
                inParams.CrossSectionArea = inJson["plenumCrossSectionArea"].numberOr(0);
                inParams.RunnerFlowRate = inJson["runnerFlowRate"].numberOr(0);
                inParams.RunnerLength = inJson["runnerLength"].numberOr(0);
                inParams.VelocityDecay = inJson["velocityDecay"].numberOr(0.5);
                inParams.InputFlowK = 0;
                intake->initialize(inParams);
            }
        }

        // --- Cylinder banks, pistons, rods, heads ---
        Crankshaft* mainCrank = engine->getCrankshaft(0);
        int globalCylIdx = 0;

        const JsonValue& banksJson = engineJson["cylinderBanks"];
        if (banksJson.isArray()) {
            for (size_t bi = 0; bi < banksJson.size() && bi < static_cast<size_t>(engine->getCylinderBankCount()); bi++) {
                const JsonValue& bankJson = banksJson[bi];
                CylinderBank* bank = engine->getCylinderBank(static_cast<int>(bi));

                int cylCount = bankJson["cylinderCount"].intOr(0);

                CylinderBank::Parameters bankParams;
                bankParams.crankshaft = mainCrank;
                bankParams.angle = bankJson["angle"].numberOr(0);
                bankParams.bore = bankJson["bore"].numberOr(0);
                bankParams.deckHeight = bankJson["deckHeight"].numberOr(0);
                bankParams.displayDepth = bankJson["displayDepth"].numberOr(0);
                bankParams.index = bankJson["index"].intOr(static_cast<int>(bi));
                bankParams.cylinderCount = cylCount;
                bankParams.positionX = bankJson["positionX"].numberOr(0);
                bankParams.positionY = bankJson["positionY"].numberOr(0);
                bank->initialize(bankParams);

                // Per-cylinder: pistons and connecting rods
                const JsonValue& cylinders = bankJson["cylinders"];
                if (cylinders.isArray()) {
                    for (size_t ci = 0; ci < cylinders.size() && ci < static_cast<size_t>(cylCount); ci++) {
                        const JsonValue& cylJson = cylinders[ci];
                        int pidx = globalCylIdx + static_cast<int>(ci);

                        if (pidx >= engine->getCylinderCount()) break;

                        // Connecting rod (must init before piston -- piston references rod)
                        ConnectingRod* rod = engine->getConnectingRod(pidx);
                        const JsonValue& rodJson = cylJson["connectingRod"];

                        ConnectingRod::Parameters rParams;
                        rParams.mass = rodJson["mass"].numberOr(0);
                        rParams.momentOfInertia = rodJson["momentOfInertia"].numberOr(0);
                        rParams.centerOfMass = rodJson["centerOfMass"].numberOr(0);
                        rParams.length = rodJson["length"].numberOr(0);
                        rParams.crankshaft = mainCrank;
                        rParams.journal = cylJson.has("rodJournalIndex")
                            ? cylJson["rodJournalIndex"].intOr(pidx) : pidx;
                        rParams.piston = engine->getPiston(pidx);
                        rod->initialize(rParams);

                        // Piston
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

                // Cylinder head
                if (bankJson.has("cylinderHead")) {
                    const JsonValue& headJson = bankJson["cylinderHead"];
                    CylinderHead* head = engine->getHead(static_cast<int>(bi));

                    // Build camshafts first (valvetrain needs them at init time)
                    Camshaft* intakeCam = nullptr;
                    Camshaft* exhaustCam = nullptr;

                    if (headJson.has("intakeCamshaft")) {
                        const JsonValue& camJson = headJson["intakeCamshaft"];
                        const JsonValue& lobes = camJson["lobeCenterlines"];
                        int lobeCount = lobes.isArray() ? static_cast<int>(lobes.size()) : 0;

                        Camshaft::Parameters camParams;
                        camParams.lobes = lobeCount;
                        camParams.advance = camJson["advance"].numberOr(0);
                        camParams.baseRadius = camJson["baseRadius"].numberOr(0);
                        camParams.crankshaft = mainCrank;
                        camParams.lobeProfile = reconstructFunction(camJson["lobeProfileSamples"]);

                        intakeCam = new Camshaft();
                        intakeCam->initialize(camParams);
                        if (lobes.isArray()) {
                            for (size_t li = 0; li < lobes.size(); li++) {
                                intakeCam->setLobeCenterline(static_cast<int>(li), lobes[li].asNumber());
                            }
                        }
                    }

                    if (headJson.has("exhaustCamshaft")) {
                        const JsonValue& camJson = headJson["exhaustCamshaft"];
                        const JsonValue& lobes = camJson["lobeCenterlines"];
                        int lobeCount = lobes.isArray() ? static_cast<int>(lobes.size()) : 0;

                        Camshaft::Parameters camParams;
                        camParams.lobes = lobeCount;
                        camParams.advance = camJson["advance"].numberOr(0);
                        camParams.baseRadius = camJson["baseRadius"].numberOr(0);
                        camParams.crankshaft = mainCrank;
                        camParams.lobeProfile = reconstructFunction(camJson["lobeProfileSamples"]);

                        exhaustCam = new Camshaft();
                        exhaustCam->initialize(camParams);
                        if (lobes.isArray()) {
                            for (size_t li = 0; li < lobes.size(); li++) {
                                exhaustCam->setLobeCenterline(static_cast<int>(li), lobes[li].asNumber());
                            }
                        }
                    }

                    // Create valvetrain with camshafts
                    StandardValvetrain* valvetrain = new StandardValvetrain();
                    StandardValvetrain::Parameters vtParams;
                    vtParams.intakeCamshaft = intakeCam;
                    vtParams.exhaustCamshaft = exhaustCam;
                    valvetrain->initialize(vtParams);

                    CylinderHead::Parameters hParams;
                    hParams.Bank = bank;
                    hParams.CombustionChamberVolume = headJson["combustionChamberVolume"].numberOr(0);
                    hParams.IntakeRunnerVolume = headJson["intakeRunnerVolume"].numberOr(0);
                    hParams.IntakeRunnerCrossSectionArea = headJson["intakeRunnerCrossSectionArea"].numberOr(0);
                    hParams.ExhaustRunnerVolume = headJson["exhaustRunnerVolume"].numberOr(0);
                    hParams.ExhaustRunnerCrossSectionArea = headJson["exhaustRunnerCrossSectionArea"].numberOr(0);
                    hParams.FlipDisplay = headJson["flipDisplay"].boolOr(false);
                    hParams.IntakePortFlow = nullptr;
                    hParams.ExhaustPortFlow = nullptr;
                    hParams.Valvetrain = valvetrain;

                    if (headJson.has("intakePortFlowSamples")) {
                        hParams.IntakePortFlow = reconstructFunction(headJson["intakePortFlowSamples"]);
                    }
                    if (headJson.has("exhaustPortFlowSamples")) {
                        hParams.ExhaustPortFlow = reconstructFunction(headJson["exhaustPortFlowSamples"]);
                    }

                    head->initialize(hParams);

                    // Wire intake and exhaust system to each cylinder in the head.
                    // The combustion chamber accesses these via head->getIntake()/getExhaustSystem().
                    // Map bank index to intake/exhaust indices (bank 0 -> intake/exhaust 0, etc.)
                    if (engine->getIntakeCount() > static_cast<int>(bi)) {
                        head->setAllIntakes(engine->getIntake(static_cast<int>(bi)));
                    } else if (engine->getIntakeCount() > 0) {
                        head->setAllIntakes(engine->getIntake(0));
                    }
                    for (int ci = 0; ci < cylCount; ci++) {
                        if (engine->getExhaustSystemCount() > static_cast<int>(bi)) {
                            head->setExhaustSystem(ci, engine->getExhaustSystem(static_cast<int>(bi)));
                        } else if (engine->getExhaustSystemCount() > 0) {
                            head->setExhaustSystem(ci, engine->getExhaustSystem(0));
                        }
                    }

                    // Per-cylinder head data
                    const JsonValue& headCyls = headJson["cylinders"];
                    if (headCyls.isArray()) {
                        for (size_t ci = 0; ci < headCyls.size(); ci++) {
                            const JsonValue& hc = headCyls[ci];
                            head->setSoundAttenuation(static_cast<int>(ci),
                                hc["soundAttenuation"].numberOr(1.0));
                            head->setHeaderPrimaryLength(static_cast<int>(ci),
                                hc["headerPrimaryLength"].numberOr(0));
                        }
                    }
                }

                globalCylIdx += cylCount;
            }
        }

        // --- Fuel ---
        if (engineJson.has("fuel")) {
            const JsonValue& fuelJson = engineJson["fuel"];
            Fuel* fuel = engine->getFuel();

            Fuel::Parameters fParams;
            fParams.maxBurningEfficiency = fuelJson["maxBurningEfficiency"].numberOr(1.0);
            fParams.burningEfficiencyRandomness = fuelJson["burningEfficiencyRandomness"].numberOr(0);
            fParams.lowEfficiencyAttenuation = fuelJson["lowEfficiencyAttenuation"].numberOr(0);
            fParams.maxTurbulenceEffect = fuelJson["maxTurbulenceEffect"].numberOr(0);
            fParams.maxDilutionEffect = fuelJson["maxDilutionEffect"].numberOr(0);
            fParams.molecularAfr = fuelJson["molecularAfr"].numberOr(14.7);
            fuel->initialize(fParams);
        }

        // --- Ignition ---
        IgnitionModule* ignition = engine->getIgnitionModule();
        IgnitionModule::Parameters igParams;
        igParams.cylinderCount = engine->getCylinderCount();
        igParams.crankshaft = mainCrank;

        Function* timingCurve = new Function;
        timingCurve->initialize(10, 1);
        timingCurve->addSample(0, 0.349066);
        timingCurve->addSample(523.599, 0.523599);
        igParams.timingCurve = timingCurve;
        igParams.revLimit = engine->getRedline() * 1.1;
        igParams.limiterDuration = 0.1;
        ignition->initialize(igParams);

        // --- Combustion chambers ---
        Function* turbFn = new Function;
        turbFn->initialize(30, 1);
        for (int i = 0; i < 30; i++) {
            turbFn->addSample(static_cast<double>(i), static_cast<double>(i) * 0.5);
        }

        CombustionChamber::Parameters ccParams;
        ccParams.CrankcasePressure = units::pressure(1.0, units::atm);
        ccParams.Fuel = engine->getFuel();
        ccParams.StartingPressure = units::pressure(1.0, units::atm);
        ccParams.StartingTemperature = units::celcius(25.0);
        ccParams.MeanPistonSpeedToTurbulence = turbFn;

        for (int i = 0; i < engine->getCylinderCount(); i++) {
            ccParams.Piston = engine->getPiston(i);
            ccParams.Head = engine->getHead(ccParams.Piston->getCylinderBank()->getIndex());
            engine->getChamber(i)->initialize(ccParams);
        }

        engine->calculateDisplacement();
        result.engine = engine;

        // --- Vehicle ---
        if (root.has("vehicle") && root["vehicle"].isObject()) {
            const JsonValue& vj = root["vehicle"];
            Vehicle::Parameters vParams;
            vParams.mass = vj["mass"].numberOr(1500);
            vParams.dragCoefficient = vj["dragCoefficient"].numberOr(0.3);
            vParams.crossSectionArea = vj["crossSectionArea"].numberOr(2.5);
            vParams.diffRatio = vj["diffRatio"].numberOr(3.42);
            vParams.tireRadius = vj["tireRadius"].numberOr(0.3);
            vParams.rollingResistance = vj["rollingResistance"].numberOr(2000);
            result.vehicle = new Vehicle();
            result.vehicle->initialize(vParams);
        } else {
            Vehicle::Parameters vParams;
            vParams.mass = units::mass(1597, units::kg);
            vParams.diffRatio = 3.42;
            vParams.tireRadius = units::distance(10, units::inch);
            vParams.dragCoefficient = 0.25;
            vParams.crossSectionArea = units::distance(6.0, units::foot) * units::distance(6.0, units::foot);
            vParams.rollingResistance = 2000.0;
            result.vehicle = new Vehicle();
            result.vehicle->initialize(vParams);
        }

        // --- Transmission ---
        // Preset compiler doesn't serialize gear ratios yet; use defaults
        const double gearRatios[] = { 2.97, 2.07, 1.43, 1.00, 0.84, 0.56 };
        Transmission::Parameters tParams;
        tParams.GearCount = 6;
        tParams.GearRatios = gearRatios;
        tParams.MaxClutchTorque = units::torque(1000.0, units::ft_lb);
        result.transmission = new Transmission();
        result.transmission->initialize(tParams);

    } catch (const std::exception& e) {
        result.error = std::string("JSON parse error: ") + e.what();
    }

    return result;
}

// Static configure methods delegate to the inline code in loadFromString
void PresetEngineFactory::configureCrankshafts(json::JsonValue& crankshaftsJson, Engine* engine) {
    // Handled inline in loadFromString
}

void PresetEngineFactory::configureCylinderBanks(json::JsonValue& banksJson, Engine* engine) {
    // Handled inline in loadFromString
}

void PresetEngineFactory::configureExhaustSystems(json::JsonValue& exhaustsJson, Engine* engine) {
    // Handled inline in loadFromString
}

void PresetEngineFactory::configureIntakes(json::JsonValue& intakesJson, Engine* engine) {
    // Handled inline in loadFromString
}

void PresetEngineFactory::configureFuel(json::JsonValue& fuelJson, Engine* engine) {
    // Handled inline in loadFromString
}

void PresetEngineFactory::configureIgnition(json::JsonValue& engineJson, Engine* engine) {
    // Handled inline in loadFromString
}

Vehicle* PresetEngineFactory::constructVehicle(json::JsonValue& vehicleJson) {
    // Handled inline in loadFromString
    return nullptr;
}

Transmission* PresetEngineFactory::constructTransmission(json::JsonValue& transJson) {
    // Handled inline in loadFromString
    return nullptr;
}
