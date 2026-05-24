// PresetSerializer.cpp - Engine serialization to JSON

#include "common/PresetSerializer.h"
#include "common/JsonWriter.h"

#include "engine.h"
#include "vehicle.h"
#include "transmission.h"
#include "crankshaft.h"
#include "cylinder_bank.h"
#include "cylinder_head.h"
#include "piston.h"
#include "connecting_rod.h"
#include "exhaust_system.h"
#include "intake.h"
#include "ignition_module.h"
#include "combustion_chamber.h"
#include "fuel.h"
#include "camshaft.h"
#include "throttle.h"
#include "direct_throttle_linkage.h"

namespace PresetSerializer {

void serializeCrankshaft(JsonWriter& j, Crankshaft* cs) {
    j.beginObject();
    j.kv("mass", cs->getMass());
    j.kv("flywheelMass", cs->getFlywheelMass());
    j.kv("momentOfInertia", cs->getMomentOfInertia());
    j.kv("crankThrow", cs->getThrow());
    j.kv("posX", cs->getPosX());
    j.kv("posY", cs->getPosY());
    j.kv("tdc", cs->getTdc());
    j.kv("frictionTorque", cs->getFrictionTorque());

    j.key("rodJournals");
    j.beginArray();
    for (int i = 0; i < cs->getRodJournalCount(); i++) {
        j.beginObject();
        j.kv("angle", cs->getRodJournalAngle(i));
        j.endObject();
    }
    j.endArray();

    j.endObject();
}

void serializeConnectingRod(JsonWriter& j, ConnectingRod* rod) {
    j.beginObject();
    j.kv("mass", rod->getMass());
    j.kv("momentOfInertia", rod->getMomentOfInertia());
    j.kv("centerOfMass", rod->getCenterOfMass());
    j.kv("length", rod->getLength());
    j.kv("journal", rod->getJournal());
    j.kv("slaveThrow", rod->getSlaveThrow());
    j.endObject();
}

void serializeExhaustSystem(JsonWriter& j, ExhaustSystem* es) {
    j.beginObject();
    j.kv("length", es->getLength());
    j.kv("collectorCrossSectionArea", es->getCollectorCrossSectionArea());
    j.kv("outletFlowRate", es->getOutletFlowRate());
    j.kv("primaryTubeLength", es->getPrimaryTubeLength());
    j.kv("primaryFlowRate", es->getPrimaryFlowRate());
    j.kv("velocityDecay", es->getVelocityDecay());
    j.kv("audioVolume", es->getAudioVolume());

    // Impulse response reference (filename for WAV lookup)
    auto* ir = es->getImpulseResponse();
    if (ir) {
        j.kv("impulseResponseFilename", ir->getFilename().c_str());
        j.kv("impulseResponseVolume", ir->getVolume());
    }

    j.endObject();
}

void serializeIntake(JsonWriter& j, Intake* intake) {
    j.beginObject();
    j.kv("runnerFlowRate", intake->getRunnerFlowRate());
    j.kv("runnerLength", intake->getRunnerLength());
    j.kv("plenumCrossSectionArea", intake->getPlenumCrossSectionArea());
    j.kv("velocityDecay", intake->getVelocityDecay());
    j.kv("inputFlowK", intake->getInputFlowK());
    j.kv("plenumVolume", intake->getVolume());
    j.kv("idleFlowK", intake->getIdleFlowK());
    j.kv("idleThrottlePlatePosition", intake->getIdleThrottlePlatePosition());
    j.kv("molecularAfr", intake->getMolecularAfr());
    j.endObject();
}

void serializeCamshaft(JsonWriter& j, Camshaft* cam, int cylinderCount) {
    j.beginObject();
    j.kv("advance", cam->getAdvance());
    j.kv("baseRadius", cam->getBaseRadius());
    j.kv("lobeCount", cam->getLobeCount());

    // Serialize lobe profile (the Function that defines valve lift)
    j.serializeFunction("lobeProfile", cam->getLobeProfile());

    // Serialize lobe centerlines (one per cylinder on this bank)
    // Convert from cam angles to crankshaft angles (×2) to match .mr convention
    j.key("lobeCenterlines");
    j.beginArray();
    for (int i = 0; i < cylinderCount; i++) {
        j.value(cam->getLobeCenterline(i) * 2);
    }
    j.endArray();

    j.endObject();
}

void serializeCylinderHead(JsonWriter& j, CylinderHead* head, int cylinderCount) {
    j.beginObject();
    j.kv("combustionChamberVolume", head->getCombustionChamberVolume());
    j.kv("intakeRunnerVolume", head->getIntakeRunnerVolume());
    j.kv("intakeRunnerCrossSectionArea", head->getIntakeRunnerCrossSectionArea());
    j.kv("exhaustRunnerVolume", head->getExhaustRunnerVolume());
    j.kv("exhaustRunnerCrossSectionArea", head->getExhaustRunnerCrossSectionArea());
    j.kvBool("flipDisplay", head->getFlipDisplay());

    // Per-cylinder head data
    j.key("cylinders");
    j.beginArray();
    for (int i = 0; i < cylinderCount; i++) {
        j.beginObject();
        j.kv("soundAttenuation", head->getSoundAttenuation(i));
        j.kv("headerPrimaryLength", head->getHeaderPrimaryLength(i));
        j.endObject();
    }
    j.endArray();

    // Camshafts
    Camshaft* intakeCam = head->getIntakeCamshaft();
    Camshaft* exhaustCam = head->getExhaustCamshaft();

    if (intakeCam) {
        j.key("intakeCamshaft");
        serializeCamshaft(j, intakeCam, cylinderCount);
    }
    if (exhaustCam) {
        j.key("exhaustCamshaft");
        serializeCamshaft(j, exhaustCam, cylinderCount);
    }

    // Port flow functions
    j.serializeFunction("intakePortFlow", head->getIntakePortFlow());
    j.serializeFunction("exhaustPortFlow", head->getExhaustPortFlow());

    j.endObject();
}

void serializeCylinderBank(JsonWriter& j, CylinderBank* bank, Engine* engine) {
    int cylCount = bank->getCylinderCount();
    j.beginObject();
    j.kv("angle", bank->getAngle());
    j.kv("bore", bank->getBore());
    j.kv("deckHeight", bank->getDeckHeight());
    j.kv("displayDepth", bank->getDisplayDepth());
    j.kv("index", bank->getIndex());
    j.kv("positionX", bank->getX());
    j.kv("positionY", bank->getY());
    j.kv("cylinderCount", cylCount);

    // Per-cylinder data
    j.key("cylinders");
    j.beginArray();
    for (int i = 0; i < cylCount; i++) {
        j.beginObject();

        Piston* piston = engine->getPiston(bank->getIndex() * cylCount + i);
        if (piston) {
            j.kv("pistonMass", piston->getMass());
            j.kv("compressionHeight", piston->getCompressionHeight());
            j.kv("wristPinPosition", piston->getWristPinLocation());
            j.kv("displacement", piston->getDisplacement());
            j.kv("blowbyK", piston->getBlowbyK());
            j.kv("cylinderIndex", piston->getCylinderIndex());
        }

        ConnectingRod* rod = engine->getConnectingRod(bank->getIndex() * cylCount + i);
        if (rod) {
            j.key("connectingRod");
            serializeConnectingRod(j, rod);
            j.kv("rodJournalIndex", rod->getJournal());
        }

        j.endObject();
    }
    j.endArray();

    // Cylinder head
    CylinderHead* head = engine->getHead(bank->getIndex());
    if (head) {
        j.key("cylinderHead");
        serializeCylinderHead(j, head, cylCount);
    }

    j.endObject();
}

void serializeCombustionChamber(JsonWriter& j, CombustionChamber* chamber) {
    j.beginObject();
    j.kv("crankcasePressure", chamber->getCrankcasePressure());
    j.kv("startingPressure", 101325.0); // 1 atm in Pascals
    j.kv("startingTemperature", 298.15); // 25°C in Kelvin

    j.serializeFunction("meanPistonSpeedToTurbulence", chamber->m_meanPistonSpeedToTurbulence);

    const auto& friction = chamber->m_frictionModel;
    j.kv("frictionCoeff", friction.frictionCoeff);
    j.kv("breakawayFriction", friction.breakawayFriction);
    j.kv("breakawayFrictionVelocity", friction.breakawayFrictionVelocity);
    j.kv("viscousFrictionCoefficient", friction.viscousFrictionCoefficient);

    j.endObject();
}

void serializeEngine(JsonWriter& j, Engine* engine) {
    j.beginObject();

    // Basic engine parameters
    j.kv("name", engine->getName().c_str());
    j.kv("cylinderBankCount", engine->getCylinderBankCount());
    j.kv("cylinderCount", engine->getCylinderCount());
    j.kv("crankshaftCount", engine->getCrankshaftCount());
    j.kv("exhaustSystemCount", engine->getExhaustSystemCount());
    j.kv("intakeCount", engine->getIntakeCount());
    j.kv("starterTorque", engine->getStarterTorque());
    j.kv("starterSpeed", engine->getStarterSpeed());
    j.kv("redline", engine->getRedline());
    j.kv("dynoMinSpeed", engine->getDynoMinSpeed());
    j.kv("dynoMaxSpeed", engine->getDynoMaxSpeed());
    j.kv("dynoHoldStep", engine->getDynoHoldStep());
    j.kv("displacement", engine->getDisplacement());
    j.kv("simulationFrequency", engine->getSimulationFrequency());
    j.kv("initialHighFrequencyGain", engine->getInitialHighFrequencyGain());
    j.kv("initialNoise", engine->getInitialNoise());
    j.kv("initialJitter", engine->getInitialJitter());

    // Throttle gamma (for DirectThrottleLinkage)
    auto* throttleObj = engine->getThrottleObject();
    if (auto* direct = dynamic_cast<DirectThrottleLinkage*>(throttleObj)) {
        j.kv("throttleGamma", direct->getGamma());
    }

    // Crankshafts
    j.key("crankshafts");
    j.beginArray();
    for (int i = 0; i < engine->getCrankshaftCount(); i++) {
        serializeCrankshaft(j, engine->getCrankshaft(i));
    }
    j.endArray();

    // Cylinder banks (includes per-cylinder data and cylinder heads)
    j.key("cylinderBanks");
    j.beginArray();
    for (int i = 0; i < engine->getCylinderBankCount(); i++) {
        serializeCylinderBank(j, engine->getCylinderBank(i), engine);
    }
    j.endArray();

    // Exhaust systems
    j.key("exhaustSystems");
    j.beginArray();
    for (int i = 0; i < engine->getExhaustSystemCount(); i++) {
        serializeExhaustSystem(j, engine->getExhaustSystem(i));
    }
    j.endArray();

    // Intakes
    j.key("intakes");
    j.beginArray();
    for (int i = 0; i < engine->getIntakeCount(); i++) {
        serializeIntake(j, engine->getIntake(i));
    }
    j.endArray();

    // Fuel
    auto* fuel = engine->getFuel();
    if (fuel) {
        j.key("fuel");
        j.beginObject();
        j.kv("maxBurningEfficiency", fuel->getMaxBurningEfficiency());
        j.kv("burningEfficiencyRandomness", fuel->getBurningEfficiencyRandomness());
        j.kv("lowEfficiencyAttenuation", fuel->getLowEfficiencyAttenuation());
        j.kv("maxTurbulenceEffect", fuel->getMaxTurbulenceEffect());
        j.kv("maxDilutionEffect", fuel->getMaxDilutionEffect());
        j.kv("molecularAfr", fuel->getMolecularAfr());
        j.kv("molecularMass", fuel->getMolecularMass());
        j.kv("energyDensity", fuel->getEnergyDensity());
        j.kv("density", fuel->getDensity());
        j.serializeFunction("turbulenceToFlameSpeedRatio", fuel->getTurbulenceToFlameSpeedRatio());
        j.endObject();
    }

    // Combustion chambers
    j.key("combustionChambers");
    j.beginArray();
    for (int i = 0; i < engine->getCylinderCount(); i++) {
        serializeCombustionChamber(j, engine->getChamber(i));
    }
    j.endArray();

    // Ignition module
    auto* ignition = engine->getIgnitionModule();
    if (ignition) {
        j.key("ignitionModule");
        j.beginObject();
        j.kv("revLimit", ignition->getRevLimit());
        j.kv("limiterDuration", ignition->getLimiterDuration());
        j.serializeFunction("timingCurve", ignition->getTimingCurve());
        j.key("firingOrder");
        j.beginArray();
        for (int i = 0; i < ignition->getCylinderCount(); i++) {
            j.value(ignition->getFiringOrder(i));
        }
        j.endArray();
        j.endObject();
    }

    j.endObject();
}

void serializeVehicle(JsonWriter& j, Vehicle* vehicle) {
    j.beginObject();
    j.kv("mass", vehicle->getMass());
    j.kv("dragCoefficient", vehicle->getDragCoefficient());
    j.kv("crossSectionArea", vehicle->getCrossSectionArea());
    j.kv("diffRatio", vehicle->getDiffRatio());
    j.kv("tireRadius", vehicle->getTireRadius());
    j.kv("rollingResistance", vehicle->getRollingResistance());
    j.kv("travelledDistance", vehicle->getTravelledDistance());
    j.endObject();
}

void serializeTransmission(JsonWriter& j, Transmission* trans) {
    j.beginObject();
    j.kv("gearCount", trans->getGearCount());
    j.kv("currentGear", trans->getGear());
    j.kv("maxClutchTorque", trans->getMaxClutchTorque());
    j.kv("clutchPressure", trans->getClutchPressure());

    j.key("gearRatios");
    j.beginArray();
    for (int i = 0; i < trans->getGearCount(); i++) {
        j.value(trans->getGearRatio(i));
    }
    j.endArray();

    j.endObject();
}

std::string serializeEngineToJson(Engine* engine, Vehicle* vehicle, Transmission* transmission) {
    JsonWriter j;
    j.beginObject();

    j.key("engine");
    serializeEngine(j, engine);

    if (vehicle) {
        j.key("vehicle");
        serializeVehicle(j, vehicle);
    }

    if (transmission) {
        j.key("transmission");
        serializeTransmission(j, transmission);
    }

    j.endObject();
    return j.str();
}

} // namespace PresetSerializer