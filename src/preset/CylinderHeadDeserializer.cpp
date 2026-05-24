#include "preset/CylinderHeadDeserializer.h"

#include "cylinder_head.h"
#include "cylinder_bank.h"
#include "camshaft.h"
#include "standard_valvetrain.h"
#include "vtec_valvetrain.h"
#include "engine.h"
#include "preset/CamshaftDeserializer.h"
#include "preset/FunctionDeserializer.h"

#include <stdexcept>

using json::JsonValue;

void CylinderHeadDeserializer::deserialize(const JsonValue& json, CylinderHead* head,
                                            CylinderBank* bank, Engine* engine, int bankIndex,
                                            const std::string& context) {
    const std::string ctx = context.empty() ? "cylinderHead" : context;
    int cylCount = bank->getCylinderCount();
    Crankshaft* crankshaft = engine->getCrankshaft(0);

    // Build camshafts first (valvetrain needs them at init time)
    Camshaft* intakeCam = nullptr;
    Camshaft* exhaustCam = nullptr;

    if (json.has("intakeCamshaft")) {
        intakeCam = CamshaftDeserializer::deserialize(
            json["intakeCamshaft"], crankshaft, ctx + ".intakeCamshaft");
    }
    if (json.has("exhaustCamshaft")) {
        exhaustCam = CamshaftDeserializer::deserialize(
            json["exhaustCamshaft"], crankshaft, ctx + ".exhaustCamshaft");
    }

    // Create valvetrain based on type
    Valvetrain* valvetrain = nullptr;
    std::string valvetrainType = "standard";
    if (json.has("valvetrainType") && json["valvetrainType"].isString()) {
        valvetrainType = json["valvetrainType"].asString();
    }

    if (valvetrainType == "vtec") {
        if (!json.has("vtecMinRpm")) {
            throw std::runtime_error("Missing required field 'vtecMinRpm' for vtec valvetrain in " + ctx);
        }
        VtecValvetrain* vtec = new VtecValvetrain();
        VtecValvetrain::Parameters vtParams;
        vtParams.intakeCamshaft = intakeCam;
        vtParams.exhaustCamshaft = exhaustCam;
        vtParams.vtecIntakeCamshaft = nullptr;
        vtParams.vtexExhaustCamshaft = nullptr;
        vtParams.engine = engine;
        vtParams.minRpm = json["vtecMinRpm"].asNumber();
        vtParams.minSpeed = 0;
        vtParams.manifoldVacuum = 0;
        vtParams.minThrottlePosition = 0;
        vtec->initialize(vtParams);
        valvetrain = vtec;
    } else {
        StandardValvetrain* stdVt = new StandardValvetrain();
        StandardValvetrain::Parameters vtParams;
        vtParams.intakeCamshaft = intakeCam;
        vtParams.exhaustCamshaft = exhaustCam;
        stdVt->initialize(vtParams);
        valvetrain = stdVt;
    }

    // Port flow functions (required)
    if (!json.has("intakePortFlow")) {
        throw std::runtime_error("Missing required field 'intakePortFlow' in " + ctx);
    }
    Function* intakePortFlow = FunctionDeserializer::deserialize(
        json["intakePortFlow"], ctx + ".intakePortFlow");

    if (!json.has("exhaustPortFlow")) {
        throw std::runtime_error("Missing required field 'exhaustPortFlow' in " + ctx);
    }
    Function* exhaustPortFlow = FunctionDeserializer::deserialize(
        json["exhaustPortFlow"], ctx + ".exhaustPortFlow");

    CylinderHead::Parameters hParams;
    hParams.Bank = bank;

    if (!json.has("combustionChamberVolume")) {
        throw std::runtime_error("Missing required field 'combustionChamberVolume' in " + ctx);
    }
    hParams.CombustionChamberVolume = json["combustionChamberVolume"].asNumber();

    if (!json.has("intakeRunnerVolume")) {
        throw std::runtime_error("Missing required field 'intakeRunnerVolume' in " + ctx);
    }
    hParams.IntakeRunnerVolume = json["intakeRunnerVolume"].asNumber();

    if (!json.has("intakeRunnerCrossSectionArea")) {
        throw std::runtime_error("Missing required field 'intakeRunnerCrossSectionArea' in " + ctx);
    }
    hParams.IntakeRunnerCrossSectionArea = json["intakeRunnerCrossSectionArea"].asNumber();

    if (!json.has("exhaustRunnerVolume")) {
        throw std::runtime_error("Missing required field 'exhaustRunnerVolume' in " + ctx);
    }
    hParams.ExhaustRunnerVolume = json["exhaustRunnerVolume"].asNumber();

    if (!json.has("exhaustRunnerCrossSectionArea")) {
        throw std::runtime_error("Missing required field 'exhaustRunnerCrossSectionArea' in " + ctx);
    }
    hParams.ExhaustRunnerCrossSectionArea = json["exhaustRunnerCrossSectionArea"].asNumber();

    hParams.FlipDisplay = json.has("flipDisplay") ? json["flipDisplay"].asBool() : false;
    hParams.IntakePortFlow = intakePortFlow;
    hParams.ExhaustPortFlow = exhaustPortFlow;
    hParams.Valvetrain = valvetrain;

    head->initialize(hParams);

    // Wire intake and exhaust system to each cylinder
    if (engine->getIntakeCount() > bankIndex) {
        head->setAllIntakes(engine->getIntake(bankIndex));
    } else if (engine->getIntakeCount() > 0) {
        head->setAllIntakes(engine->getIntake(0));
    }
    for (int ci = 0; ci < cylCount; ci++) {
        if (engine->getExhaustSystemCount() > bankIndex) {
            head->setExhaustSystem(ci, engine->getExhaustSystem(bankIndex));
        } else if (engine->getExhaustSystemCount() > 0) {
            head->setExhaustSystem(ci, engine->getExhaustSystem(0));
        }
    }

    // Per-cylinder head data
    const JsonValue& headCyls = json["cylinders"];
    if (headCyls.isArray()) {
        for (size_t ci = 0; ci < headCyls.size(); ci++) {
            const JsonValue& hc = headCyls[ci];
            if (!hc.has("soundAttenuation")) {
                throw std::runtime_error("Missing required field 'soundAttenuation' in " + ctx + ".cylinders[" + std::to_string(ci) + "]");
            }
            head->setSoundAttenuation(static_cast<int>(ci),
                hc["soundAttenuation"].asNumber());
            if (!hc.has("headerPrimaryLength")) {
                throw std::runtime_error("Missing required field 'headerPrimaryLength' in " + ctx + ".cylinders[" + std::to_string(ci) + "]");
            }
            head->setHeaderPrimaryLength(static_cast<int>(ci),
                hc["headerPrimaryLength"].asNumber());
        }
    }
}
