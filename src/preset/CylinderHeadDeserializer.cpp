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
#include <memory>

using json::JsonValue;

namespace {
    double getRequiredField(const JsonValue& json, const std::string& key,
                            const std::string& ctx) {
        if (!json.has(key)) {
            throw std::runtime_error("Missing required field '" + key + "' in " + ctx);
        }
        return json[key].asNumber();
    }

    bool getRequiredBoolField(const JsonValue& json, const std::string& key,
                              const std::string& ctx) {
        if (!json.has(key)) {
            throw std::runtime_error("Missing required field '" + key + "' in " + ctx);
        }
        return json[key].asBool();
    }

    std::pair<Camshaft*, Camshaft*> buildCamshafts(const JsonValue& json,
                                                    Crankshaft* crankshaft,
                                                    const std::string& ctx) {
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
        return {intakeCam, exhaustCam};
    }

    Valvetrain* createValvetrain(const JsonValue& json, Camshaft* intakeCam,
                                  Camshaft* exhaustCam, Engine* engine,
                                  const std::string& ctx) {
        std::string valvetrainType = "standard";
        if (json.has("valvetrainType") && json["valvetrainType"].isString()) {
            valvetrainType = json["valvetrainType"].asString();
        }

        if (valvetrainType == "vtec") {
            if (!json.has("vtecMinRpm")) {
                throw std::runtime_error(
                    "Missing required field 'vtecMinRpm' for vtec valvetrain in " + ctx);
            }
            auto vtec = std::make_unique<VtecValvetrain>();
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
            return vtec.release();
        }

        auto stdVt = std::make_unique<StandardValvetrain>();
        StandardValvetrain::Parameters vtParams;
        vtParams.intakeCamshaft = intakeCam;
        vtParams.exhaustCamshaft = exhaustCam;
        stdVt->initialize(vtParams);
        return stdVt.release();
    }

    void wireIntakeAndExhaust(CylinderHead* head, Engine* engine,
                              int bankIndex, int cylCount) {
        if (engine->getIntakeCount() > bankIndex) {
            head->setAllIntakes(engine->getIntake(bankIndex));
        } else if (engine->getIntakeCount() > 0) {
            head->setAllIntakes(engine->getIntake(0));
        }
        for (auto ci = 0; ci < cylCount; ci++) {
            if (engine->getExhaustSystemCount() > bankIndex) {
                head->setExhaustSystem(ci, engine->getExhaustSystem(bankIndex));
            } else if (engine->getExhaustSystemCount() > 0) {
                head->setExhaustSystem(ci, engine->getExhaustSystem(0));
            }
        }
    }

    void processCylinderData(CylinderHead* head, const JsonValue& json,
                             const std::string& ctx) {
        const JsonValue& headCyls = json["cylinders"];
        if (!headCyls.isArray()) {
            return;
        }
        for (auto ci = 0u; ci < headCyls.size(); ci++) {
            const JsonValue& hc = headCyls[ci];
            std::string cylCtx = ctx + ".cylinders[" + std::to_string(ci) + "]";
            head->setSoundAttenuation(static_cast<int>(ci),
                getRequiredField(hc, "soundAttenuation", cylCtx));
            head->setHeaderPrimaryLength(static_cast<int>(ci),
                getRequiredField(hc, "headerPrimaryLength", cylCtx));
        }
    }
}

void CylinderHeadDeserializer::deserialize(const JsonValue& json, CylinderHead* head,
                                            CylinderBank* bank, Engine* engine, int bankIndex,
                                            const std::string& context) {
    const std::string ctx = context.empty() ? "cylinderHead" : context;
    int cylCount = bank->getCylinderCount();
    Crankshaft* crankshaft = engine->getCrankshaft(0);

    auto [intakeCam, exhaustCam] = buildCamshafts(json, crankshaft, ctx);
    Valvetrain* valvetrain = createValvetrain(json, intakeCam, exhaustCam, engine, ctx);

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
    hParams.CombustionChamberVolume = getRequiredField(json, "combustionChamberVolume", ctx);
    hParams.IntakeRunnerVolume = getRequiredField(json, "intakeRunnerVolume", ctx);
    hParams.IntakeRunnerCrossSectionArea = getRequiredField(json, "intakeRunnerCrossSectionArea", ctx);
    hParams.ExhaustRunnerVolume = getRequiredField(json, "exhaustRunnerVolume", ctx);
    hParams.ExhaustRunnerCrossSectionArea = getRequiredField(json, "exhaustRunnerCrossSectionArea", ctx);
    hParams.FlipDisplay = getRequiredBoolField(json, "flipDisplay", ctx);
    hParams.IntakePortFlow = intakePortFlow;
    hParams.ExhaustPortFlow = exhaustPortFlow;
    hParams.Valvetrain = valvetrain;

    head->initialize(hParams);

    wireIntakeAndExhaust(head, engine, bankIndex, cylCount);
    processCylinderData(head, json, ctx);
}
