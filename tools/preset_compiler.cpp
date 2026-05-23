// preset_compiler.cpp - Build-time tool that compiles .mr scripts to JSON presets
// Runs on macOS (needs Piranha/Boost). Output JSON files are shipped with iOS app.
//
// Usage: engine-sim-preset-compiler <script.mr> <output.json> [asset_base_path]
//
// The compiler:
// 1. Compiles and executes the .mr script via Piranha
// 2. Walks the resulting Engine/Vehicle/Transmission object graph
// 3. Serializes all parameters to a portable JSON file
//
// All numeric values are in SI units (meters, kg, radians, seconds).

#include "engine_sim.h"
#include "compiler.h"
#include "simulator/ScriptCompileHelpers.h"
#include "simulator/ScriptExecutionHelpers.h"

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
#include "ignition_module.h"
#include "combustion_chamber.h"
#include "fuel.h"
#include "impulse_response.h"
#include "camshaft.h"
#include "function.h"
#include "units.h"
#include "throttle.h"
#include "direct_throttle_linkage.h"

#include <cstdio>
#include <cmath>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <filesystem>

// ============================================================================
// Minimal JSON Writer (no external dependencies)
// ============================================================================

class JsonWriter {
public:
    JsonWriter() : indent_(0), needsComma_(false) {}

    void beginObject() {
        maybeComma();
        append("{");
        indent_++;
        needsComma_ = false;
    }

    void endObject() {
        indent_--;
        newline();
        append("}");
        needsComma_ = true;
    }

    void beginArray() {
        maybeComma();
        append("[");
        indent_++;
        needsComma_ = false;
    }

    void endArray() {
        indent_--;
        newline();
        append("]");
        needsComma_ = true;
    }

    void key(const char* k) {
        maybeComma();
        newline();
        append("\"%s\": ", k);
        needsComma_ = false;
    }

    void value(double v) {
        maybeComma();
        // Handle NaN/Inf
        if (std::isnan(v)) append("null");
        else if (std::isinf(v)) append(v > 0 ? "1e308" : "-1e308");
        else {
            char buf[64];
            // Use enough precision to round-trip doubles
            snprintf(buf, sizeof(buf), "%.15g", v);
            append("%s", buf);
        }
        needsComma_ = true;
    }

    void value(int v) {
        maybeComma();
        append("%d", v);
        needsComma_ = true;
    }

    void value(const char* v) {
        maybeComma();
        // Escape basic JSON special chars
        append("\"");
        for (const char* p = v; *p; p++) {
            switch (*p) {
                case '"':  append("\\\""); break;
                case '\\': append("\\\\"); break;
                case '\n': append("\\n"); break;
                case '\r': append("\\r"); break;
                case '\t': append("\\t"); break;
                default:   buf_ += *p; break;
            }
        }
        append("\"");
        needsComma_ = true;
    }

    void valueBool(bool v) {
        maybeComma();
        append(v ? "true" : "false");
        needsComma_ = true;
    }

    // Convenience: write a key-value pair
    void kv(const char* k, double v) { key(k); value(v); }
    void kv(const char* k, int v) { key(k); value(v); }
    void kv(const char* k, const char* v) { key(k); value(v); }
    void kvBool(const char* k, bool v) { key(k); valueBool(v); }

    // Serialize Function with complete metadata including filterRadius
    void serializeFunction(const char* k, Function* fn) {
        if (!fn) return;
        key(k);
        beginObject();
        kv("filterRadius", fn->getFilterRadius());
        kv("inputScale", fn->getInputScale());
        kv("outputScale", fn->getOutputScale());

        key("samples");
        beginArray();
        for (int i = 0; i < fn->getSampleCount(); i++) {
            beginArray();
            value(fn->getX(i));
            value(fn->getY(i));
            endArray();
        }
        endArray();

        endObject();
    }

    std::string str() const { return buf_; }

private:
    void maybeComma() {
        if (needsComma_) {
            buf_ += ", ";
        }
    }

    void newline() {
        buf_ += '\n';
        for (int i = 0; i < indent_; i++) buf_ += "  ";
    }

    void append(const char* fmt, ...) {
        char tmp[512];
        va_list args;
        va_start(args, fmt);
        vsnprintf(tmp, sizeof(tmp), fmt, args);
        va_end(args);
        buf_ += tmp;
    }

    std::string buf_;
    int indent_;
    bool needsComma_;
};


// ============================================================================
// Engine Graph Walker
// ============================================================================

static void serializeCrankshaft(JsonWriter& j, Crankshaft* cs) {
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

static void serializeConnectingRod(JsonWriter& j, ConnectingRod* rod) {
    j.beginObject();
    j.kv("mass", rod->getMass());
    j.kv("momentOfInertia", rod->getMomentOfInertia());
    j.kv("centerOfMass", rod->getCenterOfMass());
    j.kv("length", rod->getLength());
    j.kv("journal", rod->getJournal());
    j.kv("slaveThrow", rod->getSlaveThrow());
    j.endObject();
}

static void serializeExhaustSystem(JsonWriter& j, ExhaustSystem* es) {
    j.beginObject();
    j.kv("length", es->getLength());
    j.kv("collectorCrossSectionArea", es->getCollectorCrossSectionArea());
    j.kv("outletFlowRate", es->getOutletFlowRate());
    j.kv("primaryTubeLength", es->getPrimaryTubeLength());
    j.kv("primaryFlowRate", es->getPrimaryFlowRate());
    j.kv("velocityDecay", es->getVelocityDecay());
    j.kv("audioVolume", es->getAudioVolume());

    // Impulse response reference (filename for WAV lookup)
    ImpulseResponse* ir = es->getImpulseResponse();
    if (ir) {
        j.kv("impulseResponseFilename", ir->getFilename().c_str());
        j.kv("impulseResponseVolume", ir->getVolume());
    }

    j.endObject();
}

static void serializeIntake(JsonWriter& j, Intake* intake) {
    j.beginObject();
    j.kv("runnerFlowRate", intake->getRunnerFlowRate());
    j.kv("runnerLength", intake->getRunnerLength());
    j.kv("plenumCrossSectionArea", intake->getPlenumCrossSectionArea());
    j.kv("velocityDecay", intake->getVelocityDecay());
    j.kv("inputFlowK", intake->getInputFlowK());
    j.kv("plenumVolume", intake->getVolume());
    j.kv("idleFlowK", intake->getIdleFlowK());
    j.kv("idleThrottlePlatePosition", intake->getIdleThrottlePlatePosition());
    j.endObject();
}

static void serializeCamshaft(JsonWriter& j, Camshaft* cam, int cylinderCount) {
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

static void serializeCylinderHead(JsonWriter& j, CylinderHead* head, int cylinderCount) {
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

static void serializeCylinderBank(JsonWriter& j, CylinderBank* bank, Engine* engine) {
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
            j.kv("rodJournalCount", rod->getRodJournalCount());
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

static void serializeEngine(JsonWriter& j, Engine* engine) {
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

    // Throttle gamma (power curve for throttle input)
    DirectThrottleLinkage *dtl = dynamic_cast<DirectThrottleLinkage *>(engine->getThrottleLinkage());
    if (dtl) {
        j.kv("throttleGamma", dtl->getGamma());
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
    Fuel* fuel = engine->getFuel();
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

    // Ignition module
    IgnitionModule* ignition = engine->getIgnitionModule();
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

static void serializeVehicle(JsonWriter& j, Vehicle* vehicle) {
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

static void serializeTransmission(JsonWriter& j, Transmission* trans) {
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

namespace {

struct PresetCompilerArgs {
    std::filesystem::path scriptPath;
    std::filesystem::path outputPath;
    std::filesystem::path simDir;
};

void printUsage(const char* programName) {
    fprintf(stderr, "Usage: %s <script.mr> <output.json> [engine_sim_dir]\n", programName);
    fprintf(stderr, "\n");
    fprintf(stderr, "  script.mr        - Engine script to compile (e.g., engines/atg-video-1/01_honda_trx520.mr)\n");
    fprintf(stderr, "  output.json      - Output JSON preset file\n");
    fprintf(stderr, "  engine_sim_dir   - Path to engine-sim root (must contain es/ and assets/ dirs)\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "The compiler generates a wrapper script that imports the engine script\n");
    fprintf(stderr, "and calls main(), then runs Piranha from the engine-sim directory so\n");
    fprintf(stderr, "that import search paths resolve correctly (../es/ -> es/).\n");
}

PresetCompilerArgs parseArgs(int argc, char* argv[]) {
    if (argc < 3) {
        printUsage(argv[0]);
        throw std::runtime_error("missing required arguments");
    }

    PresetCompilerArgs args;
    args.scriptPath = argv[1];
    args.outputPath = argv[2];
    args.simDir = (argc >= 4)
        ? std::filesystem::absolute(argv[3])
        : script_compile_helpers::findEngineSimRoot(args.scriptPath);
    return args;
}

void logCompileContext(const PresetCompilerArgs& args) {
    const auto compileTarget = script_compile_helpers::prepareScriptCompileTarget(args.scriptPath, args.simDir);
    printf("Engine-sim dir: %s\n", args.simDir.c_str());
    printf("Script import:  %s (relative to assets/)\n", compileTarget.relativeImport.c_str());
    printf("Compiling: %s\n", args.scriptPath.c_str());
    script_compile_helpers::cleanupScriptCompileTarget(compileTarget);
}

std::string buildPresetJson(const std::filesystem::path& scriptPath, const es_script::Compiler::Output& output) {
    JsonWriter j;
    j.beginObject();

    const std::string presetName = scriptPath.stem().string();
    j.kv("presetName", presetName.c_str());
    j.kv("sourceScript", scriptPath.string().c_str());

    j.key("engine");
    serializeEngine(j, output.engine);

    if (output.vehicle) {
        j.key("vehicle");
        serializeVehicle(j, output.vehicle);
    }

    if (output.transmission) {
        j.key("transmission");
        serializeTransmission(j, output.transmission);
    }

    j.endObject();
    return j.str();
}

void writeOutputFile(const std::filesystem::path& outputPath, const std::string& json) {
    std::ofstream outFile(outputPath);
    if (!outFile.is_open()) {
        throw std::runtime_error("Cannot open output file: " + outputPath.string());
    }

    outFile << json << std::endl;
}

int runPresetCompiler(int argc, char* argv[]) {
    const PresetCompilerArgs args = parseArgs(argc, argv);
    logCompileContext(args);

    const es_script::Compiler::Output output = script_execution_helpers::compileScript(args.scriptPath, args.simDir);
    if (!output.engine) {
        throw std::runtime_error("Script did not produce an engine: " + args.scriptPath.string());
    }

    printf("Engine: %s (%d cylinders, %d banks)\n",
           output.engine->getName().c_str(),
           output.engine->getCylinderCount(),
           output.engine->getCylinderBankCount());

    const std::string json = buildPresetJson(args.scriptPath, output);
    writeOutputFile(args.outputPath, json);
    printf("Output: %s (%zu bytes)\n", args.outputPath.c_str(), json.size());
    return 0;
}

}  // namespace

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
    try {
        return runPresetCompiler(argc, argv);
    } catch (const std::exception& e) {
        fprintf(stderr, "Error: %s\n", e.what());
        return 1;
    }
}
