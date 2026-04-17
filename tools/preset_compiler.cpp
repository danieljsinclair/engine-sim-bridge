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

    // Write a Function as an array of [x, y] samples
    void writeFunctionSamples(const char* k, Function* fn, int maxSamples = 64) {
        if (!fn) return;
        key(k);
        beginArray();
        double x0, x1;
        fn->getDomain(&x0, &x1);
        if (x1 > x0) {
            int n = maxSamples;
            double step = (x1 - x0) / (n - 1);
            for (int i = 0; i < n; i++) {
                double x = x0 + i * step;
                double y = fn->sampleTriangle(x);
                beginArray();
                value(x);
                value(y);
                endArray();
            }
        }
        endArray();
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
    j.kv("outletFlowRate", 0.0); // Not directly exposed as getter
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
    j.endObject();
}

static void serializeCamshaft(JsonWriter& j, Camshaft* cam, int cylinderCount) {
    j.beginObject();
    j.kv("advance", cam->getAdvance());
    j.kv("baseRadius", cam->getBaseRadius());

    // Serialize lobe profile (the Function that defines valve lift)
    j.writeFunctionSamples("lobeProfileSamples", cam->getLobeProfile(), 256);

    // Serialize lobe centerlines (one per cylinder on this bank)
    j.key("lobeCenterlines");
    j.beginArray();
    for (int i = 0; i < cylinderCount; i++) {
        j.value(cam->getLobeCenterline(i));
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
    j.endObject();
}

static void serializeTransmission(JsonWriter& j, Transmission* trans) {
    // Transmission doesn't expose gear count or ratios via public getters.
    // The preset factory will use sensible defaults from the script.
    j.beginObject();
    j.kv("note", "Transmission serialization requires adding getters to Transmission class");
    j.endObject();
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <script.mr> <output.json> [engine_sim_dir]\n", argv[0]);
        fprintf(stderr, "\n");
        fprintf(stderr, "  script.mr        - Engine script to compile (e.g., engines/atg-video-1/01_honda_trx520.mr)\n");
        fprintf(stderr, "  output.json      - Output JSON preset file\n");
        fprintf(stderr, "  engine_sim_dir   - Path to engine-sim root (must contain es/ and assets/ dirs)\n");
        fprintf(stderr, "\n");
        fprintf(stderr, "The compiler generates a wrapper script that imports the engine script\n");
        fprintf(stderr, "and calls main(), then runs Piranha from the engine-sim directory so\n");
        fprintf(stderr, "that import search paths resolve correctly (../es/ -> es/).\n");
        return 1;
    }

    const char* scriptPath = argv[1];
    const char* outputPath = argv[2];
    const char* engineSimDir = (argc >= 4) ? argv[3] : nullptr;

    if (!std::filesystem::exists(scriptPath)) {
        fprintf(stderr, "Error: Script not found: %s\n", scriptPath);
        return 1;
    }

    // Resolve engine-sim directory
    // If not provided, try to infer from the script path (look for es/ sibling)
    std::filesystem::path simDir;
    if (engineSimDir) {
        simDir = std::filesystem::absolute(engineSimDir);
    } else {
        // Walk up from script looking for es/ directory
        std::filesystem::path search = std::filesystem::absolute(scriptPath).parent_path();
        bool found = false;
        for (int i = 0; i < 10 && !found; i++) {
            if (std::filesystem::exists(search / "es" / "engine_sim.mr")) {
                simDir = search;
                found = true;
            }
            search = search.parent_path();
        }
        if (!found) {
            fprintf(stderr, "Error: Cannot find engine-sim root (looking for es/engine_sim.mr)\n");
            fprintf(stderr, "Provide the engine_sim_dir argument explicitly.\n");
            return 1;
        }
    }

    if (!std::filesystem::exists(simDir / "es" / "engine_sim.mr")) {
        fprintf(stderr, "Error: es/engine_sim.mr not found in %s\n", simDir.c_str());
        return 1;
    }

    // Compute the relative import path from the assets/ directory to the engine script
    // Scripts are typically at assets/engines/... relative to the engine-sim root
    std::filesystem::path absScriptPath = std::filesystem::absolute(scriptPath);
    std::filesystem::path assetsDir = simDir / "assets";

    std::string relativeImport;
    if (engineSimDir) {
        // When engine_sim_dir is provided explicitly, scriptPath is relative to assets/
        relativeImport = scriptPath;
    } else {
        // Compute relative path from assets/ to the script
        relativeImport = std::filesystem::relative(absScriptPath, assetsDir).string();
    }

    printf("Engine-sim dir: %s\n", simDir.c_str());
    printf("Script import:  %s (relative to assets/)\n", relativeImport.c_str());

    // Generate a temporary wrapper script in the assets/ directory
    // Piranha resolves imports relative to the importing script's parent directory.
    // By placing the wrapper next to engine scripts, `import "engines/..."` resolves
    // correctly via the assets/ directory.
    std::filesystem::path wrapperPath = assetsDir / "_escli_preset_wrapper.mr";
    {
        std::ofstream wrapper(wrapperPath);
        if (!wrapper.is_open()) {
            fprintf(stderr, "Error: Cannot create wrapper script: %s\n", wrapperPath.c_str());
            return 1;
        }
        wrapper << "import \"engine_sim.mr\"\n";
        wrapper << "import \"" << relativeImport << "\"\n";
        wrapper << "main()\n";
        wrapper.close();
    }

    printf("Compiling: %s\n", scriptPath);

    // Save current working directory and change to the engine-sim root
    // Piranha search paths are: ../../es/, ../es/, es/
    // From engine-sim/, es/ resolves to engine-sim/es/
    // The wrapper is in assets/ so imports resolve relative to assets/ (where engines/ lives)
    std::filesystem::path originalCwd = std::filesystem::current_path();
    std::filesystem::current_path(simDir);

    // Initialize and run Piranha compiler
    es_script::Compiler compiler;
    compiler.initialize();

    if (!compiler.compile(wrapperPath.string())) {
        // Restore CWD before printing error
        std::filesystem::current_path(originalCwd);
        fprintf(stderr, "Error: Failed to compile script: %s\n", scriptPath);
        return 1;
    }

    es_script::Compiler::Output output = compiler.execute();

    // Restore working directory
    std::filesystem::current_path(originalCwd);

    if (!output.engine) {
        fprintf(stderr, "Error: Script did not produce an engine: %s\n", scriptPath);
        return 1;
    }

    printf("Engine: %s (%d cylinders, %d banks)\n",
           output.engine->getName().c_str(),
           output.engine->getCylinderCount(),
           output.engine->getCylinderBankCount());

    // Serialize to JSON
    JsonWriter j;
    j.beginObject();

    // Derive preset name from script filename
    std::string presetName = std::filesystem::path(scriptPath).stem().string();
    j.kv("presetName", presetName.c_str());
    j.kv("sourceScript", scriptPath);

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

    // Write output
    std::ofstream outFile(outputPath);
    if (!outFile.is_open()) {
        fprintf(stderr, "Error: Cannot open output file: %s\n", outputPath);
        return 1;
    }

    outFile << j.str() << std::endl;
    outFile.close();

    printf("Output: %s (%zu bytes)\n", outputPath, j.str().size());

    compiler.destroy();
    return 0;
}
