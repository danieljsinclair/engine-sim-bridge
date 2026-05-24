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

#include "common/JsonWriter.h"
#include "common/PresetSerializer.h"

namespace {

// Normalize Piranha's CWD-relative impulse response paths to portable
// relative paths suitable for committed JSON. Piranha stores paths like
// "../../es/sound-library/smooth/smooth_39.wav" (relative to its CWD,
// which is the engine-sim root). We resolve these against simDir to get
// "es/sound-library/...", then strip the "es/" prefix to produce the
// portable form: "sound-library/smooth/smooth_39.wav".
void resolveImpulseResponsePaths(Engine* engine, const std::filesystem::path& simDir) {
    if (!engine) return;

    for (int i = 0; i < engine->getExhaustSystemCount(); i++) {
        ExhaustSystem* es = engine->getExhaustSystem(i);
        if (!es) continue;

        ImpulseResponse* ir = es->getImpulseResponse();
        if (!ir) continue;

        const std::string& filename = ir->getFilename();
        if (filename.empty()) continue;

        std::filesystem::path p(filename);
        if (!p.is_absolute()) {
            // Resolve the CWD-relative path against simDir to get a clean
            // relative path. For "../../es/sound-library/X.wav" relative to
            // simDir (engine-sim root), weakly_canonical produces
            // "/abs/path/es/sound-library/X.wav". We then compute the
            // relative path from simDir to get "es/sound-library/X.wav".
            std::filesystem::path resolved =
                std::filesystem::weakly_canonical(simDir / p);
            std::filesystem::path relative =
                std::filesystem::relative(resolved, simDir);

            std::string result = relative.string();

            // Strip "es/" prefix — all engine-sim assets live under es/
            // and the portable path is relative to the es/ directory.
            if (result.size() > 3 && result.substr(0, 3) == "es/") {
                result = result.substr(3);
            }

            ir->initialize(result, ir->getVolume());
        }
    }
}

}  // anonymous namespace

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
    PresetSerializer::serializeEngine(j, output.engine);

    if (output.vehicle) {
        j.key("vehicle");
        PresetSerializer::serializeVehicle(j, output.vehicle);
    }

    if (output.transmission) {
        j.key("transmission");
        PresetSerializer::serializeTransmission(j, output.transmission);
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

    // Resolve relative impulse response paths to absolute before serializing.
    // Piranha stores CWD-relative paths; canonicalize using the engine-sim root.
    resolveImpulseResponsePaths(output.engine, args.simDir);

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
