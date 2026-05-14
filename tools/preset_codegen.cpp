// preset_codegen.cpp - Build-time tool that compiles .mr scripts to C++ preset source
// Runs on macOS (needs Piranha/Boost). Output .cpp files are compiled into the bridge.
//
// Usage: engine-sim-preset-codegen --script <path> --output <path> --classname <name>
//
// The generator:
// 1. Compiles and executes the .mr script via Piranha
// 2. Walks the resulting Engine/Vehicle/Transmission object graph
// 3. Emits C++ source that constructs an equivalent engine
//
// Generated code uses EnginePresetsHelper for shared functions and
// follows the same construction pattern as EnginePresets.cpp.

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
#include "camshaft.h"
#include "ignition_module.h"
#include "combustion_chamber.h"
#include "fuel.h"
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
#include <sstream>

// ============================================================================
// C++ Code Emitter
// ============================================================================

struct CppEmitter {
    std::string buf_;
    int indent_ = 0;

    void line(const char* fmt, ...) {
        for (int i = 0; i < indent_; i++) buf_ += "    ";
        char tmp[1024];
        va_list args;
        va_start(args, fmt);
        vsnprintf(tmp, sizeof(tmp), fmt, args);
        va_end(args);
        buf_ += tmp;
        buf_ += '\n';
    }
};

// ============================================================================
// Format helpers
// ============================================================================

static std::string fmtDouble(double v) {
    if (std::isnan(v)) return "0.0  // NaN";
    if (std::isinf(v)) return v > 0 ? "1e308  // Inf" : "-1e308  // -Inf";
    char buf[64];
    snprintf(buf, sizeof(buf), "%.15g", v);
    return buf;
}

static std::string fmtInt(int v) {
    return std::to_string(v);
}

static std::string sanitizeName(const std::string& s) {
    std::string result;
    for (char c : s) {
        if (isalnum(c) || c == '_') {
            result += c;
        } else {
            result += '_';
        }
    }
    return result;
}

// ============================================================================
// Engine Graph → C++ Emitters
// ============================================================================

static void emitFunction(CppEmitter& e, const char* varName, Function* fn) {
    if (!fn) {
        e.line("%s = nullptr;", varName);
        return;
    }
    e.line("Function* %s = new Function;", varName);
    e.line("%s->initialize(%d, %s);", varName, fn->getSampleCount(), fmtDouble(fn->getFilterRadius()).c_str());
    for (int i = 0; i < fn->getSampleCount(); i++) {
        e.line("%s->addSample(%s, %s);",
            varName,
            fmtDouble(fn->getX(i)).c_str(),
            fmtDouble(fn->getY(i)).c_str());
    }
    e.line("");
}

static void emitCrankshaft(CppEmitter& e, Engine* engine, int csIdx) {
    Crankshaft* cs = engine->getCrankshaft(csIdx);
    e.line("Crankshaft::Parameters cp%d = {};", csIdx);
    e.line("cp%d.mass = %s;", csIdx, fmtDouble(cs->getMass()).c_str());
    e.line("cp%d.flywheelMass = %s;", csIdx, fmtDouble(cs->getFlywheelMass()).c_str());
    e.line("cp%d.momentOfInertia = %s;", csIdx, fmtDouble(cs->getMomentOfInertia()).c_str());
    e.line("cp%d.crankThrow = %s;", csIdx, fmtDouble(cs->getThrow()).c_str());
    e.line("cp%d.pos_x = %s;", csIdx, fmtDouble(cs->getPosX()).c_str());
    e.line("cp%d.pos_y = %s;", csIdx, fmtDouble(cs->getPosY()).c_str());
    e.line("cp%d.tdc = %s;", csIdx, fmtDouble(cs->getTdc()).c_str());
    e.line("cp%d.frictionTorque = %s;", csIdx, fmtDouble(cs->getFrictionTorque()).c_str());
    e.line("cp%d.rodJournals = %d;", csIdx, cs->getRodJournalCount());
    e.line("engine->getCrankshaft(%d)->initialize(cp%d);", csIdx, csIdx);

    for (int j = 0; j < cs->getRodJournalCount(); j++) {
        e.line("engine->getCrankshaft(%d)->setRodJournalAngle(%d, %s);",
            csIdx, j, fmtDouble(cs->getRodJournalAngle(j)).c_str());
    }
    e.line("");
}

static void emitExhaustSystem(CppEmitter& e, Engine* engine, int exIdx) {
    ExhaustSystem* es = engine->getExhaustSystem(exIdx);
    e.line("ExhaustSystem::Parameters esp%d = {};", exIdx);
    e.line("esp%d.length = %s;", exIdx, fmtDouble(es->getLength()).c_str());
    e.line("esp%d.collectorCrossSectionArea = %s;", exIdx, fmtDouble(es->getCollectorCrossSectionArea()).c_str());
    e.line("esp%d.outletFlowRate = %s;", exIdx, fmtDouble(es->getOutletFlowRate()).c_str());
    e.line("esp%d.primaryTubeLength = %s;", exIdx, fmtDouble(es->getPrimaryTubeLength()).c_str());
    e.line("esp%d.primaryFlowRate = %s;", exIdx, fmtDouble(es->getPrimaryFlowRate()).c_str());
    e.line("esp%d.velocityDecay = %s;", exIdx, fmtDouble(es->getVelocityDecay()).c_str());
    e.line("esp%d.audioVolume = %s;", exIdx, fmtDouble(es->getAudioVolume()).c_str());
    e.line("esp%d.impulseResponse = nullptr;", exIdx);
    e.line("engine->getExhaustSystem(%d)->initialize(esp%d);", exIdx, exIdx);
    e.line("");
}

static void emitIntake(CppEmitter& e, Engine* engine, int inIdx) {
    Intake* intake = engine->getIntake(inIdx);
    e.line("Intake::Parameters ip%d = {};", inIdx);
    e.line("ip%d.volume = %s;", inIdx, fmtDouble(intake->getVolume()).c_str());
    e.line("ip%d.CrossSectionArea = %s;", inIdx, fmtDouble(intake->getPlenumCrossSectionArea()).c_str());
    e.line("ip%d.InputFlowK = %s;", inIdx, fmtDouble(intake->getInputFlowK()).c_str());
    e.line("ip%d.IdleFlowK = %s;", inIdx, fmtDouble(intake->getIdleFlowK()).c_str());
    e.line("ip%d.RunnerFlowRate = %s;", inIdx, fmtDouble(intake->getRunnerFlowRate()).c_str());
    e.line("ip%d.IdleThrottlePlatePosition = %s;", inIdx, fmtDouble(intake->getIdleThrottlePlatePosition()).c_str());
    e.line("ip%d.RunnerLength = %s;", inIdx, fmtDouble(intake->getRunnerLength()).c_str());
    e.line("ip%d.VelocityDecay = %s;", inIdx, fmtDouble(intake->getVelocityDecay()).c_str());
    e.line("engine->getIntake(%d)->initialize(ip%d);", inIdx, inIdx);

    // Initialize manifold with air mixture (21% O2, 79% N2)
    e.line("{");
    e.line("    GasSystem::Mix airMix;");
    e.line("    airMix.p_fuel = 0.0;");
    e.line("    airMix.p_inert = 0.79;");
    e.line("    airMix.p_o2 = 0.21;");
    e.line("    engine->getIntake(%d)->m_system.reset(units::pressure(1.0, units::atm),", inIdx);
    e.line("                                          units::celcius(25.0), airMix);");
    e.line("}");
    e.line("");
}

static void emitCamshaft(CppEmitter& e, Camshaft* cam, const char* varName, int cylinderCount) {
    e.line("Camshaft* %s = new Camshaft();", varName);
    e.line("Camshaft::Parameters %s_params = {};", varName);
    e.line("%s_params.lobes = %d;", varName, cam->getLobeCount());
    e.line("%s_params.advance = %s;", varName, fmtDouble(cam->getAdvance()).c_str());
    e.line("%s_params.baseRadius = %s;", varName, fmtDouble(cam->getBaseRadius()).c_str());
    e.line("%s_params.crankshaft = mainCrank;", varName);
    emitFunction(e, (std::string(varName) + "_lobeProfile").c_str(), cam->getLobeProfile());
    e.line("%s_params.lobeProfile = %s_lobeProfile;", varName, varName);
    e.line("%s->initialize(%s_params);", varName, varName);

    // Lobe centerlines: JSON stores cam-angle (crank/2), setLobeCenterline divides by 2 again
    // So multiply by 2 to undo the double-division
    for (int i = 0; i < cam->getLobeCount(); i++) {
        e.line("%s->setLobeCenterline(%d, %s * 2);",
            varName, i, fmtDouble(cam->getLobeCenterline(i)).c_str());
    }
    e.line("");
}

static void emitPortFlow(CppEmitter& e, Function* portFlow, const char* varName) {
    if (!portFlow) {
        e.line("Function* %s = nullptr;", varName);
        return;
    }
    e.line("// Port flow function (%d samples, filterRadius=%s)", portFlow->getSampleCount(),
        fmtDouble(portFlow->getFilterRadius()).c_str());
    e.line("Function* %s = new Function;", varName);
    e.line("%s->initialize(%d, %s);", varName,
        portFlow->getSampleCount(),
        fmtDouble(portFlow->getFilterRadius()).c_str());
    for (int i = 0; i < portFlow->getSampleCount(); i++) {
        e.line("%s->addSample(%s, %s);",
            varName,
            fmtDouble(portFlow->getX(i)).c_str(),
            fmtDouble(portFlow->getY(i)).c_str());
    }
    e.line("");
}

static void emitCylinderBank(CppEmitter& e, Engine* engine, int bankIdx, int globalCylOffset) {
    CylinderBank* bank = engine->getCylinderBank(bankIdx);
    CylinderHead* head = engine->getHead(bankIdx);
    int cylCount = bank->getCylinderCount();

    e.line("CylinderBank::Parameters bank%d_params = {};", bankIdx);
    e.line("bank%d_params.crankshaft = mainCrank;", bankIdx);
    e.line("bank%d_params.angle = %s;", bankIdx, fmtDouble(bank->getAngle()).c_str());
    e.line("bank%d_params.bore = %s;", bankIdx, fmtDouble(bank->getBore()).c_str());
    e.line("bank%d_params.deckHeight = %s;", bankIdx, fmtDouble(bank->getDeckHeight()).c_str());
    e.line("bank%d_params.displayDepth = %s;", bankIdx, fmtDouble(bank->getDisplayDepth()).c_str());
    e.line("bank%d_params.index = %d;", bankIdx, bank->getIndex());
    e.line("bank%d_params.cylinderCount = %d;", bankIdx, cylCount);
    e.line("bank%d_params.positionX = %s;", bankIdx, fmtDouble(bank->getX()).c_str());
    e.line("bank%d_params.positionY = %s;", bankIdx, fmtDouble(bank->getY()).c_str());
    e.line("engine->getCylinderBank(%d)->initialize(bank%d_params);", bankIdx, bankIdx);
    e.line("");

    // Per-cylinder: pistons and connecting rods
    for (int ci = 0; ci < cylCount; ci++) {
        int pidx = globalCylOffset + ci;
        Piston* piston = engine->getPiston(pidx);
        ConnectingRod* rod = engine->getConnectingRod(pidx);

        // Connecting rod (must init before piston)
        e.line("ConnectingRod::Parameters rod%d = {};", pidx);
        e.line("rod%d.mass = %s;", pidx, fmtDouble(rod->getMass()).c_str());
        e.line("rod%d.momentOfInertia = %s;", pidx, fmtDouble(rod->getMomentOfInertia()).c_str());
        e.line("rod%d.centerOfMass = %s;", pidx, fmtDouble(rod->getCenterOfMass()).c_str());
        e.line("rod%d.length = %s;", pidx, fmtDouble(rod->getLength()).c_str());
        e.line("rod%d.crankshaft = mainCrank;", pidx);
        e.line("rod%d.journal = %d;", pidx, rod->getJournal());
        e.line("rod%d.piston = engine->getPiston(%d);", pidx, pidx);
        e.line("engine->getConnectingRod(%d)->initialize(rod%d);", pidx, pidx);

        // Piston
        e.line("Piston::Parameters piston%d = {};", pidx);
        e.line("piston%d.Bank = engine->getCylinderBank(%d);", pidx, bankIdx);
        e.line("piston%d.Rod = engine->getConnectingRod(%d);", pidx, pidx);
        e.line("piston%d.CylinderIndex = %d;", pidx, ci);
        e.line("piston%d.BlowbyFlowCoefficient = %s;", pidx, fmtDouble(piston->getBlowbyK()).c_str());
        e.line("piston%d.CompressionHeight = %s;", pidx, fmtDouble(piston->getCompressionHeight()).c_str());
        e.line("piston%d.WristPinPosition = %s;", pidx, fmtDouble(piston->getWristPinLocation()).c_str());
        e.line("piston%d.Displacement = %s;", pidx, fmtDouble(piston->getDisplacement()).c_str());
        e.line("piston%d.mass = %s;", pidx, fmtDouble(piston->getMass()).c_str());
        e.line("engine->getPiston(%d)->initialize(piston%d);", pidx, pidx);
        e.line("");
    }

    // Cylinder head + camshafts
    if (head) {
        Camshaft* intakeCam = head->getIntakeCamshaft();
        Camshaft* exhaustCam = head->getExhaustCamshaft();
        char intakeCamName[64], exhaustCamName[64];
        char intakePortFlowName[64], exhaustPortFlowName[64];
        char valvetrainName[64];
        snprintf(intakeCamName, sizeof(intakeCamName), "bank%d_intakeCam", bankIdx);
        snprintf(exhaustCamName, sizeof(exhaustCamName), "bank%d_exhaustCam", bankIdx);
        snprintf(intakePortFlowName, sizeof(intakePortFlowName), "bank%d_intakePortFlow", bankIdx);
        snprintf(exhaustPortFlowName, sizeof(exhaustPortFlowName), "bank%d_exhaustPortFlow", bankIdx);
        snprintf(valvetrainName, sizeof(valvetrainName), "bank%d_valvetrain", bankIdx);

        if (intakeCam) {
            emitCamshaft(e, intakeCam, intakeCamName, cylCount);
        }
        if (exhaustCam) {
            emitCamshaft(e, exhaustCam, exhaustCamName, cylCount);
        }

        // Valvetrain
        e.line("StandardValvetrain* %s = new StandardValvetrain();", valvetrainName);
        e.line("StandardValvetrain::Parameters %s_params = {};", valvetrainName);
        if (intakeCam) e.line("%s_params.intakeCamshaft = %s;", valvetrainName, intakeCamName);
        if (exhaustCam) e.line("%s_params.exhaustCamshaft = %s;", valvetrainName, exhaustCamName);
        e.line("%s->initialize(%s_params);", valvetrainName, valvetrainName);
        e.line("");

        // Head params
        e.line("CylinderHead::Parameters head%d_params = {};", bankIdx);
        e.line("head%d_params.Bank = engine->getCylinderBank(%d);", bankIdx, bankIdx);
        e.line("head%d_params.CombustionChamberVolume = %s;", bankIdx, fmtDouble(head->getCombustionChamberVolume()).c_str());
        e.line("head%d_params.IntakeRunnerVolume = %s;", bankIdx, fmtDouble(head->getIntakeRunnerVolume()).c_str());
        e.line("head%d_params.IntakeRunnerCrossSectionArea = %s;", bankIdx, fmtDouble(head->getIntakeRunnerCrossSectionArea()).c_str());
        e.line("head%d_params.ExhaustRunnerVolume = %s;", bankIdx, fmtDouble(head->getExhaustRunnerVolume()).c_str());
        e.line("head%d_params.ExhaustRunnerCrossSectionArea = %s;", bankIdx, fmtDouble(head->getExhaustRunnerCrossSectionArea()).c_str());
        e.line("head%d_params.FlipDisplay = %s;", bankIdx, head->getFlipDisplay() ? "true" : "false");
        e.line("head%d_params.Valvetrain = %s;", bankIdx, valvetrainName);

        // Port flows
        emitPortFlow(e, head->getIntakePortFlow(), intakePortFlowName);
        e.line("head%d_params.IntakePortFlow = %s;", bankIdx, intakePortFlowName);
        emitPortFlow(e, head->getExhaustPortFlow(), exhaustPortFlowName);
        e.line("head%d_params.ExhaustPortFlow = %s;", bankIdx, exhaustPortFlowName);

        e.line("engine->getHead(%d)->initialize(head%d_params);", bankIdx, bankIdx);

        // Wire intake and exhaust per-cylinder from the compiled engine's actual wiring
        for (int ci = 0; ci < cylCount; ci++) {
            Intake* wiredIntake = head->getIntake(ci);
            if (wiredIntake) {
                // Find which intake index this is
                for (int ii = 0; ii < engine->getIntakeCount(); ii++) {
                    if (engine->getIntake(ii) == wiredIntake) {
                        e.line("engine->getHead(%d)->setIntake(%d, engine->getIntake(%d));",
                            bankIdx, ci, ii);
                        break;
                    }
                }
            }
            ExhaustSystem* wiredExhaust = head->getExhaustSystem(ci);
            if (wiredExhaust) {
                for (int ei = 0; ei < engine->getExhaustSystemCount(); ei++) {
                    if (engine->getExhaustSystem(ei) == wiredExhaust) {
                        e.line("engine->getHead(%d)->setExhaustSystem(%d, engine->getExhaustSystem(%d));",
                            bankIdx, ci, ei);
                        break;
                    }
                }
            }
        }

        // Per-cylinder head data
        for (int ci = 0; ci < cylCount; ci++) {
            e.line("engine->getHead(%d)->setSoundAttenuation(%d, %s);",
                bankIdx, ci, fmtDouble(head->getSoundAttenuation(ci)).c_str());
            e.line("engine->getHead(%d)->setHeaderPrimaryLength(%d, %s);",
                bankIdx, ci, fmtDouble(head->getHeaderPrimaryLength(ci)).c_str());
        }
        e.line("");
    }
}

static void emitVehicle(CppEmitter& e, Vehicle* vehicle) {
    e.line("// Vehicle");
    e.line("Vehicle* vehicle = new Vehicle();");
    e.line("Vehicle::Parameters vp = {};");
    e.line("vp.mass = %s;", fmtDouble(vehicle->getMass()).c_str());
    e.line("vp.dragCoefficient = %s;", fmtDouble(vehicle->getDragCoefficient()).c_str());
    e.line("vp.crossSectionArea = %s;", fmtDouble(vehicle->getCrossSectionArea()).c_str());
    e.line("vp.diffRatio = %s;", fmtDouble(vehicle->getDiffRatio()).c_str());
    e.line("vp.tireRadius = %s;", fmtDouble(vehicle->getTireRadius()).c_str());
    e.line("vp.rollingResistance = %s;", fmtDouble(vehicle->getRollingResistance()).c_str());
    e.line("vehicle->initialize(vp);");
    e.line("");
}

static void emitTransmission(CppEmitter& e, Transmission* transmission) {
    e.line("// Transmission");
    int gearCount = transmission->getGearCount();
    e.line("static const double gearRatios[] = {");
    for (int i = 0; i < gearCount; i++) {
        e.line("    %s%s", fmtDouble(transmission->getGearRatio(i)).c_str(),
            (i < gearCount - 1) ? "," : "");
    }
    e.line("};");
    e.line("Transmission* transmission = new Transmission();");
    e.line("Transmission::Parameters tp = {};");
    e.line("tp.GearCount = %d;", gearCount);
    e.line("tp.GearRatios = gearRatios;");
    e.line("tp.MaxClutchTorque = %s;", fmtDouble(transmission->getMaxClutchTorque()).c_str());
    e.line("transmission->initialize(tp);");
    e.line("");
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
    std::string scriptPath, outputPath, className;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--script" && i + 1 < argc) {
            scriptPath = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            outputPath = argv[++i];
        } else if (arg == "--classname" && i + 1 < argc) {
            className = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            printf("Usage: %s --script <path.mr> --output <path.cpp> --classname <name>\n", argv[0]);
            return 0;
        }
    }

    if (scriptPath.empty() || outputPath.empty() || className.empty()) {
        fprintf(stderr, "Error: --script, --output, and --classname are required\n");
        fprintf(stderr, "Usage: %s --script <path.mr> --output <path.cpp> --classname <name>\n", argv[0]);
        return 1;
    }

    if (!std::filesystem::exists(scriptPath)) {
        fprintf(stderr, "Error: Script not found: %s\n", scriptPath.c_str());
        return 1;
    }

    // Find engine-sim root (look for es/engine_sim.mr)
    std::filesystem::path absScript = std::filesystem::absolute(scriptPath);
    std::filesystem::path search = absScript.parent_path();
    std::filesystem::path simDir;
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
        return 1;
    }

    std::filesystem::path assetsDir = simDir / "assets";
    std::string relativeImport = std::filesystem::relative(absScript, assetsDir).string();

    printf("Engine-sim dir: %s\n", simDir.c_str());
    printf("Script:        %s\n", scriptPath.c_str());
    printf("Class name:    %s\n", className.c_str());

    // Generate wrapper script for Piranha
    std::filesystem::path wrapperPath = assetsDir / "_escli_codegen_wrapper.mr";
    {
        std::ofstream wrapper(wrapperPath);
        if (!wrapper.is_open()) {
            fprintf(stderr, "Error: Cannot create wrapper: %s\n", wrapperPath.c_str());
            return 1;
        }
        wrapper << "import \"engine_sim.mr\"\n";
        wrapper << "import \"" << relativeImport << "\"\n";
        wrapper << "main()\n";
    }

    // Change to engine-sim root for Piranha include resolution
    std::filesystem::path originalCwd = std::filesystem::current_path();
    std::filesystem::current_path(simDir);

    es_script::Compiler compiler;
    compiler.initialize();

    printf("Compiling...\n");
    if (!compiler.compile(wrapperPath.string())) {
        std::filesystem::current_path(originalCwd);
        fprintf(stderr, "Error: Failed to compile: %s\n", scriptPath.c_str());
        return 1;
    }

    es_script::Compiler::Output output = compiler.execute();
    std::filesystem::current_path(originalCwd);

    // Clean up wrapper
    std::filesystem::remove(wrapperPath);

    if (!output.engine) {
        fprintf(stderr, "Error: Script did not produce an engine\n");
        compiler.destroy();
        return 1;
    }

    Engine* engine = output.engine;
    Vehicle* vehicle = output.vehicle;
    Transmission* transmission = output.transmission;
    printf("Engine: %s (%d cyl, %d banks)\n",
        engine->getName().c_str(), engine->getCylinderCount(), engine->getCylinderBankCount());
    if (vehicle) printf("Vehicle: mass=%.1f kg\n", vehicle->getMass());
    if (transmission) printf("Transmission: %d gears\n", transmission->getGearCount());

    // ========================================================================
    // Emit C++ source
    // ========================================================================
    CppEmitter e;

    e.line("// Auto-generated by engine-sim-preset-codegen from:");
    e.line("//   %s", scriptPath.c_str());
    e.line("// DO NOT EDIT — regenerate with preset_codegen");
    e.line("");
    e.line("#include \"engine.h\"");
    e.line("#include \"crankshaft.h\"");
    e.line("#include \"piston.h\"");
    e.line("#include \"connecting_rod.h\"");
    e.line("#include \"cylinder_bank.h\"");
    e.line("#include \"cylinder_head.h\"");
    e.line("#include \"camshaft.h\"");
    e.line("#include \"standard_valvetrain.h\"");
    e.line("#include \"exhaust_system.h\"");
    e.line("#include \"intake.h\"");
    e.line("#include \"fuel.h\"");
    e.line("#include \"function.h\"");
    e.line("#include \"gas_system.h\"");
    e.line("#include \"vehicle.h\"");
    e.line("#include \"transmission.h\"");
    e.line("#include \"simulator.h\"");
    e.line("#include \"piston_engine_simulator.h\"");
    e.line("#include \"units.h\"");
    e.line("#include \"direct_throttle_linkage.h\"");
    e.line("#include \"simulator/EnginePresetsHelper.h\"");
    e.line("");
    e.line("#ifndef M_PI");
    e.line("#define M_PI 3.14159265358979323846");
    e.line("#endif");
    e.line("");
    e.line("class %s {", className.c_str());
    e.line("public:");
    e.line("    static Simulator* create() {");
    e.indent_++;

    // Engine parameters
    e.line("Engine::Parameters params = {};");
    e.line("params.name = \"%s\";", engine->getName().c_str());
    e.line("params.cylinderBanks = %d;", engine->getCylinderBankCount());
    e.line("params.cylinderCount = %d;", engine->getCylinderCount());
    e.line("params.crankshaftCount = %d;", engine->getCrankshaftCount());
    e.line("params.exhaustSystemCount = %d;", engine->getExhaustSystemCount());
    e.line("params.intakeCount = %d;", engine->getIntakeCount());
    e.line("params.starterTorque = %s;", fmtDouble(engine->getStarterTorque()).c_str());
    e.line("params.starterSpeed = %s;", fmtDouble(engine->getStarterSpeed()).c_str());
    e.line("params.redline = %s;", fmtDouble(engine->getRedline()).c_str());
    e.line("params.dynoMinSpeed = %s;", fmtDouble(engine->getDynoMinSpeed()).c_str());
    e.line("params.dynoMaxSpeed = %s;", fmtDouble(engine->getDynoMaxSpeed()).c_str());
    e.line("params.dynoHoldStep = %s;", fmtDouble(engine->getDynoHoldStep()).c_str());
    e.line("params.initialSimulationFrequency = %s;", fmtDouble(engine->getSimulationFrequency()).c_str());
    e.line("params.initialHighFrequencyGain = %s;", fmtDouble(engine->getInitialHighFrequencyGain()).c_str());
    e.line("params.initialNoise = %s;", fmtDouble(engine->getInitialNoise()).c_str());
    e.line("params.initialJitter = %s;", fmtDouble(engine->getInitialJitter()).c_str());

    // DirectThrottleLinkage
    e.line("DirectThrottleLinkage* throttle = new DirectThrottleLinkage();");
    e.line("DirectThrottleLinkage::Parameters throttleParams;");
    e.line("throttleParams.gamma = 1.0;");
    e.line("throttle->initialize(throttleParams);");
    e.line("params.throttle = throttle;");
    e.line("");

    // Create engine
    e.line("Engine* engine = new Engine();");
    e.line("engine->initialize(params);");
    e.line("");

    // Crankshafts
    for (int i = 0; i < engine->getCrankshaftCount(); i++) {
        emitCrankshaft(e, engine, i);
    }
    e.line("Crankshaft* mainCrank = engine->getCrankshaft(0);");

    // Exhaust systems
    for (int i = 0; i < engine->getExhaustSystemCount(); i++) {
        emitExhaustSystem(e, engine, i);
    }

    // Intakes
    for (int i = 0; i < engine->getIntakeCount(); i++) {
        emitIntake(e, engine, i);
    }

    // Cylinder banks
    int globalCylIdx = 0;
    for (int bi = 0; bi < engine->getCylinderBankCount(); bi++) {
        emitCylinderBank(e, engine, bi, globalCylIdx);
        globalCylIdx += engine->getCylinderBank(bi)->getCylinderCount();
    }

    // Fuel
    Fuel* fuel = engine->getFuel();
    if (fuel) {
        e.line("Function* turbulenceFn = EnginePresetsHelper::createDefaultTurbulenceToFlameSpeedRatio();");
        e.line("Fuel::Parameters fuelParams = {};");
        e.line("fuelParams.maxBurningEfficiency = %s;", fmtDouble(fuel->getMaxBurningEfficiency()).c_str());
        e.line("fuelParams.burningEfficiencyRandomness = %s;", fmtDouble(fuel->getBurningEfficiencyRandomness()).c_str());
        e.line("fuelParams.lowEfficiencyAttenuation = %s;", fmtDouble(fuel->getLowEfficiencyAttenuation()).c_str());
        e.line("fuelParams.maxTurbulenceEffect = %s;", fmtDouble(fuel->getMaxTurbulenceEffect()).c_str());
        e.line("fuelParams.maxDilutionEffect = %s;", fmtDouble(fuel->getMaxDilutionEffect()).c_str());
        e.line("fuelParams.molecularAfr = %s;", fmtDouble(fuel->getMolecularAfr()).c_str());
        e.line("fuelParams.turbulenceToFlameSpeedRatio = turbulenceFn;");
        e.line("engine->getFuel()->initialize(fuelParams);");
        e.line("");
    }

    // Ignition — read actual values from the Piranha-compiled engine
    IgnitionModule* srcIgnition = engine->getIgnitionModule();
    e.line("IgnitionModule* ignition = engine->getIgnitionModule();");
    e.line("IgnitionModule::Parameters igParams = {};");
    e.line("igParams.cylinderCount = %d;", engine->getCylinderCount());
    e.line("igParams.crankshaft = mainCrank;");
    e.line("");

    // Timing curve from actual engine
    Function* srcTimingCurve = srcIgnition->getTimingCurve();
    if (srcTimingCurve) {
        emitFunction(e, "timingCurve", srcTimingCurve);
        e.line("igParams.timingCurve = timingCurve;");
    } else {
        e.line("// Warning: no timing curve on source engine");
        e.line("igParams.timingCurve = nullptr;");
    }

    e.line("igParams.revLimit = %s;", fmtDouble(srcIgnition->getRevLimit()).c_str());
    e.line("igParams.limiterDuration = %s;", fmtDouble(srcIgnition->getLimiterDuration()).c_str());
    e.line("ignition->initialize(igParams);");
    e.line("");

    // Firing order — read actual angles from compiled engine
    e.line("const double fourPi = 4.0 * M_PI;");
    for (int i = 0; i < engine->getCylinderCount(); i++) {
        e.line("ignition->setFiringOrder(%d, %s);", i, fmtDouble(srcIgnition->getFiringOrder(i)).c_str());
    }
    e.line("");

    // Combustion chambers
    e.line("Function* turbFn = EnginePresetsHelper::createMeanPistonSpeedToTurbulence();");
    e.line("CombustionChamber::Parameters ccParams = {};");
    e.line("ccParams.CrankcasePressure = units::pressure(1.0, units::atm);");
    e.line("ccParams.Fuel = engine->getFuel();");
    e.line("ccParams.StartingPressure = units::pressure(1.0, units::atm);");
    e.line("ccParams.StartingTemperature = units::celcius(25.0);");
    e.line("ccParams.MeanPistonSpeedToTurbulence = turbFn;");
    e.line("for (int i = 0; i < engine->getCylinderCount(); i++) {");
    e.line("    ccParams.Piston = engine->getPiston(i);");
    e.line("    ccParams.Head = engine->getHead(ccParams.Piston->getCylinderBank()->getIndex());");
    e.line("    engine->getChamber(i)->initialize(ccParams);");
    e.line("    engine->getChamber(i)->setEngine(engine);");
    e.line("}");
    e.line("");
    e.line("engine->calculateDisplacement();");

    // Vehicle
    if (vehicle) {
        emitVehicle(e, vehicle);
    } else {
        e.line("// No vehicle in script — creating default");
        e.line("Vehicle* vehicle = nullptr;");
        e.line("");
    }

    // Transmission
    if (transmission) {
        emitTransmission(e, transmission);
    } else {
        e.line("// No transmission in script — creating default");
        e.line("Transmission* transmission = nullptr;");
        e.line("");
    }

    // Create simulator
    e.line("auto* sim = new PistonEngineSimulator();");
    e.line("Simulator::Parameters simParams;");
    e.line("simParams.systemType = Simulator::SystemType::NsvOptimized;");
    e.line("sim->initialize(simParams);");
    e.line("sim->setSimulationFrequency(engine->getSimulationFrequency());");
    e.line("sim->setTargetSynthesizerLatency(0.05);");
    e.line("sim->loadSimulation(engine, vehicle, transmission);");
    e.line("");
    e.line("return sim;");

    e.indent_--;
    e.line("}");
    e.line("};");
    e.line("");

    // Write output
    std::ofstream outFile(outputPath);
    if (!outFile.is_open()) {
        fprintf(stderr, "Error: Cannot open output: %s\n", outputPath.c_str());
        compiler.destroy();
        return 1;
    }

    outFile << e.buf_;
    outFile.close();

    printf("Output: %s (%zu bytes)\n", outputPath.c_str(), e.buf_.size());

    compiler.destroy();
    return 0;
}
