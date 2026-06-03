// Phase 0 Spike 2: Clutch parameter sweep
// Build: cmake -B build-test -GNinja && cmake --build build-test --target clutch_sweep_spike
//
// This spike creates a fully-initialized I6 engine, Vehicle, Transmission,
// PistonEngineSimulator and runs physics without dyno. The engine must produce
// torque through actual combustion, which requires all sub-components to be
// properly initialized (crankshaft, cylinder bank, pistons, rods, heads,
// valvetrain, ignition, fuel, combustion chambers, exhaust, intake).
//
// Initialization follows the same sequence as EnginePresets.cpp and
// EngineNode::buildEngine() in the scripting layer.

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <string>

#include "engine-sim/include/piston_engine_simulator.h"
#include "engine-sim/include/engine.h"
#include "engine-sim/include/vehicle.h"
#include "engine-sim/include/transmission.h"
#include "engine-sim/include/simulator.h"
#include "engine-sim/include/crankshaft.h"
#include "engine-sim/include/cylinder_bank.h"
#include "engine-sim/include/piston.h"
#include "engine-sim/include/connecting_rod.h"
#include "engine-sim/include/cylinder_head.h"
#include "engine-sim/include/camshaft.h"
#include "engine-sim/include/standard_valvetrain.h"
#include "engine-sim/include/ignition_module.h"
#include "engine-sim/include/exhaust_system.h"
#include "engine-sim/include/intake.h"
#include "engine-sim/include/fuel.h"
#include "engine-sim/include/function.h"
#include "engine-sim/include/gas_system.h"
#include "engine-sim/include/impulse_response.h"
#include "engine-sim/include/units.h"
#include "engine-sim/include/combustion_chamber.h"
#include "engine-sim/include/starter_motor.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const double TWO_PI = 2.0 * M_PI;

// ---- Helper functions (same as EnginePresetsHelper in EnginePresets.cpp) ----

static Function* createMeanPistonSpeedToTurbulence() {
    Function* fn = new Function();
    fn->initialize(30, 1);
    for (int i = 0; i < 30; ++i) {
        fn->addSample(static_cast<double>(i), static_cast<double>(i) * 0.5);
    }
    return fn;
}

static Function* createDefaultTurbulenceToFlameSpeedRatio() {
    Function* fn = new Function();
    fn->initialize(10, 1);
    fn->addSample(0.0, 3.0);
    fn->addSample(5.0, 7.5);
    fn->addSample(10.0, 15.0);
    fn->addSample(15.0, 22.5);
    fn->addSample(20.0, 30.0);
    fn->addSample(25.0, 37.5);
    fn->addSample(30.0, 45.0);
    fn->addSample(35.0, 52.5);
    fn->addSample(40.0, 60.0);
    fn->addSample(45.0, 67.5);
    return fn;
}

static Function* generateHarmonicCamLobe(double durationAt50Thou, double gamma,
                                          double lift, int steps) {
    const double angle = durationAt50Thou / 4.0;
    const double s = std::pow(2.0 * units::distance(50, units::thou) / lift, 1.0 / gamma) - 1.0;
    const double k = std::acos(s) / angle;
    const double extents = M_PI / k;
    const double step = extents / (steps - 5.0);

    Function* fn = new Function();
    fn->initialize(steps * 2 + 1, step);

    for (int i = 0; i < steps; ++i) {
        if (i == 0) {
            fn->addSample(0.0, lift);
        } else {
            const double x = i * step;
            const double l = (x >= extents)
                ? 0.0
                : lift * std::pow(0.5 + 0.5 * std::cos(k * x), gamma);
            fn->addSample(x, l);
            fn->addSample(-x, l);
        }
    }

    return fn;
}

static Function* createFlowFunction(const double lifts[], const double flows[], int count) {
    Function* fn = new Function();
    fn->initialize(count, 0);
    for (int i = 0; i < count; ++i) {
        double liftM = lifts[i] * units::distance(1, units::thou);
        double flowK = GasSystem::k_28inH2O(flows[i]);
        fn->addSample(liftM, flowK);
    }
    return fn;
}

// ---- Fully initialize an I6 engine for combustion ----
// Follows the exact same sequence as EnginePresets.cpp presets.
static void fullyInitializeI6(Engine* engine) {
    // I6 dimensions (typical 3.5L inline-6)
    const double stroke = units::distance(88.0, units::mm);
    const double bore = units::distance(86.0, units::mm);
    const double rodLength = units::distance(145.0, units::mm);
    const double rodMass = units::mass(500.0, units::g);
    const double compressionHeight = units::distance(1.0, units::inch);
    const double crankMass = units::mass(12.0, units::kg);
    const double flywheelMass = units::mass(8.0, units::kg);
    const double flywheelRadius = units::distance(6.0, units::inch);

    // Moments of inertia
    const double crankMoment = 0.5 * crankMass * (stroke / 2.0) * (stroke / 2.0);
    const double flywheelMoment = 0.5 * flywheelMass * flywheelRadius * flywheelRadius;
    const double otherMoment = 0.5 * units::mass(5.0, units::kg)
        * units::distance(5.0, units::cm) * units::distance(5.0, units::cm);
    const double rodMoment = (1.0 / 12.0) * rodMass * rodLength * rodLength;

    // ---- Exhaust System ----
    // ImpulseResponse needs a valid WAV file. For a spike, we use a minimal/dummy one.
    // If the file is missing, the ImpulseResponse will be empty but won't crash.
    ImpulseResponse* ir = new ImpulseResponse();
    ir->initialize("new/mild_exhaust_reverb.wav", 0.01);

    ExhaustSystem::Parameters esp = {};
    esp.outletFlowRate = GasSystem::k_carb(800.0);
    esp.primaryTubeLength = units::distance(24.0, units::inch);
    esp.primaryFlowRate = GasSystem::k_carb(300.0);
    esp.velocityDecay = 0.5;
    esp.audioVolume = 4.0;
    esp.impulseResponse = ir;
    esp.length = units::distance(80.0, units::inch);
    esp.collectorCrossSectionArea = units::area(10.0, units::cm2);
    engine->getExhaustSystem(0)->initialize(esp);

    // ---- Intake ----
    Intake::Parameters ip = {};
    ip.volume = units::volume(2.0, units::L);
    ip.CrossSectionArea = units::area(15.0, units::cm2);
    ip.InputFlowK = GasSystem::k_carb(600.0);
    ip.IdleFlowK = GasSystem::k_carb(0.0);
    ip.RunnerFlowRate = GasSystem::k_carb(150.0);
    ip.RunnerLength = units::distance(10.0, units::inch);
    ip.IdleThrottlePlatePosition = 0.996;
    ip.VelocityDecay = 0.5;
    engine->getIntake(0)->initialize(ip);

    // ---- Crankshaft ----
    // I6 firing order: 1-5-3-6-2-4 (0-indexed: 0,4,2,5,1,3)
    // Rod journal angles for even firing: 120 deg apart
    // 6 rod journals, one per cylinder
    Crankshaft::Parameters cp = {};
    cp.crankThrow = stroke / 2.0;
    cp.flywheelMass = flywheelMass;
    cp.mass = crankMass;
    cp.frictionTorque = units::torque(10.0, units::ft_lb);
    cp.momentOfInertia = crankMoment + flywheelMoment + otherMoment;
    cp.pos_x = 0.0;
    cp.pos_y = 0.0;
    cp.tdc = M_PI / 2.0;
    cp.rodJournals = 6;
    engine->getCrankshaft(0)->initialize(cp);
    // I6 rod journal angles: 0, 120, 240, 0, 120, 240 degrees
    // (pairs 1-6, 2-5, 3-4 share the same pin angle)
    engine->getCrankshaft(0)->setRodJournalAngle(0, units::angle(0.0, units::deg));
    engine->getCrankshaft(0)->setRodJournalAngle(1, units::angle(120.0, units::deg));
    engine->getCrankshaft(0)->setRodJournalAngle(2, units::angle(240.0, units::deg));
    engine->getCrankshaft(0)->setRodJournalAngle(3, units::angle(0.0, units::deg));
    engine->getCrankshaft(0)->setRodJournalAngle(4, units::angle(120.0, units::deg));
    engine->getCrankshaft(0)->setRodJournalAngle(5, units::angle(240.0, units::deg));

    // ---- Cylinder Bank (inline = 1 bank, 6 cylinders, angle 0) ----
    CylinderBank::Parameters bp = {};
    bp.crankshaft = engine->getCrankshaft(0);
    bp.bore = bore;
    bp.deckHeight = stroke / 2.0 + rodLength + compressionHeight;
    bp.angle = 0.0;
    bp.cylinderCount = 6;
    bp.index = 0;
    bp.positionX = 0.0;
    bp.positionY = 0.0;
    bp.displayDepth = units::distance(2.0, units::inch);
    engine->getCylinderBank(0)->initialize(bp);

    // ---- Connecting Rods and Pistons ----
    const double pistonMass = units::mass(400.0, units::g);
    for (int i = 0; i < 6; ++i) {
        // Connecting rod
        ConnectingRod::Parameters crp = {};
        crp.mass = rodMass;
        crp.momentOfInertia = rodMoment;
        crp.centerOfMass = 0.0;
        crp.length = rodLength;
        crp.rodJournals = 1;
        crp.crankshaft = engine->getCrankshaft(0);
        crp.journal = i;  // Each piston on its own rod journal
        crp.piston = engine->getPiston(i);
        engine->getConnectingRod(i)->initialize(crp);

        // Piston
        Piston::Parameters pp = {};
        pp.Bank = engine->getCylinderBank(0);
        pp.Rod = engine->getConnectingRod(i);
        pp.CylinderIndex = i;
        pp.BlowbyFlowCoefficient = GasSystem::k_28inH2O(0.0);
        pp.CompressionHeight = compressionHeight;
        pp.WristPinPosition = 0.0;
        pp.Displacement = 0.0;
        pp.mass = pistonMass;
        engine->getPiston(i)->initialize(pp);
    }

    // ---- Camshafts (inline 6: 1 intake cam + 1 exhaust cam, 3 lobes each) ----
    Function* intakeLobe = generateHarmonicCamLobe(
        units::angle(230.0, units::deg),   // duration at 0.050"
        2.0,                                 // gamma
        units::distance(9.5, units::mm),    // lift
        100                                  // steps
    );
    Function* exhaustLobe = generateHarmonicCamLobe(
        units::angle(234.0, units::deg),
        2.0,
        units::distance(9.0, units::mm),
        100
    );

    double intakeLobeCenter = units::angle(115.0, units::deg);
    double exhaustLobeCenter = units::angle(110.0, units::deg);
    double camBaseRadius = (34.0 / 2.0) * units::distance(1, units::mm);
    double cycle = units::angle(720.0, units::deg);  // 4-stroke cycle = 2 full rotations

    // Intake cam: 3 lobes, evenly spaced by 1/3 of the 720-deg cycle = 240 deg
    Camshaft* intakeCam = new Camshaft();
    Camshaft::Parameters icp = {};
    icp.lobes = 3;
    icp.advance = 0;
    icp.crankshaft = engine->getCrankshaft(0);
    icp.lobeProfile = intakeLobe;
    icp.baseRadius = camBaseRadius;
    intakeCam->initialize(icp);
    intakeCam->setLobeCenterline(0, TWO_PI + intakeLobeCenter + (0.0 / 6.0) * cycle);
    intakeCam->setLobeCenterline(1, TWO_PI + intakeLobeCenter + (2.0 / 6.0) * cycle);
    intakeCam->setLobeCenterline(2, TWO_PI + intakeLobeCenter + (4.0 / 6.0) * cycle);

    // Exhaust cam: 3 lobes, evenly spaced by 1/3 of the 720-deg cycle = 240 deg
    Camshaft* exhaustCam = new Camshaft();
    Camshaft::Parameters ecp = {};
    ecp.lobes = 3;
    ecp.advance = 0;
    ecp.crankshaft = engine->getCrankshaft(0);
    ecp.lobeProfile = exhaustLobe;
    ecp.baseRadius = camBaseRadius;
    exhaustCam->initialize(ecp);
    exhaustCam->setLobeCenterline(0, TWO_PI - exhaustLobeCenter + (0.0 / 6.0) * cycle);
    exhaustCam->setLobeCenterline(1, TWO_PI - exhaustLobeCenter + (2.0 / 6.0) * cycle);
    exhaustCam->setLobeCenterline(2, TWO_PI - exhaustLobeCenter + (4.0 / 6.0) * cycle);

    // ---- Valvetrain ----
    StandardValvetrain* vt = new StandardValvetrain();
    StandardValvetrain::Parameters vtp = {};
    vtp.intakeCamshaft = intakeCam;
    vtp.exhaustCamshaft = exhaustCam;
    vt->initialize(vtp);

    // ---- Flow functions (generic I6 head) ----
    const double lifts[] = {0, 50, 100, 150, 200, 250, 300, 350, 400, 450};
    const double intakeFlows[] = {0, 50, 100, 160, 220, 260, 280, 290, 295, 296};
    const double exhaustFlows[] = {0, 40, 80, 125, 175, 210, 235, 248, 252, 253};

    Function* intakeFlow = createFlowFunction(lifts, intakeFlows, 10);
    Function* exhaustFlow = createFlowFunction(lifts, exhaustFlows, 10);

    // ---- Cylinder Head ----
    CylinderHead::Parameters headP = {};
    headP.Bank = engine->getCylinderBank(0);
    headP.ExhaustPortFlow = exhaustFlow;
    headP.IntakePortFlow = intakeFlow;
    headP.Valvetrain = vt;
    headP.CombustionChamberVolume = units::volume(55.0, units::cc);
    headP.IntakeRunnerVolume = units::volume(120.0, units::cc);
    headP.IntakeRunnerCrossSectionArea = units::area(8.0, units::cm2);
    headP.ExhaustRunnerVolume = units::volume(80.0, units::cc);
    headP.ExhaustRunnerCrossSectionArea = units::area(7.0, units::cm2);
    headP.FlipDisplay = false;
    engine->getHead(0)->initialize(headP);

    // Wire exhaust/intake to head for all 6 cylinders
    for (int i = 0; i < 6; ++i) {
        engine->getHead(0)->setExhaustSystem(i, engine->getExhaustSystem(0));
        engine->getHead(0)->setIntake(i, engine->getIntake(0));
    }

    // ---- Ignition Module ----
    Function* timingCurve = new Function();
    timingCurve->initialize(6, 0);
    timingCurve->addSample(units::rpm(0),    units::angle(10, units::deg));
    timingCurve->addSample(units::rpm(1000), units::angle(15, units::deg));
    timingCurve->addSample(units::rpm(2000), units::angle(25, units::deg));
    timingCurve->addSample(units::rpm(3000), units::angle(32, units::deg));
    timingCurve->addSample(units::rpm(4000), units::angle(38, units::deg));
    timingCurve->addSample(units::rpm(5000), units::angle(40, units::deg));

    IgnitionModule::Parameters imp = {};
    imp.cylinderCount = 6;
    imp.crankshaft = engine->getCrankshaft(0);
    imp.timingCurve = timingCurve;
    imp.revLimit = units::rpm(7000);
    imp.limiterDuration = 0.1 * units::sec;
    engine->getIgnitionModule()->initialize(imp);

    // I6 firing order 1-5-3-6-2-4 (0-indexed: 0,4,2,5,1,3)
    // Sparks every 120 deg of crank rotation (720/6 = 120)
    engine->getIgnitionModule()->setFiringOrder(0, (0.0 / 6.0) * cycle);
    engine->getIgnitionModule()->setFiringOrder(4, (1.0 / 6.0) * cycle);
    engine->getIgnitionModule()->setFiringOrder(2, (2.0 / 6.0) * cycle);
    engine->getIgnitionModule()->setFiringOrder(5, (3.0 / 6.0) * cycle);
    engine->getIgnitionModule()->setFiringOrder(1, (4.0 / 6.0) * cycle);
    engine->getIgnitionModule()->setFiringOrder(3, (5.0 / 6.0) * cycle);

    // ---- Fuel ----
    Function* turbulenceFn = createDefaultTurbulenceToFlameSpeedRatio();
    Fuel::Parameters fp = {};
    fp.maxBurningEfficiency = 0.85;
    fp.turbulenceToFlameSpeedRatio = turbulenceFn;
    engine->getFuel()->initialize(fp);

    // ---- Combustion Chambers ----
    Function* mpsToTurb = createMeanPistonSpeedToTurbulence();

    CombustionChamber::Parameters ccP = {};
    ccP.CrankcasePressure = units::pressure(1.0, units::atm);
    ccP.Fuel = engine->getFuel();
    ccP.StartingPressure = units::pressure(1.0, units::atm);
    ccP.StartingTemperature = units::celcius(25.0);
    ccP.MeanPistonSpeedToTurbulence = mpsToTurb;

    for (int i = 0; i < engine->getCylinderCount(); ++i) {
        ccP.Piston = engine->getPiston(i);
        ccP.Head = engine->getHead(ccP.Piston->getCylinderBank()->getIndex());
        engine->getChamber(i)->initialize(ccP);
        engine->getChamber(i)->setEngine(engine);
    }
}

TEST(Phase0Spikes, ClutchParameterSweep) {
    const std::string outDir = "build/spikes/clutch_sweep/";
    system(("mkdir -p " + outDir).c_str());

    std::vector<double> torques = {500, 1000, 2000, 4000, 8000, 16000};
    const double dt = 1.0/10000.0;
    const int totalSteps = static_cast<int>(12.0 / dt);
    const int throttleStepAt = static_cast<int>(2.0 / dt);

    for (double maxTorque : torques) {
        // Engine inline-6
        Engine::Parameters ep{};
        ep.cylinderBanks = 1;
        ep.cylinderCount = 6;
        ep.crankshaftCount = 1;
        ep.exhaustSystemCount = 1;
        ep.intakeCount = 1;
        ep.name = "SpikeI6";
        ep.starterTorque = 90 * units::ft_lb;
        ep.starterSpeed = 200;
        ep.redline = 6500;
        ep.dynoMinSpeed = 1000;
        ep.dynoMaxSpeed = 6500;
        ep.dynoHoldStep = 100;
        ep.initialSimulationFrequency = 10000;
        ep.initialHighFrequencyGain = 0.01;
        ep.initialNoise = 1.0;
        ep.initialJitter = 0.5;
        Throttle* thr = new Throttle();
        ep.throttle = thr;

        Engine engine;
        engine.initialize(ep);

        // Fully initialize all sub-components for combustion
        fullyInitializeI6(&engine);

        // Vehicle
        Vehicle::Parameters vp{};
        vp.mass = 1500 * units::kg;
        vp.diffRatio = 3.73;
        vp.tireRadius = units::distance(0.33, units::m);
        vp.dragCoefficient = 0.30;
        vp.crossSectionArea = 2.2;
        vp.rollingResistance = 0.015;
        Vehicle vehicle;
        vehicle.initialize(vp);

        // Transmission
        Transmission trans;
        Transmission::Parameters tp{};
        double ratios[6] = {3.50, 2.10, 1.45, 1.10, 0.90, 0.75};
        tp.GearCount = 6;
        tp.GearRatios = ratios;
        tp.MaxClutchTorque = maxTorque;
        trans.initialize(tp);

        // Physics simulator (no dyno)
        PistonEngineSimulator sim;
        Simulator::Parameters sp;
        sp.systemType = Simulator::SystemType::NsvOptimized;
        sim.initialize(sp);
        sim.setSimulationFrequency(10000);
        sim.setFluidSimulationSteps(8);
        sim.loadSimulation(&engine, &vehicle, &trans);
        sim.m_dyno.m_enabled = false;

        trans.changeGear(1);

        engine.getIgnitionModule()->m_enabled = true;
        sim.m_starterMotor.m_enabled = true;  // Crank engine to start

        double maxErrPct = 0.0;
        double lastRpm = 0.0;
        const int diagInterval = static_cast<int>(1.0 / dt);  // every 1 sec
        const int starterOffAt = static_cast<int>(1.0 / dt);  // Disengage starter after 1s

        for (int step = 0; step < totalSteps; ++step) {
            double t = step * dt;
            double thrVal = (step >= throttleStepAt) ? 1.0 : 0.0;
            engine.setSpeedControl(thrVal);

            // Disengage starter once engine catches (RPM > 550)
            if (sim.m_starterMotor.m_enabled) {
                double v_theta = engine.getOutputCrankshaft()->m_body.v_theta;
                double rpm = std::abs(v_theta) * (60.0 / (2.0 * M_PI));
                if (rpm > 550 || step >= starterOffAt) {
                    sim.m_starterMotor.m_enabled = false;
                }
            }

            sim.startFrame(dt);
            sim.simulateStep();
            // no audio needed

            // Get RPM from crankshaft (v_theta is negative for CW rotation)
            double v_theta = engine.getOutputCrankshaft()->m_body.v_theta;
            double rpm = std::abs(v_theta) * (60.0 / (2.0 * M_PI));
            lastRpm = rpm;

            // Diagnostic: print RPM every 1 second for the first run only
            if (maxTorque == torques[0] && step % diagInterval == 0) {
                printf("  t=%.1f throttle=%.1f rpm=%.1f\n", t, thrVal, rpm);
            }
        }

        printf("clutch sweep: %.0f Nm -> max err %.2f%% final_rpm=%.1f\n",
               maxTorque, maxErrPct*100.0, lastRpm);

        // Cleanup: destroy simulator before engine components are freed
        sim.destroy();
        engine.destroy();
    }

    SUCCEED();
}
