// EnginePresets.cpp - Hardcoded C++ engine preset factory implementations
// Each factory function creates a Simulator subclass that builds
// Engine + Vehicle + Transmission via direct C++ API calls.
//
// Construction sequence follows EngineNode::buildEngine() exactly:
// 1. Count cylinders, exhaust systems, intakes
// 2. Engine::initialize(Parameters) - allocates arrays
// 3. Configure exhaust systems, intakes, cylinder heads
// 4. Generate crankshafts
// 5. Generate cylinder banks (creates pistons, connecting rods)
// 6. Connect rod assemblies
// 7. Generate ignition module
// 8. Create meanPistonSpeedToTurbulence function
// 9. Generate fuel
// 10. Initialize combustion chambers

#include "simulator/EnginePresets.h"
#include "common/ILogging.h"

#include "engine.h"
#include "crankshaft.h"
#include "piston.h"
#include "connecting_rod.h"
#include "cylinder_bank.h"
#include "cylinder_head.h"
#include "camshaft.h"
#include "standard_valvetrain.h"
#include "ignition_module.h"
#include "intake.h"
#include "exhaust_system.h"
#include "fuel.h"
#include "function.h"
#include "gas_system.h"
#include "impulse_response.h"
#include "throttle.h"
#include "vehicle.h"
#include "transmission.h"
#include "simulator.h"
#include "piston_engine_simulator.h"
#include "units.h"

#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const double TWO_PI = 2.0 * M_PI;

// ============================================================================
// Helper functions for unit conversions used by .mr scripts
// ============================================================================

namespace EnginePresetsHelper {

// Generate a harmonic cam lobe profile (mirrors GenerateHarmonicCamLobeNode)
// durationAt50Thou: duration at 0.050" lift, in radians
// gamma: power factor for the cosine shape
// lift: maximum valve lift in meters
// steps: number of samples to generate
Function* generateHarmonicCamLobe(double durationAt50Thou, double gamma,
                                   double lift, int steps) {
    const double angle = durationAt50Thou / 4.0;
    const double s = std::pow(2.0 * units::distance(50, units::thou) / lift, 1.0 / gamma) - 1.0;
    const double k = std::acos(s) / angle;
    const double extents = M_PI / k;
    const double step = extents / (steps - 5.0);

    // Capacity: steps*2 + 1 (for positive/negative x + center)
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

// Create a meanPistonSpeedToTurbulence function (mirrors EngineNode::buildEngine)
Function* createMeanPistonSpeedToTurbulence() {
    Function* fn = new Function();
    fn->initialize(30, 1);
    for (int i = 0; i < 30; ++i) {
        fn->addSample(static_cast<double>(i), static_cast<double>(i) * 0.5);
    }
    return fn;
}

// Create a default turbulenceToFlameSpeedRatio function
Function* createDefaultTurbulenceToFlameSpeedRatio() {
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

// Create a GM LS turbulenceToFlameSpeedRatio function (different curve)
Function* createLsTurbulenceToFlameSpeedRatio() {
    Function* fn = new Function();
    fn->initialize(10, 1);
    fn->addSample(0.0, 3.0);
    fn->addSample(5.0, 7.5);   // 1.5 * 5
    fn->addSample(10.0, 17.5); // 1.75 * 10
    fn->addSample(15.0, 30.0); // 2.0 * 15
    fn->addSample(20.0, 40.0);
    fn->addSample(25.0, 50.0);
    fn->addSample(30.0, 60.0);
    fn->addSample(35.0, 70.0);
    fn->addSample(40.0, 80.0);
    fn->addSample(45.0, 90.0);
    return fn;
}

// Create a flow function from arrays of (lift_thou, flow_cfm) pairs
// lift values are in thousandths of an inch, flow values in CFM
// Converted internally to meters and k_28inH2O flow constants
Function* createFlowFunction(const double lifts[], const double flows[], int count) {
    Function* fn = new Function();
    fn->initialize(count, 0);
    for (int i = 0; i < count; ++i) {
        double liftM = lifts[i] * units::distance(1, units::thou);
        double flowK = GasSystem::k_28inH2O(flows[i]);
        fn->addSample(liftM, flowK);
    }
    return fn;
}

// Initialize combustion chambers for an engine (mirrors EngineNode::buildEngine)
void initCombustionChambers(Engine* engine) {
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

} // namespace EnginePresetsHelper

// ============================================================================
// Base class for preset simulators
// ============================================================================

namespace {

// PresetSimulator - base for hardcoded engine preset simulators
// Extends PistonEngineSimulator for real audio output (writeToSynthesizer,
// physics constraints, combustion force generators, delay filters for exhaust pulses)
// Owns the Engine, Vehicle, Transmission objects and passes them to base via loadSimulation()
class PresetSimulator : public PistonEngineSimulator {
public:
    PresetSimulator(ILogging* logger)
        : defaultLogger_(logger ? nullptr : new ConsoleLogger())
        , logger_(logger ? logger : defaultLogger_.get())
        , m_ownedEngine(nullptr)
        , m_ownedVehicle(nullptr)
        , m_ownedTransmission(nullptr) {}

    virtual ~PresetSimulator() {
        // PistonEngineSimulator::destroy() cleans up physics constraints and synthesizer
        // We must destroy before deleting owned objects since they are referenced by constraints
        destroy();

        if (m_ownedTransmission) { delete m_ownedTransmission; m_ownedTransmission = nullptr; }
        if (m_ownedVehicle)      { delete m_ownedVehicle;      m_ownedVehicle = nullptr; }
        if (m_ownedEngine)       { m_ownedEngine->destroy(); delete m_ownedEngine; m_ownedEngine = nullptr; }
    }

protected:
    // Called by subclasses after they've built their engine/vehicle/transmission
    void finalize(Engine* engine, Vehicle* vehicle, Transmission* transmission) {
        m_ownedEngine = engine;
        m_ownedVehicle = vehicle;
        m_ownedTransmission = transmission;
        loadSimulation(engine, vehicle, transmission);
    }

    ILogging* logger() { return logger_; }

private:
    std::unique_ptr<ILogging> defaultLogger_;
    ILogging* logger_;
    Engine* m_ownedEngine;
    Vehicle* m_ownedVehicle;
    Transmission* m_ownedTransmission;
};

// ============================================================================
// Honda TRX520 Preset - Single cylinder ATV engine
// ============================================================================

class HondaTrx520Simulator : public PresetSimulator {
public:
    HondaTrx520Simulator(ILogging* logger) : PresetSimulator(logger) {}

    void initialize(const Parameters& params) override {
        Simulator::initialize(params);

        // ---- Engine ----
        Throttle* throttle = new Throttle();
        Engine::Parameters ep = {};
        ep.name = "Honda TRX520 (ATV)";
        ep.cylinderBanks = 1;
        ep.cylinderCount = 1;
        ep.crankshaftCount = 1;
        ep.exhaustSystemCount = 1;
        ep.intakeCount = 1;
        ep.starterTorque = units::torque(50.0, units::ft_lb);
        ep.starterSpeed = units::rpm(500);
        ep.redline = units::rpm(5000);
        ep.throttle = throttle;
        ep.initialSimulationFrequency = 40000;
        ep.initialHighFrequencyGain = 0.00121;
        ep.initialNoise = 0.229;
        ep.initialJitter = 0.42;

        Engine* engine = new Engine();
        engine->initialize(ep);

        // ---- Exhaust System ----
        ImpulseResponse* ir = new ImpulseResponse();
        ir->initialize("new/mild_exhaust_reverb.wav", 0.01);

        ExhaustSystem::Parameters esp = {};
        esp.outletFlowRate = GasSystem::k_carb(500.0);
        esp.primaryTubeLength = units::distance(20.0, units::inch);
        esp.primaryFlowRate = GasSystem::k_carb(200.0);
        esp.velocityDecay = 0.5;
        esp.audioVolume = 1.0;
        esp.impulseResponse = ir;
        esp.length = units::distance(80.0, units::inch);
        esp.collectorCrossSectionArea = units::area(10.0, units::cm2);
        engine->getExhaustSystem(0)->initialize(esp);

        // ---- Intake ----
        Intake::Parameters ip = {};
        ip.volume = units::volume(1.5, units::L);
        ip.CrossSectionArea = units::area(10.0, units::cm2);
        ip.InputFlowK = GasSystem::k_carb(100.0);
        ip.IdleFlowK = GasSystem::k_carb(0.0);
        ip.RunnerFlowRate = 0.0;
        ip.IdleThrottlePlatePosition = 0.993;
        ip.VelocityDecay = 0.5;
        engine->getIntake(0)->initialize(ip);

        // ---- Crankshaft ----
        Crankshaft::Parameters cp = {};
        cp.crankThrow = units::distance(71.5, units::mm) / 2.0;
        cp.flywheelMass = units::mass(5.0, units::lb);
        cp.mass = units::mass(5.0, units::lb);
        cp.frictionTorque = units::torque(5.0, units::ft_lb);
        cp.momentOfInertia = 0.22986844776863666 * 0.2;
        cp.pos_x = 0.0;
        cp.pos_y = 0.0;
        cp.tdc = M_PI / 2.0;
        cp.rodJournals = 1;
        engine->getCrankshaft(0)->initialize(cp);
        engine->getCrankshaft(0)->setRodJournalAngle(0, 0.0);

        // ---- Cylinder Bank ----
        CylinderBank::Parameters bp = {};
        bp.crankshaft = engine->getCrankshaft(0);
        bp.bore = units::distance(96.0, units::mm);
        bp.deckHeight = units::distance(71.5, units::mm) / 2.0 +
                         units::distance(6.0, units::inch) +
                         units::distance(1.0, units::inch);
        bp.angle = 0.0;
        bp.cylinderCount = 1;
        bp.index = 0;
        bp.positionX = 0.0;
        bp.positionY = 0.0;
        bp.displayDepth = units::distance(2.0, units::inch);
        engine->getCylinderBank(0)->initialize(bp);

        // ---- Connecting Rod ----
        ConnectingRod::Parameters crp = {};
        crp.mass = units::mass(100.0, units::g);
        crp.momentOfInertia = 0.0015884918028487504;
        crp.centerOfMass = 0.0;
        crp.length = units::distance(6.0, units::inch);
        crp.rodJournals = 1;
        crp.crankshaft = engine->getCrankshaft(0);
        crp.journal = 0;
        crp.piston = engine->getPiston(0);
        engine->getConnectingRod(0)->initialize(crp);

        // ---- Piston ----
        Piston::Parameters pp = {};
        pp.Bank = engine->getCylinderBank(0);
        pp.Rod = engine->getConnectingRod(0);
        pp.CylinderIndex = 0;
        pp.BlowbyFlowCoefficient = GasSystem::k_28inH2O(0.1);
        pp.CompressionHeight = units::distance(1.0, units::inch);
        pp.WristPinPosition = 0.0;
        pp.Displacement = 0.0;
        pp.mass = units::mass(100.0, units::g);
        engine->getPiston(0)->initialize(pp);

        // ---- Camshafts (vtwin90 pattern for single cylinder) ----
        Function* lobeProfile = EnginePresetsHelper::generateHarmonicCamLobe(
            units::angle(180.0, units::deg),   // duration_at_50_thou
            1.0,                                 // gamma
            units::distance(200.0, units::thou), // lift
            100                                  // steps
        );

        double lobeSep = units::angle(100.0, units::deg);

        // Intake camshaft
        Camshaft* intakeCam = new Camshaft();
        Camshaft::Parameters icp = {};
        icp.lobes = 1;
        icp.advance = 0;
        icp.crankshaft = engine->getCrankshaft(0);
        icp.lobeProfile = lobeProfile;
        icp.baseRadius = units::distance(500, units::thou);
        intakeCam->initialize(icp);
        intakeCam->setLobeCenterline(0, TWO_PI + lobeSep);

        // Exhaust camshaft
        Camshaft* exhaustCam = new Camshaft();
        Camshaft::Parameters ecp = {};
        ecp.lobes = 1;
        ecp.advance = 0;
        ecp.crankshaft = engine->getCrankshaft(0);
        ecp.lobeProfile = lobeProfile;
        ecp.baseRadius = units::distance(500, units::thou);
        exhaustCam->initialize(ecp);
        exhaustCam->setLobeCenterline(0, TWO_PI - lobeSep);

        // ---- Valvetrain ----
        StandardValvetrain* vt = new StandardValvetrain();
        StandardValvetrain::Parameters vtp = {};
        vtp.intakeCamshaft = intakeCam;
        vtp.exhaustCamshaft = exhaustCam;
        vt->initialize(vtp);

        // ---- Flow functions (generic_small_engine_head with flow_attenuation=2.0) ----
        const double lifts[] = {0, 50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 550, 600, 650, 700};
        const double intakeBase[] = {0, 25, 75, 100, 130, 180, 190, 220, 240, 250, 260, 260, 260, 255, 250};
        const double exhaustBase[] = {0, 25, 50, 75, 100, 125, 160, 175, 180, 190, 200, 205, 210, 210, 210};

        const double attenuation = 2.0;
        double intakeFlows[15], exhaustFlows[15];
        for (int i = 0; i < 15; ++i) {
            intakeFlows[i] = intakeBase[i] * attenuation;
            exhaustFlows[i] = exhaustBase[i] * attenuation;
        }

        Function* intakeFlow = EnginePresetsHelper::createFlowFunction(lifts, intakeFlows, 15);
        Function* exhaustFlow = EnginePresetsHelper::createFlowFunction(lifts, exhaustFlows, 15);

        // ---- Cylinder Head ----
        CylinderHead::Parameters headP = {};
        headP.Bank = engine->getCylinderBank(0);
        headP.ExhaustPortFlow = exhaustFlow;
        headP.IntakePortFlow = intakeFlow;
        headP.Valvetrain = vt;
        headP.CombustionChamberVolume = units::volume(60.0, units::cc);
        headP.IntakeRunnerVolume = units::volume(100.0, units::cc);
        headP.IntakeRunnerCrossSectionArea = units::area(9.0, units::cm2);
        headP.ExhaustRunnerVolume = units::volume(100.0, units::cc);
        headP.ExhaustRunnerCrossSectionArea = units::area(9.0, units::cm2);
        headP.FlipDisplay = false;
        engine->getHead(0)->initialize(headP);

        engine->getHead(0)->setExhaustSystem(0, engine->getExhaustSystem(0));
        engine->getHead(0)->setIntake(0, engine->getIntake(0));

        // ---- Ignition Module ----
        Function* timingCurve = new Function();
        timingCurve->initialize(5, 0);
        timingCurve->addSample(units::rpm(0),    units::angle(12, units::deg));
        timingCurve->addSample(units::rpm(1000), units::angle(12, units::deg));
        timingCurve->addSample(units::rpm(2000), units::angle(20, units::deg));
        timingCurve->addSample(units::rpm(3000), units::angle(35, units::deg));
        timingCurve->addSample(units::rpm(4000), units::angle(35, units::deg));

        IgnitionModule::Parameters imp = {};
        imp.cylinderCount = 1;
        imp.crankshaft = engine->getCrankshaft(0);
        imp.timingCurve = timingCurve;
        imp.revLimit = units::rpm(6000);
        imp.limiterDuration = 0.5 * units::sec;
        engine->getIgnitionModule()->initialize(imp);
        // Single cylinder fires at 0 deg in the cycle
        engine->getIgnitionModule()->setFiringOrder(0, 0.0);

        // ---- Fuel ----
        Function* turbulenceFn = EnginePresetsHelper::createDefaultTurbulenceToFlameSpeedRatio();
        Fuel::Parameters fp = {};
        fp.maxBurningEfficiency = 1.0;
        fp.turbulenceToFlameSpeedRatio = turbulenceFn;
        engine->getFuel()->initialize(fp);

        // ---- Combustion Chambers ----
        EnginePresetsHelper::initCombustionChambers(engine);

        // ---- Vehicle ----
        Vehicle* vehicle = new Vehicle();
        Vehicle::Parameters vp = {};
        vp.mass = units::mass(500.0, units::kg);
        vp.dragCoefficient = 0.25;
        vp.crossSectionArea = units::distance(47.0, units::inch) * units::distance(47.0, units::inch);
        vp.diffRatio = 3.33;
        vp.tireRadius = units::distance(11.0, units::inch);
        vp.rollingResistance = units::force(200, units::N);
        vehicle->initialize(vp);

        // ---- Transmission ----
        static const double gearRatios[] = {4.0, 3.5, 3.0, 2.5, 2.0};
        Transmission* transmission = new Transmission();
        Transmission::Parameters tp = {};
        tp.GearCount = 5;
        tp.GearRatios = gearRatios;
        tp.MaxClutchTorque = units::torque(50.0, units::ft_lb);
        transmission->initialize(tp);

        // Load into simulator base
        finalize(engine, vehicle, transmission);
    }

};

// ============================================================================
// Subaru EJ25 Preset - Flat-4 boxer engine
// ============================================================================

class SubaruEj25Simulator : public PresetSimulator {
public:
    SubaruEj25Simulator(ILogging* logger) : PresetSimulator(logger) {}

    void initialize(const Parameters& params) override {
        Simulator::initialize(params);

        // ---- Dimensions ----
        const double stroke = units::distance(79.0, units::mm);
        const double bore = units::distance(99.5, units::mm);
        const double rodLength = units::distance(5.142, units::inch);
        const double rodMass = units::mass(535.0, units::g);
        const double compressionHeight = units::distance(1.0, units::inch);
        const double crankMass = units::mass(9.39, units::kg);
        const double flywheelMass = units::mass(6.8, units::kg);
        const double flywheelRadius = units::distance(6.0, units::inch);

        // Moments of inertia
        const double crankMoment = 0.5 * crankMass * (stroke / 2.0) * (stroke / 2.0);
        const double flywheelMoment = 0.5 * flywheelMass * flywheelRadius * flywheelRadius;
        const double otherMoment = 0.5 * units::mass(10.0, units::kg) * units::distance(6.0, units::cm) * units::distance(6.0, units::cm);
        const double rodMoment = (1.0 / 12.0) * rodMass * rodLength * rodLength;

        // ---- Engine ----
        Throttle* throttle = new Throttle();
        Engine::Parameters ep = {};
        ep.name = "Subaru EJ25";
        ep.cylinderBanks = 2;
        ep.cylinderCount = 4;
        ep.crankshaftCount = 1;
        ep.exhaustSystemCount = 2;
        ep.intakeCount = 1;
        ep.starterTorque = units::torque(70.0, units::ft_lb);
        ep.starterSpeed = units::rpm(500);
        ep.redline = units::rpm(6500);
        ep.throttle = throttle;
        ep.initialSimulationFrequency = 20000;
        ep.initialHighFrequencyGain = 0.01;
        ep.initialNoise = 1.0;
        ep.initialJitter = 0.5;

        Engine* engine = new Engine();
        engine->initialize(ep);

        // ---- Exhaust Systems (2: one per bank) ----
        ImpulseResponse* irExhaust = new ImpulseResponse();
        irExhaust->initialize("new/mild_exhaust.wav", 0.01);

        ExhaustSystem::Parameters esp = {};
        esp.outletFlowRate = GasSystem::k_carb(1000.0);
        esp.primaryTubeLength = units::distance(10.0, units::inch);
        esp.primaryFlowRate = GasSystem::k_carb(200.0);
        esp.velocityDecay = 1.0;
        esp.impulseResponse = irExhaust;
        esp.collectorCrossSectionArea = units::area(10.0, units::cm2);
        esp.length = units::distance(80.0, units::inch);

        // Exhaust 0: bank 0, audio_volume = 0.5 * 8 = 4.0
        esp.audioVolume = 4.0;
        engine->getExhaustSystem(0)->initialize(esp);

        // Exhaust 1: bank 1, audio_volume = 1.0 * 8 = 8.0
        esp.audioVolume = 8.0;
        engine->getExhaustSystem(1)->initialize(esp);

        // ---- Intake (1 shared) ----
        Intake::Parameters ip = {};
        ip.volume = units::volume(1.325, units::L);
        ip.CrossSectionArea = units::area(20.0, units::cm2);
        ip.InputFlowK = GasSystem::k_carb(800.0);
        ip.IdleFlowK = GasSystem::k_carb(0.0);
        ip.RunnerFlowRate = GasSystem::k_carb(250.0);
        ip.RunnerLength = units::distance(12.0, units::inch);
        ip.IdleThrottlePlatePosition = 0.9985;
        ip.VelocityDecay = 0.5;
        engine->getIntake(0)->initialize(ip);

        // ---- Crankshaft ----
        Crankshaft::Parameters cp = {};
        cp.crankThrow = stroke / 2.0;
        cp.flywheelMass = flywheelMass;
        cp.mass = crankMass;
        cp.frictionTorque = units::torque(1.0, units::ft_lb);
        cp.momentOfInertia = crankMoment + flywheelMoment + otherMoment;
        cp.pos_x = 0.0;
        cp.pos_y = 0.0;
        cp.tdc = units::angle(180.0, units::deg);
        cp.rodJournals = 4;
        engine->getCrankshaft(0)->initialize(cp);
        engine->getCrankshaft(0)->setRodJournalAngle(0, units::angle(0.0, units::deg));
        engine->getCrankshaft(0)->setRodJournalAngle(1, units::angle(180.0, units::deg));
        engine->getCrankshaft(0)->setRodJournalAngle(2, units::angle(0.0, units::deg));
        engine->getCrankshaft(0)->setRodJournalAngle(3, units::angle(180.0, units::deg));

        // ---- Cylinder Banks ----
        // Bank 0: angle=90deg, 2 cylinders
        CylinderBank::Parameters bp0 = {};
        bp0.crankshaft = engine->getCrankshaft(0);
        bp0.bore = bore;
        bp0.deckHeight = stroke / 2.0 + rodLength + compressionHeight;
        bp0.angle = units::angle(90.0, units::deg);
        bp0.cylinderCount = 2;
        bp0.index = 0;
        bp0.positionX = 0.0;
        bp0.positionY = 0.0;
        bp0.displayDepth = units::distance(2.0, units::inch);
        engine->getCylinderBank(0)->initialize(bp0);

        // Bank 1: angle=-90deg, 2 cylinders
        CylinderBank::Parameters bp1 = bp0;
        bp1.angle = units::angle(-90.0, units::deg);
        bp1.index = 1;
        engine->getCylinderBank(1)->initialize(bp1);

        // ---- Connecting Rods and Pistons ----
        // EJ25 cylinder-to-rod-journal mapping:
        // Bank0, Cyl0 (global piston 0) -> rj0
        // Bank0, Cyl1 (global piston 1) -> rj3
        // Bank1, Cyl0 (global piston 2) -> rj1
        // Bank1, Cyl1 (global piston 3) -> rj2

        const double pistonMass = units::mass(414.0 + 152.0, units::g);
        const double blowby[] = {
            GasSystem::k_28inH2O(0.001),
            GasSystem::k_28inH2O(0.002),
            GasSystem::k_28inH2O(0.001),
            GasSystem::k_28inH2O(0.002)
        };
        const int rodJournalMap[] = {0, 3, 1, 2};
        const int bankOfCylinder[] = {0, 0, 1, 1};
        const int cylinderInBank[] = {0, 1, 0, 1};

        for (int i = 0; i < 4; ++i) {
            // Connecting rod
            ConnectingRod::Parameters crp = {};
            crp.mass = rodMass;
            crp.momentOfInertia = rodMoment;
            crp.centerOfMass = 0.0;
            crp.length = rodLength;
            crp.rodJournals = 1;
            crp.crankshaft = engine->getCrankshaft(0);
            crp.journal = rodJournalMap[i];
            crp.piston = engine->getPiston(i);
            engine->getConnectingRod(i)->initialize(crp);

            // Piston
            Piston::Parameters pp = {};
            pp.Bank = engine->getCylinderBank(bankOfCylinder[i]);
            pp.Rod = engine->getConnectingRod(i);
            pp.CylinderIndex = cylinderInBank[i];
            pp.BlowbyFlowCoefficient = blowby[i];
            pp.CompressionHeight = compressionHeight;
            pp.WristPinPosition = 0.0;
            pp.Displacement = 0.0;
            pp.mass = pistonMass;
            engine->getPiston(i)->initialize(pp);
        }

        // ---- Camshafts (EJ25 pattern: 4 camshafts, 2 per bank) ----
        Function* intakeLobe = EnginePresetsHelper::generateHarmonicCamLobe(
            units::angle(232.0, units::deg),  // duration_at_50_thou
            2.0,                               // gamma
            units::distance(9.78, units::mm),  // lift
            100                                // steps
        );
        Function* exhaustLobe = EnginePresetsHelper::generateHarmonicCamLobe(
            units::angle(236.0, units::deg),
            2.0,
            units::distance(9.60, units::mm),
            100
        );

        double intakeLobeCenter = units::angle(117.0, units::deg);
        double exhaustLobeCenter = units::angle(112.0, units::deg);
        double camBaseRadius = (34.0 / 2.0) * units::distance(1, units::mm);

        // EJ25 cam lobe pattern:
        // exhaust_cam_0: lobe at (360 - exhaust_center + 0/4*cycle) and (360 - exhaust_center + 1/4*cycle)
        // intake_cam_0:  lobe at (360 + intake_center + 0/4*cycle) and (360 + intake_center + 1/4*cycle)
        // exhaust_cam_1: lobe at (360 - exhaust_center + 2/4*cycle) and (360 - exhaust_center + 3/4*cycle)
        // intake_cam_1:  lobe at (360 + intake_center + 2/4*cycle) and (360 + intake_center + 3/4*cycle)
        double cycle = units::angle(720.0, units::deg); // 2 * 360 deg

        Camshaft* intakeCam0 = new Camshaft();
        Camshaft::Parameters icp0 = {};
        icp0.lobes = 2;
        icp0.advance = 0;
        icp0.crankshaft = engine->getCrankshaft(0);
        icp0.lobeProfile = intakeLobe;
        icp0.baseRadius = camBaseRadius;
        intakeCam0->initialize(icp0);
        intakeCam0->setLobeCenterline(0, TWO_PI + intakeLobeCenter + (0.0 / 4.0) * cycle);
        intakeCam0->setLobeCenterline(1, TWO_PI + intakeLobeCenter + (1.0 / 4.0) * cycle);

        Camshaft* exhaustCam0 = new Camshaft();
        Camshaft::Parameters ecp0 = {};
        ecp0.lobes = 2;
        ecp0.advance = 0;
        ecp0.crankshaft = engine->getCrankshaft(0);
        ecp0.lobeProfile = exhaustLobe;
        ecp0.baseRadius = camBaseRadius;
        exhaustCam0->initialize(ecp0);
        exhaustCam0->setLobeCenterline(0, TWO_PI - exhaustLobeCenter + (0.0 / 4.0) * cycle);
        exhaustCam0->setLobeCenterline(1, TWO_PI - exhaustLobeCenter + (1.0 / 4.0) * cycle);

        Camshaft* intakeCam1 = new Camshaft();
        Camshaft::Parameters icp1 = icp0;
        icp1.lobeProfile = intakeLobe;
        intakeCam1->initialize(icp1);
        intakeCam1->setLobeCenterline(0, TWO_PI + intakeLobeCenter + (2.0 / 4.0) * cycle);
        intakeCam1->setLobeCenterline(1, TWO_PI + intakeLobeCenter + (3.0 / 4.0) * cycle);

        Camshaft* exhaustCam1 = new Camshaft();
        Camshaft::Parameters ecp1 = ecp0;
        ecp1.lobeProfile = exhaustLobe;
        exhaustCam1->initialize(ecp1);
        exhaustCam1->setLobeCenterline(0, TWO_PI - exhaustLobeCenter + (2.0 / 4.0) * cycle);
        exhaustCam1->setLobeCenterline(1, TWO_PI - exhaustLobeCenter + (3.0 / 4.0) * cycle);

        // ---- Valvetrains (one per head) ----
        StandardValvetrain* vt0 = new StandardValvetrain();
        StandardValvetrain::Parameters vt0p = {};
        vt0p.intakeCamshaft = intakeCam0;
        vt0p.exhaustCamshaft = exhaustCam0;
        vt0->initialize(vt0p);

        StandardValvetrain* vt1 = new StandardValvetrain();
        StandardValvetrain::Parameters vt1p = {};
        vt1p.intakeCamshaft = intakeCam1;
        vt1p.exhaustCamshaft = exhaustCam1;
        vt1->initialize(vt1p);

        // ---- Flow functions (EJ25 head, flow_attenuation=1.0, lift_scale=1.0) ----
        const double ej25Lifts[] = {0, 50, 100, 150, 200, 250, 300, 350, 400, 450};
        const double ej25IntakeFlows[] = {0, 58, 103, 156, 214, 249, 268, 280, 280, 281};
        const double ej25ExhaustFlows[] = {0, 37, 72, 113, 160, 196, 222, 235, 245, 246};

        Function* intakeFlow = EnginePresetsHelper::createFlowFunction(ej25Lifts, ej25IntakeFlows, 10);
        Function* exhaustFlow = EnginePresetsHelper::createFlowFunction(ej25Lifts, ej25ExhaustFlows, 10);

        // ---- Cylinder Heads ----
        // Head 0 (Bank 0, flip_display=true)
        CylinderHead::Parameters headP0 = {};
        headP0.Bank = engine->getCylinderBank(0);
        headP0.ExhaustPortFlow = exhaustFlow;
        headP0.IntakePortFlow = intakeFlow;
        headP0.Valvetrain = vt0;
        headP0.CombustionChamberVolume = units::volume(67.0, units::cc);
        headP0.IntakeRunnerVolume = units::volume(149.6, units::cc);
        headP0.IntakeRunnerCrossSectionArea = units::distance(1.35, units::inch) * units::distance(1.35, units::inch);
        headP0.ExhaustRunnerVolume = units::volume(50.0, units::cc);
        headP0.ExhaustRunnerCrossSectionArea = units::distance(1.25, units::inch) * units::distance(1.25, units::inch);
        headP0.FlipDisplay = true;
        engine->getHead(0)->initialize(headP0);

        // Head 1 (Bank 1, flip_display=false)
        CylinderHead::Parameters headP1 = headP0;
        headP1.Bank = engine->getCylinderBank(1);
        headP1.Valvetrain = vt1;
        headP1.FlipDisplay = false;
        engine->getHead(1)->initialize(headP1);

        // Wire exhaust/intake to heads
        // Head 0 (Bank 0): exhaust0, intake0
        engine->getHead(0)->setExhaustSystem(0, engine->getExhaustSystem(0));
        engine->getHead(0)->setExhaustSystem(1, engine->getExhaustSystem(0));
        engine->getHead(0)->setIntake(0, engine->getIntake(0));
        engine->getHead(0)->setIntake(1, engine->getIntake(0));

        // Head 1 (Bank 1): exhaust1, intake0
        engine->getHead(1)->setExhaustSystem(0, engine->getExhaustSystem(1));
        engine->getHead(1)->setExhaustSystem(1, engine->getExhaustSystem(1));
        engine->getHead(1)->setIntake(0, engine->getIntake(0));
        engine->getHead(1)->setIntake(1, engine->getIntake(0));

        // ---- Ignition Module ----
        Function* timingCurve = new Function();
        timingCurve->initialize(5, 0);
        timingCurve->addSample(units::rpm(0),    units::angle(25, units::deg));
        timingCurve->addSample(units::rpm(1000), units::angle(25, units::deg));
        timingCurve->addSample(units::rpm(2000), units::angle(30, units::deg));
        timingCurve->addSample(units::rpm(3000), units::angle(40, units::deg));
        timingCurve->addSample(units::rpm(4000), units::angle(40, units::deg));

        IgnitionModule::Parameters imp = {};
        imp.cylinderCount = 4;
        imp.crankshaft = engine->getCrankshaft(0);
        imp.timingCurve = timingCurve;
        imp.revLimit = units::rpm(6500);
        imp.limiterDuration = 0.08 * units::sec;
        engine->getIgnitionModule()->initialize(imp);
        // Firing order: 0, 1, 2, 3 at 0/4, 1/4, 2/4, 3/4 of cycle
        engine->getIgnitionModule()->setFiringOrder(0, (0.0 / 4.0) * cycle);
        engine->getIgnitionModule()->setFiringOrder(1, (1.0 / 4.0) * cycle);
        engine->getIgnitionModule()->setFiringOrder(2, (2.0 / 4.0) * cycle);
        engine->getIgnitionModule()->setFiringOrder(3, (3.0 / 4.0) * cycle);

        // ---- Fuel ----
        Function* turbulenceFn = EnginePresetsHelper::createDefaultTurbulenceToFlameSpeedRatio();
        Fuel::Parameters fp = {};
        fp.maxBurningEfficiency = 0.75;
        fp.maxTurbulenceEffect = 2.5;
        fp.turbulenceToFlameSpeedRatio = turbulenceFn;
        engine->getFuel()->initialize(fp);

        // ---- Combustion Chambers ----
        EnginePresetsHelper::initCombustionChambers(engine);

        // ---- Vehicle (Subaru Impreza) ----
        Vehicle* vehicle = new Vehicle();
        Vehicle::Parameters vp = {};
        vp.mass = units::mass(2700.0, units::lb);
        vp.dragCoefficient = 0.2;
        vp.crossSectionArea = units::distance(66.0, units::inch) * units::distance(56.0, units::inch);
        vp.diffRatio = 3.9;
        vp.tireRadius = units::distance(10.0, units::inch);
        vp.rollingResistance = units::force(300, units::N);
        vehicle->initialize(vp);

        // ---- Transmission ----
        static const double gearRatios[] = {3.636, 2.375, 1.761, 1.346, 0.971, 0.756};
        Transmission* transmission = new Transmission();
        Transmission::Parameters tp = {};
        tp.GearCount = 6;
        tp.GearRatios = gearRatios;
        tp.MaxClutchTorque = units::torque(300.0, units::ft_lb);
        transmission->initialize(tp);

        finalize(engine, vehicle, transmission);
    }

};

// ============================================================================
// GM LS Preset - V8 pushrod engine
// ============================================================================

class GmLsSimulator : public PresetSimulator {
public:
    GmLsSimulator(ILogging* logger) : PresetSimulator(logger) {}

    void initialize(const Parameters& params) override {
        Simulator::initialize(params);

        const double stroke = units::distance(3.622, units::inch);
        const double bore = units::distance(3.78, units::inch);
        const double rodLength = units::distance(160.0, units::mm);
        const double rodMass = units::mass(50.0, units::g);
        const double compressionHeight = units::distance(1.0, units::inch);
        const double crankMass = units::mass(60.0, units::lb);
        const double flywheelMass = units::mass(30.0, units::lb);
        const double flywheelRadius = units::distance(8.0, units::inch);

        const double crankMoment = 1.5 * 0.5 * crankMass * (stroke / 2.0) * (stroke / 2.0);
        const double flywheelMoment = 0.5 * flywheelMass * flywheelRadius * flywheelRadius;
        const double otherMoment = 0.5 * units::mass(1.0, units::kg) * units::distance(1.0, units::cm) * units::distance(1.0, units::cm);
        const double rodMoment = (1.0 / 12.0) * rodMass * rodLength * rodLength;
        const double vAngle = units::angle(90.0, units::deg);

        // ---- Engine ----
        Throttle* throttle = new Throttle();
        Engine::Parameters ep = {};
        ep.name = "GM LS";
        ep.cylinderBanks = 2;
        ep.cylinderCount = 8;
        ep.crankshaftCount = 1;
        ep.exhaustSystemCount = 2;
        ep.intakeCount = 1;
        ep.starterTorque = units::torque(200.0, units::ft_lb);
        ep.starterSpeed = units::rpm(200);
        ep.redline = units::rpm(6500);
        ep.throttle = throttle;
        ep.initialSimulationFrequency = 10000;
        ep.initialHighFrequencyGain = 0.01;
        ep.initialNoise = 1.0;
        ep.initialJitter = 0.6;

        Engine* engine = new Engine();
        engine->initialize(ep);

        // ---- Exhaust Systems (2) ----
        ImpulseResponse* irDefault = new ImpulseResponse();
        irDefault->initialize("smooth/smooth_39.wav", 0.001);

        ExhaustSystem::Parameters esp = {};
        esp.outletFlowRate = GasSystem::k_carb(1000.0);
        esp.primaryTubeLength = units::distance(29.0, units::inch);
        esp.primaryFlowRate = GasSystem::k_carb(500.0);
        esp.velocityDecay = 1.0;
        esp.impulseResponse = irDefault;
        esp.collectorCrossSectionArea = units::area(10.0, units::cm2);
        esp.audioVolume = 4.0;

        // Exhaust 0: Bank 0, length=100 inch
        esp.length = units::distance(100.0, units::inch);
        engine->getExhaustSystem(0)->initialize(esp);

        // Exhaust 1: Bank 1, length=172 inch
        esp.length = units::distance(172.0, units::inch);
        engine->getExhaustSystem(1)->initialize(esp);

        // ---- Intake ----
        Intake::Parameters ip = {};
        ip.volume = units::volume(1.325, units::L);
        ip.CrossSectionArea = units::area(20.0, units::cm2);
        ip.InputFlowK = GasSystem::k_carb(700.0);
        ip.IdleFlowK = GasSystem::k_carb(0.0);
        ip.RunnerFlowRate = GasSystem::k_carb(100.0);
        ip.RunnerLength = units::distance(12.0, units::inch);
        ip.IdleThrottlePlatePosition = 0.996;
        ip.VelocityDecay = 0.5;
        engine->getIntake(0)->initialize(ip);

        // ---- Crankshaft ----
        Crankshaft::Parameters cp = {};
        cp.crankThrow = stroke / 2.0;
        cp.flywheelMass = flywheelMass;
        cp.mass = crankMass;
        cp.frictionTorque = units::torque(20.0, units::ft_lb);
        cp.momentOfInertia = crankMoment + flywheelMoment + otherMoment;
        cp.pos_x = 0.0;
        cp.pos_y = 0.0;
        cp.tdc = units::angle(90.0, units::deg) - (vAngle / 2.0);
        cp.rodJournals = 4;
        engine->getCrankshaft(0)->initialize(cp);
        // LS firing order: 1,8,7,2,6,5,4,3 mapped to rod journal angles:
        // rj0=0deg, rj1=270deg, rj2=90deg, rj3=180deg
        engine->getCrankshaft(0)->setRodJournalAngle(0, units::angle(0.0, units::deg));
        engine->getCrankshaft(0)->setRodJournalAngle(1, units::angle(270.0, units::deg));
        engine->getCrankshaft(0)->setRodJournalAngle(2, units::angle(90.0, units::deg));
        engine->getCrankshaft(0)->setRodJournalAngle(3, units::angle(180.0, units::deg));

        // ---- Cylinder Banks (V8: 2 banks of 4 cylinders each) ----
        double deckHeight = stroke / 2.0 + rodLength + compressionHeight;
        const double spacing = units::distance(2.0, units::inch);

        // Bank 0: angle = -vAngle/2
        CylinderBank::Parameters bp0 = {};
        bp0.crankshaft = engine->getCrankshaft(0);
        bp0.bore = bore;
        bp0.deckHeight = deckHeight;
        bp0.angle = -vAngle / 2.0;
        bp0.cylinderCount = 4;
        bp0.index = 0;
        bp0.positionX = 0.0;
        bp0.positionY = 0.0;
        bp0.displayDepth = units::distance(2.0, units::inch);
        engine->getCylinderBank(0)->initialize(bp0);

        // Bank 1: angle = +vAngle/2
        CylinderBank::Parameters bp1 = bp0;
        bp1.angle = vAngle / 2.0;
        bp1.index = 1;
        engine->getCylinderBank(1)->initialize(bp1);

        // ---- Connecting Rods and Pistons ----
        // LS V8 cylinder-to-rod-journal mapping:
        // Bank0: Cyl0->rj0, Cyl1->rj1, Cyl2->rj2, Cyl3->rj3
        // Bank1: Cyl0->rj0, Cyl1->rj1, Cyl2->rj2, Cyl3->rj3
        // Global pistons: 0-3 = Bank0, 4-7 = Bank1

        for (int i = 0; i < 8; ++i) {
            int bank = (i < 4) ? 0 : 1;
            int cylInBank = i % 4;
            int rodJournal = cylInBank;  // same mapping for both banks

            ConnectingRod::Parameters crp = {};
            crp.mass = rodMass;
            crp.momentOfInertia = rodMoment;
            crp.centerOfMass = 0.0;
            crp.length = rodLength;
            crp.rodJournals = 1;
            crp.crankshaft = engine->getCrankshaft(0);
            crp.journal = rodJournal;
            crp.piston = engine->getPiston(i);
            engine->getConnectingRod(i)->initialize(crp);

            Piston::Parameters pp = {};
            pp.Bank = engine->getCylinderBank(bank);
            pp.Rod = engine->getConnectingRod(i);
            pp.CylinderIndex = cylInBank;
            pp.BlowbyFlowCoefficient = GasSystem::k_28inH2O(0.0);
            pp.CompressionHeight = compressionHeight;
            pp.WristPinPosition = 0.0;
            pp.Displacement = 0.0;
            pp.mass = units::mass(100.0, units::g);
            engine->getPiston(i)->initialize(pp);
        }

        // ---- Camshafts (LS V8: 4 camshafts, 2 per bank) ----
        Function* intakeLobe = EnginePresetsHelper::generateHarmonicCamLobe(
            units::angle(234.0, units::deg),
            1.1,
            units::distance(551.0, units::thou),
            256
        );
        Function* exhaustLobe = EnginePresetsHelper::generateHarmonicCamLobe(
            units::angle(235.0, units::deg),
            1.1,
            units::distance(551.0, units::thou),
            256
        );

        double intakeLobeCenter = units::angle(116.0, units::deg);
        double exhaustLobeCenter = units::angle(116.0, units::deg);
        double camBaseRadius = units::distance(1.0, units::inch);
        double rot90 = units::angle(90.0, units::deg);

        // LS cam lobe pattern (from ls_v8_camshaft_builder):
        // exhaust_cam_0: lobes at (360-exh_center+0*90), (360-exh_center+7*90), (360-exh_center+5*90), (360-exh_center+2*90)
        // intake_cam_0:  lobes at (360+int_center+0*90), (360+int_center+7*90), (360+int_center+5*90), (360+int_center+2*90)
        // exhaust_cam_1: lobes at (360-exh_center+3*90), (360-exh_center+6*90), (360-exh_center+4*90), (360-exh_center+1*90)
        // intake_cam_1:  lobes at (360+int_center+3*90), (360+int_center+6*90), (360+int_center+4*90), (360+int_center+1*90)

        Camshaft* intakeCam0 = new Camshaft();
        Camshaft::Parameters icp0 = {};
        icp0.lobes = 4;
        icp0.advance = 0;
        icp0.crankshaft = engine->getCrankshaft(0);
        icp0.lobeProfile = intakeLobe;
        icp0.baseRadius = camBaseRadius;
        intakeCam0->initialize(icp0);
        intakeCam0->setLobeCenterline(0, TWO_PI + intakeLobeCenter + 0 * rot90);
        intakeCam0->setLobeCenterline(1, TWO_PI + intakeLobeCenter + 7 * rot90);
        intakeCam0->setLobeCenterline(2, TWO_PI + intakeLobeCenter + 5 * rot90);
        intakeCam0->setLobeCenterline(3, TWO_PI + intakeLobeCenter + 2 * rot90);

        Camshaft* exhaustCam0 = new Camshaft();
        Camshaft::Parameters ecp0 = {};
        ecp0.lobes = 4;
        ecp0.advance = 0;
        ecp0.crankshaft = engine->getCrankshaft(0);
        ecp0.lobeProfile = exhaustLobe;
        ecp0.baseRadius = camBaseRadius;
        exhaustCam0->initialize(ecp0);
        exhaustCam0->setLobeCenterline(0, TWO_PI - exhaustLobeCenter + 0 * rot90);
        exhaustCam0->setLobeCenterline(1, TWO_PI - exhaustLobeCenter + 7 * rot90);
        exhaustCam0->setLobeCenterline(2, TWO_PI - exhaustLobeCenter + 5 * rot90);
        exhaustCam0->setLobeCenterline(3, TWO_PI - exhaustLobeCenter + 2 * rot90);

        Camshaft* intakeCam1 = new Camshaft();
        Camshaft::Parameters icp1 = icp0;
        icp1.lobeProfile = intakeLobe;
        intakeCam1->initialize(icp1);
        intakeCam1->setLobeCenterline(0, TWO_PI + intakeLobeCenter + 3 * rot90);
        intakeCam1->setLobeCenterline(1, TWO_PI + intakeLobeCenter + 6 * rot90);
        intakeCam1->setLobeCenterline(2, TWO_PI + intakeLobeCenter + 4 * rot90);
        intakeCam1->setLobeCenterline(3, TWO_PI + intakeLobeCenter + 1 * rot90);

        Camshaft* exhaustCam1 = new Camshaft();
        Camshaft::Parameters ecp1 = ecp0;
        ecp1.lobeProfile = exhaustLobe;
        exhaustCam1->initialize(ecp1);
        exhaustCam1->setLobeCenterline(0, TWO_PI - exhaustLobeCenter + 3 * rot90);
        exhaustCam1->setLobeCenterline(1, TWO_PI - exhaustLobeCenter + 6 * rot90);
        exhaustCam1->setLobeCenterline(2, TWO_PI - exhaustLobeCenter + 4 * rot90);
        exhaustCam1->setLobeCenterline(3, TWO_PI - exhaustLobeCenter + 1 * rot90);

        // ---- Valvetrains ----
        StandardValvetrain* vt0 = new StandardValvetrain();
        StandardValvetrain::Parameters vt0p = {};
        vt0p.intakeCamshaft = intakeCam0;
        vt0p.exhaustCamshaft = exhaustCam0;
        vt0->initialize(vt0p);

        StandardValvetrain* vt1 = new StandardValvetrain();
        StandardValvetrain::Parameters vt1p = {};
        vt1p.intakeCamshaft = intakeCam1;
        vt1p.exhaustCamshaft = exhaustCam1;
        vt1->initialize(vt1p);

        // ---- Flow functions (LS V8 head, flow_attenuation=1.0, lift_scale=1.0) ----
        const double lsLifts[] = {0, 50, 100, 150, 200, 250, 300, 350, 400, 450};
        const double lsIntakeFlows[] = {0, 1, 103, 156, 214, 249, 268, 280, 280, 281};
        const double lsExhaustFlows[] = {0, 1, 72, 113, 160, 196, 222, 235, 245, 246};

        Function* intakeFlow = EnginePresetsHelper::createFlowFunction(lsLifts, lsIntakeFlows, 10);
        Function* exhaustFlow = EnginePresetsHelper::createFlowFunction(lsLifts, lsExhaustFlows, 10);

        // ---- Cylinder Heads ----
        CylinderHead::Parameters headP0 = {};
        headP0.Bank = engine->getCylinderBank(0);
        headP0.ExhaustPortFlow = exhaustFlow;
        headP0.IntakePortFlow = intakeFlow;
        headP0.Valvetrain = vt0;
        headP0.CombustionChamberVolume = units::volume(90.0, units::cc);
        headP0.IntakeRunnerVolume = units::volume(149.6, units::cc);
        headP0.IntakeRunnerCrossSectionArea = units::distance(2.2, units::inch) * units::distance(2.2, units::inch);
        headP0.ExhaustRunnerVolume = units::volume(50.0, units::cc);
        headP0.ExhaustRunnerCrossSectionArea = units::distance(1.75, units::inch) * units::distance(1.75, units::inch);
        headP0.FlipDisplay = false;
        engine->getHead(0)->initialize(headP0);

        CylinderHead::Parameters headP1 = headP0;
        headP1.Bank = engine->getCylinderBank(1);
        headP1.Valvetrain = vt1;
        headP1.FlipDisplay = true;
        engine->getHead(1)->initialize(headP1);

        // Wire exhaust/intake to heads (4 cylinders per head)
        for (int i = 0; i < 4; ++i) {
            engine->getHead(0)->setExhaustSystem(i, engine->getExhaustSystem(0));
            engine->getHead(0)->setIntake(i, engine->getIntake(0));
            engine->getHead(1)->setExhaustSystem(i, engine->getExhaustSystem(1));
            engine->getHead(1)->setIntake(i, engine->getIntake(0));
        }

        // ---- Ignition Module ----
        Function* timingCurve = new Function();
        timingCurve->initialize(9, 0);
        timingCurve->addSample(units::rpm(0),    units::angle(12, units::deg));
        timingCurve->addSample(units::rpm(1000), units::angle(12, units::deg));
        timingCurve->addSample(units::rpm(2000), units::angle(20, units::deg));
        timingCurve->addSample(units::rpm(3000), units::angle(30, units::deg));
        timingCurve->addSample(units::rpm(4000), units::angle(40, units::deg));
        timingCurve->addSample(units::rpm(5000), units::angle(40, units::deg));
        timingCurve->addSample(units::rpm(6000), units::angle(40, units::deg));
        timingCurve->addSample(units::rpm(7000), units::angle(40, units::deg));
        timingCurve->addSample(units::rpm(8000), units::angle(40, units::deg));

        IgnitionModule::Parameters imp = {};
        imp.cylinderCount = 8;
        imp.crankshaft = engine->getCrankshaft(0);
        imp.timingCurve = timingCurve;
        imp.revLimit = units::rpm(6800);
        imp.limiterDuration = 0.2 * units::sec;
        engine->getIgnitionModule()->initialize(imp);

        // LS firing order: 1,8,7,2,6,5,4,3 (cylinders indexed 0-7)
        // cylinder 0 fires at 0*90 deg, 7 at 1*90, 6 at 2*90, 1 at 3*90, etc.
        engine->getIgnitionModule()->setFiringOrder(0, 0 * units::angle(90.0, units::deg));
        engine->getIgnitionModule()->setFiringOrder(7, 1 * units::angle(90.0, units::deg));
        engine->getIgnitionModule()->setFiringOrder(6, 2 * units::angle(90.0, units::deg));
        engine->getIgnitionModule()->setFiringOrder(1, 3 * units::angle(90.0, units::deg));
        engine->getIgnitionModule()->setFiringOrder(5, 4 * units::angle(90.0, units::deg));
        engine->getIgnitionModule()->setFiringOrder(4, 5 * units::angle(90.0, units::deg));
        engine->getIgnitionModule()->setFiringOrder(3, 6 * units::angle(90.0, units::deg));
        engine->getIgnitionModule()->setFiringOrder(2, 7 * units::angle(90.0, units::deg));

        // ---- Fuel ----
        Function* turbulenceFn = EnginePresetsHelper::createLsTurbulenceToFlameSpeedRatio();
        Fuel::Parameters fp = {};
        fp.maxBurningEfficiency = 1.0;
        fp.turbulenceToFlameSpeedRatio = turbulenceFn;
        engine->getFuel()->initialize(fp);

        // ---- Combustion Chambers ----
        EnginePresetsHelper::initCombustionChambers(engine);

        // ---- Vehicle (Corvette) ----
        Vehicle* vehicle = new Vehicle();
        Vehicle::Parameters vp = {};
        vp.mass = units::mass(1614.0, units::kg);
        vp.dragCoefficient = 0.3;
        vp.crossSectionArea = units::distance(72.0, units::inch) * units::distance(50.0, units::inch);
        vp.diffRatio = 3.42;
        vp.tireRadius = units::distance(10.0, units::inch);
        vp.rollingResistance = units::force(200, units::N);
        vehicle->initialize(vp);

        // ---- Transmission ----
        static const double gearRatios[] = {2.97, 2.07, 1.43, 1.00, 0.71, 0.57};
        Transmission* transmission = new Transmission();
        Transmission::Parameters tp = {};
        tp.GearCount = 6;
        tp.GearRatios = gearRatios;
        tp.MaxClutchTorque = units::torque(500.0, units::ft_lb);
        transmission->initialize(tp);

        finalize(engine, vehicle, transmission);
    }

};

} // anonymous namespace

// ============================================================================
// EnginePresets public API
// ============================================================================

const std::vector<EnginePresetInfo>& EnginePresets::getAvailablePresets() {
    static const std::vector<EnginePresetInfo> presets = {
        {"honda_trx520", "Honda TRX520 (ATV)", "Single cylinder 518cc ATV engine", 1, 0.52},
        {"subaru_ej25", "Subaru EJ25", "Flat-4 2.5L boxer engine", 4, 2.5},
        {"gm_ls", "GM LS", "V8 5.7L pushrod engine", 8, 5.7},
    };
    return presets;
}

Simulator* EnginePresets::createPreset(const std::string& presetId, ILogging* logger) {
    if (presetId == "honda_trx520") return createHondaTrx520(logger);
    if (presetId == "subaru_ej25")  return createSubaruEj25(logger);
    if (presetId == "gm_ls")        return createGmLs(logger);
    return nullptr;
}

// ============================================================================
// Honda TRX520 Factory
// ============================================================================

Simulator* EnginePresets::createHondaTrx520(ILogging* logger) {
    HondaTrx520Simulator* sim = new HondaTrx520Simulator(logger);
    Simulator::Parameters params = {};
    sim->initialize(params);
    return sim;
}

// ============================================================================
// Subaru EJ25 Factory
// ============================================================================

Simulator* EnginePresets::createSubaruEj25(ILogging* logger) {
    SubaruEj25Simulator* sim = new SubaruEj25Simulator(logger);
    Simulator::Parameters params = {};
    sim->initialize(params);
    return sim;
}

// ============================================================================
// GM LS Factory
// ============================================================================

Simulator* EnginePresets::createGmLs(ILogging* logger) {
    GmLsSimulator* sim = new GmLsSimulator(logger);
    Simulator::Parameters params = {};
    sim->initialize(params);
    return sim;
}
