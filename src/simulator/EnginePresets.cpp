// EnginePresets.cpp - Helper functions for building engine presets
// Hand-coded engine classes removed — all engines loaded via JSON presets.
// See PresetEngineFactory for the JSON loading path.
// Helper functions retained for use by PresetEngineFactory and tests.

#include "simulator/EnginePresetsHelper.h"

#include "engine.h"
#include "piston.h"
#include "connecting_rod.h"
#include "cylinder_bank.h"
#include "cylinder_head.h"
#include "function.h"
#include "gas_system.h"
#include "units.h"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace EnginePresetsHelper {

Function* generateHarmonicCamLobe(double durationAt50Thou, double gamma,
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

Function* createMeanPistonSpeedToTurbulence() {
    Function* fn = new Function();
    fn->initialize(30, 1);
    for (int i = 0; i < 30; ++i) {
        fn->addSample(static_cast<double>(i), static_cast<double>(i) * 0.5);
    }
    return fn;
}

Function* createDefaultTurbulenceToFlameSpeedRatio() {
    Function* fn = new Function();
    fn->initialize(10, 5);
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

Function* createLsTurbulenceToFlameSpeedRatio() {
    Function* fn = new Function();
    fn->initialize(10, 5);
    fn->addSample(0.0, 3.0);
    fn->addSample(5.0, 7.5);
    fn->addSample(10.0, 17.5);
    fn->addSample(15.0, 30.0);
    fn->addSample(20.0, 40.0);
    fn->addSample(25.0, 50.0);
    fn->addSample(30.0, 60.0);
    fn->addSample(35.0, 70.0);
    fn->addSample(40.0, 80.0);
    fn->addSample(45.0, 90.0);
    return fn;
}

Function* createFlowFunction(const double lifts[], const double flows[], int count) {
    Function* fn = new Function();
    fn->initialize(count, 50 * units::distance(1, units::thou));
    for (int i = 0; i < count; ++i) {
        double liftM = lifts[i] * units::distance(1, units::thou);
        double flowK = GasSystem::k_28inH2O(flows[i]);
        fn->addSample(liftM, flowK);
    }
    return fn;
}

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
