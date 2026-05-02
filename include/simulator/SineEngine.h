// SineEngine.h - Minimal Engine stub for sine wave mode
// No physics — throttle directly controls RPM with no lag.
#ifndef ENGINE_SIM_BRIDGE_SINE_ENGINE_H
#define ENGINE_SIM_BRIDGE_SINE_ENGINE_H

#include "engine.h"
#include "throttle.h"
#include "units.h"

/// @brief A minimal Engine implementation for sine wave mode, where throttle directly controls RPM with no lag.
/// Essentially a dummy engine that does nothing but allows the plumbing to function
class SineEngine : public Engine {
public:
    SineEngine() {
        auto* throttle = new Throttle();
        Parameters p = {};
        p.name              = "Sine Wave Test Engine";
        p.cylinderBanks     = 0;
        p.cylinderCount     = 0;
        p.crankshaftCount   = 1;
        p.exhaustSystemCount = 1;  // Keep one input channel alive for synthesizer pipeline
        p.intakeCount       = 0;
        p.throttle          = throttle;
        initialize(p);

        Crankshaft::Parameters cp = {};
        cp.mass              = units::mass(10, units::kg);
        cp.flywheelMass      = units::mass(5, units::kg);
        cp.momentOfInertia   = 0.1;
        cp.crankThrow        = units::distance(50, units::mm);
        cp.rodJournals       = 0;
        getCrankshaft(0)->initialize(cp);
    }

    double getRpm() const override {
        // Engine::getSpeedControl() is non-const in the base API despite being a pure read.
        return 800.0 + const_cast<SineEngine*>(this)->getSpeedControl() * 5200.0;
    }

    double getSpeed() const override {
        // Base contract: return rad/s so units::toRpm() in the bridge gives correct RPM.
        return getRpm() * 0.104719755;
    }
};

#endif // ENGINE_SIM_BRIDGE_SINE_ENGINE_H
