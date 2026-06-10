// CrankingController.h - Engine cranking state machine controller
// Manages starter motor engagement and engine phase transitions
// for the Stopped/Cranking/Running/Stopping state machine.
//
// Phase is NOT stored here — it lives on ICombustionEngine (single source of truth).
// Controller returns TransitionDecision; callers apply via BridgeSimulator::applyTransition().

#ifndef CRANKING_CONTROLLER_H
#define CRANKING_CONTROLLER_H

#include "simulation/EnginePhase.h"
#include "simulator/EngineSimTypes.h"

class ICombustionEngine;
class ILogging;

class CrankingController {
public:
    struct State {
        double startingThrottle;
        bool starterEngaged;
        EnginePhase phase;
    };

    // Decision methods (return TransitionDecision instead of mutating engine)
    TransitionDecision engageStarter(ICombustionEngine& engine, bool startStopButton, bool inputIgnition);
    TransitionDecision step(ICombustionEngine& engine, double userThrottle, bool ignition);

    void reset();

private:
    int ticks_ = 0;

    static constexpr int BASELINE_TICKS = 10;
    static constexpr int ROLLOVER_FALLBACK_TICKS = 3;
    static constexpr double CATCH_RATIO = 2.0;
    static constexpr double MIN_CATCH_RPM = 500.0;
    static constexpr double STOPPED_RPM = 5.0;

    double exhaustFlowSum_ = 0.0;
    double exhaustFlowBaseline_ = 0.0;

    bool engineCaught(const EngineSimStats& stats, bool inputIgnition) const;
    bool engineCanCatch(const EngineSimStats& stats, bool inputIgnition) const;
};

#endif // CRANKING_CONTROLLER_H
