// CrankingController.h - Engine cranking state machine controller
// Manages starter motor engagement and engine phase transitions
// for the Stopped/Cranking/Running/Stopping state machine.

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

    void engageStarter(ICombustionEngine& engine, bool starterButton);
    State step(ICombustionEngine& engine, double userThrottle, bool ignition, ILogging* logger);
    void setInitialPhase(EnginePhase phase, ICombustionEngine* engine = nullptr);
    EnginePhase currentPhase() const { return phase_; }
    void reset();

private:
    EnginePhase phase_ = EnginePhase::Stopped;
    int ticks_ = 0;

    static constexpr int BASELINE_TICKS = 10;
    static constexpr double CATCH_RATIO = 2.0;
    static constexpr double MIN_CATCH_RPM = 500.0;
    static constexpr double STOPPED_RPM = 5.0;

    double exhaustFlowSum_ = 0.0;
    double exhaustFlowBaseline_ = 0.0;

    bool engineCaught(const EngineSimStats& stats, bool inputIgnition) const;
};

#endif // CRANKING_CONTROLLER_H
