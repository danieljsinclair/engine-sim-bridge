// CrankingController.cpp - Engine cranking state machine implementation
// Phase is stored on ICombustionEngine (single source of truth), not here.

#include "simulation/CrankingController.h"
#include "simulator/ICombustionEngine.h"

// ============================================================================
// Decision methods - return TransitionDecision instead of mutating engine
// ============================================================================

TransitionDecision CrankingController::engageStarter(
    ICombustionEngine& engine, bool startStopButton, bool inputIgnition) {
    TransitionDecision decision{engine.getEnginePhase(), false, 0.0, false};

    if (!startStopButton) {
        return decision;
    }

    switch (decision.targetPhase) {
        case EnginePhase::Stopped:
            reset();
            decision.starterMotor = true;
            decision.targetPhase = EnginePhase::Cranking;
            decision.isTransition = true;
            break;

        case EnginePhase::Cranking:
            decision.starterMotor = false;
            decision.targetPhase = EnginePhase::Stopped;
            decision.isTransition = true;
            break;

        case EnginePhase::Stopping:
        case EnginePhase::Rollover:
            if (engineCanCatch(engine.getStats(), inputIgnition)) {
                decision.targetPhase = EnginePhase::Running;
                decision.isTransition = true;
            } else {
                decision.starterMotor = true;
                decision.targetPhase = EnginePhase::Cranking;
                decision.isTransition = true;
            }
            break;

        default:
            break;
    }

    return decision;
}

TransitionDecision CrankingController::step(
    ICombustionEngine& engine, double userThrottle, bool inputIgnition) {
    EnginePhase phase = engine.getEnginePhase();
    EngineSimStats stats = engine.getStats();
    double effectiveThrottle = userThrottle;
    constexpr double CRANKING_THROTTLE = 0.55;

    TransitionDecision decision{phase, false, effectiveThrottle, false};

    switch(phase) {
        case EnginePhase::Running:
            if (!inputIgnition) {
                decision.targetPhase = EnginePhase::Stopping;
                decision.isTransition = true;
            } else if (stats.currentRPM < STOPPED_RPM) {
                decision.targetPhase = EnginePhase::Stopped;
                decision.isTransition = true;
            }
            break;

        case EnginePhase::Stopping:
            if (engineCanCatch(stats, inputIgnition)) {
                decision.targetPhase = EnginePhase::Rollover;
                decision.isTransition = true;
            } else if (stats.currentRPM < MIN_CATCH_RPM) {
                decision.targetPhase = EnginePhase::Stopped;
                decision.isTransition = true;
            }
            break;

        case EnginePhase::Cranking:
            if (++ticks_ <= BASELINE_TICKS) {
                exhaustFlowSum_ += stats.exhaustFlow;
                if (ticks_ == BASELINE_TICKS) {
                    exhaustFlowBaseline_ = exhaustFlowSum_ / BASELINE_TICKS;
                }
                decision.starterMotor = true;
            } else if (engineCaught(stats, inputIgnition)) {
                decision.starterMotor = false;
                decision.targetPhase = EnginePhase::Running;
                decision.isTransition = true;
            } else {
                decision.starterMotor = true;
            }
            decision.effectiveThrottle = CRANKING_THROTTLE;
            break;

        case EnginePhase::Rollover:
            if (engineCanCatch(stats, inputIgnition)) {
                decision.targetPhase = EnginePhase::Running;
                decision.isTransition = true;
            } else {
                ++ticks_;
                if (stats.currentRPM < STOPPED_RPM && ticks_ > ROLLOVER_FALLBACK_TICKS) {
                    decision.starterMotor = true;
                    decision.targetPhase = EnginePhase::Cranking;
                    decision.isTransition = true;
                }
            }
            break;

        case EnginePhase::Stopped:
        default:
            break;
    }

    return decision;
}

void CrankingController::reset() {
    ticks_ = 0;
    exhaustFlowSum_ = 0.0;
    exhaustFlowBaseline_ = 0.0;
}

bool CrankingController::engineCaught(const EngineSimStats& stats, bool inputIgnition) const {
    return engineCanCatch(stats, inputIgnition) && stats.exhaustFlow > exhaustFlowBaseline_ * CATCH_RATIO;
}

bool CrankingController::engineCanCatch(const EngineSimStats& stats, bool inputIgnition) const {
    return inputIgnition && stats.currentRPM > MIN_CATCH_RPM;
}