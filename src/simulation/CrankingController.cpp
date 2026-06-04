// CrankingController.cpp - Engine cranking state machine implementation
// Phase is stored on ICombustionEngine (single source of truth), not here.

#include "simulation/CrankingController.h"
#include "simulator/ICombustionEngine.h"
#include "common/ILogging.h"

void CrankingController::engageStarter(ICombustionEngine& engine, bool startStopButton) {
    if (startStopButton) {
        EnginePhase phase = engine.getEnginePhase();
        switch(phase) {
            case EnginePhase::Stopped:
                reset();
                engine.setStarterMotor(true);
                engine.setEnginePhase(EnginePhase::Cranking);
                break;

            case EnginePhase::Cranking:
                engine.setStarterMotor(false);
                engine.setEnginePhase(EnginePhase::Stopped);
                break;

            case EnginePhase::Rollover:
                engine.setStarterMotor(true);
                engine.setEnginePhase(EnginePhase::Cranking);
                break;

            default:
                break;
        }
    }
}

CrankingController::State CrankingController::step(
    ICombustionEngine& engine,
    double userThrottle,
    bool inputIgnition,
    ILogging* logger)
{
    EnginePhase phase = engine.getEnginePhase();
    EngineSimStats stats = engine.getStats();
    double effectiveThrottle = userThrottle;
    constexpr double CRANKING_THROTTLE = 0.55;
    (void)logger;

    switch(phase) {
        case EnginePhase::Running:
            if (!inputIgnition) {
                engine.setEnginePhase(EnginePhase::Stopping);
                phase = EnginePhase::Stopping;
            } else if (stats.currentRPM < STOPPED_RPM) {
                engine.setEnginePhase(EnginePhase::Stopped);
                phase = EnginePhase::Stopped;
            }
            break;

        case EnginePhase::Stopping:
            if (stats.currentRPM < STOPPED_RPM) {
                engine.setEnginePhase(EnginePhase::Stopped);
                phase = EnginePhase::Stopped;
            }
            break;

        case EnginePhase::Cranking:
            if (++ticks_ <= BASELINE_TICKS) {
                exhaustFlowSum_ += stats.exhaustFlow;
                if (ticks_ == BASELINE_TICKS) {
                    exhaustFlowBaseline_ = exhaustFlowSum_ / BASELINE_TICKS;
                }
            } else if (engineCaught(stats, inputIgnition)) {
                engine.setStarterMotor(false);
                engine.setEnginePhase(EnginePhase::Running);
                phase = EnginePhase::Running;
            }
            effectiveThrottle = CRANKING_THROTTLE;
            break;

        case EnginePhase::Rollover:
            if (engineCanCatch(stats, inputIgnition)) {
                engine.setEnginePhase(EnginePhase::Running);
                phase = EnginePhase::Running;
            } else if (stats.currentRPM < STOPPED_RPM && ++ticks_ > ROLLOVER_FALLBACK_TICKS) {
                engine.setStarterMotor(true);
                engine.setEnginePhase(EnginePhase::Cranking);
                phase = EnginePhase::Cranking;
            }
            break;

        case EnginePhase::Stopped:
        default:
            break;
    }

    return {effectiveThrottle, phase == EnginePhase::Cranking, phase};
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