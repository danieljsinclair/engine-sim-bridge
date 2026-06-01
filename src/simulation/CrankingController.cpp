// CrankingController.cpp - Engine cranking state machine implementation

#include "simulation/CrankingController.h"
#include "simulator/ICombustionEngine.h"
#include "common/ILogging.h"

#define CRANKING_DEBUG false
#if CRANKING_DEBUG
#define IF_CRANKING_DEBUG(x) x
#else
#define IF_CRANKING_DEBUG(x)
#endif

void CrankingController::engageStarter(ICombustionEngine& engine, bool starterButton) {
    if (starterButton) {
        switch(phase_) {
            case EnginePhase::Stopped:
                reset();
                engine.setStarterMotor(true);
                phase_ = EnginePhase::Cranking;
                break;
                
            case EnginePhase::Cranking:
                engine.setStarterMotor(false);
                phase_ = EnginePhase::Stopped;
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
    EngineSimStats stats = engine.getStats();
    double effectiveThrottle = userThrottle;
    constexpr double CRANKING_THROTTLE = 0.55;
    (void)logger;

    switch(phase_) {
        case EnginePhase::Running:
            if (!inputIgnition) {
                phase_ = EnginePhase::Stopping;
            } else if (stats.currentRPM < STOPPED_RPM) {
                phase_ = EnginePhase::Stopped;
            }
            break;

        case EnginePhase::Stopping:
            if (stats.currentRPM < STOPPED_RPM) {
                phase_ = EnginePhase::Stopped;
            } else if (engineCaught(stats, inputIgnition)) {
                phase_ = EnginePhase::Running;
            }
            break;

        case EnginePhase::Cranking:
            IF_CRANKING_DEBUG(
                if (ticks_ <= BASELINE_TICKS || ticks_ % 20 == 0) {
                    logger->info(LogMask::BRIDGE,
                        "Crank tick %d: RPM=%.0f exhaust=%.6f baseline=%.6f",
                        ticks_, stats.currentRPM, stats.exhaustFlow,
                        exhaustFlowBaseline_);
                }
            );

            if (++ticks_ <= BASELINE_TICKS) {
                exhaustFlowSum_ += stats.exhaustFlow;
                if (ticks_ == BASELINE_TICKS) {
                    exhaustFlowBaseline_ = exhaustFlowSum_ / BASELINE_TICKS;
                }
            } else if (engineCaught(stats, inputIgnition)) {
                engine.setStarterMotor(false);
                phase_ = EnginePhase::Running;
            }
            effectiveThrottle = CRANKING_THROTTLE;
            break;

        case EnginePhase::Stopped:
        default:
            break;
    }

    return {effectiveThrottle, phase_ == EnginePhase::Cranking, phase_};
}

void CrankingController::setInitialPhase(EnginePhase phase) {
    phase_ = phase;
}

void CrankingController::reset() {
    phase_ = EnginePhase::Stopped;
    ticks_ = 0;
    exhaustFlowSum_ = 0.0;
    exhaustFlowBaseline_ = 0.0;
}

bool CrankingController::engineCaught(const EngineSimStats& stats, bool inputIgnition) const {
    return inputIgnition
        && stats.exhaustFlow > exhaustFlowBaseline_ * CATCH_RATIO
        && stats.currentRPM > MIN_CATCH_RPM;
}
