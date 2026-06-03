// ICombustionEngine.h - Interface for combustion engine simulators
// Extends ISimulator with combustion-specific controls and phase management.
// Non-combustion simulators (sine, electric) only implement ISimulator.
// EnginePhase is owned by the engine (BridgeSimulator) — single source of truth.

#ifndef I_COMBUSTION_ENGINE_H
#define I_COMBUSTION_ENGINE_H

#include "simulator/ISimulator.h"

class ICombustionEngine : public ISimulator {
public:
    virtual ~ICombustionEngine() = default;

    // Combustion engine controls — high-level signals, not internal state
    virtual void setStarterMotor(bool on) = 0;
    virtual void setIgnition(bool on) = 0;

    // Phase management — ICombustionEngine owns the single source of truth
    // ISimulator provides read-only getEnginePhase(); this adds the write path
    // for CrankingController state transitions.
    virtual void setEnginePhase(EnginePhase phase) = 0;
};

#endif // I_COMBUSTION_ENGINE_H
