// ICombustionEngine.h - Interface for combustion engine simulators
// Extends ISimulator with combustion-specific controls and phase management.
// Non-combustion simulators (sine, electric) only implement ISimulator.
// EnginePhase is owned by the engine (BridgeSimulator) -- single source of truth.
// Phase transitions are applied atomically via applyTransition().

#ifndef I_COMBUSTION_ENGINE_H
#define I_COMBUSTION_ENGINE_H

#include "simulator/ISimulator.h"
#include "simulation/EnginePhase.h"

class ICombustionEngine : public ISimulator {
public:
    virtual ~ICombustionEngine() = default;

    // Combustion engine controls -- high-level signals, not internal state
    virtual void setStarterMotor(bool on) = 0;
    virtual void setIgnition(bool on) = 0;

    // Atomic phase transition -- sets phase + starter motor together
    virtual void applyTransition(const TransitionDecision& decision) = 0;
};

#endif // I_COMBUSTION_ENGINE_H
