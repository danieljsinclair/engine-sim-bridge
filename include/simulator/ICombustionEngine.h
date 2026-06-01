// ICombustionEngine.h - Interface for combustion engine simulators
// Extends ISimulator with combustion-specific controls.
// Non-combustion simulators (sine, electric) only implement ISimulator.
// Phase is managed internally by CrankingController — not exposed for writing.

#ifndef I_COMBUSTION_ENGINE_H
#define I_COMBUSTION_ENGINE_H

#include "simulator/ISimulator.h"

class ICombustionEngine : public ISimulator {
public:
    virtual ~ICombustionEngine() = default;

    // Combustion engine controls — high-level signals, not internal state
    virtual void setStarterMotor(bool on) = 0;
    virtual void setIgnition(bool on) = 0;
};

#endif // I_COMBUSTION_ENGINE_H
