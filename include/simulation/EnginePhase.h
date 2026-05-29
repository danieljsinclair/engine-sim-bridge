// EnginePhase.h - Engine operational state machine phase
// Single source of truth for engine state: Stopped/Cranking/Running/Stopping
// Owned by CrankingController in SimulationLoop, consumed by presentation and app

#ifndef ENGINE_PHASE_H
#define ENGINE_PHASE_H

enum class EnginePhase {
    Stopped,    // Engine not running
    Cranking,   // Starter motor engaged, waiting for catch
    Running,    // Engine self-sustaining
    Stopping    // Ignition off, waiting for RPM to drop
};

constexpr const char *EnginePhaseName(EnginePhase phase) noexcept {
    switch (phase) {
        case  EnginePhase::Stopped: return "\033[31m Stopped\033[0m"; // RED
        case EnginePhase::Cranking: return "\033[33mCranking\033[0m"; // YELLOW
        case  EnginePhase::Running: return "\033[32m Running\033[0m"; // GREEN
        case EnginePhase::Stopping: return "\033[35mStopping\033[0m"; // ORANGE
        default: return "invalid";
    }
}
#endif // ENGINE_PHASE_H
 