// IDemoSpeedEnhancer.h - Interface for speed data enhancement
// Enhances EngineInput with speed-based physics for --connect-demo mode

#ifndef I_DEMO_SPEED_ENHANCER_H
#define I_DEMO_SPEED_ENHANCER_H

#include "simulator/EngineSimTypes.h"

namespace input {

// Interface for speed-based input enhancement
// DemoInputProvider implements this to add speed data to base EngineInput
class IDemoSpeedEnhancer {
public:
    virtual ~IDemoSpeedEnhancer() = default;

    // Enhance the base EngineInput with speed data
    // Returns the enhanced EngineInput (may modify or replace baseInput)
    virtual EngineInput enhanceInput(const EngineInput& baseInput, double dt) = 0;

    // Provide engine stats feedback to the enhancer (for cranking transition, etc.)
    virtual void provideFeedback(const struct EngineSimStats& stats) = 0;
};

} // namespace input

#endif // I_DEMO_SPEED_ENHANCER_H
