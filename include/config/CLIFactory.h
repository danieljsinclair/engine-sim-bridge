#ifndef CLI_FACTORY_H
#define CLI_FACTORY_H

#include "config/CLIConfig.h"

namespace input { class IInputProvider; }

namespace config {

// Factory function to create input provider based on CLI configuration
// Stub for RED phase - will be implemented in Task #5
input::IInputProvider* createInputProvider(const CLIConfig& config);

} // namespace config

#endif
