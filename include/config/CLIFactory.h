#ifndef CLI_FACTORY_H
#define CLI_FACTORY_H

#include "config/BridgeAppConfig.h"

namespace input { class IInputProvider; }

namespace config {

// Factory function to create input provider based on bridge app configuration
// Stub for RED phase - will be implemented in Task #5
input::IInputProvider* createInputProvider(const BridgeAppConfig& config);

} // namespace config

#endif
