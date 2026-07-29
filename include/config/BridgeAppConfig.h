// BridgeAppConfig.h
//
// Bridge-side application-level configuration DTO. Carries the runtime
// behaviour knobs that engine-sim-app passes down through the bridge, and is
// consumed by config::createInputProvider() (see CLIFactory.h) to select the
// app-mode input provider -- e.g. modelDemo routes the factory to a
// DemoInputProvider. This is distinct from the CLI's CommandLineArgs
// (engine-sim-cli/src/config/CLIconfig.h), which parses argv on the CLI side.
#ifndef BRIDGE_APP_CONFIG_H
#define BRIDGE_APP_CONFIG_H

namespace config {

struct BridgeAppConfig {
    bool modelDemo = false;   // Run in model-demo mode (selects DemoInputProvider).
    bool playAudio = false;   // Enable audio playback of the simulation.
    bool interactive = false; // Enable interactive (user-driven) input.
};

} // namespace config

#endif
