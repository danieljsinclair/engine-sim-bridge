// EnginePresetsHelper.h - Helper functions for building engine presets
//
// These are common helper functions used by both EnginePresets (hardcoded C++ presets)
// and PresetEngineFactory (JSON presets). They avoid code duplication.
//
// SRP: Single responsibility - provide reusable builder functions
// DRY: Eliminates duplicate function data across multiple files

#ifndef ENGINE_PRESETS_HELPER_H
#define ENGINE_PRESETS_HELPER_H

#include "engine.h"
#include "function.h"

namespace EnginePresetsHelper {

// Create a meanPistonSpeedToTurbulence function
// Models turbulence increasing linearly with mean piston speed
Function* createMeanPistonSpeedToTurbulence();

// Create a default turbulenceToFlameSpeedRatio function
// Maps turbulence to flame speed multiplier for combustion calculations
// Required for combustion: Fuel::flameSpeed() dereferences this pointer
Function* createDefaultTurbulenceToFlameSpeedRatio();

// Create a GM LS-specific turbulenceToFlameSpeedRatio function
// Different curve for GM LS engines (higher values at high turbulence)
Function* createLsTurbulenceToFlameSpeedRatio();

// Create a port flow function from lift/flow arrays
// lift values are in thousandths of an inch, flow values in CFM
// Converted internally to meters and k_28inH2O flow constants
Function* createFlowFunction(const double lifts[], const double flows[], int count);

// Initialize combustion chambers for an engine
// Sets up all combustion chambers with common parameters
void initCombustionChambers(Engine* engine);

} // namespace EnginePresetsHelper

#endif // ENGINE_PRESETS_HELPER_H
