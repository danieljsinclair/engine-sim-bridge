// ISimulator.cpp - Static method implementation for ISimulator interface
// Phase 2: Provides getVersion() static method implementation

#include "simulator/ISimulator.h"
#include "simulator/EngineSimTypes.h"

const char* ISimulator::getVersion() {
    return EngineSimGetVersion();
}
