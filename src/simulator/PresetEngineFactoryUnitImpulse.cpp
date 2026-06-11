// PresetEngineFactoryUnitImpulse.cpp - Generate minimal unit impulse responses for preset-loaded engines
// Addresses the root cause identified in task #6 investigation:
// "PresetFactory never sets exhaustParams.impulseResponse, leaving it as nullptr.
// This causes synthesizer crash when renderAudioOnDemand() dereferences null m_shiftRegister."

#include "simulator/PresetEngineFactory.h"
#include "simulator/simulator.h"
#include <array>
#include <cstdint>

// Apply preset-specific fix: generate unit impulse responses during preset load
// This ensures ConvolutionFilter::m_shiftRegister is initialized, preventing null pointer crash
// Even though convolution is typically disabled (convolution=0.0f), the filter must be initialized.

void applyPresetEngineFactoryUnitImpulseFix(PistonEngineSimulator* pistonSim) {
    if (!pistonSim) return;

    Engine* engine = pistonSim->getEngine();
    if (!engine) return;

    const int exhaustCount = engine->getExhaustSystemCount();
    if (exhaustCount == 0) return;

    // Unit impulse: [INT16_MAX] represents a single sample at full scale
    // This creates an identity transform in the convolution filter (like SineSimulator does)
    static const std::array<int16_t, 1> kUnitImpulse = { INT16_MAX };

    // Initialize each exhaust system's convolution filter with unit impulse
    for (int i = 0; i < exhaustCount; ++i) {
        pistonSim->synthesizer().initializeImpulseResponse(
            kUnitImpulse.data(),
            1,                       // 1 sample
            1.0f,                     // full volume
            i                           // exhaust index
        );
    }

    // Disable convolution to save CPU (unit impulse makes it identity transform)
    // Presets don't have real impulse response data, so this is expected
    Synthesizer::AudioParameters params = pistonSim->synthesizer().getAudioParameters();
    params.convolution = 0.0f;
    pistonSim->synthesizer().setAudioParameters(params);
}
