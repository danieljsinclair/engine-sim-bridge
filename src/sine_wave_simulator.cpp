#include "sine_wave_simulator.h"

SineWaveSimulator::SineWaveSimulator()
    : m_sampleRate(48000)
    , m_cylinders(4)
    , m_rpm(0.0)
    , m_frequency(0.0)
    , m_phase(0.0)
    , m_volume(0.5f)
{
}

void SineWaveSimulator::initialize(int sampleRate, int cylinders) {
    m_sampleRate = sampleRate;
    m_cylinders = cylinders > 0 ? cylinders : 4;
    // Default to idle RPM
    m_rpm = 800.0;
    m_frequency = m_rpm / 60.0 * (m_cylinders / 2.0);
    m_phase = 0.0;
}

void SineWaveSimulator::setRPM(double rpm) {
    m_rpm = rpm;
    m_frequency = m_rpm / 60.0 * (m_cylinders / 2.0);
}

void SineWaveSimulator::generate(float* buffer, int32_t frames) {
    if (frames <= 0 || !buffer) {
        return;
    }

    const float amplitude = m_volume;
    const float phaseIncrement = TWO_PI * static_cast<float>(m_frequency) / static_cast<float>(m_sampleRate);

    for (int32_t i = 0; i < frames; ++i) {
        // Generate sine wave
        float sample = amplitude * std::sin(m_phase);

        // Store as stereo (both channels same)
        buffer[i * 2] = sample;         // Left
        buffer[i * 2 + 1] = sample;     // Right

        // Advance phase
        m_phase += phaseIncrement;
        if (m_phase >= TWO_PI) {
            m_phase -= TWO_PI;
        }
    }
}

void SineWaveSimulator::generateInt16(int16_t* buffer, int32_t samples) {
    if (samples <= 0 || !buffer) {
        return;
    }

    // Calculate phase increment for mono output
    const float phaseIncrement = TWO_PI * static_cast<float>(m_frequency) / static_cast<float>(m_sampleRate);

    for (int32_t i = 0; i < samples; ++i) {
        // Generate sine wave at full scale (will be scaled by volume)
        float sample = m_volume * std::sin(m_phase);

        // Convert to int16 with full-scale range
        // Use 32767 as max to avoid potential clipping on extreme values
        buffer[i] = static_cast<int16_t>(sample * 32767.0f);

        // Advance phase
        m_phase += phaseIncrement;
        if (m_phase >= TWO_PI) {
            m_phase -= TWO_PI;
        }
    }
}