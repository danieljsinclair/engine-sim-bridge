#ifndef ENGINE_SIM_BRIDGE_SINE_WAVE_SIMULATOR_H
#define ENGINE_SIM_BRIDGE_SINE_WAVE_SIMULATOR_H

#include <cstdint>
#include <cmath>

/**
 * SineWaveSimulator - A simple sine wave generator for testing/development.
 *
 * Generates a pure sine wave at a frequency proportional to RPM:
 *   frequency = RPM / 60 * (cylinders / 2) Hz
 *
 * For a 4-cylinder engine at 3000 RPM: frequency = 3000/60*2 = 100 Hz
 *
 * This class is used when config.sineMode == 1 to bypass the full engine
 * simulation for quick audio testing without requiring script loading.
 */
class SineWaveSimulator {
public:
    SineWaveSimulator();
    ~SineWaveSimulator() = default;

    /**
     * Initialize the sine wave generator with sample rate and cylinder count.
     */
    void initialize(int sampleRate, int cylinders = 4);

    /**
     * Set the current RPM for frequency calculation.
     */
    void setRPM(double rpm);

    /**
     * Generate stereo float audio samples.
     * @param buffer Output buffer for interleaved stereo samples (L, R, L, R...)
     * @param frames Number of frames to generate
     */
    void generate(float* buffer, int32_t frames);

    /**
     * Generate mono int16 audio samples (matching synthesizer output format).
     * @param buffer Output buffer for mono samples
     * @param samples Number of samples to generate
     */
    void generateInt16(int16_t* buffer, int32_t samples);

    /**
     * Get the current frequency in Hz.
     */
    double getFrequency() const { return m_frequency; }

    /**
     * Set the output volume (0.0 to 1.0).
     */
    void setVolume(float volume) { m_volume = volume; }

private:
    int m_sampleRate;
    int m_cylinders;
    double m_rpm;
    double m_frequency;
    double m_phase;
    float m_volume;

    static constexpr float TWO_PI = static_cast<float>(2.0 * M_PI);
};

#endif // ENGINE_SIM_BRIDGE_SINE_WAVE_SIMULATOR_H