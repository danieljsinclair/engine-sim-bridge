#ifndef ATG_ENGINE_SIM_BRIDGE_H
#define ATG_ENGINE_SIM_BRIDGE_H

#include <stdint.h>
#include <stddef.h>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class ILogging;
class Simulator;

// ============================================================================
// DEFAULTS — single source of truth for simulation parameters
// ============================================================================
//
// Constant relationships:
//
// SIMULATION_FREQUENCY (10000 Hz) — physics step rate
//   |-- Synthesizer inputSampleRate = SIMULATION_FREQUENCY
//   |   |-- Upsampling ratio = SAMPLE_RATE / SIMULATION_FREQUENCY = 44100/10000 = 4.41x
//   |   |-- Each simulateStep() advances of synth input write pointer by this ratio
//   |-- Steps per 60Hz tick = SIMULATION_FREQUENCY / UPDATE_RATE_HZ = 10000/60 ~ 167
//   |   |-- Dynamically adjusted +/-10% by TARGET_SYNTH_LATENCY feedback loop
//   |-- Physics timestep = 1/SIMULATION_FREQUENCY = 0.1ms
//       |-- Overridable via --sim-freq CLI flag
//
// SAMPLE_RATE (44100 Hz) — audio output rate, matches upstream engine-sim
//   |-- Synthesizer audioSampleRate
//   |-- CoreAudio hardware device rate
//   |-- AUDIO_BUFFER_SIZE = SAMPLE_RATE * 2 seconds = 88200
//   |-- FRAMES_PER_UPDATE = SAMPLE_RATE / 60 = 735

namespace EngineSimDefaults {
    // Primary constants
    constexpr int32_t SAMPLE_RATE            = 44100;   // Hz — matches upstream engine-sim (simulator.cpp:226)
    constexpr int32_t SIMULATION_FREQUENCY   = 10000;   // Hz — physics step rate
    constexpr int32_t FLUID_SIMULATION_STEPS = 8;       // Substeps per physics step
    constexpr double  TARGET_SYNTH_LATENCY   = 0.02;    // seconds — M4 Pro best latency tested (0.01 too fast for threaded mode) - can be overridden with --synth-latency CLI flag

    // Derived constants — relationships are explicit
    constexpr int32_t UPDATE_RATE_HZ             = 60;                              // Main loop tick rate (from original 60 FPS GUI)
    constexpr double  BUFFER_DURATION_SECONDS    = 2.0;                             // Ring buffer safety margin for threaded mode
    constexpr int32_t AUDIO_BUFFER_SIZE          = SAMPLE_RATE * static_cast<int32_t>(BUFFER_DURATION_SECONDS);  // 88200
    constexpr int32_t INPUT_BUFFER_SIZE          = 1024;                            // Per-channel input ring buffer
    constexpr int32_t FRAMES_PER_UPDATE          = SAMPLE_RATE / UPDATE_RATE_HZ;   // 735 frames per 60Hz tick
    constexpr double  UPDATE_INTERVAL            = 1.0 / UPDATE_RATE_HZ;           // 16.67ms

    // Audio channel constants
    constexpr int32_t AUDIO_CHANNELS_MONO        = 1;   // Synthesizer output is mono (summed exhaust)
    constexpr int32_t AUDIO_CHANNELS_STEREO      = AUDIO_CHANNELS_MONO * 2;  // Hardware output is stereo (L+R duplicate)

    // Audio I/O constants
    constexpr int32_t MAX_AUDIO_CHUNK_FRAMES     = 4096; // Max frames per single read/drain operation

    // Simulation defaults
    constexpr double  DEFAULT_DURATION_SECONDS   = 3.0;  // Default non-interactive simulation duration
    constexpr float   DEFAULT_HARDWARE_VOLUME    = 1.0f; // Default hardware output volume (0.0 to 1.0)
    constexpr int32_t DEFAULT_PREFILL_MS         = 50;   // Default pre-fill buffer duration in ms for sync-pull mode
    constexpr double  DYNO_MAX_TORQUE_FT_LBS     = 500.0; // Base dyno brake torque — ~1.5x typical V8 peak, gives usable range

    // Display conversion constants
    constexpr double  KMH_TO_MPH                 = 0.621371; // km/h to mph conversion factor

    // Physics conversion constants
    constexpr double  TWO_PI = 2.0 * M_PI;
    constexpr double  MIN_TO_SECONDS             = 60.0;                    // minutes to seconds
    constexpr double  MS_TO_KMH                  = 3.6;                     // m/s to km/h (3600s/h / 1000m/km)
    constexpr double  RAD_PER_SEC_TO_RPM         = MIN_TO_SECONDS / TWO_PI;   // rad/s to RPM - One revolution is 2π radians, and there are 60 seconds in a minute, so the factor is 60 / (2π)
    constexpr double  MS_TO_SECONDS              = 1.0 / 1000.0; // milliseconds to seconds

    // Twin state machine thresholds
    constexpr double  TELEMETRY_TIMEOUT_S         = 5.0;   // Seconds without valid telemetry before OFF transition
    constexpr double  CRANKING_THROTTLE           = 0.6;   // Throttle fraction during cranking
    constexpr double  IDLE_SUSTAIN_THROTTLE       = 0.05;  // Minimum throttle to sustain combustion at idle; floor applied in IDLE so the engine never coasts through the Stopped latch at the CRANKING->IDLE handoff before the driver throttle arrives
    constexpr double  RELIEF_IDLE_SUSTAIN_THROTTLE = 0.055; // Stronger idle-sustain floor applied when the creep-drag relief opens the clutch (engine decoupled from the wheels). The normal 5% floor lets the M156 settle ~750-800 rpm decoupled at 6 mph (below the 950 idle target — the no-load throttle curve is steep: 5%≈800, 7%≈1650, 12%≈2600, so a small bump shifts rpm a lot). 5.5% holds the engine at ~950-1000 through the relief so the decoupled engine idles near its coupled target instead of drooping.
    // Clutch pressure rate limits (per second). The relief and re-engage are
    // RATE-LIMITED, not instant steps, so the clutch never goes 0%↔100% in one
    // frame (the binary slam that crashed the engine 4700→771 rpm). Asymmetric:
    // the release is FAST (a lugging engine must be decoupled before it death-
    // spirals), the engage is SMOOTH (no slam when the slip-lock takes over).
    constexpr double  CLUTCH_RELEASE_RATE_PER_SEC = 10.0;  // relief open: 0.076→0 in ~1 frame
    constexpr double  CLUTCH_ENGAGE_RATE_PER_SEC  = 3.0;   // re-engage: 0→1 in ~0.33s (smooth)
    // The creep-relief fires just BEFORE the engine crosses idle (idle + this
    // margin) so the clutch opens with a head-start and the relief idle-sustain
    // catches the engine at ~idle instead of letting downward momentum carry it
    // to ~750. Small enough that a healthy/high engine (4700 rpm) never triggers
    // it; large enough to preempt the droop.
    constexpr double  CREEP_RELIEF_TRIGGER_MARGIN_RPM = 60.0;
    constexpr double  STANDSTILL_SPEED_MS         = 0.001; // Below this speed (m/s), vehicle is considered stopped

    // Display thresholds
    constexpr int     RPM_DISPLAY_FLOOR          = 10;    // RPM below this is displayed as 0 (suppresses transient noise)
    constexpr double  DYNO_IDLE_RPM              = 700.0; // Idle RPM — dyno won't brake below this speed
}

// ISimulatorConfig — Configuration for ISimulator implementations
// Inline initializers from EngineSimDefaults (single source of truth)
//
// Field ownership:
// - sampleRate: CROSS-CUTTING value shared across ISimulator, IAudioBuffer, and IAudioHardwareProvider
//   - ISimulator: computes dt = frames / sampleRate for renderOnDemand()
//   - IAudioBuffer: receives sampleRate as parameter to initialize()
//   - IAudioHardwareProvider: receives sampleRate via AudioStreamFormat for hardware configuration
//   - Canonical source is ISimulatorConfig.sampleRate (set from EngineSimDefaults::SAMPLE_RATE)
// - simulationFrequency, fluidSimulationSteps, synthLatency: ISimulator-only (factory sets on Simulator subclass)
// - maxChunkFrames, volume, convolutionLevel: ISimulator-only (runtime use by BridgeSimulator)
//
// Note: volume and convolutionLevel are runtime-tunable defaults, not constants
struct ISimulatorConfig {
    int32_t sampleRate = EngineSimDefaults::SAMPLE_RATE;
    int32_t simulationFrequency = 0;  // 0 = use engine's actual frequency; >0 = explicit override
    int32_t fluidSimulationSteps = EngineSimDefaults::FLUID_SIMULATION_STEPS;
    int32_t maxChunkFrames = EngineSimDefaults::MAX_AUDIO_CHUNK_FRAMES;
    double targetSynthesizerLatency = EngineSimDefaults::TARGET_SYNTH_LATENCY;
    float volume = 0.5f;           // Runtime-tunable default
    float convolutionLevel = 0.5f; // Runtime-tunable default
    // Output-stage span taming amount (--span-tame CLI arg, [0,1], default 0
    // = OFF = bit-identical audio). Flows to the synthesizer's
    // AudioParameters.spanTame via SimulatorInitHelpers::applySpanTame at
    // factory build time. See engine-sim include/span_tame.h for the pinned
    // parameterization (soft-knee compressor + makeup + safety soft-clip,
    // applied just before the int16 conversion in renderAudio).
    float spanTame = 0.0f;
    // Output-stage volume leveling (--volume-tame CLI arg, [0,1], default 0
    // = OFF = fully bypassed = bit-identical audio). Applied at the mono
    // int16 -> stereo float conversion seam by BridgeSimulator's VolumeTamer
    // (see simulator/OutputStageDynamics.h): quiet-on-decel sections lift
    // toward the session average, loud sections sit back, gain moves are
    // smoothed so the processor itself never steps the amplitude.
    float volumeTame = 0.0f;
    // When true the simulation is paced to a recording (deterministic replay or
    // live/replay telemetry with a warm-start prefix) rather than free-running
    // real-time audio. In paced mode the loop thread owns core advancement at a
    // fixed timestep, so the audio-latency self-adjustment of m_steps in
    // Simulator::startFrame must be DISABLED: it reads a timing-dependent buffer
    // size and would make the per-frame substep count (and thus the whole run)
    // nondeterministic, tipping the warm-start into the reversion attractor.
    bool pacedReplay = false;
};

// Runtime statistics
struct EngineSimStats {
    double currentRPM = 0.0;       // raw crank rpm
    double filteredRPM = 0.0;      // tach-sensor rpm (first-order, tau=0.1s)
    double currentLoad = 0.0;
    double exhaustFlow = 0.0;
    double manifoldPressure = 0.0;
    int32_t activeChannels = 0;
    double processingTimeMs = 0.0;
    // Synth output level of the last rendered audio block: post-leveler,
    // PRE-volume RMS in int16 output scale — the "what you would hear at
    // volume 1" quantity. -1.0 = nothing rendered yet (honest-absent).
    double synthOutputRms = -1.0;

    // Dyno state (0.0 when dyno disabled)
    double dynoTorque = 0.0;         // Current dyno applied torque (ft*lbs)
    double dynoTargetRPM = 0.0;      // Dyno target RPM (0 = disabled)
    double dynoTorqueScale = 1.0;    // Current torque scale (0-1)
    int gear = 0;                    // Current gear (0 = neutral)

    // Vehicle telemetry
    double vehicleSpeedKmh = 0.0;
    double engineTorqueNm = 0.0;
    double drivetrainTorqueNm = 0.0;

    // Gear selector state
    int gearSelector = 0;            // GearSelector value
    bool gearAutoMode = false;       // true=auto(ZF), false=manual

    // Computed accessors (vehicleSpeedKmh is single source of truth)
    double speedMph() const { return vehicleSpeedKmh * EngineSimDefaults::KMH_TO_MPH; }
};

namespace EngineSimAudio {
    constexpr int STEREO = EngineSimDefaults::AUDIO_CHANNELS_STEREO;

// Converts mono int16 samples to stereo float32 (interleaved) - balanced channels
inline void convertInt16ToStereoFloat(
        const int16_t* input,
        int32_t frameCount,
        float* output,
        float volume,
        float convolutionLevel) {
    constexpr float scale = 1.0f / 32768.0f;  // int16_t range [-32768, 32767] normalized to [-1.0, 1.0]
    for (int32_t i = 0; i < frameCount; ++i) {
        const float sample = static_cast<float>(input[i]) * scale;
        // Interleaved stereo layout: [L, R, L, R, ...]
        output[i * STEREO] = sample * volume;           // Left channel
        output[i * STEREO + 1] = sample * convolutionLevel;  // Right channel
    }
}

// Converts mono int16 samples to stereo float32 with clipping protection
inline void convertInt16ToStereoFloatClipped(const int16_t* input, float* output, int32_t frameCount) {
    constexpr float scale = 1.0f / 32768.0f;  // int16_t range [-32768, 32767] normalized to [-1.0, 1.0]
    for (int32_t i = 0; i < frameCount; ++i) {
        float sample = static_cast<float>(input[i]) * scale;
        // Clamp to valid float audio range [-1.0, 1.0]
        if (sample > 1.0f) sample = 1.0f;
        if (sample < -1.0f) sample = -1.0f;
        // Interleaved stereo layout: [L, R, L, R, ...]
        output[i * STEREO] = sample;      // Left channel
        output[i * STEREO + 1] = sample;  // Right channel
    }
}

// Fills a stereo float buffer with silence (zeros)
inline void fillSilence(float* buffer, int32_t frames) {
    // STEREO channels per frame, sizeof(float) bytes per sample
    std::memset(buffer, 0, frames * STEREO * sizeof(float));
}

// Inaudible non-zero floor for startup-crackle fades. float 0.0003 maps to
// int16 +9 (~-88 dBFS). Chosen so that:
//   (a) every silence-transition frame is inaudible, AND
//   (b) the linear fade from a real sample to this floor never produces an
//       intermediate float whose int16 quantization collapses to exactly 0
//       (the knock detector's zero-sample proxy). At 0.0001f the fade from a
//       near-zero sample dipped below 1 LSB (int16 0) during cranking
//       startup — the residual zero_samples. 0.0003 keeps the fade band
//       comfortably above the int16 quantization step.
// Shared by audioRenderCallback (not-playing fill) and SyncPullStrategy
// (fillSilenceFaded / applyFadeIn) so the not-playing -> playing handoff
// stays continuous.
constexpr float SILENCE_FLOOR = 0.0003f;
constexpr int FADE_SAMPLES = 256; // ~5.8ms at 44100Hz

} // namespace EngineSimAudio

inline const char* EngineSimGetVersion() {
    return "1.0.0";
}

#endif // ATG_ENGINE_SIM_BRIDGE_H
