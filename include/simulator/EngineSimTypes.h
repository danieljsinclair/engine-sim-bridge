#ifndef ATG_ENGINE_SIM_BRIDGE_H
#define ATG_ENGINE_SIM_BRIDGE_H

#include <stdint.h>
#include <stddef.h>
#include <cstring>
#include <string>

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
};

// Afterfire ("pop on overrun") tuning, expressed in bridge-level terms so callers
// never need to include engine-sim headers. Mapped onto
// CombustionChamber::AfterfireParameters by BridgeSimulator::configureAfterfire.
//
// These are PHYSICAL properties of exhaust-gas auto-ignition, not a firing
// schedule. There is no throttle cutoff, RPM floor, probability, cooldown or
// per-decel event cap here any more: a pop happens when unburnt fuel in a hot
// runner completes its Arrhenius induction period before being scavenged out.
// Overrun produces that condition by itself, so it needs no explicit gate.
struct AfterfireConfig {
    bool enabled = false;

    // Stage 1 — misfire. Manifold pressure (Pa) below which the charge is so
    // diluted that combustion breaks down and raw fuel is pumped into the
    // exhaust. This is what selects overrun: measured MAP is 98 kPa at WOT,
    // 52 kPa at part throttle and idle, but 25-29 kPa on a coast.
    double misfireManifoldPressurePa = 40530.0;   // ~0.4 atm

    // Throttle position below which afterfire is allowed.
    // Speed control s where s=1 = wide open, s=0 = shut.
    // This prevents firing at steady part-throttle where MAP may also be low.
    double throttleCutoff = 0.1;

    // Stage 2 — auto-ignition.
    // tau(T) = ignitionDelayRefS * exp(activationTempK * (1/T - 1/refTempK)).
    // Tuned toward "cold": 0.02 -> 0.3 lengthens the induction period so the
    // crackle sequence spreads into the coast-down (pops keep firing as the runner
    // cools through 1000-800K) instead of one near-instant burst on the hottest
    // runner. Note tau is ~50x more sensitive to runner temp than to this knob, so
    // the very first pop on a ~1700K runner is still quick; dial this up/down, or
    // adjust activationTempK, to taste. Tests override this default.
    double ignitionDelayRefS = 0.3;
    double activationTempK   = 8000.0;
    double refTempK          = 1000.0;

    // Auto-ignition temperature of gasoline vapour (K). Below this, no light-off.
    double autoIgnitionTempK = 750.0;

    // Reactants must be present for a reaction to occur.
    double minRawFuelFraction    = 0.0005;
    double minOxygenMoleFraction = 0.01;

    // Trim on released energy; 1.0 = the fuel's real energy density.
    double energyScale = 1.0;

    // Custom impulse response for afterfire pops.
    // Can be a single file path or a glob pattern (e.g., "es/sound-library/new/*.wav").
    // Resolved relative to the executable directory. If a glob, one matching file
    // is chosen randomly for each pop.
    std::string afterfireWavPath;

    bool diagnostics = false;
};

// Per-chamber afterfire counters, mirrored out of engine-sim. eventCount is the
// field the acceptance test asserts on; the rest explain WHY a pop did not
// happen — each names the missing PHYSICAL precondition. maxIgnitionProgress is
// the most informative: it reports how close the induction integral ever came
// to completing, which separates "never hot enough" from "always scavenged
// first" without guesswork.
struct AfterfireDiagnostics {
    int eventCount = 0;
    int skippedTooCold = 0;
    int skippedNoFuel = 0;
    int skippedNoOxygen = 0;
    int skippedNotReady = 0;
    int skippedThrottle = 0;
    int misfireCycles = 0;
    double maxIgnitionProgress = 0.0;
    double maxRunnerTempK = 0.0;
    double maxRawFuelFraction = 0.0;
    double minManifoldPressurePa = 0.0;
    double lastEventRpm = 0.0;
    double lastEventThrottle = 0.0;
    double lastEventPeakPressure = 0.0;
    double lastEventEnergyReleased = 0.0;
    double lastEventRunnerTempK = 0.0;
};

// Runtime statistics
struct EngineSimStats {
    double currentRPM = 0.0;
    double currentLoad = 0.0;
    double exhaustFlow = 0.0;
    double manifoldPressure = 0.0;
    int32_t activeChannels = 0;
    double processingTimeMs = 0.0;

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

} // namespace EngineSimAudio

inline const char* EngineSimGetVersion() {
    return "1.0.0";
}

#endif // ATG_ENGINE_SIM_BRIDGE_H
