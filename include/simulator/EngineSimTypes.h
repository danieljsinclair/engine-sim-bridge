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

// How a pop arriving while another is still sounding on the same exhaust channel
// is admitted. Bridge-level MIRROR of engine-sim's PopOverlapMode: EngineSimTypes.h
// is the header callers include INSTEAD of engine-sim's, so it cannot pull in
// one_shot_sample_mixer.h. BridgeSimulator::configureAfterfire translates this to
// the engine-sim enum, and a static_assert there pins the numeric values together
// so a divergence is a compile error rather than a silently wrong mode.
//
// Both modes share one rule: a sounding pop is NEVER restarted or truncated.
enum class AfterfirePopOverlap {
    // Drop the new pop while one is still sounding, so the sounding crack always
    // completes. The OPT-OUT: it makes a frequently popping engine read as
    // discrete cracks, at the cost of discarding overlapping events.
    SuppressWhilePlaying = 0,

    // Layer the new pop ON TOP of the sounding one, both playing to completion.
    // DEFAULT: physically honest for a shared exhaust collector, where two
    // cylinders' cracks genuinely add. Bounded by the mixer's voice count and by
    // the inter-pop floor (minPopIntervalMs).
    SumOnTop = 1,
};

// Default inter-pop floor, in audio milliseconds. Mirrors
// OneShotSampleMixer::DefaultMinPopIntervalMs, which this header cannot include
// (see AfterfirePopOverlap above). A constant rather than a literal in the struct
// so the static_assert in BridgeSimulator::configureAfterfire has something
// constexpr to pin against — AfterfireConfig itself is not a literal type.
constexpr double DEFAULT_AFTERFIRE_MIN_POP_INTERVAL_MS = 50.0;

// Default added-decay divisor for a pop: 0 = OFF, so a pop plays as stored.
// Mirrors OneShotSampleMixer::DefaultDecayTimeConstantDivisor, which this header
// cannot include (see AfterfirePopOverlap above); the static_assert in
// BridgeSimulator::configureAfterfire pins the two together.
constexpr double DEFAULT_AFTERFIRE_POP_DECAY_DIVISOR = 0.0;

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
    //
    // This is the induction time at refTempK (1000 K), NOT the delay you hear.
    // The runner on a real overrun sits far hotter than the reference — measured
    // ~1880-1910 K on the C63_M156_V3 rev-and-cut — and at 1900 K the Arrhenius
    // factor is exp(8000*(1/1900 - 1/1000)) ~= 0.023, so the effective tau in the
    // pipe is ~44x SHORTER than the number written here. That compression is why
    // a physical-looking 0.02 fired the first pop on the very tick of the cut.
    //
    // Measured on that scenario, one value per process (first pop after the cut /
    // pop count / spread between first and last pop):
    //     0.02 -> 0.000 s, 30 pops, 1.000 s   (instant, machine-gun)
    //     0.3  -> 0.050 s, 20 pops, 0.317 s
    //     1.0  -> 0.067 s, 13 pops, 0.517 s
    //     3.0  -> 0.100 s,  7 pops, 0.333 s   (current)
    //     6.0  -> 0.167 s,  4 pops, 0.083 s
    //     8.0  -> 0.250 s,  2 pops, 0.083 s
    //    10.0  -> no pops at all              (scavenging always wins)
    //
    // 3.0 puts the first pop ~100 ms behind the cut — a beat, not a coincidence —
    // and thins 30 pops to 7 that still spread over a third of a second, so the
    // crackle reads as a sequence rather than one burst. Beyond ~6 the sequence
    // collapses to a couple of isolated cracks and 10 kills the effect outright,
    // so this keeps real margin to that cliff.
    //
    // Measure isolated: several values swept inside ONE process interfere (a
    // later value read 0 pops in-process but 4 on its own), so re-tune with one
    // value per run. Repeatability at 3.0: three runs, identical timings.
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

    // Afterfire MASTER VOLUME (0..10). Scales the pop's loudness as a whole: it
    // attenuates the combustion energy that enters the exhaust runner (so the
    // physical crackle is quieter) AND the WAV overlay mixed on top. 0 = silent,
    // 1 = full physical crackle + full WAV, higher = louder than physical.
    // Default 0.6.
    double customGain = 0.6;

    // ISOLATION SWITCH: play ONLY the WAV overlay, with no physical pop.
    //
    // The audible afterfire has two independent components: the PHYSICAL crackle
    // (the exhaust-runner pressure spike the synthesizer reads out of the gas
    // state) and the WAV overlay mixed onto the same channel. When this is true
    // the combustion energy is withheld from the runner, so no pressure spike
    // forms and the physical crackle is silent, while the WAV still plays.
    //
    // The event itself is unchanged: it fires on the same physics at the same
    // instant, consumes the same fuel and increments the same counters, so the
    // event stream matches the ungated build exactly. That is what makes it a
    // DIAGNOSTIC — if a pop still sounds wrong with this on, the WAV is the
    // culprit, not the physics.
    //
    // Must MATCH CombustionChamber::AfterfireParameters::wavOnly's default: this
    // struct is pushed unconditionally through configureAfterfire, so a stale
    // default here would silently override the chamber's rather than defer to it.
    bool wavOnly = false;

    // Custom impulse response for afterfire pops.
    // Can be a single file path or a glob pattern (e.g., "es/sound-library/new/*.wav").
    // Resolved relative to the executable directory. If a glob, one matching file
    // is chosen randomly for each pop.
    std::string afterfireWavPath;

    // --- Pop PLAYBACK behaviour (WAV overlay only, not the physics) -----------
    // A V-engine shares one exhaust channel across cylinders, so on an engine
    // that pops constantly overlap is the normal case. SumOnTop (the default)
    // layers them; SuppressWhilePlaying is the opt-out that lets the sounding
    // crack finish and drops the new pop. Neither ever restarts a sounding pop.
    //
    // Must MATCH CombustionChamber::AfterfireParameters' default: this struct is
    // pushed unconditionally through configureAfterfire, so a stale default here
    // would silently override the chamber's rather than defer to it.
    AfterfirePopOverlap popOverlapMode = AfterfirePopOverlap::SumOnTop;

    // Minimum spacing, in audio milliseconds, between ACCEPTED pops on one
    // exhaust channel. This is what makes SumOnTop safe: without it a frequently
    // popping engine layers cracks faster than they decay. 0 disables the floor.
    // Default mirrors OneShotSampleMixer::DefaultMinPopIntervalMs; the
    // static_assert in BridgeSimulator::configureAfterfire pins them together.
    double minPopIntervalMs = DEFAULT_AFTERFIRE_MIN_POP_INTERVAL_MS;

    // Added exponential decay applied to a pop on playback, as a divisor of the
    // pop's own length: tau = length / divisor. 0 (THE DEFAULT) = no added decay;
    // larger = decays faster.
    //
    // DEFAULT IS PASSTHROUGH: a pop plays exactly as stored — full duration,
    // correct rate, its own shape intact — because a pop WAV is already a
    // recording of a bang and does not need an envelope imposed on it. Applying
    // one to an already-shaped sample multiplies the two envelopes and buries
    // every crack after the first: measured on a real backfire, that collapsed
    // the trailing/leading energy ratio from 0.226 to 0.045, roughly 14 dB.
    //
    // Set it (3.0 is the classic value) for a SYNTHESISED or flat pop, which has
    // no shape of its own and needs the decay to read as a crack with audible
    // troughs between successive pops.
    //
    // Default mirrors OneShotSampleMixer::DefaultDecayTimeConstantDivisor; the
    // static_assert in BridgeSimulator::configureAfterfire pins them together.
    double popDecayDivisor = DEFAULT_AFTERFIRE_POP_DECAY_DIVISOR;

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
