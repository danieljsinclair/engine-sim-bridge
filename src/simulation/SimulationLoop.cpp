// SimulationLoop.cpp - Simulation loop implementation
// Extracted from engine_sim_cli.cpp for SOLID SRP compliance
// Phase E: Uses ISimulator* instead of EngineSimHandle/EngineSimAPI&
// Phase F: Moved to engine-sim-bridge for reusability (GUI, iOS, headless)
// Phase G: Refactored from free function to class with injected dependencies

#include "simulation/SimulationLoop.h"
#include "simulation/audioRenderCallback.h"
#include "simulation/CrankingController.h"
#include "simulation/PresentationStateBuilders.h"
#include "session/ISimulatorSession.h"
#include "input/IVehicleControlSink.h"

#include "simulator/ISimulator.h"
#include "simulator/ICombustionEngine.h"
#include "simulator/BridgeSimulator.h"
#include "input/VirtualIceInputProvider.h"
#include "input/IReplayTimeline.h"
#include "input/IArrivalStatePrimer.h"
#include "simulator/EngineSimTypes.h"
#include "simulator/SimulatorFactory.h"

#include "hardware/IAudioHardwareProvider.h"
#include "strategy/IAudioBuffer.h"
#include "io/IInputProvider.h"
#include "io/IPresentation.h"
#include "common/ILogging.h"
#include "common/PresetExceptions.h"
#include "common/wav_writer.h"
#include "telemetry/ITelemetryProvider.h"
#include "common/Verification.h"

#include <cstring>
#include <cstdio>


namespace {

// Timed input simulation constants
constexpr double THROTTLE_RAMP_DURATION_SECONDS = 0.5;  // Time to ramp from 0 to 1
constexpr double FULL_THROTTLE = 1.0;                     // Maximum throttle value
constexpr double SECONDS_TO_MILLISECONDS = 1000.0;

// Instant --start-from: sim seconds of suppressed settling at the HELD
// arrival operating point before the first emitted frame. Long enough for
// the engine to catch through the wheel pin (~0.2s family-A) AND for the
// gas path (runners/chambers/exhaust basin) to relax to the quasi-steady
// state at the arrival rpm/throttle; bounded so the instant contract holds
// (~1.4s wall on the quiet M4 bench at the measured 0.35x compute ratio).
// CONSTANT, independent of the offset — that independence is the contract.
constexpr double ARRIVAL_SETTLE_SECONDS = 4.0;

} // anonymous namespace — constants only

// ============================================================================
// SimulationLoop - Private methods (file scope, access members directly)
// ============================================================================

void SimulationLoop::applyStartStopDecision(LoopState& state, bool lightReportedByTelemetry) {
    // Opinion = a vehicle-control signal exists this frame: telemetry reported
    // the brake light, the keyboard brake level is non-zero, or a drive gear
    // (D/R) is selected. With no opinion the provider keeps start/stop
    // authority (e.g. replay autoStart's frame-0 starter pulse). Once any
    // opinion is seen the controller keeps authority: it is a state machine
    // (crank delay, stop latch) that must not be suspended mid-decision.
    const auto gear = static_cast<bridge::GearSelector>(state.engineInput.gearSelector);
    if (const bool driveSelected =
            gear == bridge::GearSelector::DRIVE || gear == bridge::GearSelector::REVERSE;
        !startStopEngaged_ &&
        !lightReportedByTelemetry &&
        state.engineInput.brakeLevel <= 0.0 &&
        !driveSelected) {
        return;
    }
    startStopEngaged_ = true;

    startStopController_.update(config_.updateInterval(),
                                state.engineInput.brakeLight.value_or(false),
                                gear);

    // Flatten the decision into the input: the controller only writes these
    // two fields; CrankingController downstream stays the ignition/starter
    // actuator authority, exactly as the removed StartStopInputAdapter did.
    state.engineInput.ignition = startStopObserver_.ignition_;
    state.engineInput.starterButton = starterPulseFromLevel(startStopObserver_.starter_);

    // Command twin-based live providers with the same level. The twin gates all
    // of its processing (throttle/gearbox/cranking) on ignition and defaults
    // OFF, so without this push the live path could never process throttle even
    // after a legal start — and conversely the twin can never self-start before
    // the controller's first decision, which now only happens on a real input.
    // Providers without the seam (keyboard, demo, replay) are unaffected.
    if (auto* ignitionSink = dynamic_cast<input::IVehicleControlSink*>(inputProvider_)) {
        ignitionSink->setIgnition(startStopObserver_.ignition_);
    }
}

bool SimulationLoop::starterPulseFromLevel(bool controllerStarterLevel) {
    // The controller can hold starter=true across many frames (e.g. a held
    // brake crank). CrankingController::engageStarter TOGGLES on a held-high
    // button (Stopped -> Cranking -> Stopped), which would abort the crank.
    // Emit a single-frame pulse on the rising edge; hold low until released.
    const bool pulse = controllerStarterLevel && !prevStarterLevel_;
    prevStarterLevel_ = controllerStarterLevel;
    return pulse;
}

input::EngineInput SimulationLoop::pollInput(double currentTime, double updateInterval, bool isFirstTick) {
    if (inputProvider_) {
        return inputProvider_->OnUpdateSimulation(updateInterval);
    }
    input::EngineInput timed;
    timed.throttle = currentTime < THROTTLE_RAMP_DURATION_SECONDS
        ? currentTime / THROTTLE_RAMP_DURATION_SECONDS : FULL_THROTTLE;

    // Send starter button on first tick for non-interactive mode to auto-start engine
    if (isFirstTick) {
        timed.starterButton = true;
    }
    return timed;
}

// One suppressed settle tick. Returns false when settling must stop, each
// for a reason the main loop's machinery already handles: a stop request
// (CTRL+C/SIGTERM — the main loop's stop check returns immediately), a
// disconnected source (its IsConnected check exits cleanly), or a
// non-Continue step result (Stop: duration already reached — e.g. a
// --duration shorter than the settle window — so step() can never advance
// currentTime further; handing over avoids spinning on a frozen clock).
bool SimulationLoop::settleTick(LoopState& state) {
    if (stopRequested_->load(std::memory_order_seq_cst)) return false;
    if (!inputProvider_->IsConnected()) return false;
    if (step(state) != StepResult::Continue) return false;
    state.engineInput = pollInput(state.currentTime, config_.updateInterval(), state.isFirstTick);
    state.isFirstTick = false;
    inputProvider_->provideFeedback(state.previousStats);
    return true;
}

void SimulationLoop::settleAtArrivalPoint(LoopState& state,
                                          input::IArrivalStatePrimer& primer,
                                          double offsetS) {
    logger_->info(LogMask::BRIDGE,
        __ilog_format("Instant start-from %.3fs: priming arrival state + %.1fs core settle (no pre-offset rows simulated)",
            offsetS, ARRIVAL_SETTLE_SECONDS));

    // (1) Provider prime: twin warm-boot from the arrival row + clock anchor +
    //     arrival-row HOLD (constant synthetic input for the settle below).
    primer.primeArrivalState();

    // (2) Core settle: step the FULL per-tick path (engine core + twin via
    //     step(), physics tick via audioBuffer_.updateSimulation) at the held
    //     arrival operating point, suppressing ALL output — CSV write
    //     (emitCsv_=false), presentation, and audio queueing (emitAudio_=false;
    //     the physics tick still runs, only the playback ring stops being
    //     filled). Bounded and offset-independent: the settle constructs the
    //     steady state AT the operating point; it never replays rows.
    emitCsv_ = false;
    emitAudio_ = false;
    if (presentation_) presentation_->setCsvEmissionEnabled(false);
    while (state.currentTime < ARRIVAL_SETTLE_SECONDS && settleTick(state)) {
    }

    // (3) Handoff: release the hold (rows emit from the arrival row onward),
    //     anchor the loop clock at the offset (duration + telemetry timestamps
    //     read the true recording-relative time), resume emission, and start
    //     audio CLEAN at the offset — the ring is reset and the schedule
    //     resynced so playback begins at the --start-from point. Identical
    //     handoff acts to the retired warm-start prefix.
    primer.releaseArrivalHold();
    state.currentTime = offsetS;
    emitCsv_ = true;
    emitAudio_ = true;
    audioBuffer_.resetBufferAfterWarmup();
    clock_->resync();  // un-paced settle left the schedule in the past
    if (presentation_) presentation_->setCsvEmissionEnabled(true);
}

void SimulationLoop::updatePresentation(
                        const EngineSimStats& stats,
                        const CrankingController::State& crankingState,
                        const input::EngineInput& input,
                        double tickTime) {

    if (!presentation_) return;

    telemetry::AudioTimingTelemetry timing;
    if (telemetryReader_) {
        timing = telemetryReader_->getAudioTiming();
    }

    presentation::EngineState state;
    state.engine = presentation::builders::buildEngineState(stats, crankingState);
    state.drivetrain = presentation::builders::buildDrivetrainState(stats, input);
    state.controls = presentation::builders::buildControlState(input, crankingState);
    state.audio = presentation::builders::buildAudioState(timing, telemetryReader_, audioBuffer_, config_, tickTime, simulator_);
    state.presetShortName = simulator_.getName() ? simulator_.getName() : "";
    presentation_->ShowSimulatorStates(state);
}

// ============================================================================
// File-local helpers — pure functions and internal types
// ============================================================================

// Named audio render callback -- bridges AudioBufferView to strategy->render().
// Pure function (no session/globals/hardware); declared in
// simulation/audioRenderCallback.h for direct unit testing.
int audioRenderCallback(IAudioBuffer* strategy, AudioBufferView& buffer) {
    if (!strategy->isPlaying()) {
        // Engine is cranking/not yet playing. The old code hard-memset the
        // buffer to exact-zero — a discontinuity at the audio->silence edge
        // heard as a startup crackle, and it writes exact-zero int16 frames to
        // the --output WAV (the knock detector's zero-sample proxy then flags
        // the startup silence as dropouts). Fill with SILENCE_FLOOR (±9 LSB,
        // ~-88 dBFS, inaudible) instead so the transition is continuous and
        // the zero-sample rate reflects only the audio's natural
        // zero-crossings. This is the SAME floor the strategy's fillSilenceFaded
        // ramps toward, keeping the not-playing -> playing handoff continuous.
        if (float* dst = buffer.asFloat(); dst) {
            const size_t totalSamples = static_cast<size_t>(buffer.frameCount) * buffer.channelCount;
            constexpr float kStartupFloor = EngineSimAudio::SILENCE_FLOOR;
            for (size_t i = 0; i < totalSamples; ++i) {
                dst[i] = kStartupFloor;
            }
        }
        return 0;
    }

    strategy->render(buffer);
    return 0;
}

namespace {

// Create and initialize the audio hardware provider. Throws on failure.
std::unique_ptr<IAudioHardwareProvider> createHardwareProvider(
    int sampleRate,
    const IAudioHardwareProvider::AudioCallback& callback,
    ILogging* logger)
{
    auto provider = AudioHardwareProviderFactory::createProvider(logger);
    provider->registerAudioCallback(callback);

    // Use AudioStreamFormat defaults (stereo float32 interleaved), only override sampleRate
    AudioStreamFormat format;
    format.sampleRate = sampleRate;

    if (!provider->initialize(format)) {
        throw SimulatorException("Failed to initialize audio hardware");
    }

    return provider;
}

// Apply gear changes from keyboard input ([/] keys). Clamps at gear 0 (neutral).
void applyGearChange(ISimulator& simulator, int gearDelta, ILogging* logger) {

    if (simulator.changeGear(gearDelta)) {
        logger->info(LogMask::BRIDGE, __ilog_format("New gear: %+d", simulator.getGear()));
    }
}

void applyDecision(ICombustionEngine* combustionEngine, const TransitionDecision& decision) {
    if (!combustionEngine) return;
    combustionEngine->applyTransition(decision);
}

} // anonymous namespace — file-local helpers

// ============================================================================
// SimulationLoop - Private methods (file scope, access members directly)
// ============================================================================

CrankingController::State SimulationLoop::applyCrankingDecision(
                                                ICombustionEngine* combustionEngine,
                                                const input::EngineInput& engineInput) {

    auto crankingDecision = TransitionDecision{EnginePhase::Running, false, engineInput.throttle, false};

    if (combustionEngine) {
        auto starterDecision = crankingController_.engageStarter(*combustionEngine, engineInput.starterButton, engineInput.ignition);
        applyDecision(combustionEngine, starterDecision);

        crankingDecision = crankingController_.step(*combustionEngine, engineInput.throttle, engineInput.ignition);
        applyDecision(combustionEngine, crankingDecision);
    }

    return CrankingController::State{crankingDecision.effectiveThrottle, combustionEngine && crankingDecision.starterMotor, crankingDecision.targetPhase};
}

void applyDynoControl(ISimulator& simulator, double scale, double& lastScale) {
    if (scale == lastScale) return;  // OK: no-op when unchanged
    if (scale < 0.0) return;  // OK: -1 is a sentinel for "no change" (keep last scale)
    simulator.setDynoTorqueScale(scale);
    lastScale = scale;
}

void SimulationLoop::applyVehicleControls(
    ICombustionEngine* combustionEngine,
    const input::EngineInput& input, const CrankingController::State& crankingState,
    double& lastDynoTorqueScale) {

    simulator_.setThrottle(crankingState.startingThrottle);

    if (combustionEngine) {
        combustionEngine->setIgnition(input.ignition);
    }

    // Apply gear changes: twin gearAbsolute takes priority over keyboard gearDelta
    if (input.gearAbsolute >= 0) {
        simulator_.setGear(input.gearAbsolute);
    } else {
        applyGearChange(simulator_, input.gearDelta, logger_);
    }

    // Vehicle controls (gear, dyno). The vehicle-speed constraint (PIN mode)
    // takes precedence: it drives the wheels directly and the dyno stays off.
    // FREE mode has no speed constraint; the twin signals dyno braking during
    // cranking (dynoTorqueScale > 0) to give the starter a resistive load
    // so the CrankingController can detect engine catch (RPM > 500). Without
    // this load FREE mode's engine free-revs with no resistance and the RPM
    // never climbs, leaving the twin stuck in CRANKING indefinitely.
    if (auto* bridgeSim = dynamic_cast<BridgeSimulator*>(combustionEngine)) {
        if (input.vehicleSpeedTargetKmh >= 0.0) {
            // PIN mode: constraint solver drives the wheels directly.
            bridgeSim->setVehicleSpeedTarget(input.vehicleSpeedTargetKmh);
        } else if (!input.gearAutoMode && input.roadSpeedKmh >= 0.0) {
            // Legacy dyno fallback — manual road-speed input only. In --auto
            // mode the gearbox provider owns speed tracking via the constraint;
            // PARK/NEUTRAL frames must NOT fall through to the dyno (free-rev).
            bridgeSim->setSpeedTrackingTarget(input.roadSpeedKmh, input.engineRpmFloor);
        } else if (input.dynoTorqueScale > 0.0) {
            // FREE mode: twin-requested dyno braking (cranking load). Applied
            // when no speed constraint is active so FREE mode can build RPM.
            applyDynoControl(simulator_, input.dynoTorqueScale, lastDynoTorqueScale);
        }

        // MATCH (Torque) mode: inject the recorded drivetrain torque at the
        // transmission input so the constraint solver integrates road speed from
        // it (engine RPM emerges via the clutch). 0.0 in FREE/PIN is a true
        // no-op on the rotating mass, so this is unconditional — no mode gate.
        bridgeSim->setDrivetrainInputTorque(input.drivetrainInputTorqueNm);
    } else if (!crankingState.starterEngaged) {
        // Non-bridge path: dyno only when starter not engaged (legacy).
        applyDynoControl(simulator_, input.dynoTorqueScale, lastDynoTorqueScale);
    }
    if (crankingState.starterEngaged && !lastStarterEngaged_) {
        logger_->info(LogMask::BRIDGE, "Cranking: starter engaged, dyno disabled - consider using the clutch instead");
    }
    lastStarterEngaged_ = crankingState.starterEngaged;

    // Twin clutch control (direct pressure, overrides applyGearChange's hardwired clutch)
    if (input.clutchPressure >= 0.0) {
        simulator_.setClutchPressure(input.clutchPressure);
    }

    // Brake — physics consumes the analog level (keyboard 'B' only). The CSV
    // brake light is an indicator, not a pedal: it never reaches this call.
    simulator_.setBrakePressure(input.brakeLevel);
}

void SimulationLoop::writeTelemetry(double currentTime, double throttle, bool ignition, bool starterEngaged) {
    if (!telemetryWriter_) return; // we allow skipping telemetry providers for some reason, so this is an allowed no-op

    telemetry::VehicleInputsTelemetry inputs;
    inputs.throttlePosition = throttle;
    inputs.ignitionOn = ignition;
    inputs.starterMotorEngaged = starterEngaged;
    telemetryWriter_->writeVehicleInputs(inputs);

    // Push simulator metrics
    telemetry::SimulatorMetricsTelemetry metrics;
    metrics.timestamp = currentTime;
    telemetryWriter_->writeSimulatorMetrics(metrics);
}

// ============================================================================
// File-local helpers (continued) + SimulatorSession
// ============================================================================
namespace {

// Initialize the simulator: create with audio config.
// Script loading is handled by SimulatorFactory before this is called.
// Throws std::runtime_error on failure.
void initializeSimulator(
    ISimulator& simulator,
    const SimulationConfig& config,
    ILogging* logger,
    telemetry::ITelemetryWriter* telemetryWriter,
    const ISimulatorConfig* engineConfig)
{
    // Use provided label directly, no internal logic about simulator type
    const std::string& label = config.simulatorLabel;
    logger->info(LogMask::BRIDGE, __ilog_format("Loading simulator: %s", label.c_str()));

    if (!simulator.create(*engineConfig, logger, telemetryWriter)) {
        throw SimulatorException("Failed to create simulator: " + simulator.getLastError());
    }
}

void cleanupSimulation(IAudioHardwareProvider* hardwareProvider, ISimulator& simulator) {
    if (hardwareProvider) {
        hardwareProvider->stopPlayback();
        hardwareProvider->cleanup();
    }
    simulator.destroy();
}

// ============================================================================
// SimulatorSession - Concrete session managing audio hardware + simulator lifecycle
// Owns audio hardware for session lifetime. Reuses runSimulationLoop() for the main tick loop.
// Hot-swap is triggered by initSimulation(existingSession) — the session swaps its internal simulator pointer.
// Composes SimulationLoop for the main tick execution.
// ============================================================================

class SimulatorSession : public ISimulatorSession {
public:
    SimulatorSession(
        const SimulationConfig& config,
        std::unique_ptr<ISimulator> simulator,
        const SessionDependencies& deps,
        std::unique_ptr<IAudioHardwareProvider> hardwareProvider,
        std::unique_ptr<WavWriter> wavWriter)
        : config_(config)
        , simulator_(std::move(simulator))
        , audioBuffer_(deps.audioBuffer)
        , hardwareProvider_(std::move(hardwareProvider))
        , wavWriter_(std::move(wavWriter))
        , inputProvider_(deps.inputProvider)
        , presentation_(deps.presentation)
        , telemetryWriter_(deps.telemetryWriter)
        , telemetryReader_(deps.telemetryReader)
        , logger_(deps.logger)
    {}

    ~SimulatorSession() override {
        if (!closed_) {
            doClose();
        }
    }

    int run() override {
        if (closed_) {
            logger_->warning(LogMask::BRIDGE, "Session is closed — ignoring run() call");
            return 0;
        }

        // Start audio only if not already playing (hot-swap keeps audio running)
        if (!audioBuffer_->isPlaying()) {
            if (!audioBuffer_->startPlayback(simulator_.get())) {
                throw SimulatorException("Failed to start audio playback");
            }
            hardwareProvider_->setVolume(config_.volume);
            audioBuffer_->prepareBuffer();

            if (!hardwareProvider_->startPlayback()) {
                logger_->error(LogMask::AUDIO, "Failed to start hardware playback");
            }
        }

        // Create loop with injected dependencies — no parameter plumbing.
        // Deterministic mode paces with the no-op clock: the run advances at
        // CPU speed with zero wall-clock dependence (input rows are consumed
        // on the loop's fixed sim clock, physics on the same thread).
        SessionDependencies loopDeps{
            audioBuffer_,
            &crankingController_,
            &stopRequested_,
            inputProvider_,
            presentation_,
            telemetryWriter_,
            telemetryReader_,
            logger_,
            config_.deterministic ? &fakeClock_ : nullptr
        };
        SimulationLoop loop(*simulator_, config_, loopDeps);

        int exitCode = loop.run();

        // Stop audio only on final exit, not on preset cycle
        if (exitCode != EXIT_BUT_CONTINUE_NEXT) {
            audioBuffer_->stopPlayback(simulator_.get());
        }

        return exitCode;
    }

    void stop() override {
        stopRequested_.store(true, std::memory_order_seq_cst);
    }

    bool hasDrivetrainMomentum(const BridgeSimulator::DrivetrainSnapshot& snapshot) const {
        return snapshot.gear >= 0 && std::abs(snapshot.vehicleMassVtheta) > 1.0;
    }

    void transferDrivetrainState(ISimulator& newSimulator, const ISimulator& oldSimulator, ILogging* logger) {
        // Transfer drivetrain state from old simulator to new
        const auto* oldBridge = dynamic_cast<const BridgeSimulator*>(&oldSimulator);
        auto* newBridge = dynamic_cast<BridgeSimulator*>(&newSimulator);

        if (oldBridge && newBridge) {
            auto snapshot = oldBridge->captureDrivetrainState();
            newBridge->restoreDrivetrainState(snapshot);

            if (snapshot.enginePhase == EnginePhase::Stopped) {
                logger->info(LogMask::BRIDGE, "Old engine was Stopped — no state to transfer");
                return;
            }

            crankingController_.reset();
            auto* combustion = dynamic_cast<ICombustionEngine*>(&newSimulator);
            if (!combustion) return;

            if (hasDrivetrainMomentum(snapshot)) {
                auto rolloverDecision = TransitionDecision{EnginePhase::Rollover, false, 0.0, true};
                applyDecision(combustion, rolloverDecision);
                logger->info(LogMask::BRIDGE, __ilog_format("Hot-swap → Rollover (gear=%d, vtheta=%.1f)", snapshot.gear, snapshot.vehicleMassVtheta));
            } else {
                auto decision = crankingController_.engageStarter(*combustion, true, true);
                applyDecision(combustion, decision);
                logger->info(LogMask::BRIDGE, "Hot-swap → Cranking (neutral, starter engaged)");
            }
        } else {
            logger->error(LogMask::BRIDGE, "Drivetrain transfer skipped — one or both simulators are not BridgeSimulators");
        }
    }

    bool handoverSession(const std::string& presetFilePath, std::unique_ptr<ISimulator> newSimulator) {
        ASSERT(!closed_, "handoverSession: session is closed");
        ASSERT(newSimulator, "handoverSession: null simulator provided");
        ASSERT(simulator_, "handoverSession: current simulator is null");

        logger_->info(LogMask::BRIDGE, __ilog_format("handoverSession: loading %s", presetFilePath.c_str()));

        // Initialize the new simulator (audio config only, not pipeline)
        initializeSimulator(*newSimulator, config_, logger_, telemetryWriter_, &config_.engineConfig);

        // pre-init the new simulator with the engine state, road speed, gears etc
        //as if we swapped the engine in a running vehicle. no effect on a non combustion sim
        transferDrivetrainState(*newSimulator, *simulator_, logger_);

        // Keep old simulator alive in previousSimulator_ to prevent
        // use-after-free in the audio callback (SyncPullStrategy holds a raw pointer)
        previousSimulator_ = std::move(simulator_);

        // Swap to the new simulator
        simulator_ = std::move(newSimulator);

        // Update the audio buffer's pointer to the new simulator
        audioBuffer_->swapSimulator(simulator_.get());

        // Update config path
        config_.configPath = presetFilePath;

        // Reset stopped flag so the loop runs again when session->run() is called
        stopRequested_.store(false, std::memory_order_seq_cst);

        logger_->info(LogMask::BRIDGE, "handoverSession: complete");
        return true;
    }

    void close() override {
        if (closed_) return;
        doClose();
    }

    void doClose() {
        cleanupSimulation(hardwareProvider_.get(), *simulator_);
        closed_ = true;
    }

    ISimulator* getSimulator() const override {
        return simulator_.get();
    }

private:
    SimulationConfig config_;
    std::unique_ptr<ISimulator> simulator_;
    std::unique_ptr<ISimulator> previousSimulator_;
    IAudioBuffer* audioBuffer_;
    std::unique_ptr<IAudioHardwareProvider> hardwareProvider_;
    std::unique_ptr<WavWriter> wavWriter_;
    input::IInputProvider* inputProvider_;
    presentation::IPresentation* presentation_;
    telemetry::ITelemetryWriter* telemetryWriter_;
    telemetry::ITelemetryReader* telemetryReader_;
    ILogging* logger_;
    CrankingController crankingController_;
    std::atomic<bool> stopRequested_{false};
    // No-op clock for deterministic mode (unpaced, CPU-speed replay).
    FakeLoopClock fakeClock_;
    bool closed_{false};
};

} // anonymous namespace

// ============================================================================
// SimulationLoop - Implementation
// ============================================================================

SimulationLoop::SimulationLoop(
    ISimulator& simulator,
    const SimulationConfig& config,
    const SessionDependencies& deps)
        : simulator_(simulator)
        , config_(config)
        , audioBuffer_(*deps.audioBuffer)
        , crankingController_(*deps.crankingController)
        , stopRequested_(deps.stopRequested)
        , inputProvider_(deps.inputProvider)
        , presentation_(deps.presentation)
        , telemetryWriter_(deps.telemetryWriter)
        , telemetryReader_(deps.telemetryReader)
        , logger_(deps.logger)
        , clock_(deps.clock ? deps.clock : nullptr)
        , steadyClock_(deps.clock ? nullptr : std::make_unique<SteadyClockLoopClock>(config.updateInterval()))
    {
        // Point clock_ to steadyClock_ if no clock was injected
        if (!clock_) {
            clock_ = steadyClock_.get();
        }
    }

int SimulationLoop::run() {
    logger_->info(LogMask::BRIDGE, __ilog_format("SimulationLoop starting with %s", config_.simulatorLabel.c_str()));

    // Initialize loop state
    LoopState state;
    state.currentTime = 0.0;
    state.isFirstTick = true;
    state.lastDynoTorqueScale = -1.0;
    state.combustionEngine = dynamic_cast<ICombustionEngine*>(&simulator_);

    // Pre-loop: provide initial feedback to input provider
    if (inputProvider_) inputProvider_->provideFeedback(state.previousStats);

    // Instant --start-from — FILE TRACES ONLY (durationS() >= 0). The owner
    // contract: rows before the offset NEVER existed — no frame of them is
    // simulated, whatever the offset (start, middle, end; a capture may only
    // hold the middle of a drive). The old warm-start prefix stepped the full
    // sim from 0 to the offset at CPU speed (~0.35x real time — 31s of compute
    // for a 90s offset) to carry the gas path's history; the owner has
    // rejected that trade. The replacement synthesizes the arrival state:
    //
    //   (1) PROVIDER PRIME (IArrivalStatePrimer): the twin warm-boots seeded
    //       from the ARRIVAL row (first row at/after the offset: throttle,
    //       road speed, selector) and settles its gearbox/coupling at that
    //       operating point; the replay clock cold-jumps onto the arrival
    //       row's timecode. Twin-only, microseconds.
    //   (2) CORE SETTLE (here): a BOUNDED window (kArrivalSettleSeconds of
    //       sim time) stepping the full per-tick path with the HELD arrival
    //       row as a CONSTANT input and all emission suppressed. The wheel
    //         pin (PIN/TC coupling) drags the drivetrain to the recorded
    //       road speed — the engine catches through its own physics (the
    //       same bump-start the live attach uses, family-A ~0.17s) — and the
    //       gas path (intake runners, chambers, exhaust basin) relaxes to
    //       the quasi-steady state CONSISTENT with that rpm/throttle: the
    //       steady state is an attractor of the operating point, so holding
    //       the point reaches it without any pre-offset history. This is the
    //       same "construct a running engine at time T" model as preset
    //       hot-swap (transferDrivetrainState) — momentum transfer through
    //       the drivetrain, never a hand-set crank speed.
    //   (3) HANDOFF: release the hold (rows emit from the arrival row on),
    //       anchor the loop clock at the offset, reset the audio ring and
    //       resync the schedule — identical handoff acts to the old prefix.
    //
    // LIVE streams (durationS() < 0, e.g. stdin) SKIP this block exactly as
    // before: a live stream cannot be seeked, only consumed; the live
    // provider implements its own instant contract (unpaced pre-window
    // discard + warm-boot prime + display offset — see
    // LiveTelemetryProvider::setStartFromS) and must stay unchanged.
    //
    // A provider with no offset (startFromS_ <= 0) skips this entirely ->
    // zero behavior change for from-0 runs (byte-identical). Only
    // IReplayTimeline providers (replay/live) carry an offset;
    // keyboard/demo/manual providers don't, and the cast is null for them.
    if (inputProvider_) {
        const auto* timeline = dynamic_cast<const input::IReplayTimeline*>(inputProvider_);
        if (timeline && timeline->getStartFromS() > 0.0
                && timeline->durationS() >= 0.0) {
            auto* primer = dynamic_cast<input::IArrivalStatePrimer*>(inputProvider_);
            ASSERT(primer, "file-trace --start-from provider must implement "
                           "IArrivalStatePrimer (instant arrival-state prime)");
            settleAtArrivalPoint(state, *primer, timeline->getStartFromS());
        }
    }

    // Main loop: thin wrapper calling step()
    for (;;) {
        // Execute one simulation tick
        StepResult result = step(state);

        // Loop control: pacing, input polling, feedback
        clock_->waitUntilNextTick();
        state.engineInput = pollInput(state.currentTime, config_.updateInterval(), state.isFirstTick);
        state.isFirstTick = false;

        // Feed stats back to input provider for next tick's decisions
        if (inputProvider_) inputProvider_->provideFeedback(state.previousStats);

        // Handle step results (after feedback, matching original behavior)
        if (result == StepResult::PresetCycle) {
            return EXIT_BUT_CONTINUE_NEXT;
        }
        if (result == StepResult::Stop) {
            return 0;
        }

        // Check stop flag
        if (stopRequested_->load(std::memory_order_seq_cst)) {
            return 0;
        }

        // Streaming-source EOF (live-telemetry stdin / piped CSV): when the
        // provider reports it is no longer connected, the input is exhausted and
        // every buffered row has been delivered — terminate cleanly instead of
        // looping forever on stale telemetry. Only a streaming provider ever
        // disconnects mid-run (keyboard/demo/replay/manual stay connected and
        // stop via duration / the user); interactive mode has no provider (the
        // guard skips it).
        if (inputProvider_ && !inputProvider_->IsConnected()) {
            return 0;
        }
    }
}

// ============================================================================
// step() - Per-tick simulation logic (sleep-free, deterministic)
// Extracted from run() for unit testing
// ============================================================================

StepResult SimulationLoop::step(LoopState& state) {
    // Duration check: stop when currentTime reaches or exceeds duration
    if (config_.duration > 0.0 && state.currentTime >= config_.duration) {
        return StepResult::Stop;
    }

    // Brake-light assembly — the SINGLE derivation point. brakeLight is the
    // canonical display/start-stop signal; brakeLevel is the physics control
    // (keyboard 'B' is its only writer). Telemetry (CSV brake_light column)
    // supplies the light directly; when no telemetry reports it, the local
    // brake level derives it. The light never writes the level (physics).
    // The pre-assembly presence of the value tells the start/stop decision
    // below whether telemetry reported an opinion this frame.
    const bool lightReportedByTelemetry = state.engineInput.brakeLight.has_value();
    if (!lightReportedByTelemetry) {
        state.engineInput.brakeLight = state.engineInput.brakeLevel > 0.0;
    }

    // Vehicle start/stop — the ONE decision site every input mode traverses
    // (keyboard, demo, replay, live). Runs off the canonical light + gear so
    // the consumer cannot tell the sources apart.
    applyStartStopDecision(state, lightReportedByTelemetry);

    // Per-tick simulation logic
    CrankingController::State crankingState = applyCrankingDecision(state.combustionEngine, state.engineInput);

    applyVehicleControls(state.combustionEngine, state.engineInput, crankingState, state.lastDynoTorqueScale);

    audioBuffer_.updateSimulation(&simulator_, config_.updateInterval() * SECONDS_TO_MILLISECONDS);
    // Physics tick (updateSimulation -> simulator->update) ALWAYS runs — it must
    // advance identically in settle and main loop or the gas path stays cold at
    // the handoff (the bug behind sick --start-from runs). Audio QUEUEING is
    // gated: during the suppressed settle no rendered samples enter the
    // playback ring (silent settle; the ring is drained/reset at handoff).
    if (emitAudio_) {
        audioBuffer_.fillBufferFromEngine(&simulator_, config_.framesPerUpdate());
    }

    // CSV telemetry write is suppressed during the suppressed settle (emitCsv_
    // gate); the engine is stepped normally (above).
    if (emitCsv_) {
        writeTelemetry(state.currentTime, crankingState.startingThrottle, state.engineInput.ignition, crankingState.starterEngaged);
    }

    EngineSimStats stats = simulator_.getStats();
    state.currentTime += config_.updateInterval();
    // Presentation suppressed during the suppressed settle (emitCsv_ false) so
    // console progress lines don't print during the silent settle; the audio
    // simulation advancement above keeps running regardless.
    if (emitCsv_) {
        updatePresentation(stats, crankingState, state.engineInput, state.currentTime);
    }

    // Update cross-tick state
    state.previousStats = stats;

    // Check for preset cycle request
    if (state.engineInput.presetCycle) {
        return StepResult::PresetCycle;
    }

    return StepResult::Continue;
}

// ============================================================================
// Session factory - GLOBAL scope
// ============================================================================

std::unique_ptr<ISimulatorSession> createSession(
    const SimulationConfig& config,
    const std::string& scriptPath,
    std::unique_ptr<ISimulator> simulator,
    SessionDependencies& deps,
    std::unique_ptr<ISimulatorSession> existingSession,
    std::unique_ptr<IAudioHardwareProvider> audioProvider)
{
    auto* audioBuffer = deps.audioBuffer;
    auto* inputProvider = deps.inputProvider;
    auto* presentation = deps.presentation;
    auto* telemetryWriter = deps.telemetryWriter;
    auto* telemetryReader = deps.telemetryReader;
    auto* logger = deps.logger;

    // Hot-swap path: caller passed an existing session — swap the simulator within it
    if (existingSession) {
        ASSERT(simulator, "simulator must be provided for hot-swap");
        if (auto* session = static_cast<SimulatorSession*>(existingSession.get());
            !session || !session->handoverSession(scriptPath, std::move(simulator))) {
            throw SimulatorException("Failed to swap preset within existing session: " + scriptPath);
        }
        return existingSession;
    }

    // First-run path: create a new session with audio hardware
    ASSERT(simulator, "simulator must be provided");
    ASSERT(audioBuffer, "audioBuffer must be provided");
    ASSERT(config.engineConfig.sampleRate > 0, "config.sampleRate must be set");
    ASSERT(config.updateInterval() > 0.0, "config.updateInterval must be set");
    ASSERT(config.framesPerUpdate() > 0, "config.framesPerUpdate must be set");

    // Initialize the simulator
    initializeSimulator(*simulator, config, logger, telemetryWriter, &config.engineConfig);
    SimulationConfig sessionConfig = config;
    sessionConfig.configPath = scriptPath;

    // Bind the live BridgeSimulator to the input provider so it can install the
    // fluid-coupling torque converter on the transmission when --coupling-model
    // torque-converter is selected (the provider is constructed BEFORE the
    // simulator exists, so the install is deferred until here).
    if (auto* virtualIce = dynamic_cast<input::VirtualIceInputProvider*>(inputProvider)) {
        virtualIce->setBridgeSimulator(dynamic_cast<::BridgeSimulator*>(simulator.get()));
    }

    // Initialize audio buffer and create hardware provider (first run only)
    AudioBufferConfig strategyConfig;
    strategyConfig.channels = EngineSimAudio::STEREO;
    strategyConfig.synthLatency = config.engineConfig.targetSynthesizerLatency;
    audioBuffer->initialize(strategyConfig, config.sampleRate());

    // Optional WAV writer for --output <path>. Taps the rendered output AFTER
    // the strategy fills the buffer (the true rendered-output seam) but BEFORE
    // the hardware provider applies the --silent volume mute — so --silent does
    // NOT suppress --output (the file is diagnostic and captures full-scale
    // samples). Ownership passes to the SimulatorSession (lives for the session
    // lifetime); the audio callback captures a raw pointer that stays valid for
    // as long as the session does. No writer == no --output (byte-identical to
    // legacy behavior).
    std::unique_ptr<WavWriter> wavWriter;
    if (config.outputWav && config.outputWav[0] != '\0') {
        auto w = std::make_unique<WavWriter>();
        if (w->open(config.outputWav,
                   static_cast<uint32_t>(config.sampleRate()),
                   static_cast<uint32_t>(EngineSimAudio::STEREO))) {
            wavWriter = std::move(w);
        } else {
            logger->error(LogMask::AUDIO,
                std::string("Failed to open --output WAV for writing: ") + config.outputWav);
        }
    }

    // Stable raw pointer for the callback lambda. The session owns the writer
    // (via wavWriter_) and outlives every callback invocation, so this pointer
    // is valid for the callback's entire lifetime.
    WavWriter* wavWriterPtr = wavWriter.get();
    auto callback = [audioBuffer, wavWriterPtr](AudioBufferView& buffer) {
        int rc = audioRenderCallback(audioBuffer, buffer);
        // Tap rendered output for WAV writing (post-render, pre-silent-mute).
        // asFloat() is non-null for the float32 render path; guard anyway.
        if (wavWriterPtr && wavWriterPtr->isOpen()) {
            if (const float* dst = buffer.asFloat(); dst) {
                wavWriterPtr->writeFrames(dst, static_cast<uint64_t>(buffer.frameCount));
            }
        }
        return rc;
    };

    // Use an injected provider when supplied (headless/testing); otherwise create
    // the real platform provider via createHardwareProvider(). Behavior-identical
    // when audioProvider is null.
    std::unique_ptr<IAudioHardwareProvider> hardwareProvider;
    if (audioProvider) {
        hardwareProvider = std::move(audioProvider);
    } else {
        hardwareProvider = createHardwareProvider(config.sampleRate(), callback, logger);
    }

    logger->info(LogMask::AUDIO, __ilog_format("Audio initialized: strategy=%s, sr=%d", audioBuffer->getName(), config.sampleRate()));

    SessionDependencies sessionDeps{
        audioBuffer,
        nullptr,  // crankingController not needed at session construction
        nullptr,  // stopRequested set by session internally
        inputProvider,
        presentation,
        telemetryWriter,
        telemetryReader,
        logger
    };
    return std::make_unique<SimulatorSession>(
        sessionConfig,
        std::move(simulator),
        sessionDeps,
        std::move(hardwareProvider),
        std::move(wavWriter));
}// GLOBAL scope — session factory, no access to SimulationLoop members
