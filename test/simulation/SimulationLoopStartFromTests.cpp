// SimulationLoopStartFromTests.cpp - Contract tests for instant --start-from
// (file traces) in SimulationLoop::run().
//
// The owner contract: rows before the --start-from offset are NEVER
// simulated. The loop must (1) ask the provider (IArrivalStatePrimer) to
// prime the arrival state, (2) settle the engine core for a BOUNDED window
// at the held arrival operating point with all emission suppressed, and
// (3) emit from the offset onward — the total simulated ticks must not grow
// with the offset. These tests pin that contract with a fake provider that
// implements IReplayTimeline + IArrivalStatePrimer over a counting audio
// buffer (updateSimulation == one sim tick) and a recording telemetry writer
// (first emitted metrics timestamp == the loop clock at first emission).

#include "simulation/SimulationLoop.h"
#include "simulator/BridgeSimulator.h"
#include "simulator/SineSimulator.h"
#include "simulator/SineEngine.h"
#include "simulator/SineVehicle.h"
#include "simulator/SineTransmission.h"
#include "simulator/EngineSimTypes.h"
#include "simulator/ISimulator.h"
#include "simulator/ICombustionEngine.h"
#include "io/IInputProvider.h"
#include "io/IPresentation.h"
#include "telemetry/ITelemetryProvider.h"
#include "strategy/IAudioBuffer.h"
#include "common/ILogging.h"
#include "simulation/CrankingController.h"
#include "simulation/ILoopClock.h"
#include "input/IReplayTimeline.h"
#include "input/IArrivalStatePrimer.h"

#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <string>
#include <vector>

namespace {

using namespace input;
using namespace telemetry;

// Fake replay provider: IReplayTimeline (file trace: durationS >= 0) +
// IArrivalStatePrimer. Records the prime/release call order and, while held,
// reports the arrival row's constant input; after release its clock advances
// normally so the loop sees a live trace.
class FakeStartFromProvider : public IInputProvider,
                              public IReplayTimeline,
                              public IArrivalStatePrimer {
public:
    EngineInput OnUpdateSimulation(double dt) override {
        if (!holdActive_) {
            clockS_ += dt;
        }
        EngineInput input;
        input.throttle = holdActive_ ? arrivalThrottle_ : 0.1;
        input.ignition = true;
        input.replayTimestampS = holdActive_ ? arrivalTimeS_ : clockS_;
        if (holdActive_) {
            ++holdPolls_;
        } else {
            ++postPolls_;
        }
        return input;
    }
    void provideFeedback(const EngineSimStats&) override {}
    bool Initialize() override { return true; }
    void Shutdown() override {}
    bool IsConnected() const override { return true; }
    std::string GetProviderName() const override { return "FakeStartFromProvider"; }
    std::string GetLastError() const override { return ""; }

    // IReplayTimeline
    double durationS() const override { return 600.0; }
    void setEndAtS(double) override {}
    double getStartFromS() const override { return startFromS_; }

    // IArrivalStatePrimer
    void primeArrivalState() override {
        holdActive_ = true;
        primed_ = true;
        ++primeCalls_;
    }
    void releaseArrivalHold() override {
        holdActive_ = false;
        ++releaseCalls_;
    }

    void configure(double startFromS, double arrivalTimeS, double arrivalThrottle) {
        startFromS_ = startFromS;
        arrivalTimeS_ = arrivalTimeS;
        arrivalThrottle_ = arrivalThrottle;
    }

    int primeCalls_ = 0;
    int releaseCalls_ = 0;
    int holdPolls_ = 0;
    int postPolls_ = 0;
    bool primed_ = false;
    bool holdActive_ = false;
    double startFromS_ = -1.0;
    double arrivalTimeS_ = 0.0;
    double arrivalThrottle_ = 0.0;
    double clockS_ = 0.0;
};

class CountingAudioBuffer : public IAudioBuffer {
public:
    const char* getName() const override { return "CountingAudioBuffer"; }
    bool isEnabled() const override { return true; }
    bool isPlaying() const override { return true; }
    bool render(AudioBufferView&) override { return true; }
    bool AddFrames(float*, int) override { return true; }
    bool initialize(const AudioBufferConfig&, int) override { return true; }
    void prepareBuffer() override {}
    bool startPlayback(ISimulator*) override { return true; }
    void stopPlayback(ISimulator*) override {}
    void swapSimulator(ISimulator*) override {}
    void resetBufferAfterWarmup() override { ++resetCount_; }
    bool shouldDrainDuringWarmup() const override { return false; }
    void updateSimulation(ISimulator*, double) override { ++updateCount_; }
    void fillBufferFromEngine(ISimulator*, int) override { ++fillCount_; }
    void reset() override {}
    std::string getModeString() const override { return "CountingAudioBuffer"; }

    int updateCount_ = 0;
    int fillCount_ = 0;
    int resetCount_ = 0;
};

class RecordingTelemetryWriter : public ITelemetryWriter {
public:
    void writeEngineState(const EngineStateTelemetry&) override {}
    void writeFramePerformance(const FramePerformanceTelemetry&) override {}
    void writeAudioDiagnostics(const AudioDiagnosticsTelemetry&) override {}
    void writeAudioTiming(const AudioTimingTelemetry&) override {}
    void writeVehicleInputs(const VehicleInputsTelemetry&) override {}
    void writeSimulatorMetrics(const SimulatorMetricsTelemetry& m) override {
        metricTimestamps_.push_back(m.timestamp);
    }
    void reset() override {}
    const char* getName() const override { return "RecordingTelemetryWriter"; }

    std::vector<double> metricTimestamps_;
};

class NullTelemetryReader : public ITelemetryReader {
public:
    AudioTimingTelemetry getAudioTiming() const override { return AudioTimingTelemetry{}; }
    EngineStateTelemetry getEngineState() const override { return EngineStateTelemetry{}; }
    FramePerformanceTelemetry getFramePerformance() const override { return FramePerformanceTelemetry{}; }
    AudioDiagnosticsTelemetry getAudioDiagnostics() const override { return AudioDiagnosticsTelemetry{}; }
    VehicleInputsTelemetry getVehicleInputs() const override { return VehicleInputsTelemetry{}; }
    SimulatorMetricsTelemetry getSimulatorMetrics() const override { return SimulatorMetricsTelemetry{}; }
    const char* getName() const override { return "NullTelemetryReader"; }
};

class NullPresentation : public presentation::IPresentation {
public:
    bool Initialize(const presentation::PresentationConfig&) override { return true; }
    void Shutdown() override {}
    void ShowMessage(const std::string&) override {}
    void ShowError(const std::string&) override {}
    void ShowProgress(double, double) override {}
    void Update(double) override {}
    void ShowSimulatorStates(const presentation::EngineState&) override {}
    void setCsvEmissionEnabled(bool enabled) override { csvEmissionCalls_.push_back(enabled); }

    std::vector<bool> csvEmissionCalls_;
};

class NullLogger : public ILogging {
public:
    void setMask(uint32_t) override {}
    uint32_t getMask() const override { return 0; }
protected:
    void _write(uint32_t, const std::string&) override {}
};

// Deterministic pacing probe: counts waitUntilNextTick calls so tests can
// assert WHERE pacing happens without wall-clock time or per-iteration
// hand-modelling of the loop's shape.
class CountingLoopClock : public FakeLoopClock {
public:
    void waitUntilNextTick() override { ++waitCount_; }
    int waitCount_ = 0;
};

class SimulationLoopStartFromTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto sineSim = std::make_unique<SineSimulator>();
        Simulator::Parameters simParams;
        simParams.systemType = Simulator::SystemType::NsvOptimized;
        sineSim->initialize(simParams);
        sineSim->setSimulationFrequency(EngineSimDefaults::SIMULATION_FREQUENCY);
        sineSim->setFluidSimulationSteps(EngineSimDefaults::FLUID_SIMULATION_STEPS);
        sineSim->setTargetSynthesizerLatency(EngineSimDefaults::TARGET_SYNTH_LATENCY);
        sineSim->loadSimulation(new SineEngine(), new SineVehicle(), new SineTransmission());

        simulator_ = std::make_unique<BridgeSimulator>(std::move(sineSim), "StartFromTestSim");
        ISimulatorConfig config;
        config.sampleRate = EngineSimDefaults::SAMPLE_RATE;
        config.simulationFrequency = EngineSimDefaults::SIMULATION_FREQUENCY;
        config.fluidSimulationSteps = EngineSimDefaults::FLUID_SIMULATION_STEPS;
        config.targetSynthesizerLatency = EngineSimDefaults::TARGET_SYNTH_LATENCY;
        ASSERT_TRUE(simulator_->create(config, nullptr, nullptr));

        crankingController_ = std::make_unique<CrankingController>();
        stopRequested_ = std::make_unique<std::atomic<bool>>(false);
        clock_ = std::make_unique<CountingLoopClock>();
    }

    SessionDependencies buildDeps(input::IInputProvider* provider) {
        return SessionDependencies{
            audioBuffer_.get(),
            crankingController_.get(),
            stopRequested_.get(),
            provider,
            presentation_.get(),
            telemetryWriter_.get(),
            telemetryReader_.get(),
            logger_.get(),
            clock_.get()
        };
    }

    // Run the loop to completion with the given provider + config duration.
    int runLoop(FakeStartFromProvider& provider, SimulationConfig cfg) {
        SimulationLoop loop(*simulator_, cfg, buildDeps(&provider));
        return loop.run();
    }

    std::unique_ptr<BridgeSimulator> simulator_;
    std::unique_ptr<CountingAudioBuffer> audioBuffer_ = std::make_unique<CountingAudioBuffer>();
    std::unique_ptr<RecordingTelemetryWriter> telemetryWriter_ = std::make_unique<RecordingTelemetryWriter>();
    std::unique_ptr<NullTelemetryReader> telemetryReader_ = std::make_unique<NullTelemetryReader>();
    std::unique_ptr<NullPresentation> presentation_ = std::make_unique<NullPresentation>();
    std::unique_ptr<NullLogger> logger_ = std::make_unique<NullLogger>();
    std::unique_ptr<CrankingController> crankingController_;
    std::unique_ptr<std::atomic<bool>> stopRequested_;
    std::unique_ptr<CountingLoopClock> clock_;
};

// The core owner contract: the number of simulated ticks must NOT grow with
// the offset. A 90s offset and a 5s offset with the same emission window
// simulate the SAME number of ticks (settle + emission), which is bounded —
// the old warm-start prefix would have simulated 90s * 60 = 5400 ticks.
TEST_F(SimulationLoopStartFromTest, SimulatedTicksDoNotGrowWithOffset) {
    FakeStartFromProvider nearProvider;
    nearProvider.configure(/*startFromS=*/5.0, /*arrivalTimeS=*/5.0, /*throttle=*/0.5);
    SimulationConfig nearCfg;
    nearCfg.duration = 5.0 + 0.2;  // 0.2s of emission (12 ticks)
    runLoop(nearProvider, nearCfg);
    const int nearTicks = audioBuffer_->updateCount_;
    const int nearEmitted = static_cast<int>(telemetryWriter_->metricTimestamps_.size());

    // Fresh counting fixtures for the far-offset run.
    audioBuffer_ = std::make_unique<CountingAudioBuffer>();
    telemetryWriter_ = std::make_unique<RecordingTelemetryWriter>();

    FakeStartFromProvider farProvider;
    farProvider.configure(/*startFromS=*/90.0, /*arrivalTimeS=*/90.0, /*throttle=*/0.5);
    SimulationConfig farCfg;
    farCfg.duration = 90.0 + 0.2;
    runLoop(farProvider, farCfg);
    const int farTicks = audioBuffer_->updateCount_;
    const int farEmitted = static_cast<int>(telemetryWriter_->metricTimestamps_.size());

    EXPECT_EQ(nearEmitted, farEmitted);  // same 0.2s emission window
    // Same settle + emission budget regardless of a 5s vs 90s offset.
    EXPECT_EQ(nearTicks, farTicks);
    // Bounded: far below the 5400 ticks the old 90s prefix would simulate.
    EXPECT_LT(farTicks, 600);
}

// The loop primes exactly once (before any emission), releases exactly once
// (after the settle, before the first emitted frame), and polls the HELD
// arrival input during the settle.
TEST_F(SimulationLoopStartFromTest, PrimesThenReleasesAroundBoundedSettle) {
    FakeStartFromProvider provider;
    provider.configure(90.0, 90.0, 0.5);
    SimulationConfig cfg;
    cfg.duration = 90.0 + 0.1;
    runLoop(provider, cfg);

    EXPECT_EQ(provider.primeCalls_, 1);
    EXPECT_EQ(provider.releaseCalls_, 1);
    EXPECT_TRUE(provider.primed_);
    // The settle polled the held arrival input (constant operating point).
    EXPECT_GT(provider.holdPolls_, 0);
    // And the post-release trace was polled for the emission window.
    EXPECT_GT(provider.postPolls_, 0);
}

// The loop clock is ANCHORED at the offset for the first emitted frame: the
// first telemetry metrics timestamp lands AT/after the --start-from offset
// (the loop snaps its clock there at the settle handoff; the exact stamp is
// pinned end-to-end by the replay benches, so the unit contract only needs
// the tolerant bound — an exact == would be an FP-accumulation trap).
TEST_F(SimulationLoopStartFromTest, FirstEmittedFrameIsAnchoredAtOffset) {
    FakeStartFromProvider provider;
    provider.configure(90.0, 90.0, 0.5);
    SimulationConfig cfg;
    cfg.duration = 90.0 + 0.1;
    runLoop(provider, cfg);

    ASSERT_FALSE(telemetryWriter_->metricTimestamps_.empty());
    EXPECT_GE(telemetryWriter_->metricTimestamps_.front(), 90.0);
}

// The settle window is SILENT and UNPACED: no rendered audio is queued from
// pre-emission work (audio queueing is gated during the settle), and no
// pacing wait happens for it (the settle runs at CPU speed; only the emission
// loop waits on the clock). Aggregate-count assertions only — no per-iteration
// shape, no exact counts.
TEST_F(SimulationLoopStartFromTest, SettleIsSilentAndUnpaced) {
    FakeStartFromProvider provider;
    provider.configure(90.0, 90.0, 0.5);
    SimulationConfig cfg;
    cfg.duration = 90.0 + 0.2;
    runLoop(provider, cfg);

    // Settle ticks ran (physics advanced) but queued no audio: fewer fills
    // than physics ticks, with a wide margin (settle is seconds of ticks).
    EXPECT_LT(audioBuffer_->fillCount_, audioBuffer_->updateCount_);
    EXPECT_LT(audioBuffer_->fillCount_, 30);  // ~0.2s emission window only
    // Pacing waits only cover the emission window, not the settle ticks.
    EXPECT_LT(clock_->waitCount_, audioBuffer_->updateCount_);
    EXPECT_LT(clock_->waitCount_, 30);
}

// No offset (from-0): the primer is never invoked and NO settle ticks run —
// only the emission window is simulated. This is the from-0 byte-identity
// guard at the loop level.
TEST_F(SimulationLoopStartFromTest, NoOffsetNeverPrimesOrSettles) {
    FakeStartFromProvider provider;  // startFromS_ stays -1
    SimulationConfig cfg;
    cfg.duration = 0.2;  // 12 ticks of emission only
    runLoop(provider, cfg);

    EXPECT_EQ(provider.primeCalls_, 0);
    EXPECT_EQ(provider.releaseCalls_, 0);
    EXPECT_EQ(provider.holdPolls_, 0);
    // 0.2s at 60Hz = 12 ticks (13 with FP tick accumulation on the boundary).
    EXPECT_GE(audioBuffer_->updateCount_, 11);
    EXPECT_LE(audioBuffer_->updateCount_, 13);
    EXPECT_EQ(telemetryWriter_->metricTimestamps_.size(),
              static_cast<size_t>(audioBuffer_->updateCount_));
    EXPECT_DOUBLE_EQ(telemetryWriter_->metricTimestamps_.front(), 0.0);
}

// A --duration SHORTER than the settle window with a positive offset must
// terminate cleanly, not spin: step() returns Stop (duration reached) without
// advancing currentTime once currentTime >= duration, so the settle loop must
// break on that result. (The retired warm-start prefix had the same unguarded
// shape with a far larger exposure window: any duration < offset.) A run
// whose duration is below the anchored offset emits zero rows — the loop
// clock jumps to the offset, already past the stop time.
TEST_F(SimulationLoopStartFromTest, ShortDurationWithOffsetTerminatesCleanly) {
    FakeStartFromProvider provider;
    provider.configure(90.0, 90.0, 0.5);
    SimulationConfig cfg;
    cfg.duration = 2.0;  // < the 4s settle window, and << the 90s offset
    const int exitCode = runLoop(provider, cfg);

    EXPECT_EQ(exitCode, 0);
    EXPECT_EQ(provider.primeCalls_, 1);
    EXPECT_EQ(provider.releaseCalls_, 1);
    // Nothing emits: the anchored clock (90s) is already past the 2s duration.
    EXPECT_TRUE(telemetryWriter_->metricTimestamps_.empty());
    // Bounded stepping: ~2s at 60Hz — NOT an unbounded settle spin.
    EXPECT_LE(audioBuffer_->updateCount_, 130);
}

}  // namespace
