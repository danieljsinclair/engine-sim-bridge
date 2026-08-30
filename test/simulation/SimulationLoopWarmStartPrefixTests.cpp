// SimulationLoopWarmStartPrefixTests.cpp - Contract tests for the replay
// --start-from warm-start prefix in SimulationLoop::run().
//
// The landed --start-from contract (owner-verified E2E on feat/sliplock-tune):
//   1. UNPACED fast-forward: the prefix [0, startFromS) steps the full sim at
//      CPU speed with ZERO wall-clock pacing (no waitUntilNextTick before the
//      first emitted frame).
//   2. Silent warm-up: no audio frames queued and no presentation/CSV emission
//      during the prefix, while the physics tick (updateSimulation) runs every
//      tick so the gas path arrives warm.
//   3. Handoff at the offset: the FIRST emitted frame is the first tick at or
//      after startFromS, stamped with the replay timecode of that tick — the
//      "[MM:SS.016] of the requested offset" rule.
//   4. No offset (startFromS <= 0) skips the prefix entirely: a from-0 run is
//      byte-for-byte unaffected by this machinery.
//   5. A stop request arriving mid-prefix breaks out cleanly (the prefix can
//      be tens of CPU-seconds; CTRL+C must not be swallowed).
//
// These were previously pinned only by CLI-level E2E runs; the loop-level
// seam had no coverage, which is why a 2026-08-30 owner report ("not instant,
// wrong first timecode") could not be distinguished from regression without
// manual bisection. These tests make that distinction automatic.

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

#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <vector>

namespace {

using namespace input;
using namespace telemetry;

constexpr double kTick = 1.0 / 60.0;  // SimulationConfig::updateInterval()

// Replay-timeline input provider: advances a CSV-relative elapsed clock by dt
// per poll (exactly how ReplayTelemetryProvider::applyTimeSlicing drives
// elapsedS_) and stamps the polled EngineInput with that replay timecode
// (the real provider's input.replayTimestampS = sampleAt(elapsedS_).timeS).
// Optionally trips an external stop flag at the Nth poll to emulate CTRL+C
// arriving mid-prefix.
class ReplayTimelineInputProvider : public IInputProvider, public IReplayTimeline {
public:
    explicit ReplayTimelineInputProvider(std::atomic<bool>* stopFlag = nullptr,
                                         int stopAtPoll = -1)
        : stopFlag_(stopFlag), stopAtPoll_(stopAtPoll) {}

    EngineInput OnUpdateSimulation(double dt) override {
        elapsedS_ += dt;
        ++pollCount_;
        EngineInput input;
        input.replayTimestampS = elapsedS_;
        if (stopFlag_ && stopAtPoll_ > 0 && pollCount_ == stopAtPoll_) {
            stopFlag_->store(true, std::memory_order_seq_cst);
        }
        return input;
    }
    void provideFeedback(const EngineSimStats&) override {}
    bool Initialize() override { return true; }
    void Shutdown() override {}
    bool IsConnected() const override { return true; }
    std::string GetProviderName() const override { return "ReplayTimelineInputProvider"; }
    std::string GetLastError() const override { return ""; }

    void setStartFrom(double s) { startFromS_ = s; }
    double getStartFromS() const override { return startFromS_; }
    double durationS() const override { return 1.0; }  // file trace (>= 0)
    void setEndAtS(double) override {}

    int pollCount() const { return pollCount_; }

private:
    double elapsedS_ = 0.0;
    double startFromS_ = -1.0;
    int pollCount_ = 0;
    std::atomic<bool>* stopFlag_ = nullptr;
    int stopAtPoll_ = -1;
};

// Counts pacing waits: a wait before the first emitted frame would mean the
// prefix is wall-clock paced (the exact regression class these tests guard).
class CountingLoopClock : public ILoopClock {
public:
    void waitUntilNextTick() override { ++waitCount_; }
    void resync() override {}
    int waitCount() const { return waitCount_; }
private:
    int waitCount_ = 0;
};

class RecordingAudioBuffer : public IAudioBuffer {
public:
    const char* getName() const override { return "RecordingAudioBuffer"; }
    bool isEnabled() const override { return true; }
    bool isPlaying() const override { return true; }
    bool render(AudioBufferView&) override { return true; }
    bool AddFrames(float*, int) override { return true; }
    bool initialize(const AudioBufferConfig&, int) override { return true; }
    void prepareBuffer() override {}
    bool startPlayback(ISimulator*) override { return true; }
    void stopPlayback(ISimulator*) override {}
    void swapSimulator(ISimulator*) override {}
    void resetBufferAfterWarmup() override { ++resetAfterWarmupCount_; }
    bool shouldDrainDuringWarmup() const override { return false; }
    void updateSimulation(ISimulator*, double) override { ++updateSimulationCount_; }
    void fillBufferFromEngine(ISimulator*, int) override { ++fillBufferFromEngineCount_; }
    void reset() override {}
    std::string getModeString() const override { return "RecordingAudioBuffer"; }

    int updateSimulationCount_ = 0;
    int fillBufferFromEngineCount_ = 0;
    int resetAfterWarmupCount_ = 0;
};

class RecordingPresentation : public presentation::IPresentation {
public:
    bool Initialize(const presentation::PresentationConfig&) override { return true; }
    void Shutdown() override {}
    void ShowMessage(const std::string&) override {}
    void ShowError(const std::string&) override {}
    void ShowProgress(double, double) override {}
    void Update(double) override {}
    void ShowSimulatorStates(const presentation::EngineState& state) override {
        emittedReplayTimestampsS_.push_back(state.drivetrain.replayTimestampS);
    }
    void setCsvEmissionEnabled(bool enabled) override {
        emissionToggleTrace_.push_back(enabled);
    }

    const std::vector<double>& emittedReplayTimestampsS() const {
        return emittedReplayTimestampsS_;
    }
    const std::vector<bool>& emissionToggleTrace() const { return emissionToggleTrace_; }

private:
    std::vector<double> emittedReplayTimestampsS_;
    std::vector<bool> emissionToggleTrace_;
};

class FakeTelemetryWriter : public ITelemetryWriter {
public:
    void writeEngineState(const EngineStateTelemetry&) override {}
    void writeFramePerformance(const FramePerformanceTelemetry&) override {}
    void writeAudioDiagnostics(const AudioDiagnosticsTelemetry&) override {}
    void writeAudioTiming(const AudioTimingTelemetry&) override {}
    void writeVehicleInputs(const VehicleInputsTelemetry&) override {}
    void writeSimulatorMetrics(const SimulatorMetricsTelemetry&) override {}
    void reset() override {}
    const char* getName() const override { return "FakeTelemetryWriter"; }
};

class FakeTelemetryReader : public ITelemetryReader {
public:
    AudioTimingTelemetry getAudioTiming() const override { return AudioTimingTelemetry{}; }
    EngineStateTelemetry getEngineState() const override { return EngineStateTelemetry{}; }
    FramePerformanceTelemetry getFramePerformance() const override { return FramePerformanceTelemetry{}; }
    AudioDiagnosticsTelemetry getAudioDiagnostics() const override { return AudioDiagnosticsTelemetry{}; }
    VehicleInputsTelemetry getVehicleInputs() const override { return VehicleInputsTelemetry{}; }
    SimulatorMetricsTelemetry getSimulatorMetrics() const override { return SimulatorMetricsTelemetry{}; }
    const char* getName() const override { return "FakeTelemetryReader"; }
};

class FakeLogger : public ILogging {
public:
    void setMask(uint32_t) override {}
    uint32_t getMask() const override { return 0; }
protected:
    void _write(uint32_t, const std::string&) override {}
};

class SimulationLoopWarmStartPrefixTest : public ::testing::Test {
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

        simulator_ = std::make_unique<BridgeSimulator>(std::move(sineSim), "WarmStartTestSim");
        ISimulatorConfig config;
        config.sampleRate = EngineSimDefaults::SAMPLE_RATE;
        config.simulationFrequency = EngineSimDefaults::SIMULATION_FREQUENCY;
        config.fluidSimulationSteps = EngineSimDefaults::FLUID_SIMULATION_STEPS;
        config.targetSynthesizerLatency = EngineSimDefaults::TARGET_SYNTH_LATENCY;
        ASSERT_TRUE(simulator_->create(config, nullptr, nullptr));

        audioBuffer_ = std::make_unique<RecordingAudioBuffer>();
        presentation_ = std::make_unique<RecordingPresentation>();
        telemetryWriter_ = std::make_unique<FakeTelemetryWriter>();
        telemetryReader_ = std::make_unique<FakeTelemetryReader>();
        logger_ = std::make_unique<FakeLogger>();
        crankingController_ = std::make_unique<CrankingController>();
        stopRequested_ = std::make_unique<std::atomic<bool>>(false);
        clock_ = std::make_unique<CountingLoopClock>();
    }

    // Runs the loop with the given provider and duration, returning run()'s code.
    int runLoop(IInputProvider* provider, ReplayTimelineInputProvider* timeline,
                double durationS) {
        SimulationConfig cfg;
        cfg.duration = durationS;
        cfg.simulatorLabel = "WarmStartPrefix";
        SessionDependencies deps{
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
        SimulationLoop loop(*simulator_, cfg, deps);
        (void)timeline;  // provider implements IReplayTimeline itself
        return loop.run();
    }

    std::unique_ptr<BridgeSimulator> simulator_;
    std::unique_ptr<RecordingAudioBuffer> audioBuffer_;
    std::unique_ptr<RecordingPresentation> presentation_;
    std::unique_ptr<FakeTelemetryWriter> telemetryWriter_;
    std::unique_ptr<FakeTelemetryReader> telemetryReader_;
    std::unique_ptr<FakeLogger> logger_;
    std::unique_ptr<CrankingController> crankingController_;
    std::unique_ptr<std::atomic<bool>> stopRequested_;
    std::unique_ptr<CountingLoopClock> clock_;
};

// Contract 1+2+3: with startFromS=0.12 the prefix runs 8 ticks (8 * 1/60 =
// 0.1333 >= 0.12), silent and unpaced; the first EMITTED frame is tick 9,
// stamped with the replay timecode of that tick (>= the requested offset),
// and playback audio starts only there.
TEST_F(SimulationLoopWarmStartPrefixTest, PrefixIsUnpacedSilentAndHandsOffAtOffset) {
    auto provider = std::make_unique<ReplayTimelineInputProvider>();
    provider->setStartFrom(0.12);

    const int result = runLoop(provider.get(), provider.get(), 0.2);

    EXPECT_EQ(result, 0);
    // Physics ticked every step, prefix included (warm gas path). Duration
    // 0.2 = 12 ticks in exact math, but accumulated 12 * (1/60) is
    // 0.19999... < 0.2, so a 13th step runs: 8 prefix + 5 emitted steps.
    EXPECT_EQ(audioBuffer_->updateSimulationCount_, 13);
    // No audio frames were queued during the 8 prefix ticks; audio starts at
    // the handoff tick and runs for the 5 emitted ticks.
    EXPECT_EQ(audioBuffer_->fillBufferFromEngineCount_, 5);
    // The ring is reset exactly once at handoff (no stale warm-up audio).
    EXPECT_EQ(audioBuffer_->resetAfterWarmupCount_, 1);
    // Exactly 5 emitted frames, none during the prefix.
    EXPECT_EQ(presentation_->emittedReplayTimestampsS().size(), 5u);
    // First emitted frame lands on the first tick at/after the offset: 8/60.
    ASSERT_FALSE(presentation_->emittedReplayTimestampsS().empty());
    EXPECT_NEAR(presentation_->emittedReplayTimestampsS().front(), 8.0 * kTick, 1e-9);
    EXPECT_GE(presentation_->emittedReplayTimestampsS().front(), 0.12);
    // Emission was switched off for the prefix and back on at handoff.
    ASSERT_EQ(presentation_->emissionToggleTrace().size(), 2u);
    EXPECT_FALSE(presentation_->emissionToggleTrace()[0]);
    EXPECT_TRUE(presentation_->emissionToggleTrace()[1]);
    // UNPACED: zero pacing waits during the 8 prefix polls. The main loop
    // waits once per iteration — 5 emitted ticks plus the terminal iteration
    // whose step() returns Stop — so exactly 6 waits, never 14 (paced).
    EXPECT_EQ(clock_->waitCount(), 6);
    // The provider was polled once per step (8 prefix + 5 emitted) and once
    // more in the terminal main-loop iteration before Stop is observed.
    EXPECT_EQ(provider->pollCount(), 14);
}

// Contract 4: no offset -> the prefix block is skipped entirely; emission,
// audio, and pacing run from tick 1 exactly as a from-0 run.
TEST_F(SimulationLoopWarmStartPrefixTest, NoOffsetSkipsPrefixAndEmitsFromZero) {
    auto provider = std::make_unique<ReplayTimelineInputProvider>();
    provider->setStartFrom(-1.0);

    const int result = runLoop(provider.get(), provider.get(), 0.2);

    EXPECT_EQ(result, 0);
    // 13 steps run (12 * 1/60 accumulates to 0.19999... < 0.2), all emitted.
    EXPECT_EQ(audioBuffer_->updateSimulationCount_, 13);
    EXPECT_EQ(audioBuffer_->fillBufferFromEngineCount_, 13);  // audio from tick 1
    EXPECT_EQ(audioBuffer_->resetAfterWarmupCount_, 0);       // no handoff reset
    ASSERT_EQ(presentation_->emittedReplayTimestampsS().size(), 13u);
    // Tick 1 emits the DEFAULT input stamp (-1): the loop polls the provider
    // after step(), so engineInput is unprimed on the very first tick — real
    // presentations fall back to their own clock for that row (the from-0
    // run's first row is the provider's from tick 2 on).
    EXPECT_EQ(presentation_->emittedReplayTimestampsS()[0], -1.0);
    EXPECT_NEAR(presentation_->emittedReplayTimestampsS()[1], kTick, 1e-9);
    EXPECT_TRUE(presentation_->emissionToggleTrace().empty());  // never toggled
    EXPECT_EQ(clock_->waitCount(), 14);  // paced every iteration incl. terminal
}

// Contract 5: a stop request arriving mid-prefix is honoured promptly — the
// prefix breaks, at most the break-point tick is emitted, and run() returns 0
// instead of grinding through the remaining prefix (CTRL+C window).
TEST_F(SimulationLoopWarmStartPrefixTest, StopRequestedMidPrefixExitsCleanly) {
    auto provider = std::make_unique<ReplayTimelineInputProvider>(
        stopRequested_.get(), /*stopAtPoll=*/3);
    provider->setStartFrom(0.5);  // 30 prefix ticks — far more than we run

    const int result = runLoop(provider.get(), provider.get(), 1000.0);

    EXPECT_EQ(result, 0);
    // Stop tripped at poll 3; the loop breaks, runs one handoff step, exits.
    EXPECT_EQ(provider->pollCount(), 4);
    EXPECT_EQ(audioBuffer_->updateSimulationCount_, 4);
    EXPECT_LE(audioBuffer_->fillBufferFromEngineCount_, 1);
    EXPECT_LE(presentation_->emittedReplayTimestampsS().size(), 1u);
}

}  // namespace
