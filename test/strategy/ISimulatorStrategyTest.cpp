// ISimulatorStrategyTest.cpp - Extracted from engine-sim-cli/test/unit/ISimulatorTest.cpp
//
// Purpose: Verify that strategy methods (ThreadedStrategy/SyncPullStrategy)
// drive ISimulator correctly. Moved into the bridge so coverage
// instrumentation exercises these paths (ISimulator.cpp was 0% — tested
// only in the cli). The cli retains the original file (duplication accepted,
// like the shared helpers).
//
// GROUP 3: IAudioBuffer uses ISimulator* instead of EngineSimHandle/API
//
// TARGET: Strategy methods take ISimulator* instead of raw engine types.
// This decouples the audio pipeline from the bridge implementation.

#include <gtest/gtest.h>
#include <memory>

#include "simulator/ISimulator.h"
#include "MockSimulator.h"
#include "strategy/IAudioBuffer.h"
#include "strategy/ThreadedStrategy.h"
#include "strategy/SyncPullStrategy.h"
#include "AudioTestConstants.h"
#include "AudioTestHelpers.h"
#include "common/ILogging.h"

using namespace test::constants;

class ISimulatorStrategyTest : public ::testing::Test {
protected:
    void SetUp() override {
        logger_ = std::make_unique<ConsoleLogger>();
        mockSim_ = std::make_unique<MockSimulator>();
    }

    void TearDown() override {}

    std::unique_ptr<ConsoleLogger> logger_;
    std::unique_ptr<MockSimulator> mockSim_;
};

TEST_F(ISimulatorStrategyTest, ThreadedStrategy_FillBufferFromEngine_UsesISimulator) {
    // Arrange: Create strategy
    auto strategy = std::make_unique<ThreadedStrategy>(logger_.get());
    AudioBufferConfig config;
    config.channels = STEREO_CHANNELS;
    ASSERT_TRUE(strategy->initialize(config, DEFAULT_SAMPLE_RATE));

    // Act: fillBufferFromEngine takes ISimulator* instead of EngineSimHandle/EngineSimAPI&
    strategy->fillBufferFromEngine(mockSim_.get(), 512);

    // Assert: No crash -- strategy uses ISimulator interface
    SUCCEED();
}

TEST_F(ISimulatorStrategyTest, ThreadedStrategy_UpdateSimulation_UsesISimulator) {
    // Arrange
    auto strategy = std::make_unique<ThreadedStrategy>(logger_.get());
    AudioBufferConfig config;
// Removed sampleRate - now passed as separate parameter
    config.channels = STEREO_CHANNELS;
    ASSERT_TRUE(strategy->initialize(config, DEFAULT_SAMPLE_RATE));

    // Act: updateSimulation takes ISimulator* instead of EngineSimHandle/EngineSimAPI&
    strategy->updateSimulation(mockSim_.get(), 16.667);

    // Assert: No crash
    SUCCEED();
}

TEST_F(ISimulatorStrategyTest, ThreadedStrategy_StartPlayback_UsesISimulator) {
    // Arrange
    auto strategy = std::make_unique<ThreadedStrategy>(logger_.get());
    AudioBufferConfig config;
// Removed sampleRate - now passed as separate parameter
    config.channels = STEREO_CHANNELS;
    ASSERT_TRUE(strategy->initialize(config, DEFAULT_SAMPLE_RATE));

    // Act: startPlayback takes ISimulator* instead of EngineSimHandle/EngineSimAPI*
    bool result = strategy->startPlayback(mockSim_.get());

    // Assert: Should succeed with MockSimulator
    EXPECT_TRUE(result);
    EXPECT_TRUE(strategy->isPlaying());

    // Cleanup
    strategy->stopPlayback(mockSim_.get());
}

TEST_F(ISimulatorStrategyTest, SyncPullStrategy_StartPlayback_UsesISimulator) {
    // Arrange
    auto strategy = std::make_unique<SyncPullStrategy>(logger_.get());
    AudioBufferConfig config;
// Removed sampleRate - now passed as separate parameter
    config.channels = STEREO_CHANNELS;
    ASSERT_TRUE(strategy->initialize(config, DEFAULT_SAMPLE_RATE));

    // Act: startPlayback takes ISimulator*
    bool result = strategy->startPlayback(mockSim_.get());

    // Assert
    EXPECT_TRUE(result);
    EXPECT_TRUE(strategy->isPlaying());

    strategy->stopPlayback(mockSim_.get());
}

TEST_F(ISimulatorStrategyTest, ThreadedStrategy_FullPipeline_WithMockSimulator) {
    // Arrange: Create strategy with ISimulator
    auto strategy = std::make_unique<ThreadedStrategy>(logger_.get());
    AudioBufferConfig config;
// Removed sampleRate - now passed as separate parameter
    config.channels = STEREO_CHANNELS;
    ASSERT_TRUE(strategy->initialize(config, DEFAULT_SAMPLE_RATE));

    // Act: Fill buffer from ISimulator
    strategy->fillBufferFromEngine(mockSim_.get(), 512);

    // Act: Render what we got
    AudioBufferView audioBuffer = createAudioBuffer(TEST_FRAME_COUNT);
    bool renderResult = strategy->render(audioBuffer);

    // Assert: Should succeed (may be silence from mock, but no crash)
    EXPECT_TRUE(renderResult);

    freeAudioBuffer(audioBuffer);
}

TEST_F(ISimulatorStrategyTest, SyncPullStrategy_Render_UsesISimulator) {
    // Arrange: SyncPullStrategy with ISimulator
    auto strategy = std::make_unique<SyncPullStrategy>(logger_.get());
    AudioBufferConfig config;
// Removed sampleRate - now passed as separate parameter
    config.channels = STEREO_CHANNELS;
    ASSERT_TRUE(strategy->initialize(config, DEFAULT_SAMPLE_RATE));

    // Act: Start playback with ISimulator
    ASSERT_TRUE(strategy->startPlayback(mockSim_.get()));

    // Act: Render audio (SyncPull renders on-demand from ISimulator)
    AudioBufferView audioBuffer = createAudioBuffer(TEST_FRAME_COUNT);
    bool renderResult = strategy->render(audioBuffer);

    // Assert: Should succeed -- SyncPull calls renderOnDemand() on ISimulator
    EXPECT_TRUE(renderResult);

    freeAudioBuffer(audioBuffer);
    strategy->stopPlayback(mockSim_.get());
}
