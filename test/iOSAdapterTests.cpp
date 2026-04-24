// iOSAdapterTests.cpp - TDD tests for iOS audio adapter
//
// Purpose: Test the AudioBufferView and AVAudioEngineHardwareProvider
// that enable the engine-sim-bridge to run on iOS.
//
// Architecture:
//   Group 1: AudioBufferView (pure C++, runs everywhere)
//   Groups 2-6: AVAudioEngineHardwareProvider (iOS only, behind TARGET_OS_IPHONE)
//
// Build: cmake -DBUILD_IOS_ADAPTER_TESTS=ON
//   macOS: Only Group 1 tests compile/run (3 tests)
//   iOS: All 15 tests compile/run
//
// Phase G: Uses AudioBufferView + simplified AudioCallback signature

#include <gtest/gtest.h>
#include <memory>
#include <atomic>
#include <thread>
#include <chrono>
#include <vector>
#include <cmath>
#include <cstring>

// Phase 1 header - platform-agnostic buffer type
#include "hardware/AudioTypes.h"

// Phase 2 header - iOS hardware provider (declaration always visible,
// but implementation only exists when compiled for iOS)
#include "hardware/AVAudioEngineHardwareProvider.h"

// Shared headers
#include "hardware/IAudioHardwareProvider.h"
#include "common/ILogging.h"

#if TARGET_OS_IPHONE
#include "simulator/ISimulator.h"
#include "simulator/BridgeSimulator.h"
#include "simulator/SineSimulator.h"
#include "strategy/IAudioBuffer.h"
#include "strategy/SyncPullStrategy.h"
#include "strategy/ThreadedStrategy.h"
#include "simulator/EngineSimTypes.h"
#endif

// ============================================================================
// GROUP 1: AudioBufferView construction and data access
//
// Pure C++ tests -- no platform dependencies.
// These run on macOS, iOS, and any other platform.
// ============================================================================

class AudioBufferViewTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(AudioBufferViewTest, ConstructsWithValidParams) {
    // Arrange: Create a buffer descriptor for 256 stereo float frames
    const int frames = 256;
    const int channels = 2;
    std::vector<float> buffer(frames * channels, 0.0f);

    // Act
    AudioBufferView desc(buffer.data(), frames, channels);

    // Assert
    EXPECT_NE(desc.asFloat(), nullptr);
    EXPECT_EQ(desc.frameCount, frames);
    EXPECT_EQ(desc.channelCount, channels);
}

TEST_F(AudioBufferViewTest, DataPointerMatchesProvidedBuffer) {
    // Arrange
    const int frames = 128;
    const int channels = 2;
    std::vector<float> buffer(frames * channels, 0.0f);
    float* expectedPtr = buffer.data();

    // Act
    AudioBufferView desc(expectedPtr, frames, channels);

    // Assert: The descriptor references the same memory
    EXPECT_EQ(desc.asFloat(), expectedPtr);
}

TEST_F(AudioBufferViewTest, SampleCountEqualsFramesTimesChannels) {
    // Arrange
    const int frames = 64;
    const int channels = 2;

    std::vector<float> buffer(frames * channels, 0.0f);
    AudioBufferView desc(buffer.data(), frames, channels);

    // Assert: Total sample count is frames * channels
    EXPECT_EQ(desc.frameCount * desc.channelCount, frames * channels);
}

// ============================================================================
// GROUPS 2-6: AVAudioEngineHardwareProvider tests (iOS only)
//
// These tests require AVAudioEngineHardwareProvider implementation which
// is only compiled for iOS targets (TARGET_OS_IPHONE). On macOS they
// are compiled away to avoid linker errors.
// ============================================================================

#if TARGET_OS_IPHONE

// ============================================================================
// GROUP 2: AVAudioEngineHardwareProvider lifecycle
// ============================================================================

class AVAudioEngineProviderTest : public ::testing::Test {
protected:
    void SetUp() override {
        logger_ = std::make_unique<ConsoleLogger>();
    }

    void TearDown() override {
        if (provider_) {
            provider_->stopPlayback();
            provider_->cleanup();
        }
    }

    std::unique_ptr<ConsoleLogger> logger_;
    std::unique_ptr<IAudioHardwareProvider> provider_;
};

TEST_F(AVAudioEngineProviderTest, Initialize_SucceedsWithValidFormat) {
    // Arrange: Create provider with valid 48kHz stereo float format
    provider_ = std::make_unique<AVAudioEngineHardwareProvider>(logger_.get());

    AudioStreamFormat format;
    format.sampleRate = EngineSimDefaults::SAMPLE_RATE;
    format.channels = 2;
    format.bitsPerSample = 32;
    format.isFloat = true;
    format.isInterleaved = true;

    // Act
    bool result = provider_->initialize(format);

    // Assert: Initialization should succeed on iOS / simulator
    EXPECT_TRUE(result);

    // Assert: Hardware state reflects initialization
    auto state = provider_->getHardwareState();
    EXPECT_TRUE(state.isInitialized);
}

TEST_F(AVAudioEngineProviderTest, StartPlayback_AfterInit_Succeeds) {
    // Arrange: Initialize provider
    provider_ = std::make_unique<AVAudioEngineHardwareProvider>(logger_.get());

    AudioStreamFormat format;
    format.sampleRate = EngineSimDefaults::SAMPLE_RATE;
    format.channels = 2;
    format.isFloat = true;
    format.isInterleaved = true;

    ASSERT_TRUE(provider_->initialize(format));

    // Register a silence-producing callback (required before start)
    auto callback = [](AudioBufferView& buffer) -> int {
        if (buffer.asFloat()) {
            std::memset(buffer.asFloat(), 0,
                        static_cast<size_t>(buffer.frameCount) * buffer.channelCount * sizeof(float));
        }
        return 0;
    };
    ASSERT_TRUE(provider_->registerAudioCallback(callback));

    // Act: Start playback
    bool result = provider_->startPlayback();

    // Assert
    EXPECT_TRUE(result);
    auto state = provider_->getHardwareState();
    EXPECT_TRUE(state.isPlaying);
}

TEST_F(AVAudioEngineProviderTest, StopPlayback_StopsCallbackThread) {
    // Arrange: Init, register callback, start
    provider_ = std::make_unique<AVAudioEngineHardwareProvider>(logger_.get());

    AudioStreamFormat format;
    format.sampleRate = EngineSimDefaults::SAMPLE_RATE;
    format.channels = 2;
    format.isFloat = true;
    format.isInterleaved = true;
    ASSERT_TRUE(provider_->initialize(format));

    auto callback = [](AudioBufferView& buffer) -> int {
        if (buffer.asFloat()) {
            std::memset(buffer.asFloat(), 0, 512 * 2 * sizeof(float));
        }
        return 0;
    };
    ASSERT_TRUE(provider_->registerAudioCallback(callback));
    ASSERT_TRUE(provider_->startPlayback());

    // Act: Stop playback
    provider_->stopPlayback();

    // Assert: No longer playing
    auto state = provider_->getHardwareState();
    EXPECT_FALSE(state.isPlaying);
    EXPECT_FALSE(state.isCallbackActive);
}

TEST_F(AVAudioEngineProviderTest, SetVolume_ClampsToRange) {
    // Arrange
    provider_ = std::make_unique<AVAudioEngineHardwareProvider>(logger_.get());

    AudioStreamFormat format;
    format.sampleRate = EngineSimDefaults::SAMPLE_RATE;
    format.channels = 2;
    format.isFloat = true;
    format.isInterleaved = true;
    ASSERT_TRUE(provider_->initialize(format));

    // Act: Set volume above 1.0
    provider_->setVolume(2.0);
    EXPECT_DOUBLE_EQ(provider_->getVolume(), 1.0);

    // Act: Set volume below 0.0
    provider_->setVolume(-0.5);
    EXPECT_DOUBLE_EQ(provider_->getVolume(), 0.0);
}

// ============================================================================
// GROUP 3: Callback wiring -- the critical tests
// ============================================================================

TEST_F(AVAudioEngineProviderTest, CallbackReceivesFrames_WhenPlaybackStarted) {
    // Arrange: Create provider, register counter callback
    provider_ = std::make_unique<AVAudioEngineHardwareProvider>(logger_.get());

    AudioStreamFormat format;
    format.sampleRate = EngineSimDefaults::SAMPLE_RATE;
    format.channels = 2;
    format.isFloat = true;
    format.isInterleaved = true;
    ASSERT_TRUE(provider_->initialize(format));

    std::atomic<int> callbackCount{0};
    std::atomic<int> lastFrameCount{0};

    auto callback = [&callbackCount, &lastFrameCount](AudioBufferView& buffer) -> int {
        callbackCount.fetch_add(1);
        lastFrameCount.store(buffer.frameCount);

        // Fill silence
        if (buffer.asFloat()) {
            std::memset(buffer.asFloat(), 0,
                        static_cast<size_t>(buffer.frameCount) * buffer.channelCount * sizeof(float));
        }
        return 0;
    };
    ASSERT_TRUE(provider_->registerAudioCallback(callback));

    // Act: Start playback
    ASSERT_TRUE(provider_->startPlayback());

    // Wait up to 200ms for the callback to fire
    bool callbackFired = false;
    for (int i = 0; i < 40; ++i) {
        if (callbackCount.load() > 0) {
            callbackFired = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // Assert: Callback must have fired
    ASSERT_TRUE(callbackFired) << "Audio callback was never invoked";
    EXPECT_GT(lastFrameCount.load(), 0) << "Callback received 0 frames";
}

TEST_F(AVAudioEngineProviderTest, CallbackBufferIsWritable) {
    // Arrange: Create provider, register callback that writes to buffer
    provider_ = std::make_unique<AVAudioEngineHardwareProvider>(logger_.get());

    AudioStreamFormat format;
    format.sampleRate = EngineSimDefaults::SAMPLE_RATE;
    format.channels = 2;
    format.isFloat = true;
    format.isInterleaved = true;
    ASSERT_TRUE(provider_->initialize(format));

    std::atomic<bool> bufferWasValid{false};
    std::atomic<bool> writeSucceeded{false};

    auto callback = [&bufferWasValid, &writeSucceeded](AudioBufferView& buffer) -> int {
        if (buffer.asFloat()) {
            bufferWasValid.store(true);
            // Try to write to the buffer
            std::memset(buffer.asFloat(), 0,
                        static_cast<size_t>(buffer.frameCount) * buffer.channelCount * sizeof(float));
            writeSucceeded.store(true);
        }
        return 0;
    };
    ASSERT_TRUE(provider_->registerAudioCallback(callback));
    ASSERT_TRUE(provider_->startPlayback());

    // Wait for callback
    for (int i = 0; i < 40; ++i) {
        if (bufferWasValid.load()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // Assert: Buffer was valid and writable
    EXPECT_TRUE(bufferWasValid.load()) << "Callback received null or invalid buffer";
    EXPECT_TRUE(writeSucceeded.load()) << "Could not write to audio buffer";
}

// ============================================================================
// GROUP 4: Full pipeline integration with real engine (sine mode)
// ============================================================================

class iOSPipelineIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        logger_ = std::make_unique<ConsoleLogger>();
    }

    void TearDown() override {
        if (hardware_) {
            hardware_->stopPlayback();
            hardware_->cleanup();
        }
    }

    // Helper: Create a BridgeSimulator in sine mode for testing
    std::unique_ptr<BridgeSimulator> createSineSimulator() {
        ISimulatorConfig config;

        auto sineSim = std::make_unique<SineSimulator>();
        Simulator::Parameters simParams;
        simParams.systemType = Simulator::SystemType::NsvOptimized;
        sineSim->initialize(simParams);
        sineSim->setSimulationFrequency(config.simulationFrequency);
        sineSim->setFluidSimulationSteps(config.fluidSimulationSteps);
        sineSim->setTargetSynthesizerLatency(config.targetSynthesizerLatency);
        sineSim->init();
        auto sim = std::make_unique<BridgeSimulator>(std::move(sineSim));
        if (!sim->create(config)) return nullptr;
        return sim;
    }

    std::unique_ptr<ConsoleLogger> logger_;
    std::unique_ptr<IAudioHardwareProvider> hardware_;
};

TEST_F(iOSPipelineIntegrationTest, SineModeProducesAudio) {
    // Arrange: Create real pipeline components
    auto simulator = createSineSimulator();
    ASSERT_NE(simulator, nullptr) << "Failed to create sine-mode simulator";

    auto strategy = std::make_unique<SyncPullStrategy>(logger_.get());
    AudioStrategyConfig config;
    config.sampleRate = ISimulatorConfig().sampleRate;
    config.channels = 2;
    config.synthLatency = ISimulatorConfig().targetSynthesizerLatency;
    ASSERT_TRUE(strategy->initialize(config));
    ASSERT_TRUE(strategy->startPlayback(simulator.get()));

    // Create real hardware provider
    hardware_ = std::make_unique<AVAudioEngineHardwareProvider>(logger_.get());
    AudioStreamFormat format;
    format.sampleRate = ISimulatorConfig().sampleRate;
    format.channels = 2;
    format.isFloat = true;
    format.isInterleaved = true;
    ASSERT_TRUE(hardware_->initialize(format));

    // Register callback that uses the strategy to render
    std::atomic<bool> gotNonSilentAudio{false};
    BridgeSimulator* simPtr = simulator.get();

    auto callback = [&gotNonSilentAudio, simPtr](AudioBufferView& buffer) -> int {
        if (!buffer.asFloat()) return 0;

        // Advance simulation and render
        int32_t written = 0;
        simPtr->renderOnDemand(buffer.asFloat(), buffer.frameCount, &written);

        if (written > 0) {
            // Check if non-silent
            for (int i = 0; i < written * 2; ++i) {
                if (std::abs(buffer.asFloat()[i]) > 1e-6f) {
                    gotNonSilentAudio.store(true);
                    break;
                }
            }
        }

        return 0;
    };
    ASSERT_TRUE(hardware_->registerAudioCallback(callback));
    ASSERT_TRUE(hardware_->startPlayback());

    // Act: Run simulation for 200ms
    for (int i = 0; i < 12; ++i) {  // 12 * 16.667ms ~ 200ms
        simulator->update(0.016667);
        std::this_thread::sleep_for(std::chrono::milliseconds(17));
    }

    // Assert: Pipeline produced non-silent audio
    EXPECT_TRUE(gotNonSilentAudio.load())
        << "Pipeline did not produce non-silent audio in 200ms";
}

TEST_F(iOSPipelineIntegrationTest, CleanShutdownAfterPlayback) {
    // Arrange
    auto simulator = createSineSimulator();
    ASSERT_NE(simulator, nullptr);

    auto strategy = std::make_unique<SyncPullStrategy>(logger_.get());
    AudioStrategyConfig config;
    config.sampleRate = EngineSimDefaults::SAMPLE_RATE;
    config.channels = 2;
    config.synthLatency = 0.0;  // Use default
    ASSERT_TRUE(strategy->initialize(config));
    ASSERT_TRUE(strategy->startPlayback(simulator.get()));

    hardware_ = std::make_unique<AVAudioEngineHardwareProvider>(logger_.get());
    AudioStreamFormat format;
    format.sampleRate = EngineSimDefaults::SAMPLE_RATE;
    format.channels = 2;
    format.isFloat = true;
    format.isInterleaved = true;
    ASSERT_TRUE(hardware_->initialize(format));

    auto callback = [](AudioBufferView& buffer) -> int {
        if (buffer.asFloat()) {
            std::memset(buffer.asFloat(), 0, 512 * 2 * sizeof(float));
        }
        return 0;
    };
    ASSERT_TRUE(hardware_->registerAudioCallback(callback));
    ASSERT_TRUE(hardware_->startPlayback());

    // Act: Run for 500ms then stop
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    strategy->stopPlayback(simulator.get());
    hardware_->stopPlayback();
    hardware_->cleanup();
    hardware_.reset();  // Prevent double-cleanup in TearDown

    // Assert: No crash, no deadlock -- reaching here is the assertion
    SUCCEED();
}

TEST_F(iOSPipelineIntegrationTest, SyncPullAndThreadedBothWork) {
    // Arrange/Act: Run once with SyncPull, once with Threaded
    for (const char* mode : {"SyncPull", "Threaded"}) {
        auto simulator = createSineSimulator();
        ASSERT_NE(simulator, nullptr) << "Failed to create simulator for " << mode;

        std::unique_ptr<IAudioBuffer> strategy;
        if (std::string(mode) == "SyncPull") {
            strategy = std::make_unique<SyncPullStrategy>(logger_.get());
        } else {
            strategy = std::make_unique<ThreadedStrategy>(logger_.get());
        }

        AudioStrategyConfig config;
        config.sampleRate = EngineSimDefaults::SAMPLE_RATE;
        config.channels = 2;
        config.synthLatency = 0.0;  // Use default
        ASSERT_TRUE(strategy->initialize(config));

        auto hw = std::make_unique<AVAudioEngineHardwareProvider>(logger_.get());
        AudioStreamFormat format;
        format.sampleRate = EngineSimDefaults::SAMPLE_RATE;
        format.channels = 2;
        format.isFloat = true;
        format.isInterleaved = true;
        ASSERT_TRUE(hw->initialize(format));

        std::atomic<bool> callbackFired{false};
        auto cb = [&callbackFired](AudioBufferView& buffer) -> int {
            (void)buffer;
            callbackFired.store(true);
            return 0;
        };
        ASSERT_TRUE(hw->registerAudioCallback(cb));

        ASSERT_TRUE(strategy->startPlayback(simulator.get()));
        ASSERT_TRUE(hw->startPlayback());

        // Wait for callback
        for (int i = 0; i < 40; ++i) {
            if (callbackFired.load()) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        // Assert: Callback fired for both modes
        EXPECT_TRUE(callbackFired.load())
            << mode << " strategy: hardware callback never fired";

        hw->stopPlayback();
        hw->cleanup();
        strategy->stopPlayback(simulator.get());
        simulator->destroy();
    }
}

// ============================================================================
// GROUP 5: Callback stress tests
// ============================================================================

class iOSCallbackStressTest : public ::testing::Test {
protected:
    void SetUp() override {
        logger_ = std::make_unique<ConsoleLogger>();
    }

    void TearDown() override {}

    std::unique_ptr<ConsoleLogger> logger_;
};

TEST_F(iOSCallbackStressTest, CallbackDoesNotDeadlock_WhenStopPlaybackCalledFromCallback) {
    // This tests the shutdown race condition (same as CLI task #64).
    // The callback calls stopPlayback() from the audio thread.
    // This MUST not deadlock.

    auto provider = std::make_unique<AVAudioEngineHardwareProvider>(logger_.get());

    AudioStreamFormat format;
    format.sampleRate = EngineSimDefaults::SAMPLE_RATE;
    format.channels = 2;
    format.isFloat = true;
    format.isInterleaved = true;
    ASSERT_TRUE(provider->initialize(format));

    std::atomic<int> callbackCount{0};
    IAudioHardwareProvider* providerPtr = provider.get();

    auto callback = [&callbackCount, providerPtr](AudioBufferView& buffer) -> int {
        // Fill silence
        if (buffer.asFloat()) {
            std::memset(buffer.asFloat(), 0,
                        static_cast<size_t>(buffer.frameCount) * buffer.channelCount * sizeof(float));
        }

        int count = callbackCount.fetch_add(1);

        // On 10th callback, call stopPlayback from the audio thread
        if (count == 9) {
            providerPtr->stopPlayback();
        }
        return 0;
    };

    ASSERT_TRUE(provider->registerAudioCallback(callback));
    ASSERT_TRUE(provider->startPlayback());

    // Act: Wait up to 2 seconds for the deadlock test to resolve
    bool stopped = false;
    for (int i = 0; i < 200; ++i) {
        auto state = provider->getHardwareState();
        if (!state.isPlaying) {
            stopped = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Assert: Must have stopped without deadlock
    ASSERT_TRUE(stopped) << "stopPlayback() called from callback caused deadlock";

    // Cleanup
    provider->cleanup();
}

TEST_F(iOSCallbackStressTest, RapidStartStopCycle_DoesNotLeakOrCrash) {
    // Arrange: Create and initialize provider
    auto provider = std::make_unique<AVAudioEngineHardwareProvider>(logger_.get());

    AudioStreamFormat format;
    format.sampleRate = EngineSimDefaults::SAMPLE_RATE;
    format.channels = 2;
    format.isFloat = true;
    format.isInterleaved = true;
    ASSERT_TRUE(provider->initialize(format));

    auto callback = [](AudioBufferView& buffer) -> int {
        if (buffer.asFloat()) {
            std::memset(buffer.asFloat(), 0,
                        static_cast<size_t>(buffer.frameCount) * buffer.channelCount * sizeof(float));
        }
        return 0;
    };
    ASSERT_TRUE(provider->registerAudioCallback(callback));

    // Act: Rapid start/stop cycle 50 times
    for (int i = 0; i < 50; ++i) {
        ASSERT_TRUE(provider->startPlayback())
            << "startPlayback failed on iteration " << i;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        provider->stopPlayback();
    }

    // Assert: Hardware state is clean
    auto state = provider->getHardwareState();
    EXPECT_FALSE(state.isPlaying);
    EXPECT_FALSE(state.isCallbackActive);

    // Cleanup
    provider->cleanup();
}

// ============================================================================
// GROUP 6: ObjC++ wrapper lifecycle (destructor safety)
// ============================================================================

TEST_F(AVAudioEngineProviderTest, DestroyWhilePlaying_DoesNotCrash) {
    // Arrange: Create provider, init, start
    auto provider = std::make_unique<AVAudioEngineHardwareProvider>(logger_.get());

    AudioStreamFormat format;
    format.sampleRate = EngineSimDefaults::SAMPLE_RATE;
    format.channels = 2;
    format.isFloat = true;
    format.isInterleaved = true;
    ASSERT_TRUE(provider->initialize(format));

    auto callback = [](AudioBufferView& buffer) -> int {
        if (buffer.asFloat()) {
            std::memset(buffer.asFloat(), 0,
                        static_cast<size_t>(buffer.frameCount) * buffer.channelCount * sizeof(float));
        }
        return 0;
    };
    ASSERT_TRUE(provider->registerAudioCallback(callback));
    ASSERT_TRUE(provider->startPlayback());

    // Act: Destroy while playing (simulates iOS app kill)
    // Destructor should call stopPlayback() + cleanup()
    provider.reset();

    // Assert: Reaching here without crash or deadlock is success
    SUCCEED();
}

#endif // TARGET_OS_IPHONE
