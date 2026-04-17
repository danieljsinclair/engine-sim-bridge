// iOSAdapterTests.cpp - TDD RED-phase tests for iOS audio adapter
//
// Purpose: Test the AudioBufferDescriptor and AVAudioEngineHardwareProvider
// that enable the engine-sim-bridge to run on iOS.
//
// RED PHASE: These tests will NOT compile until:
//   - Phase 1: AudioBufferDescriptor exists (replaces CoreAudio AudioBufferList)
//   - Phase 2: AVAudioEngineHardwareProvider exists (implements IAudioHardwareProvider)
//
// Test count target: 15 (cut from 29 for value over volume)
//
// Layers tested:
//   Layer 1: AVAudioEngineHardwareProvider lifecycle and playback
//   Layer 3: Full pipeline integration with real engine (sine mode)
//   Layer 4: Callback stress tests (shutdown race, rapid start/stop)
//
// NOT tested here (will be separate files):
//   Layer 2: ObjC++ wrapper tests (needs Swift/ObjC++ interop)
// Phase G: Uses AudioBufferDescriptor + simplified AudioCallback signature

#include <gtest/gtest.h>
#include <memory>
#include <atomic>
#include <thread>
#include <chrono>
#include <vector>
#include <cmath>
#include <cstring>

// Phase 1 header
#include "hardware/AudioTypes.h"

// Phase 2 header -- does not exist yet (RED phase)
#include "hardware/AVAudioEngineHardwareProvider.h"

// Existing headers that DO exist
#include "hardware/IAudioHardwareProvider.h"
#include "simulator/ISimulator.h"
#include "simulator/BridgeSimulator.h"
#include "strategy/IAudioBuffer.h"
#include "strategy/SyncPullStrategy.h"
#include "strategy/ThreadedStrategy.h"
#include "simulator/engine_sim_bridge.h"
#include "common/ILogging.h"

// ============================================================================
// GROUP 1: AudioBufferDescriptor construction and data access (Phase 1)
//
// AudioBufferDescriptor replaces the CoreAudio AudioBufferList dependency.
// It provides a platform-agnostic buffer descriptor that IAudioBuffer::render()
// uses instead of AudioBufferList*.
// ============================================================================

class AudioBufferDescriptorTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(AudioBufferDescriptorTest, ConstructsWithValidParams) {
    // Arrange: Create a buffer descriptor for 256 stereo float frames
    const int frames = 256;
    const int channels = 2;
    std::vector<float> buffer(frames * channels, 0.0f);

    // Act
    AudioBufferDescriptor desc(buffer.data(), frames, channels);

    // Assert: Data pointer is valid
    EXPECT_NE(desc.buffer, nullptr);
    EXPECT_EQ(desc.frameCount, frames);
    EXPECT_EQ(desc.channelCount, channels);
}

TEST_F(AudioBufferDescriptorTest, DataPointerMatchesProvidedBuffer) {
    // Arrange
    const int frames = 128;
    const int channels = 2;
    std::vector<float> buffer(frames * channels, 0.0f);
    float* expectedPtr = buffer.data();

    // Act
    AudioBufferDescriptor desc(expectedPtr, frames, channels);

    // Assert: The descriptor references the same memory
    EXPECT_EQ(desc.buffer, expectedPtr);
}

TEST_F(AudioBufferDescriptorTest, SampleCountEqualsFramesTimesChannels) {
    // Arrange
    const int frames = 64;
    const int channels = 2;

    std::vector<float> buffer(frames * channels, 0.0f);
    AudioBufferDescriptor desc(buffer.data(), frames, channels);

    // Assert
    EXPECT_EQ(desc.frameCount * desc.channelCount, frames * channels);
}

// ============================================================================
// GROUP 2: AVAudioEngineHardwareProvider lifecycle (Phase 2)
//
// AVAudioEngineHardwareProvider implements IAudioHardwareProvider using
// iOS AVAudioEngine. Tests verify lifecycle, playback, and callback wiring.
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
    format.sampleRate = 48000;
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
    format.sampleRate = 48000;
    format.channels = 2;
    format.isFloat = true;
    format.isInterleaved = true;

    ASSERT_TRUE(provider_->initialize(format));

    // Register a silence-producing callback (required before start)
    auto callback = [](AudioBufferDescriptor& buffer) -> int {
        // Fill silence
        if (buffer.buffer) {
            std::memset(buffer.buffer, 0,
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
    format.sampleRate = 48000;
    format.channels = 2;
    format.isFloat = true;
    format.isInterleaved = true;
    ASSERT_TRUE(provider_->initialize(format));

    auto callback = [](AudioBufferDescriptor& buffer) -> int {
        if (buffer.buffer) {
            std::memset(buffer.buffer, 0, 512 * 2 * sizeof(float));
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
    format.sampleRate = 48000;
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
//
// These tests prove the AudioUnit/AVAudioEngine pipeline is correctly wired.
// They register a callback and verify it actually gets invoked by the
// hardware with real frame data.
// ============================================================================

TEST_F(AVAudioEngineProviderTest, CallbackReceivesFrames_WhenPlaybackStarted) {
    // Arrange: Create provider, register counter callback
    provider_ = std::make_unique<AVAudioEngineHardwareProvider>(logger_.get());

    AudioStreamFormat format;
    format.sampleRate = 48000;
    format.channels = 2;
    format.isFloat = true;
    format.isInterleaved = true;
    ASSERT_TRUE(provider_->initialize(format));

    std::atomic<int> callbackCount{0};
    std::atomic<int> lastFrameCount{0};

    auto callback = [&callbackCount, &lastFrameCount](AudioBufferDescriptor& buffer) -> int {
        callbackCount.fetch_add(1);
        lastFrameCount.store(buffer.frameCount);

        // Fill silence
        if (buffer.buffer) {
            std::memset(buffer.buffer, 0,
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
    format.sampleRate = 48000;
    format.channels = 2;
    format.isFloat = true;
    format.isInterleaved = true;
    ASSERT_TRUE(provider_->initialize(format));

    std::atomic<bool> bufferWasValid{false};
    std::atomic<bool> writeSucceeded{false};

    auto callback = [&bufferWasValid, &writeSucceeded](AudioBufferDescriptor& buffer) -> int {
        // Verify buffer is valid
        if (buffer.buffer) {
            bufferWasValid.store(true);
            // Try to write to the buffer
            std::memset(buffer.buffer, 0,
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
//
// These tests wire together real components: BridgeSimulator (sine mode),
// SyncPullStrategy or ThreadedStrategy, and AVAudioEngineHardwareProvider.
// They prove the entire Holy Trinity pipeline works on iOS.
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
        EngineSimConfig config{};
        config.sampleRate = 48000;
        config.sineMode = 1;
        config.simulationFrequency = 10000;

        auto sim = std::make_unique<BridgeSimulator>();
        if (!sim->create(config)) return nullptr;
        if (!sim->loadScript("", "")) return nullptr;
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
    config.sampleRate = 48000;
    config.channels = 2;
    ASSERT_TRUE(strategy->initialize(config));
    ASSERT_TRUE(strategy->startPlayback(simulator.get()));

    // Create real hardware provider
    hardware_ = std::make_unique<AVAudioEngineHardwareProvider>(logger_.get());
    AudioStreamFormat format;
    format.sampleRate = 48000;
    format.channels = 2;
    format.isFloat = true;
    format.isInterleaved = true;
    ASSERT_TRUE(hardware_->initialize(format));

    // Register callback that uses the strategy to render
    std::atomic<bool> gotNonSilentAudio{false};
    BridgeSimulator* simPtr = simulator.get();

    auto callback = [&gotNonSilentAudio, simPtr](AudioBufferDescriptor& buffer) -> int {
        if (!buffer.buffer) return 0;

        // Advance simulation and render
        int32_t written = 0;
        simPtr->renderOnDemand(buffer.buffer, buffer.frameCount, &written);

        if (written > 0) {
            // Check if non-silent
            for (int i = 0; i < written * 2; ++i) {
                if (std::abs(buffer.buffer[i]) > 1e-6f) {
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
    config.sampleRate = 48000;
    config.channels = 2;
    ASSERT_TRUE(strategy->initialize(config));
    ASSERT_TRUE(strategy->startPlayback(simulator.get()));

    hardware_ = std::make_unique<AVAudioEngineHardwareProvider>(logger_.get());
    AudioStreamFormat format;
    format.sampleRate = 48000;
    format.channels = 2;
    format.isFloat = true;
    format.isInterleaved = true;
    ASSERT_TRUE(hardware_->initialize(format));

    auto callback = [](AudioBufferDescriptor& buffer) -> int {
        if (buffer.buffer) {
            std::memset(buffer.buffer, 0, 512 * 2 * sizeof(float));
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
        config.sampleRate = 48000;
        config.channels = 2;
        ASSERT_TRUE(strategy->initialize(config));

        auto hw = std::make_unique<AVAudioEngineHardwareProvider>(logger_.get());
        AudioStreamFormat format;
        format.sampleRate = 48000;
        format.channels = 2;
        format.isFloat = true;
        format.isInterleaved = true;
        ASSERT_TRUE(hw->initialize(format));

        std::atomic<bool> callbackFired{false};
        auto cb = [&callbackFired](AudioBufferDescriptor& buffer) -> int {
            (void)buffer;
            callbackFired.store(true);
            return 0;
        };
        ASSERT_TRUE(hw->registerAudioCallback(cb));

        if (std::string(mode) == "SyncPull") {
            ASSERT_TRUE(strategy->startPlayback(simulator.get()));
        } else {
            ASSERT_TRUE(strategy->startPlayback(simulator.get()));
        }

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
//
// These tests catch race conditions and resource leaks.
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
    format.sampleRate = 48000;
    format.channels = 2;
    format.isFloat = true;
    format.isInterleaved = true;
    ASSERT_TRUE(provider->initialize(format));

    std::atomic<int> callbackCount{0};
    IAudioHardwareProvider* providerPtr = provider.get();

    auto callback = [&callbackCount, providerPtr](AudioBufferDescriptor& buffer) -> int {
        // Fill silence
        if (buffer.buffer) {
            std::memset(buffer.buffer, 0,
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
    format.sampleRate = 48000;
    format.channels = 2;
    format.isFloat = true;
    format.isInterleaved = true;
    ASSERT_TRUE(provider->initialize(format));

    auto callback = [](AudioBufferDescriptor& buffer) -> int {
        if (buffer.buffer) {
            std::memset(buffer.buffer, 0,
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
// GROUP 6: ObjC++ wrapper lifecycle (Layer 2 subset)
//
// These tests verify the wrapper creates and destroys components correctly.
// Full Layer 2 tests will be in a separate file when the wrapper is implemented.
// ============================================================================

TEST_F(AVAudioEngineProviderTest, DestroyWhilePlaying_DoesNotCrash) {
    // Arrange: Create provider, init, start
    auto provider = std::make_unique<AVAudioEngineHardwareProvider>(logger_.get());

    AudioStreamFormat format;
    format.sampleRate = 48000;
    format.channels = 2;
    format.isFloat = true;
    format.isInterleaved = true;
    ASSERT_TRUE(provider->initialize(format));

    auto callback = [](AudioBufferDescriptor& buffer) -> int {
        if (buffer.buffer) {
            std::memset(buffer.buffer, 0,
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
