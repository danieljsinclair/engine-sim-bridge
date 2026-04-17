// AVAudioEngineHardwareProvider.mm - iOS AVAudioEngine implementation of IAudioHardwareProvider
// Wraps AVAudioEngine + AVAudioSourceNode for iOS platform audio output
// Thread-safe callback handling with proper resource management
// Phase G: Created for iOS platform support

#include "hardware/AVAudioEngineHardwareProvider.h"

#if TARGET_OS_IPHONE

#import <AVFoundation/AVFoundation.h>
#import <AudioToolbox/AudioToolbox.h>
#import <memory>

// ================================================================
// AVAudioEngineHardwareProvider Implementation
// ================================================================

AVAudioEngineHardwareProvider::AVAudioEngineHardwareProvider(ILogging* logger)
    : audioEngine_(nullptr),
      sourceNode_(nullptr),
      audioSession_(nullptr),
      isPlaying_(false),
      currentVolume_(1.0),
      underrunCount_(0),
      overrunCount_(0),
      defaultLogger_(logger ? nullptr : new ConsoleLogger()),
      logger_(logger ? logger : defaultLogger_.get()),
      audioCallback_(nullptr) {
}

AVAudioEngineHardwareProvider::~AVAudioEngineHardwareProvider() {
    cleanup();
}

bool AVAudioEngineHardwareProvider::initialize(const AudioStreamFormat& format) {
    logger_->info(LogMask::AUDIO, "AVAudioEngineHardwareProvider::initialize() - sr=%d, ch=%d",
                 format.sampleRate, format.channels);

    format_ = format;

    // Setup AVAudioSession
    if (!setupAudioSession()) {
        logger_->error(LogMask::AUDIO, "Failed to setup AVAudioSession");
        return false;
    }

    // Setup AVAudioEngine
    if (!setupAudioEngine()) {
        logger_->error(LogMask::AUDIO, "Failed to setup AVAudioEngine");
        return false;
    }

    // Configure audio format
    if (!configureAudioFormat(format)) {
        logger_->error(LogMask::AUDIO, "Failed to configure audio format");
        return false;
    }

    // Create source node
    if (!createSourceNode()) {
        logger_->error(LogMask::AUDIO, "Failed to create source node");
        return false;
    }

    logger_->info(LogMask::AUDIO, "AVAudioEngineHardwareProvider initialized successfully");
    return true;
}

void AVAudioEngineHardwareProvider::cleanup() {
    if (audioEngine_) {
        stopPlayback();

        // Stop AVAudioEngine
        [audioEngine_ stop];

        // Detach and release source node
        if (sourceNode_) {
            [audioEngine_ detachNode:sourceNode_];
            sourceNode_ = nullptr;
        }

        audioEngine_ = nullptr;
    }

    if (audioSession_) {
        // Deactivate audio session
        NSError* error = nil;
        [audioSession_ setActive:NO error:&error];
        if (error) {
            logger_->warning(LogMask::AUDIO, "Failed to deactivate AVAudioSession: %s",
                          [[error localizedDescription] UTF8String]);
        }
        audioSession_ = nullptr;
    }

    isPlaying_ = false;
    logger_->info(LogMask::AUDIO, "AVAudioEngineHardwareProvider cleaned up");
}

bool AVAudioEngineHardwareProvider::startPlayback() {
    if (!audioEngine_) {
        logger_->error(LogMask::AUDIO, "Cannot start playback - AVAudioEngine not initialized");
        return false;
    }

    logger_->debug(LogMask::AUDIO, "Starting AVAudioEngine playback");

    NSError* error = nil;
    BOOL success = [audioEngine_ startAndReturnError:&error];

    if (!success) {
        logger_->error(LogMask::AUDIO, "Failed to start AVAudioEngine: %s",
                      [[error localizedDescription] UTF8String]);
        return false;
    }

    isPlaying_ = true;
    logger_->info(LogMask::AUDIO, "AVAudioEngine playback started");
    return true;
}

void AVAudioEngineHardwareProvider::stopPlayback() {
    if (!audioEngine_) {
        logger_->warning(LogMask::AUDIO, "Cannot stop playback - AVAudioEngine not initialized");
        return;
    }

    if (isPlaying_) {
        logger_->debug(LogMask::AUDIO, "Stopping AVAudioEngine playback");

        [audioEngine_ stop];
        isPlaying_ = false;
        logger_->info(LogMask::AUDIO, "AVAudioEngine playback stopped");
    }
}

void AVAudioEngineHardwareProvider::setVolume(double volume) {
    if (!audioEngine_) {
        logger_->warning(LogMask::AUDIO, "Cannot set volume - AVAudioEngine not initialized");
        return;
    }

    // Clamp volume to valid range
    double clampedVolume = std::max(0.0, std::min(1.0, volume));

    logger_->debug(LogMask::AUDIO, "Setting volume to %.2f", clampedVolume);

    // Use AVAudioEngine's mainMixerNode for volume control
    if (audioEngine_) {
        audioEngine_.mainMixerNode.outputVolume = static_cast<float>(clampedVolume);
        currentVolume_ = clampedVolume;
    } else {
        currentVolume_ = clampedVolume;
    }
}

double AVAudioEngineHardwareProvider::getVolume() const {
    return currentVolume_;
}

bool AVAudioEngineHardwareProvider::registerAudioCallback(const AudioCallback& callback) {
    if (!callback) {
        logger_->error(LogMask::AUDIO, "Cannot register null callback");
        return false;
    }

    audioCallback_ = callback;
    logger_->info(LogMask::AUDIO, "Audio callback registered");

    // Note: The callback is captured by the AVAudioSourceNode render block created
    // during initialize(). If we need to update the callback, we would recreate the source node.
    return true;
}

AudioHardwareState AVAudioEngineHardwareProvider::getHardwareState() const {
    AudioHardwareState state;
    state.isInitialized = (audioEngine_ != nullptr);
    state.isPlaying = isPlaying_;
    state.isCallbackActive = isPlaying_;  // Callback is active when playing
    state.currentVolume = currentVolume_;
    state.underrunCount = underrunCount_;
    state.overrunCount = overrunCount_;
    return state;
}

void AVAudioEngineHardwareProvider::resetDiagnostics() {
    underrunCount_ = 0;
    overrunCount_ = 0;
    logger_->debug(LogMask::AUDIO, "Hardware diagnostics reset");
}

// ================================================================
// Private Helper Methods
// ================================================================

bool AVAudioEngineHardwareProvider::setupAudioSession() {
    // Get shared audio session
    audioSession_ = [AVAudioSession sharedInstance];

    // Set audio session category to playback
    NSError* error = nil;
    [audioSession_ setCategory:AVAudioSessionCategoryPlayback error:&error];
    if (error) {
        logger_->error(LogMask::AUDIO, "Failed to set AVAudioSession category: %s",
                      [[error localizedDescription] UTF8String]);
        return false;
    }

    // Activate audio session
    [audioSession_ setActive:YES error:&error];
    if (error) {
        logger_->error(LogMask::AUDIO, "Failed to activate AVAudioSession: %s",
                      [[error localizedDescription] UTF8String]);
        return false;
    }

    logger_->info(LogMask::AUDIO, "AVAudioSession configured successfully");
    return true;
}

bool AVAudioEngineHardwareProvider::setupAudioEngine() {
    // Create AVAudioEngine instance
    audioEngine_ = [[AVAudioEngine alloc] init];
    if (!audioEngine_) {
        logger_->error(LogMask::AUDIO, "Failed to create AVAudioEngine");
        return false;
    }

    logger_->info(LogMask::AUDIO, "AVAudioEngine created successfully");
    return true;
}

bool AVAudioEngineHardwareProvider::configureAudioFormat(const AudioStreamFormat& format) {
    if (!audioEngine_) {
        return false;
    }

    // Create AVAudioFormat from our platform-agnostic format
    AVAudioFormat* audioFormat = [[AVAudioFormat alloc] initWithCommonFormat:AVAudioPCMFormatFloat32
                                                                  sampleRate:static_cast<double>(format.sampleRate)
                                                                    channels:static_cast<AVAudioChannelCount>(format.channels)
                                                                 interleaved:YES];
    if (!audioFormat) {
        logger_->error(LogMask::AUDIO, "Failed to create AVAudioFormat");
        return false;
    }

    // Note: output format is established via the connection in createSourceNode(),
    // not by setting it directly on the mixer node.

    logger_->info(LogMask::AUDIO, "Audio format configured: %dHz, %d channels, float32 interleaved",
                  format.sampleRate, format.channels);
    return true;
}

bool AVAudioEngineHardwareProvider::createSourceNode() {
    if (!audioEngine_) {
        return false;
    }

    // Create AVAudioFormat matching our configuration
    AVAudioFormat* audioFormat = [[AVAudioFormat alloc] initWithCommonFormat:AVAudioPCMFormatFloat32
                                                                  sampleRate:static_cast<double>(format_.sampleRate)
                                                                    channels:static_cast<AVAudioChannelCount>(format_.channels)
                                                                 interleaved:YES];

    if (!audioFormat) {
        logger_->error(LogMask::AUDIO, "Failed to create AVAudioFormat for source node");
        return false;
    }

    // Create render block matching AVAudioSourceNodeRenderBlock signature:
    //   OSStatus (^)(BOOL* isSilence, const AudioTimeStamp* timestamp,
    //                AVAudioFrameCount frameCount, AudioBufferList* outputData)
    //
    // This block is called by AVAudioEngine on the render thread when it needs audio samples.
    // IMPORTANT: Avoid blocking calls, ObjC message sends, and allocations in this block.
    AVAudioSourceNodeRenderBlock renderBlock = ^(BOOL* isSilence,
                                                  const AudioTimeStamp* timestamp,
                                                  AVAudioFrameCount frameCount,
                                                  AudioBufferList* outputData) {
        // Suppress unused parameters
        (void)timestamp;

        // Get pointer to the output buffer (interleaved float data)
        AudioBuffer* buffer = &outputData->mBuffers[0];
        float* channelData = static_cast<float*>(buffer->mData);

        if (!channelData) {
            if (isSilence) *isSilence = YES;
            underrunCount_++;
            return static_cast<OSStatus>(noErr);
        }

        // Invoke user-provided callback if registered
        if (audioCallback_) {
            AudioBufferView descriptor(channelData, static_cast<int>(frameCount), format_.channels);
            audioCallback_(descriptor);
        } else {
            // Fill with silence if no callback registered
            std::memset(channelData, 0, frameCount * format_.channels * sizeof(float));
        }

        if (isSilence) *isSilence = NO;
        return static_cast<OSStatus>(noErr);
    };

    // Create AVAudioSourceNode with format and render block
    @try {
        sourceNode_ = [[AVAudioSourceNode alloc] initWithFormat:audioFormat
                                                    renderBlock:renderBlock];
        if (!sourceNode_) {
            logger_->error(LogMask::AUDIO, "Failed to create AVAudioSourceNode");
            return false;
        }
    } @catch (NSException* exception) {
        logger_->error(LogMask::AUDIO, "Failed to create AVAudioSourceNode: %s",
                      [[exception reason] UTF8String]);
        return false;
    }

    // Attach source node to engine
    @try {
        [audioEngine_ attachNode:sourceNode_];
    } @catch (NSException* exception) {
        logger_->error(LogMask::AUDIO, "Failed to attach source node to engine: %s",
                      [[exception reason] UTF8String]);
        sourceNode_ = nullptr;
        return false;
    }

    // Connect source node to main mixer (format established via connection)
    @try {
        [audioEngine_ connect:sourceNode_ to:audioEngine_.mainMixerNode fromBus:0 toBus:0 format:audioFormat];
    } @catch (NSException* exception) {
        logger_->error(LogMask::AUDIO, "Failed to connect source node to mixer: %s",
                      [[exception reason] UTF8String]);
        return false;
    }

    logger_->info(LogMask::AUDIO, "AVAudioSourceNode created and connected successfully");
    return true;
}

#endif // TARGET_OS_IPHONE
