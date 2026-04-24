// AVAudioEngineHardwareProvider.mm - iOS RemoteIO AudioUnit implementation of IAudioHardwareProvider
// Uses CoreAudio AudioUnit API (kAudioUnitSubType_RemoteIO) — same pattern as CoreAudioHardwareProvider
// Phase G: Created for iOS platform support

#include "hardware/AVAudioEngineHardwareProvider.h"

#if TARGET_OS_IPHONE

#include <cstring>
#include <thread>
#include <chrono>

// ================================================================
// AVAudioEngineHardwareProvider Implementation
// ================================================================

AVAudioEngineHardwareProvider::AVAudioEngineHardwareProvider(ILogging* logger)
    : audioUnit_(nullptr),
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

    // Setup AudioUnit (RemoteIO)
    if (!setupAudioUnit()) {
        logger_->error(LogMask::AUDIO, "Failed to setup AudioUnit");
        return false;
    }

    // Configure audio format (interleaved float32)
    if (!configureAudioFormat(format)) {
        logger_->error(LogMask::AUDIO, "Failed to configure audio format");
        return false;
    }

    // Register render callback
    if (!registerCallbackWithAudioUnit()) {
        logger_->error(LogMask::AUDIO, "Failed to register audio callback");
        return false;
    }

    // Initialize AudioUnit (required before start)
    OSStatus status = AudioUnitInitialize(audioUnit_);
    if (status != noErr) {
        logAudioError("AudioUnitInitialize", status);
        return false;
    }

    logger_->info(LogMask::AUDIO, "AVAudioEngineHardwareProvider initialized successfully");
    return true;
}

void AVAudioEngineHardwareProvider::cleanup() {
    if (audioUnit_) {
        stopPlayback();

        // Allow in-flight render callbacks to drain
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Uninitialize and dispose AudioUnit
        OSStatus status = AudioUnitUninitialize(audioUnit_);
        if (status != noErr) {
            logger_->warning(LogMask::AUDIO, "AudioUnitUninitialize failed: %d", status);
        }

        status = AudioComponentInstanceDispose(audioUnit_);
        if (status != noErr) {
            logger_->warning(LogMask::AUDIO, "AudioComponentInstanceDispose failed: %d", status);
        }

        audioUnit_ = nullptr;
    }

    isPlaying_ = false;
    logger_->info(LogMask::AUDIO, "AVAudioEngineHardwareProvider cleaned up");
}

bool AVAudioEngineHardwareProvider::startPlayback() {
    if (!audioUnit_) {
        logger_->error(LogMask::AUDIO, "Cannot start playback - AudioUnit not initialized");
        return false;
    }

    logger_->debug(LogMask::AUDIO, "Starting AudioUnit playback");

    OSStatus status = AudioOutputUnitStart(audioUnit_);
    if (status != noErr) {
        logAudioError("AudioOutputUnitStart", status);
        return false;
    }

    isPlaying_ = true;
    logger_->info(LogMask::AUDIO, "AudioUnit playback started");
    return true;
}

void AVAudioEngineHardwareProvider::stopPlayback() {
    if (!audioUnit_) {
        logger_->warning(LogMask::AUDIO, "Cannot stop playback - AudioUnit not initialized");
        return;
    }

    if (isPlaying_) {
        logger_->debug(LogMask::AUDIO, "Stopping AudioUnit playback");

        OSStatus status = AudioOutputUnitStop(audioUnit_);
        if (status != noErr) {
            logAudioError("AudioOutputUnitStop", status);
        }

        isPlaying_ = false;
        logger_->info(LogMask::AUDIO, "AudioUnit playback stopped");
    }
}

void AVAudioEngineHardwareProvider::setVolume(double volume) {
    if (!audioUnit_) {
        logger_->warning(LogMask::AUDIO, "Cannot set volume - AudioUnit not initialized");
        return;
    }

    double clampedVolume = std::max(0.0, std::min(1.0, volume));
    logger_->debug(LogMask::AUDIO, "Setting volume to %.2f", clampedVolume);

    OSStatus status = AudioUnitSetParameter(
        audioUnit_,
        kHALOutputParam_Volume,
        kAudioUnitScope_Global,
        0,
        static_cast<Float32>(clampedVolume),
        0
    );

    if (status != noErr) {
        logAudioError("AudioUnitSetParameter (volume)", status);
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
    return true;
}

AudioHardwareState AVAudioEngineHardwareProvider::getHardwareState() const {
    AudioHardwareState state;
    state.isInitialized = (audioUnit_ != nullptr);
    state.isPlaying = isPlaying_;
    state.isCallbackActive = isPlaying_;
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

bool AVAudioEngineHardwareProvider::setupAudioUnit() {
    AudioComponentDescription desc = {};
    desc.componentType = kAudioUnitType_Output;
    desc.componentSubType = kAudioUnitSubType_RemoteIO;
    desc.componentManufacturer = kAudioUnitManufacturer_Apple;
    desc.componentFlags = 0;
    desc.componentFlagsMask = 0;

    AudioComponent component = AudioComponentFindNext(nullptr, &desc);
    if (!component) {
        logger_->error(LogMask::AUDIO, "Failed to find RemoteIO AudioComponent");
        return false;
    }

    OSStatus status = AudioComponentInstanceNew(component, &audioUnit_);
    if (status != noErr || !audioUnit_) {
        logAudioError("AudioComponentInstanceNew", status, "RemoteIO unavailable");
        return false;
    }

    return true;
}

bool AVAudioEngineHardwareProvider::configureAudioFormat(const AudioStreamFormat& format) {
    if (!audioUnit_) return false;

    AudioStreamBasicDescription streamFormat = {};
    streamFormat.mSampleRate = static_cast<Float64>(format.sampleRate);
    streamFormat.mFormatID = kAudioFormatLinearPCM;
    streamFormat.mFormatFlags = kLinearPCMFormatFlagIsFloat | kLinearPCMFormatFlagIsPacked;
    streamFormat.mBitsPerChannel = 32;
    streamFormat.mFramesPerPacket = 1;
    streamFormat.mBytesPerPacket = format.channels * sizeof(float);
    streamFormat.mBytesPerFrame = format.channels * sizeof(float);
    streamFormat.mChannelsPerFrame = static_cast<UInt32>(format.channels);
    streamFormat.mReserved = 0;

    OSStatus status = AudioUnitSetProperty(
        audioUnit_,
        kAudioUnitProperty_StreamFormat,
        kAudioUnitScope_Input,
        0,
        &streamFormat,
        sizeof(streamFormat)
    );

    if (status != noErr) {
        logAudioError("AudioUnitSetProperty (format)", status, "Interleaved float32 not supported");
        return false;
    }

    logger_->info(LogMask::AUDIO, "Audio format configured: %dHz, %d channels, float32 interleaved",
                  format.sampleRate, format.channels);
    return true;
}

bool AVAudioEngineHardwareProvider::registerCallbackWithAudioUnit() {
    if (!audioUnit_ || !audioCallback_) return false;

    AURenderCallbackStruct callbackStruct;
    callbackStruct.inputProc = &remoteIORenderCallbackWrapper;
    callbackStruct.inputProcRefCon = this;

    OSStatus status = AudioUnitSetProperty(
        audioUnit_,
        kAudioUnitProperty_SetRenderCallback,
        kAudioUnitScope_Input,
        0,
        &callbackStruct,
        sizeof(callbackStruct)
    );

    if (status != noErr) {
        logAudioError("AudioUnitSetProperty (callback)", status);
        return false;
    }

    return true;
}

OSStatus AVAudioEngineHardwareProvider::remoteIORenderCallbackWrapper(
    void* refCon,
    AudioUnitRenderActionFlags* actionFlags,
    const AudioTimeStamp* timeStamp,
    UInt32 busNumber,
    UInt32 numberFrames,
    AudioBufferList* ioData
) {
    (void)actionFlags;
    (void)timeStamp;
    (void)busNumber;

    AVAudioEngineHardwareProvider* self = static_cast<AVAudioEngineHardwareProvider*>(refCon);
    if (!self || !self->audioCallback_) return noErr;

    AudioBuffer* buffer = &ioData->mBuffers[0];
    float* channelData = static_cast<float*>(buffer->mData);
    int channels = static_cast<int>(buffer->mNumberChannels);

    if (!channelData) {
        self->underrunCount_++;
        return noErr;
    }

    AudioBufferView view(channelData, static_cast<int>(numberFrames), channels);
    return static_cast<OSStatus>(self->audioCallback_(view));
}

void AVAudioEngineHardwareProvider::logAudioError(const char* operation, OSStatus status, const char* additional) {
    const char* description = "unknown error";
    switch (status) {
        case noErr: description = "no error"; break;
        case kAudioUnitErr_FormatNotSupported: description = "format not supported"; break;
        case kAudioUnitErr_Initialized: description = "already initialized"; break;
        case kAudioUnitErr_InvalidParameter: description = "invalid parameter"; break;
        case kAudioUnitErr_InvalidProperty: description = "invalid property"; break;
        case kAudioUnitErr_InvalidElement: description = "invalid element"; break;
        case kAudioUnitErr_NoConnection: description = "no connection"; break;
        case kAudioUnitErr_Uninitialized: description = "hardware uninitialized"; break;
        case kAudioUnitErr_TooManyFramesToProcess: description = "too many frames"; break;
        default: description = "unknown error"; break;
    }

    if (additional) {
        logger_->error(LogMask::AUDIO, "AudioUnit error in %s: %s (%d) - %s",
                      operation, description, status, additional);
    } else {
        logger_->error(LogMask::AUDIO, "AudioUnit error in %s: %s (%d)",
                      operation, description, status);
    }
}

#endif // TARGET_OS_IPHONE
