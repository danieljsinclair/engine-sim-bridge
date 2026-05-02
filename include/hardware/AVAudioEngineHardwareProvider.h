// AVAudioEngineHardwareProvider.h - iOS AVAudioEngine implementation of IAudioHardwareProvider
// Wraps AVAudioEngine + AVAudioSourceNode for iOS platform audio output
// OCP: iOS-specific implementation, abstracted behind IAudioHardwareProvider interface
// SRP: Single responsibility - manage AVAudioEngine lifecycle
// DIP: Depends on IAudioHardwareProvider abstraction, not exposed to clients
// Phase G: Created for iOS platform support

#ifndef AV_AUDIO_ENGINE_HARDWARE_PROVIDER_H
#define AV_AUDIO_ENGINE_HARDWARE_PROVIDER_H

#include "hardware/IAudioHardwareProvider.h"
#include "hardware/AudioTypes.h"
#include "common/ILogging.h"

#include <memory>
#include <functional>

#if TARGET_OS_IPHONE
#import <AudioToolbox/AudioToolbox.h>
#endif

/**
 * AVAudioEngineHardwareProvider - iOS AVAudioEngine implementation of IAudioHardwareProvider
 *
 * This class wraps CoreAudio RemoteIO AudioUnit for iOS behind the
 * platform-agnostic IAudioHardwareProvider interface. Matches the
 * CoreAudioHardwareProvider pattern on macOS for consistency.
 *
 * Responsibilities:
 * - AudioUnit lifecycle (create, initialize, cleanup)
 * - Audio format configuration
 * - Playback control (start/stop)
 * - Volume control
 * - Callback registration via render callback
 * - Diagnostic state tracking
 *
 * Design Decisions:
 * - RemoteIO AudioUnit: Low-level but consistent across Apple platforms
 *   (unlike AVAudioEngine which has format constraints and extra buffering)
 * - Matches CoreAudioHardwareProvider pattern — single clean callback
 */
class AVAudioEngineHardwareProvider : public IAudioHardwareProvider {
public:
    /**
     * Constructor with optional logger injection
     * @param logger Optional logger for diagnostics. If nullptr, uses default logger.
     */
    explicit AVAudioEngineHardwareProvider(ILogging* logger = nullptr);

    ~AVAudioEngineHardwareProvider() override;

    // ================================================================
    // IAudioHardwareProvider Implementation
    // ================================================================

    bool initialize(const AudioStreamFormat& format) override;
    void cleanup() override;

    bool startPlayback() override;
    void stopPlayback() override;

    void setVolume(double volume) override;
    double getVolume() const override;

    bool registerAudioCallback(const AudioCallback& callback) override;

    AudioHardwareState getHardwareState() const override;
    void resetDiagnostics() override;

private:
#if TARGET_OS_IPHONE
    // AudioUnit instance (RemoteIO)
    AudioUnit audioUnit_;

    // State
    bool isPlaying_;
    double currentVolume_;
    int underrunCount_;
    int overrunCount_;
    AudioStreamFormat format_;

    // Logging
    std::unique_ptr<ILogging> defaultLogger_;
    ILogging* logger_;

    // Callback
    AudioCallback audioCallback_;

    // Helpers
    bool setupAudioUnit();
    bool configureAudioFormat(const AudioStreamFormat& format);
    bool registerCallbackWithAudioUnit();
    static OSStatus remoteIORenderCallbackWrapper(
        void* refCon,
        AudioUnitRenderActionFlags* actionFlags,
        const AudioTimeStamp* timeStamp,
        UInt32 busNumber,
        UInt32 numberFrames,
        AudioBufferList* ioData
    );
    void logAudioError(const char* operation, OSStatus status, const char* additional = nullptr);

#endif
};

#endif // AV_AUDIO_ENGINE_HARDWARE_PROVIDER_H
