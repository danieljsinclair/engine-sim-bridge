// CoreAudioHardwareProvider.h - macOS CoreAudio implementation of IAudioHardwareProvider
// Wraps CoreAudio AudioUnit for macOS platform audio output
// OCP: macOS-specific implementation, abstracted behind IAudioHardwareProvider interface
// SRP: Single responsibility - manage CoreAudio AudioUnit lifecycle
// DIP: Depends on IAudioHardwareProvider abstraction, not exposed to clients
// Phase F: Moved to engine-sim-bridge submodule

#ifndef CORE_AUDIO_HARDWARE_PROVIDER_H
#define CORE_AUDIO_HARDWARE_PROVIDER_H

#include "hardware/IAudioHardwareProvider.h"
#include "common/ILogging.h"

#include <AudioUnit/AudioUnit.h>
#include <AudioToolbox/AudioToolbox.h>
#include <memory>

/**
 * CoreAudioHardwareProvider - macOS CoreAudio implementation of IAudioHardwareProvider
 *
 * This class wraps all CoreAudio AudioUnit operations behind the platform-agnostic
 * IAudioHardwareProvider interface.
 *
 * Responsibilities:
 * - AudioUnit lifecycle (create, initialize, cleanup)
 * - Audio format configuration
 * - Playback control (start/stop)
 * - Volume control
 * - Callback registration and management
 * - Diagnostic state tracking
 *
 * Thread Safety:
 * - Callback must be thread-safe (real-time audio thread)
 * - State access uses appropriate synchronization
 */
class CoreAudioHardwareProvider : public IAudioHardwareProvider {
public:
    /**
     * Constructor with optional logger injection
     * @param logger Optional logger for diagnostics. If nullptr, uses default logger.
     */
    explicit CoreAudioHardwareProvider(ILogging* logger = nullptr);

    ~CoreAudioHardwareProvider() override;

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
    // ================================================================
    // CoreAudio-specific Members
    // ================================================================

    AudioUnit audioUnit;              // CoreAudio AudioUnit instance
    AudioDeviceID deviceID;          // Default output device ID
    bool isPlaying;                  // Playback state
    double currentVolume;              // Current volume level (0.0 to 1.0)
    int underrunCount;               // Buffer underrun counter
    int overrunCount;                // Buffer overrun counter

    // ================================================================
    // Logging
    // ================================================================

    std::unique_ptr<ILogging> defaultLogger_;
    ILogging* logger_;                // Non-null, points to defaultLogger_ or injected logger

    // ================================================================
    // Callback Management
    // ================================================================

    AudioCallback audioCallback_;     // Registered audio callback

    // ================================================================
    // Private Helper Methods
    // ================================================================

    bool setupAudioUnit();
    bool configureAudioFormat(const AudioStreamFormat& format);
    bool registerCallbackWithAudioUnit();

    static OSStatus coreAudioCallbackWrapper(
        void* refCon,
        AudioUnitRenderActionFlags* actionFlags,
        const AudioTimeStamp* timeStamp,
        UInt32 busNumber,
        UInt32 numberFrames,
        AudioBufferList* ioData
    );

    static const char* getStatusDescription(OSStatus status);

    void logCoreAudioError(const char* operation, OSStatus status, const char* additional = nullptr);
};

#endif // CORE_AUDIO_HARDWARE_PROVIDER_H
