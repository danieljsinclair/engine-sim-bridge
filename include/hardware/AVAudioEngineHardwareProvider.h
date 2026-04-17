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
#import <AVFoundation/AVFoundation.h>
#import <AudioToolbox/AudioToolbox.h>
#endif

/**
 * AVAudioEngineHardwareProvider - iOS AVAudioEngine implementation of IAudioHardwareProvider
 *
 * This class wraps all AVAudioEngine operations behind the platform-agnostic
 * IAudioHardwareProvider interface. Uses AVAudioSourceNode for real-time
 * audio generation callback.
 *
 * Responsibilities:
 * - AVAudioEngine lifecycle (create, initialize, cleanup)
 * - AVAudioSession management (.playback category, activation)
 * - Audio format configuration
 * - Playback control (start/stop)
 * - Volume control
 * - Callback registration via AVAudioSourceNode
 * - Diagnostic state tracking
 *
 * Thread Safety:
 * - AVAudioSourceNode callback must be thread-safe (real-time audio thread)
 * - AVAudioEngine calls should be on main thread
 *
 * Design Decisions:
 * - AVAudioEngine over RemoteIO: Higher-level API, easier lifecycle management
 * - KISS: No unnecessary complexity for this use case
 * - AVAudioSourceNode: Clean callback interface for real-time audio generation
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
    // ================================================================
    // iOS-specific Members
    // ================================================================

    AVAudioEngine* audioEngine_;         // AVAudioEngine instance
    AVAudioSourceNode* sourceNode_;      // Source node for real-time audio generation
    AVAudioSession* audioSession_;       // Shared audio session instance
    bool isPlaying_;                    // Playback state
    double currentVolume_;               // Current volume level (0.0 to 1.0)
    int underrunCount_;                 // Buffer underrun counter
    int overrunCount_;                  // Buffer overrun counter
    AudioStreamFormat format_;           // Stored audio format

    // ================================================================
    // Logging
    // ================================================================

    std::unique_ptr<ILogging> defaultLogger_;
    ILogging* logger_;                  // Non-null, points to defaultLogger_ or injected logger

    // ================================================================
    // Callback Management
    // ================================================================

    AudioCallback audioCallback_;         // Registered audio callback

    // ================================================================
    // Private Helper Methods
    // ================================================================

    bool setupAudioSession();
    bool setupAudioEngine();
    bool configureAudioFormat(const AudioStreamFormat& format);
    bool createSourceNode();

#endif
};

#endif // AV_AUDIO_ENGINE_HARDWARE_PROVIDER_H
