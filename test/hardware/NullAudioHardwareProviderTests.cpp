// NullAudioHardwareProviderTests.cpp - Contract tests for the headless audio provider.
//
// NullAudioHardwareProvider is the no-op IAudioHardwareProvider strategy that lets
// tests build a seed/headless session (including the Q2 hot-swap path) without a
// real audio backend (see NullAudioHardwareProvider.h). Its contract:
//   - initialize / startPlayback / registerAudioCallback: succeed (true) without hardware
//   - cleanup / stopPlayback / setVolume / resetDiagnostics: tolerated no-ops
//   - getVolume: constant 0.0 (setVolume is a documented discard, not storage)
//   - getHardwareState: the AudioHardwareState default ctor's report —
//     not-initialized / not-playing / no callback / volume 1.0 / no over- or underruns
//
// Coverage note: this TU was previously all-cold in lcov — the only test "use" was
// construction (SimulationLoopHandoverTests), and construction runs the inline header
// ctor, never the out-of-line .cpp bodies. These tests drive every out-of-line method.
//
// Construction uses the real provider; no mocks of the class under test.

#include "hardware/NullAudioHardwareProvider.h"
#include "hardware/IAudioHardwareProvider.h"

#include <gtest/gtest.h>

using NullAudio = NullAudioHardwareProvider;

TEST(NullAudioHardwareProviderTest, InitializeSucceedsWithoutHardware) {
    NullAudio provider;
    AudioStreamFormat format{};  // default ctor: stereo/32-bit-float (IAudioHardwareProvider.h)

    // Contract: the headless provider always reports a successful initialize
    // (a headless session must boot exactly like a hardware-backed one).
    EXPECT_TRUE(provider.initialize(format));
}

TEST(NullAudioHardwareProviderTest, StartPlaybackSucceedsAfterInitialize) {
    NullAudio provider;
    AudioStreamFormat format{};

    provider.initialize(format);
    // Contract: playback "starts" successfully without any device behind it.
    EXPECT_TRUE(provider.startPlayback());
}

TEST(NullAudioHardwareProviderTest, GetVolumeIsConstantZeroAndSetVolumeIsADiscard) {
    NullAudio provider;
    AudioStreamFormat format{};

    provider.initialize(format);

    // Contract (NullAudioHardwareProvider.cpp): getVolume is a constant 0.0 and
    // setVolume is a documented no-op — the null provider holds no volume state.
    EXPECT_DOUBLE_EQ(provider.getVolume(), 0.0);
    provider.setVolume(0.5);
    EXPECT_DOUBLE_EQ(provider.getVolume(), 0.0)
        << "setVolume must remain a discard: the headless provider stores no volume";
}

TEST(NullAudioHardwareProviderTest, RegisterAudioCallbackAcceptsAnyCallback) {
    NullAudio provider;
    AudioStreamFormat format{};

    provider.initialize(format);

    // Contract: any well-formed AudioCallback is accepted (true); the null
    // provider never invokes it, so acceptance is the whole observable.
    IAudioHardwareProvider::AudioCallback callback =
        [](AudioBufferView&) { return 0; };
    EXPECT_TRUE(provider.registerAudioCallback(callback));
}

TEST(NullAudioHardwareProviderTest, GetHardwareStateReportsNotInitializedAndNotPlaying) {
    NullAudio provider;
    AudioStreamFormat format{};

    provider.initialize(format);
    provider.startPlayback();

    // Contract (NullAudioHardwareProvider.h): getHardwareState always reports
    // the default diagnostics — initialization and playback state come from the
    // AudioHardwareState default ctor, NOT from the calls above. Pin every field
    // so a future change to the reported diagnostics is a conscious one.
    const AudioHardwareState state = provider.getHardwareState();
    EXPECT_FALSE(state.isInitialized);
    EXPECT_FALSE(state.isPlaying);
    EXPECT_FALSE(state.isCallbackActive);
    EXPECT_DOUBLE_EQ(state.currentVolume, 1.0);  // AudioHardwareState ctor default
    EXPECT_EQ(state.underrunCount, 0);
    EXPECT_EQ(state.overrunCount, 0);
}

TEST(NullAudioHardwareProviderTest, MethodsAreSafeToCallInAnyOrderAndRepeatedly) {
    NullAudio provider;
    AudioStreamFormat format{};

    // Before initialize: every method tolerates being called (headless safety).
    IAudioHardwareProvider::AudioCallback callback =
        [](AudioBufferView&) { return 0; };
    provider.cleanup();
    provider.stopPlayback();
    provider.setVolume(0.5);
    provider.registerAudioCallback(callback);
    provider.getHardwareState();
    provider.resetDiagnostics();

    // Normal lifecycle, exercised twice over: initialize -> play -> cleanup,
    // then again after cleanup — the no-op provider must never become stateful.
    for (int cycle = 0; cycle < 2; ++cycle) {
        EXPECT_TRUE(provider.initialize(format));
        EXPECT_TRUE(provider.startPlayback());
        provider.stopPlayback();
        provider.cleanup();
    }
    EXPECT_DOUBLE_EQ(provider.getVolume(), 0.0);

    const AudioHardwareState state = provider.getHardwareState();
    EXPECT_FALSE(state.isInitialized);
    EXPECT_FALSE(state.isPlaying);
}

TEST(NullAudioHardwareProviderTest, ResetDiagnosticsIsAToleratedNoop) {
    NullAudio provider;
    AudioStreamFormat format{};

    provider.initialize(format);
    provider.resetDiagnostics();
    provider.resetDiagnostics();  // repeated: no state, so no crash and no change

    const AudioHardwareState state = provider.getHardwareState();
    EXPECT_EQ(state.underrunCount, 0);
    EXPECT_EQ(state.overrunCount, 0);
}
