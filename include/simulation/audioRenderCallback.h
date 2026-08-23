// audioRenderCallback.h - Internal test-access seam for the audio render callback.
//
// audioRenderCallback is a PURE function (no session state, no globals, no audio
// hardware). It is declared here so it can be exercised directly under unit tests
// with a fake IAudioBuffer + fake AudioBufferView, covering the !strategy->isPlaying()
// zeroing branch without real audio.
//
// This header is INTERNAL to the bridge (not part of the public API). Production
// behavior is unchanged; createSession still calls through it identically.

#ifndef AUDIO_RENDER_CALLBACK_H
#define AUDIO_RENDER_CALLBACK_H

#include "strategy/IAudioBuffer.h"
#include "hardware/AudioTypes.h"
#include "io/IAudioSink.h"

// Render one audio buffer via the given buffer strategy.
// Zeroes the destination when the strategy is not playing; otherwise forwards to
// strategy->render(buffer). Returns 0 (platform success code) for both paths.
//
// When sink is non-null, the frames the callback produced are also handed to the
// sink — including the zeroed not-playing frames, so a capture keeps a
// continuous timeline instead of silently compressing gaps. Passing nullptr
// (the default) is the no-capture path and is behaviour-identical to before.
int audioRenderCallback(IAudioBuffer* strategy, AudioBufferView& buffer,
                        io::IAudioSink* sink = nullptr);

#endif // AUDIO_RENDER_CALLBACK_H
