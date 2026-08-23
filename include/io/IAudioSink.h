// IAudioSink.h - Destination for rendered audio frames.
//
// The render callback produces interleaved float frames and, when a capture
// destination is configured, hands the same frames to an IAudioSink. The sink
// decides what "storing" means: a WAV file (WavFileSink), a socket, a test spy.
//
// DIP: SimulationLoop's callback depends on this abstraction, never on a file
// format. OCP: a new capture destination is a new implementation, not an edit
// to the render path.
//
// Threading contract: writeFrames() is called from the real-time audio thread.
// finalize() is called from the owning thread AFTER audio hardware has stopped,
// so no further writeFrames() can be in flight. Implementations must still be
// safe against the two being called from different threads.

#ifndef IAUDIO_SINK_H
#define IAUDIO_SINK_H

namespace io {

class IAudioSink {
public:
    virtual ~IAudioSink() = default;

    /// Append frameCount interleaved frames of channelCount channels each.
    /// Called from the audio render thread; must not throw.
    virtual void writeFrames(const float* interleaved, int frameCount, int channelCount) = 0;

    /// Flush and complete the destination. Returns false if the destination
    /// could not be completed (e.g. the file could not be written).
    /// Idempotent: a second call is a no-op returning the first result.
    virtual bool finalize() = 0;
};

} // namespace io

#endif // IAUDIO_SINK_H
