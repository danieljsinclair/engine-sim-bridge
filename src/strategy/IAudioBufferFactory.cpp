// IAudioBufferFactory.cpp - Factory for creating audio buffers
// Implements the factory pattern to create appropriate IAudioBuffer implementations

#include "strategy/IAudioBuffer.h"
#include "strategy/ThreadedStrategy.h"
#include "strategy/SyncPullStrategy.h"
#include "telemetry/ITelemetryProvider.h"

#include <memory>

std::unique_ptr<IAudioBuffer> IAudioBufferFactory::createBuffer(
    AudioMode mode,
    ILogging* logger,
    telemetry::ITelemetryWriter* telemetry
) {
    switch (mode) {
        case AudioMode::Threaded: return std::make_unique<ThreadedStrategy>(logger, telemetry);
        case AudioMode::SyncPull: return std::make_unique<SyncPullStrategy>(logger, telemetry);
    }

    return nullptr;
}
