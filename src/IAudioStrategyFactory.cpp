// IAudioStrategyFactory.cpp - Factory for creating audio strategies
// Implements the factory pattern to create appropriate IAudioStrategy implementations

#include "IAudioStrategy.h"
#include "ThreadedStrategy.h"
#include "SyncPullStrategy.h"
#include "ITelemetryProvider.h"

#include <memory>

std::unique_ptr<IAudioStrategy> IAudioStrategyFactory::createStrategy(
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