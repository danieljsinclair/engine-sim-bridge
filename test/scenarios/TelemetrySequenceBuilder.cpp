#include "TelemetrySequenceBuilder.h"

namespace test::scenarios {

std::vector<input::UpstreamSignal> TelemetrySequenceBuilder::buildAccelerationTelemetry(double durationS, double dt) {
    std::vector<input::UpstreamSignal> signals;
    int steps = static_cast<int>(durationS / dt);

    for (int i = 0; i < steps; ++i) {
        double progress = static_cast<double>(i) / steps;
        double throttle = progress;
        double speedKmh = progress * 100.0;
        signals.push_back(makeSignal(static_cast<uint64_t>(i * dt * 1000), throttle, speedKmh));
    }
    return signals;
}

std::vector<input::UpstreamSignal> TelemetrySequenceBuilder::buildCruiseTelemetry(double speedKmh, double throttle, double durationS, double dt) {
    std::vector<input::UpstreamSignal> signals;
    int steps = static_cast<int>(durationS / dt);

    for (int i = 0; i < steps; ++i) {
        signals.push_back(makeSignal(static_cast<uint64_t>(i * dt * 1000), throttle, speedKmh));
    }
    return signals;
}

std::vector<input::UpstreamSignal> TelemetrySequenceBuilder::buildDecelerationTelemetry(double startSpeedKmh, double durationS, double dt) {
    std::vector<input::UpstreamSignal> signals;
    int steps = static_cast<int>(durationS / dt);

    for (int i = 0; i < steps; ++i) {
        double progress = static_cast<double>(i) / steps;
        double speedKmh = startSpeedKmh * (1.0 - progress);
        signals.push_back(makeSignal(static_cast<uint64_t>(i * dt * 1000), 0.0, speedKmh));
    }
    return signals;
}

std::vector<input::UpstreamSignal> TelemetrySequenceBuilder::buildKickdownTelemetry(double cruiseSpeedKmh, double cruiseThrottle, double kickdownThrottle, double durationS, double dt) {
    std::vector<input::UpstreamSignal> signals;
    int steps = static_cast<int>(durationS / dt);

    for (int i = 0; i < steps; ++i) {
        double t = i * dt;
        double throttle = (t < 0.5) ? cruiseThrottle : kickdownThrottle;
        signals.push_back(makeSignal(static_cast<uint64_t>(t * 1000), throttle, cruiseSpeedKmh));
    }
    return signals;
}

std::vector<input::UpstreamSignal> TelemetrySequenceBuilder::buildLaunchTelemetry(double endSpeedKmh, double throttle, double durationS, double dt) {
    std::vector<input::UpstreamSignal> signals;
    int steps = static_cast<int>(durationS / dt);

    for (int i = 0; i < steps; ++i) {
        double progress = static_cast<double>(i) / steps;
        double speedKmh = progress * endSpeedKmh;
        signals.push_back(makeSignal(static_cast<uint64_t>(i * dt * 1000), throttle, speedKmh));
    }
    return signals;
}

std::vector<input::UpstreamSignal> TelemetrySequenceBuilder::buildStandstillTelemetry(double durationS, double dt) {
    std::vector<input::UpstreamSignal> signals;
    int steps = static_cast<int>(durationS / dt);

    for (int i = 0; i < steps; ++i) {
        signals.push_back(makeSignal(static_cast<uint64_t>(i * dt * 1000), 0.0, 0.0));
    }
    return signals;
}

input::UpstreamSignal TelemetrySequenceBuilder::makeSignal(uint64_t timestampMs, double throttle, double speedKmh) {
    input::UpstreamSignal sig;
    sig.throttleFraction = throttle;
    sig.speedKmh = speedKmh;
    sig.timestampUtcMs = timestampMs;
    sig.isValid = true;
    return sig;
}

}
