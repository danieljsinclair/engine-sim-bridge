#ifndef TELEMETRY_SEQUENCE_BUILDER_H
#define TELEMETRY_SEQUENCE_BUILDER_H

#include <vector>
#include <io/UpstreamSignal.h>

namespace test::scenarios {

struct TelemetrySequenceBuilder {
    static std::vector<input::UpstreamSignal> buildAccelerationTelemetry(double durationS, double dt);
    static std::vector<input::UpstreamSignal> buildCruiseTelemetry(double speedKmh, double throttle, double durationS, double dt);
    static std::vector<input::UpstreamSignal> buildDecelerationTelemetry(double startSpeedKmh, double durationS, double dt);
    static std::vector<input::UpstreamSignal> buildKickdownTelemetry(double cruiseSpeedKmh, double cruiseThrottle, double kickdownThrottle, double durationS, double dt);
    static std::vector<input::UpstreamSignal> buildLaunchTelemetry(double endSpeedKmh, double throttle, double durationS, double dt);
    static std::vector<input::UpstreamSignal> buildStandstillTelemetry(double durationS, double dt);

    static input::UpstreamSignal makeSignal(uint64_t timestampMs, double throttle, double speedKmh);
};

}
#endif
