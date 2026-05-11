#include "input/DemoInputProvider.h"
#include <chrono>

namespace input {

namespace {
uint64_t getCurrentTimeMs() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration);
    return static_cast<uint64_t>(millis.count());
}
}

DemoInputProvider::DemoInputProvider(
    std::unique_ptr<IThrottleSource> throttleSource,
    const twin::IceVehicleProfile& profile,
    const DemoVehiclePhysicsConfig& physicsConfig
)
    : profile_(profile)
    , throttleSource_(std::move(throttleSource))
    , twinProvider_(profile_)
    , physics_(physicsConfig) {
}

DemoInputProvider::~DemoInputProvider() {
    Shutdown();
}

bool DemoInputProvider::Initialize() {
    if (initialized_) {
        lastError_ = "Already initialized";
        return false;
    }

    if (!twinProvider_.Initialize()) {
        lastError_ = twinProvider_.GetLastError();
        return false;
    }

    initialized_ = true;
    return true;
}

void DemoInputProvider::Shutdown() {
    if (initialized_) {
        twinProvider_.Shutdown();
        initialized_ = false;
    }
}

bool DemoInputProvider::IsConnected() const {
    return initialized_ && twinProvider_.IsConnected();
}

EngineInput DemoInputProvider::OnUpdateSimulation(double dt) {
    if (!initialized_ || !throttleSource_) {
        EngineInput input;
        input.shouldContinue = false;
        return input;
    }

    double throttle = throttleSource_->pollThrottle();
    physics_.update(dt, throttle);

    roadSpeedKmh_ = physics_.getSpeedKmh();

    UpstreamSignal signal;
    signal.throttleFraction = throttle;
    signal.speedKmh = roadSpeedKmh_;
    signal.accelerationG = physics_.getAccelerationG();
    signal.timestampUtcMs = getCurrentTimeMs();
    signal.isValid = true;

    twinProvider_.setUpstreamSignal(signal);

    EngineInput input = twinProvider_.OnUpdateSimulation(dt);
    currentGear_ = input.gearAbsolute;

    if (!throttleSource_->shouldContinue()) {
        input.shouldContinue = false;
    }

    return input;
}

std::string DemoInputProvider::GetProviderName() const {
    return "DemoInputProvider";
}

std::string DemoInputProvider::GetLastError() const {
    return lastError_;
}

double DemoInputProvider::getDemoRoadSpeedKmh() const {
    return roadSpeedKmh_;
}

int DemoInputProvider::getDemoGear() const {
    return currentGear_;
}

}
