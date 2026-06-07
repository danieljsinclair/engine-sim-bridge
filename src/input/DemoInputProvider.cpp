#include "input/DemoInputProvider.h"
#include "input/DemoThrottleSource.h"
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
    std::unique_ptr<GearSelectorInput> gearSelector,
    std::unique_ptr<IgnitionInput> ignition,
    const twin::IceVehicleProfile& profile,
    const DemoVehiclePhysicsConfig& physicsConfig
)
    : profile_(profile)
    , throttleSource_(std::move(throttleSource))
    , gearSelector_(std::move(gearSelector))
    , ignition_(std::move(ignition))
    , twinProvider_(profile_)
    , physics_(physicsConfig)
    , demoThrottle_(static_cast<DemoThrottleSource*>(throttleSource_.get())) {
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
        return EngineInput{};
    }

    double throttle = throttleSource_->pollThrottle();
    double brake = brakeInput_.pollLevel();
    physics_.update(dt, throttle, brake);

    roadSpeedKmh_ = physics_.getSpeedKmh();

    UpstreamSignal signal;
    signal.throttleFraction = throttle;
    signal.speedKmh = roadSpeedKmh_;
    signal.accelerationG = physics_.getAccelerationG();
    signal.timestampUtcMs = getCurrentTimeMs();
    signal.isValid = true;

    twinProvider_.setUpstreamSignal(signal);

    // Forward gear selector and ignition state to twin
    if (gearSelector_) {
        int currentSelector = gearSelector_->getState();
        if (currentSelector != lastForwardedSelector_) {
            twinProvider_.setGearSelector(currentSelector);
            lastForwardedSelector_ = currentSelector;
        }
    }

    if (ignition_) {
        twinProvider_.setIgnition(ignition_->isOn());
    }

    EngineInput input = twinProvider_.OnUpdateSimulation(dt);
    currentGear_ = input.gearAbsolute;

    // NOTE: brakeLevel is tracked but has no physics effect yet.
    // Dyno-based approach was investigated but dyno is a velocity-targeting
    // measurement constraint, not a brake. Needs Vehicle drag/force API.
    // TODO: circle-back — proper vehicle braking
    input.brakeLevel = brake;

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

void DemoInputProvider::provideFeedback(const EngineSimStats& stats) {
    twinProvider_.provideFeedback(stats);
}

// IDemoControls implementations
void DemoInputProvider::setThrottle(double level) {
    if (demoThrottle_) {
        demoThrottle_->setThrottleLevel(level);
    }
}

void DemoInputProvider::shiftUp() {
    if (gearSelector_) {
        gearSelector_->shiftUp();
    }
}

void DemoInputProvider::shiftDown() {
    if (gearSelector_) {
        gearSelector_->shiftDown();
    }
}

int DemoInputProvider::getGearSelectorState() const {
    if (gearSelector_) {
        return gearSelector_->getState();
    }
    return 0;
}

void DemoInputProvider::setIgnition(bool on) {
    if (ignition_) {
        ignition_->setOn(on);
    }
}

void DemoInputProvider::toggleIgnition() {
    if (ignition_) {
        ignition_->toggle();
    }
}

bool DemoInputProvider::isIgnitionOn() const {
    if (ignition_) {
        return ignition_->isOn();
    }
    return false;
}

void DemoInputProvider::requestExit() {
    if (demoThrottle_) {
        demoThrottle_->requestExit();
    }
}

void DemoInputProvider::setGearboxLogger(twin::IGearboxLogger* logger) {
    twinProvider_.setGearboxLogger(logger);
}

void DemoInputProvider::setBrake(double level) {
    brakeInput_.setLevel(level);
}

}
