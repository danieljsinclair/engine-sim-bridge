// BridgeSimulator.cpp - Production ISimulator wrapping EngineSimAPI
// Forwards all ISimulator calls to the C-style EngineSim bridge functions.

#include "BridgeSimulator.h"
#include "ILogging.h"

// Static ISimulator::getVersion() -- delegates to bridge C API
const char* ISimulator::getVersion() {
    return EngineSimGetVersion();
}

BridgeSimulator::BridgeSimulator() = default;

BridgeSimulator::~BridgeSimulator() {
    if (created_ && handle_) {
        api_.Destroy(handle_);
        handle_ = nullptr;
        created_ = false;
    }
}

bool BridgeSimulator::create(const EngineSimConfig& config) {
    // Apply sensible defaults for any zero-valued fields
    EngineSimConfig effective = config;
    if (effective.sampleRate <= 0) effective.sampleRate = 48000;
    if (effective.inputBufferSize <= 0) effective.inputBufferSize = 1024;
    if (effective.audioBufferSize <= 0) effective.audioBufferSize = 96000;
    if (effective.simulationFrequency <= 0) effective.simulationFrequency = 10000;
    if (effective.fluidSimulationSteps <= 0) effective.fluidSimulationSteps = 8;
    if (effective.targetSynthesizerLatency <= 0.0) effective.targetSynthesizerLatency = 0.05;

    EngineSimResult result = api_.Create(&effective, &handle_);
    if (result != ESIM_SUCCESS || !handle_) {
        return false;
    }
    created_ = true;

    // Apply any pending logger that was set before create()
    if (pendingLogger_) {
        api_.SetLogging(handle_, pendingLogger_);
        pendingLogger_ = nullptr;
    }

    return true;
}

bool BridgeSimulator::loadScript(const std::string& path, const std::string& assetBase) {
    if (!handle_) return false;
    const char* pathPtr = path.empty() ? nullptr : path.c_str();
    const char* assetPtr = assetBase.empty() ? nullptr : assetBase.c_str();
    EngineSimResult result = api_.LoadScript(handle_, pathPtr, assetPtr);
    return result == ESIM_SUCCESS;
}

bool BridgeSimulator::setLogging(ILogging* logger) {
    if (!handle_) {
        // Store for later application after create()
        pendingLogger_ = logger;
        return true;
    }
    EngineSimResult result = api_.SetLogging(handle_, logger);
    return result == ESIM_SUCCESS;
}

void BridgeSimulator::setTelemetryWriter(telemetry::ITelemetryWriter* writer) {
    telemetryWriter_ = writer;
}

void BridgeSimulator::destroy() {
    if (!handle_) return;
    api_.Destroy(handle_);
    handle_ = nullptr;
    created_ = false;
}

std::string BridgeSimulator::getLastError() const {
    if (!handle_) return "";
    return api_.GetLastError(handle_);
}

void BridgeSimulator::update(double deltaTime) {
    if (!handle_) return;
    api_.Update(handle_, deltaTime);

    // Push engine state to telemetry if writer is set
    if (telemetryWriter_) {
        EngineSimStats stats = {};
        api_.GetStats(handle_, &stats);

        telemetry::EngineStateTelemetry engine;
        engine.currentRPM = stats.currentRPM;
        engine.currentLoad = stats.currentLoad;
        engine.exhaustFlow = stats.exhaustFlow;
        engine.manifoldPressure = stats.manifoldPressure;
        engine.activeChannels = stats.activeChannels;
        telemetryWriter_->writeEngineState(engine);

        telemetry::FramePerformanceTelemetry perf;
        perf.processingTimeMs = stats.processingTimeMs;
        telemetryWriter_->writeFramePerformance(perf);
    }
}

EngineSimStats BridgeSimulator::getStats() const {
    EngineSimStats stats = {};
    if (handle_) {
        api_.GetStats(handle_, &stats);
    }
    return stats;
}

void BridgeSimulator::setThrottle(double position) {
    if (!handle_) return;
    api_.SetThrottle(handle_, position);
}

void BridgeSimulator::setIgnition(bool on) {
    if (!handle_) return;
    api_.SetIgnition(handle_, on ? 1 : 0);
}

void BridgeSimulator::setStarterMotor(bool on) {
    if (!handle_) return;
    api_.SetStarterMotor(handle_, on ? 1 : 0);
}

bool BridgeSimulator::renderOnDemand(float* buffer, int32_t frames, int32_t* written) {
    if (!handle_) return false;
    EngineSimResult result = api_.RenderOnDemand(handle_, buffer, frames, written);
    return result == ESIM_SUCCESS;
}

bool BridgeSimulator::readAudioBuffer(float* buffer, int32_t frames, int32_t* read) {
    if (!handle_) return false;
    EngineSimResult result = api_.ReadAudioBuffer(handle_, buffer, frames, read);
    return result == ESIM_SUCCESS;
}

bool BridgeSimulator::start() {
    if (!handle_) return false;
    EngineSimResult result = api_.StartAudioThread(handle_);
    return result == ESIM_SUCCESS;
}
