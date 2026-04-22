// ThreadedStrategy.cpp - Cursor-chasing audio strategy
// SRP: Single responsibility - only implements threaded cursor-chasing rendering
// OCP: New strategies can be added without modifying existing code
// DIP: Depends on abstractions (ISimulator), not concrete implementations
// Phase F: Moved to engine-sim-bridge submodule

#include "strategy/ThreadedStrategy.h"
#include "simulator/ISimulator.h"
#include "common/ILogging.h"
#include "common/Verification.h"
#include "telemetry/NullTelemetryWriter.h"

#include <cstring>
#include <algorithm>
#include <chrono>

using namespace std::chrono;

// ============================================================================
// ThreadedStrategy Implementation
// ============================================================================

ThreadedStrategy::ThreadedStrategy(ILogging* logger, telemetry::ITelemetryWriter* telemetry)
    : defaultLogger_(logger ? nullptr : new ConsoleLogger())
    , logger_(logger ? logger : defaultLogger_.get())
    , defaultTelemetry_(telemetry ? nullptr : new NullTelemetryWriter())
    , telemetry_(telemetry ? telemetry : defaultTelemetry_.get())
{
    ASSERT(logger_, "ThreadedStrategy: logger must not be null");
    ASSERT(telemetry_, "ThreadedStrategy: telemetry must not be null");
}

// ============================================================================
// IAudioBuffer Implementation
// ============================================================================

const char* ThreadedStrategy::getName() const {
    return "Threaded";
}

bool ThreadedStrategy::isEnabled() const {
    return true;
}

bool ThreadedStrategy::isPlaying() const {
    return audioState_.isPlaying.load();
}

bool ThreadedStrategy::shouldDrainDuringWarmup() const {
    return true;
}

void ThreadedStrategy::fillBufferFromEngine(ISimulator* simulator, int defaultFramesPerUpdate) {
    if (!simulator) return;

    // Calculate current lead distance based on total frames written vs read
    int64_t leadFrames = totalFramesWritten_ - totalFramesRead_;
    int sampleRate = audioState_.sampleRate;

    // Target lead: derived from TARGET_SYNTH_LATENCY
    // Maximum lead: 2x target (prevents buffer from growing too large)
    int targetLeadFrames = static_cast<int>(sampleRate * EngineSimDefaults::TARGET_SYNTH_LATENCY);
    int maxLeadFrames = targetLeadFrames * 2;

    // Self-correction: if lead has drifted too far, reset buffer
    int bufferSize = static_cast<int>(circularBuffer_.capacity());
    if (leadFrames > maxLeadFrames * 2) {
        logger_->warning(LogMask::AUDIO,
                      "ThreadedStrategy: Lead drift detected (%.0fms > 200ms), resetting to target",
                      leadFrames * 1000.0 / sampleRate);
        // Reset buffer by draining excess
        int framesToDiscard = static_cast<int>(leadFrames - maxLeadFrames);
        std::vector<float> discard(framesToDiscard * 2);
        circularBuffer_.read(discard.data(), framesToDiscard);
        totalFramesRead_ += framesToDiscard;
        leadFrames = totalFramesWritten_ - totalFramesRead_;
    }

    // Calculate how many frames to write based on current lead
    int framesToWrite;
    if (leadFrames < targetLeadFrames) {
        // Buffer is low - catch up by writing more
        int deficit = targetLeadFrames - static_cast<int>(leadFrames);
        framesToWrite = defaultFramesPerUpdate + deficit;
    } else if (leadFrames > maxLeadFrames) {
        // Buffer is too full - slow down or skip writing
        framesToWrite = 0;
    } else {
        // Buffer is in target range - write default amount
        framesToWrite = defaultFramesPerUpdate;
    }

    // Clamp to reasonable limits
    constexpr int MAX_FRAMES_PER_READ = EngineSimDefaults::MAX_AUDIO_CHUNK_FRAMES;
    framesToWrite = std::min(framesToWrite, MAX_FRAMES_PER_READ);
    framesToWrite = std::min(framesToWrite, bufferSize);

    // Read from simulator and write to buffer
    std::vector<float> buffer(framesToWrite * 2);
    int totalRead = 0;
    simulator->readAudioBuffer(buffer.data(), framesToWrite, &totalRead);

    if (totalRead > 0) {
        AddFrames(buffer.data(), totalRead);
        totalFramesWritten_ += totalRead;
    }
}

// ============================================================================
// Lifecycle Method Implementations
// ============================================================================

bool ThreadedStrategy::initialize(const AudioStrategyConfig& config) {
    ASSERT(logger_, "ThreadedStrategy::initialize: logger must not be null");
    ASSERT(telemetry_, "ThreadedStrategy::initialize: telemetry must not be null");

    // Initialize circular buffer with appropriate capacity
    int bufferCapacity = config.sampleRate * 2;

    if (!circularBuffer_.initialize(bufferCapacity)) {
        logger_->error(LogMask::AUDIO, "ThreadedStrategy::initialize: Failed to initialize circular buffer");
        return false;
    }

    audioState_.sampleRate = config.sampleRate;
    audioState_.isPlaying = false;
    diagnostics_.setSampleRate(config.sampleRate);
    circularBuffer_.reset();

    logger_->info(LogMask::AUDIO,
                  "ThreadedStrategy initialized: bufferCapacity=%d frames (%.2f seconds)",
                  bufferCapacity, bufferCapacity / static_cast<double>(config.sampleRate));

    return true;
}

void ThreadedStrategy::prepareBuffer() {
    // Reset tracking counters
    totalFramesWritten_ = 0;
    totalFramesRead_ = 0;

    // Pre-fill circular buffer with silence for smooth playback start
    int preFillFrames = static_cast<int>(audioState_.sampleRate * EngineSimDefaults::TARGET_SYNTH_LATENCY);
    int capacity = static_cast<int>(circularBuffer_.capacity());
    preFillFrames = std::min(preFillFrames, capacity);

    std::vector<float> silence(preFillFrames * 2);
    std::fill(silence.begin(), silence.end(), 0.0f);

    size_t framesWritten = circularBuffer_.write(silence.data(), preFillFrames);
    totalFramesWritten_ += static_cast<int64_t>(framesWritten);  // Track the pre-fill

    logger_->debug(LogMask::AUDIO, "ThreadedStrategy::prepareBuffer: Pre-filled %d frames with silence", static_cast<int>(framesWritten));
}

bool ThreadedStrategy::startPlayback(ISimulator* simulator) {
    if (!simulator) {
        logger_->error(LogMask::AUDIO, "ThreadedStrategy::startPlayback: Invalid simulator");
        return false;
    }

    bool result = simulator->start();
    if (!result) {
        logger_->error(LogMask::AUDIO, "ThreadedStrategy::startPlayback: Failed to start audio thread");
        return false;
    }

    simulator_ = simulator;
    audioState_.isPlaying.store(true);
    lastThroughputTime_ = std::chrono::steady_clock::now();

    logger_->info(LogMask::AUDIO, "ThreadedStrategy::startPlayback: Audio thread started");

    return true;
}

void ThreadedStrategy::stopPlayback(ISimulator* simulator) {
    if (simulator) {
        logger_->debug(LogMask::AUDIO, "ThreadedStrategy::stopPlayback: Audio thread will stop with simulation");
    }

    simulator_ = nullptr;
    audioState_.isPlaying.store(false);

    logger_->info(LogMask::AUDIO, "ThreadedStrategy::stopPlayback: Playback stopped");
}

void ThreadedStrategy::resetBufferAfterWarmup() {
    circularBuffer_.reset();
    // Reset tracking counters to start fresh after warmup
    totalFramesWritten_ = 0;
    totalFramesRead_ = 0;

    logger_->info(LogMask::AUDIO, "ThreadedStrategy::resetBufferAfterWarmup: Buffer reset complete");
}

void ThreadedStrategy::updateSimulation(ISimulator* simulator, double deltaTimeMs) {
    if (simulator && audioState_.isPlaying.load()) {
        double deltaTimeSeconds = deltaTimeMs / 1000.0;
        simulator->update(deltaTimeSeconds);
    }
}

bool ThreadedStrategy::render(AudioBufferView& buffer) {
    float* dst = buffer.asFloat();
    if (!dst) {
        return false;
    }

    if (!circularBuffer_.isInitialized()) {
        return false;
    }

    int availableFrames = getAvailableFrames();
    int framesToRead = std::min(buffer.frameCount, availableFrames);

    if (framesToRead <= 0) {
        size_t totalSamples = static_cast<size_t>(buffer.frameCount) * buffer.channelCount;
        std::memset(dst, 0, totalSamples * sizeof(float));
        updateDiagnostics(0, buffer.frameCount);
        diagnostics_.recordRender(0.0, 0, buffer.frameCount);
        publishAudioTiming();
        return true;
    }

    circularBuffer_.read(dst, framesToRead);

    // Track total frames read for lead distance calculation
    totalFramesRead_ += framesToRead;

    updateDiagnostics(availableFrames, buffer.frameCount);
    diagnostics_.recordRender(0.0, framesToRead, buffer.frameCount);
    publishAudioTiming();

    return true;
}

bool ThreadedStrategy::AddFrames(
    float* buffer,
    int frameCount
) {
    if (!buffer || frameCount <= 0) {
        return false;
    }

    if (!circularBuffer_.isInitialized()) {
        return false;
    }

    size_t framesWritten = circularBuffer_.write(buffer, frameCount);

    if (framesWritten != static_cast<size_t>(frameCount)) {
        logger_->warning(LogMask::AUDIO, "ThreadedStrategy::AddFrames: Only wrote %zu/%d frames",
                      framesWritten, frameCount);
    }

    return true;
}

void ThreadedStrategy::reset() {
    logger_->debug(LogMask::AUDIO, "ThreadedStrategy::reset");
}

std::string ThreadedStrategy::getModeString() const {
    return "THREADED";
}

// ============================================================================
// Private Helper Methods
// ============================================================================

int ThreadedStrategy::getAvailableFrames() const {
    return static_cast<int>(circularBuffer_.available());
}

void ThreadedStrategy::updateDiagnostics(
    int availableFrames,
    int requestedFrames
) {
    if (availableFrames < requestedFrames) {
        underrunCount_++;
    } else {
        underrunCount_ = 0;
    }

    double bufferHealthPct = 0.0;
    if (circularBuffer_.capacity() > 0) {
        bufferHealthPct = (static_cast<double>(availableFrames) /
                          static_cast<double>(circularBuffer_.capacity())) * 100.0;
    }

    publishAudioDiagnostics(underrunCount_, bufferHealthPct);
}

void ThreadedStrategy::publishAudioDiagnostics(int underrunCount, double bufferHealthPct) {
    telemetry::AudioDiagnosticsTelemetry diag;
    diag.underrunCount = underrunCount;
    diag.bufferHealthPct = bufferHealthPct;
    telemetry_->writeAudioDiagnostics(diag);
}

void ThreadedStrategy::publishAudioTiming() {
    // Update throughput rates once per second
    auto now = std::chrono::steady_clock::now();
    double elapsedSec = std::chrono::duration<double>(now - lastThroughputTime_).count();
    if (elapsedSec >= 1.0) {
        diagnostics_.updateThroughput(elapsedSec);
        lastThroughputTime_ = now;
    }

    auto snap = diagnostics_.getSnapshot();
    telemetry::AudioTimingTelemetry timing;
    timing.renderMs = snap.lastRenderMs;
    timing.headroomMs = snap.lastHeadroomMs;
    timing.budgetPct = snap.lastBudgetPct;
    timing.framesRequested = snap.lastFramesRequested;
    timing.framesRendered = snap.lastFramesRendered;
    timing.callbackRateHz = snap.callbackRateHz;
    timing.generatingRateFps = snap.generatingRateFps;
    timing.trendPct = snap.trendPct;
    telemetry_->writeAudioTiming(timing);
}
