#include "../include/mock_engine_sim_internal.h"

#include <string>
#include <cstring>
#include <mutex>
#include <atomic>
#include <thread>
#include <condition_variable>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <iostream>

// Wrap all functions in extern "C" for C linkage
extern "C" {

// ============================================================================
// INTERNAL IMPLEMENTATION STRUCTURES
// ============================================================================

struct MockEngineSimContext {
    // Configuration
    EngineSimConfig config;

    // Engine state
    std::atomic<double> throttlePosition;
    std::atomic<double> targetRPM;
    std::atomic<double> currentRPM;
    std::atomic<double> currentLoad;
    std::atomic<int> ignitionEnabled;
    std::atomic<int> starterMotorEnabled;
    std::atomic<int> currentGear;
    std::atomic<double> clutchPressure;
    std::atomic<int> dynoEnabled;
    std::atomic<int> dynoHoldEnabled;
    std::atomic<double> dynoSpeed;

    // Throttle smoothing parameters
    std::atomic<double> smoothedThrottle;
    double throttleSmoothingRatio;

    // RPM controller parameters (P-only)
    double kp; // Proportional gain
    double ki; // Integral gain
    double integral;
    double rpmControllerLastError;
    double lastThrottle;
    bool firstUpdate;

    // Starter motor parameters
    const double minSustainedRPM = 550.0;
    const double starterCutoffRPM = 600.0;

    // Audio generation
    std::atomic<double> sinePhase;
    std::atomic<double> lastSineFrequency;
    double sinePhaseValue;  // Non-atomic copy for arithmetic operations

    // Threading
    std::thread audioThread;
    std::atomic<bool> audioThreadRunning;
    std::mutex audioMutex;
    std::condition_variable audioWaitCond;
    std::condition_variable audioFrameReadyCond;
    std::atomic<bool> audioFrameReady;

    // Audio buffer (asynchronous mode)
    std::vector<float> audioBuffer;
    std::atomic<int> audioBufferWritePos;
    std::atomic<int> audioBufferReadPos;
    std::atomic<int> audioBufferFrameCount;

    // Error handling
    double lastError;
    std::mutex errorMutex;

    // Statistics
    EngineSimStats stats;
    double processingTimeMs;

    // Warmup phase
    const double warmupDuration = 2.0; // 2 seconds warmup
    bool inWarmupPhase;
    std::chrono::steady_clock::time_point warmupStartTime;

    MockEngineSimContext()
        : throttlePosition(0.0)
        , targetRPM(800.0)
        , currentRPM(0.0)
        , currentLoad(0.0)
        , ignitionEnabled(0)
        , starterMotorEnabled(0)
        , currentGear(0)
        , clutchPressure(1.0)
        , dynoEnabled(0)
        , dynoHoldEnabled(0)
        , dynoSpeed(0.0)
        , smoothedThrottle(0.0)
        , throttleSmoothingRatio(0.8) // Start with moderate smoothing
        , kp(0.002) // Proportional gain for RPM control
        , ki(0.0001) // Integral gain
        , integral(0.0)
        , rpmControllerLastError(0.0)
        , lastThrottle(0.0)
        , firstUpdate(true)
        , sinePhase(0.0)
        , lastSineFrequency(0.0)
        , sinePhaseValue(0.0)
        , audioThreadRunning(false)
        , audioFrameReady(false)
        , audioBufferWritePos(0)
        , audioBufferReadPos(0)
        , audioBufferFrameCount(0)
        , processingTimeMs(0.0)
        , inWarmupPhase(true)
        , warmupStartTime(std::chrono::steady_clock::now()) {

        std::memset(&config, 0, sizeof(config));
        std::memset(&stats, 0, sizeof(stats));

        // Initialize audio buffer (96000 frames at 48kHz = 2 seconds)
        resizeAudioBuffer(config.audioBufferSize);
    }

    ~MockEngineSimContext() {
        if (audioThreadRunning) {
            audioThreadRunning = false;
            audioWaitCond.notify_all();
            if (audioThread.joinable()) {
                audioThread.join();
            }
        }
    }

    void resizeAudioBuffer(int32_t size) {
        audioBuffer.resize(size * 2, 0.0f); // Stereo
        audioBufferWritePos = 0;
        audioBufferReadPos = 0;
        audioBufferFrameCount = 0;
    }

    void setError(const std::string& msg) {
        std::lock_guard<std::mutex> lock(errorMutex);
        lastError = msg.length();  // Store length as double
    }

    std::string getError() const {
        std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(errorMutex));
        return std::to_string(static_cast<int>(lastError));
    }

    // Throttle smoothing with adaptive behavior
    double applyThrottleSmoothing(double targetThrottle, double deltaTime) {
        // Adaptive smoothing based on throttle level
        double smoothingRatio;
        if (targetThrottle < 0.1) {  // Below 10% - very responsive
            smoothingRatio = 0.9;  // 90% new, 10% old
        } else if (targetThrottle < 0.3) {  // 10-30% - responsive but smooth
            smoothingRatio = 0.8;  // 80% new, 20% old
        } else {  // Above 30% - more damping
            smoothingRatio = 0.7;  // 70% new, 30% old
        }

        // Gradually adjust smoothing ratio
        throttleSmoothingRatio = throttleSmoothingRatio * 0.95 + smoothingRatio * 0.05;

        double newThrottle = targetThrottle * throttleSmoothingRatio +
                           smoothedThrottle * (1.0 - throttleSmoothingRatio);

        smoothedThrottle = newThrottle;
        return newThrottle;
    }

    // RPM controller with P-only logic
    double updateRPMController(double currentRPM, double deltaTime) {
        if (targetRPM <= 0) return 0.0;

        // Only enable RPM control above minimum RPM to prevent hunting at idle
        if (currentRPM < 100.0) {
            return 0.1; // Minimum throttle to keep engine running
        }

        double error = targetRPM - currentRPM;

        // Enhanced P-term calculation with damping for P-only instability
        double pTerm = error * kp;

        // Add damping to prevent oscillations at low throttle
        if (currentRPM < 2000.0) {
            double rpmChange = currentRPM - rpmControllerLastError;
            pTerm -= rpmChange * 0.0001; // Small damping factor
        }

        // Integral term with anti-windup for better low-throttle response
        integral += error * deltaTime;
        // Anti-windup: limit integral term
        integral = std::max(-0.5, std::min(0.5, integral));
        double iTerm = integral * ki;

        // Combine P and I terms
        double rawThrottle = pTerm + iTerm;

        // Apply throttle curve shaping for better low-throttle response
        double curvedThrottle;
        if (rawThrottle < 0.5) {
            // Curve for low throttle - more responsive
            curvedThrottle = rawThrottle * rawThrottle * 2.0;
        } else {
            // Linear for high throttle
            curvedThrottle = rawThrottle;
        }

        // Apply rate limiting for smooth transitions
        double rateLimitedThrottle;
        double maxRate = deltaTime * 0.5; // Max change per update
        if (curvedThrottle - lastThrottle > maxRate) {
            rateLimitedThrottle = lastThrottle + maxRate;
        } else if (curvedThrottle - lastThrottle < -maxRate) {
            rateLimitedThrottle = lastThrottle - maxRate;
        } else {
            rateLimitedThrottle = curvedThrottle;
        }

        // Update last values
        rpmControllerLastError = error;
        lastThrottle = rateLimitedThrottle;

        // Clamp throttle to bounds
        double finalThrottle = std::max(0.0, std::min(1.0, rateLimitedThrottle));

        // Debug output to track P-term calculation
        if (fabs(error) > 100) {
            std::cout << "Mock RPM Debug - Error: " << error << ", P: " << pTerm
                      << ", I: " << iTerm << ", Raw: " << rawThrottle
                      << ", Final: " << finalThrottle << std::endl;
        }

        return finalThrottle;
    }

    // Update RPM based on throttle and engine state
    void updateEngineState(double deltaTime) {
        if (ignitionEnabled == 0) {
            // Engine off - rapid RPM decay
            currentRPM = std::max(0.0, currentRPM - deltaTime * 500.0);
            targetRPM = 0.0;
            return;
        }

        double targetThrottle = throttlePosition.load(std::memory_order_relaxed);

            // Handle warmup phase
        if (inWarmupPhase) {
            auto currentTime = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
                currentTime - warmupStartTime
            ).count();

            // Exit warmup if time elapsed OR RPM reached minimum operating threshold
            if (elapsed >= warmupDuration || currentRPM > minSustainedRPM) {
                inWarmupPhase = false;
                std::cout << "[WARMUP] Warmup completed - elapsed: " << elapsed
                          << "s, RPM: " << currentRPM << std::endl;
            } else {
                // Gradual throttle increase during warmup
                // Linear ramp from 0 to 1 over 2 seconds
                double throttleProgress = elapsed / warmupDuration;
                targetThrottle = throttleProgress * 0.3; // Max 30% throttle during warmup
                targetThrottle = applyThrottleSmoothing(targetThrottle, deltaTime);
                throttlePosition.store(targetThrottle, std::memory_order_relaxed);
            }
        }

        // Apply throttle smoothing
        double smoothedThrottle = applyThrottleSmoothing(targetThrottle, deltaTime);

        // RPM control in automatic mode
        if (targetRPM > 0) {
            double autoThrottle = updateRPMController(currentRPM, deltaTime);
            smoothedThrottle = std::max(smoothedThrottle, autoThrottle);
        }

        // Simple engine physics model
        double targetRPMFromThrottle = 800.0 + smoothedThrottle * 5200.0; // 800-6000 RPM

        // Add starter motor effect
        if (starterMotorEnabled != 0 && currentRPM < starterCutoffRPM) {
            targetRPMFromThrottle = std::min(targetRPMFromThrottle, 1000.0);
            targetRPMFromThrottle += deltaTime * 500.0; // Starter acceleration
        }

        // Smooth RPM transitions
        double rpmChangeRate = 5000.0; // RPM per second
        if (targetRPMFromThrottle > currentRPM) {
            currentRPM = std::min(targetRPMFromThrottle,
                                 currentRPM + deltaTime * rpmChangeRate);
        } else {
            currentRPM = std::max(targetRPMFromThrottle,
                                 currentRPM - deltaTime * rpmChangeRate * 0.3); // Slower deceleration
        }

        // Ensure RPM doesn't go negative
        if (currentRPM < 0.0) currentRPM = 0.0;

        // Update statistics
        currentLoad = smoothedThrottle;
        stats.currentRPM = currentRPM;
        stats.currentLoad = currentLoad;
        stats.exhaustFlow = smoothedThrottle * 0.1; // Simplified exhaust flow
        stats.manifoldPressure = 101325 * (1.0 + smoothedThrottle * 0.3); // Atmospheric + boost
        stats.processingTimeMs = 0.5; // Fixed mock processing time

        // Auto-disable starter motor (including during warmup)
        if (starterMotorEnabled != 0 && currentRPM > starterCutoffRPM) {
            starterMotorEnabled = 0;
            std::cout << "[STARTER] Starter motor disabled - RPM: " << currentRPM << std::endl;
        }
    }

    // Generate sine wave based on current RPM - OPTIMIZED FOR BATCH PROCESSING
    void generateSineWave(float* buffer, int32_t frames, double sampleRate) {
        // Ensure minimum RPM for audio generation
        double safeRPM = currentRPM;
        if (ignitionEnabled == 0) {
            // No ignition - generate silence
            std::memset(buffer, 0, frames * 2 * sizeof(float));
            return;
        }
        
        // Use minimum RPM of 100Hz if RPM is too low
        if (safeRPM < 100.0) {
            safeRPM = 100.0;
        }
        
        double frequency = (safeRPM / 600.0) * 100.0; // 600 RPM -> 100Hz, 6000 RPM -> 1000Hz

        // Optimize batch processing by avoiding per-frame phase updates
        double phaseIncrement = 2.0 * M_PI * frequency / sampleRate;

        // Maintain continuous phase across calls
        // For batch generation, accumulate phase from previous generation
        double totalPhaseAdvance = phaseIncrement * frames;
        sinePhaseValue = sinePhase.load(std::memory_order_relaxed);

        // Optimize sine calculation using phase accumulation
        for (int i = 0; i < frames; ++i) {
            // Pre-calculate phase for this sample
            double phase = sinePhaseValue + phaseIncrement * i;
            double sample = sin(phase);

            // Apply volume
            sample *= config.volume;

            // Stereo output - identical channels
            buffer[i * 2] = static_cast<float>(sample);     // Left channel
            buffer[i * 2 + 1] = static_cast<float>(sample); // Right channel
        }
        
        // Update phase for next call
        sinePhaseValue += totalPhaseAdvance;
        sinePhase.store(sinePhaseValue, std::memory_order_relaxed);

        // Update phase with total advance for next call
        sinePhaseValue += totalPhaseAdvance;
        if (sinePhaseValue > 2.0 * M_PI) {
            sinePhaseValue -= 2.0 * M_PI;
        }
        sinePhase.store(sinePhaseValue, std::memory_order_relaxed);
        lastSineFrequency = frequency;
    }

    // Audio thread function - OPTIMIZED FOR BATCH AUDIO GENERATION
    void audioThreadFunction() {
        while (audioThreadRunning) {
            auto start = std::chrono::steady_clock::now();
            
            // Check if we should exit immediately
            if (!audioThreadRunning) break;

            // Update engine state
            updateEngineState(1.0 / 60.0); // 60Hz simulation

            // Generate sine wave and write to buffer
            std::lock_guard<std::mutex> lock(audioMutex);

            // ENHANCED: Generate larger batches to match consumption rate
            // Calculate available buffer space to prevent overflow
            int bufferCapacity = static_cast<int>(audioBuffer.size()) / 2;
            int framesToGenerate;

            // Check if buffer needs pre-filling (startup or underrun recovery)
            if (audioBufferFrameCount == 0) {
                // Buffer is empty - generate immediately
                framesToGenerate = std::min(config.inputBufferSize * 8, bufferCapacity / 4); // Large initial fill
                std::cout << "[AUDIO] Buffer empty - initial fill: " << framesToGenerate << " frames\n";
            } else if (audioBufferFrameCount < bufferCapacity / 4) {
                // Buffer is critically low - generate a large batch
                framesToGenerate = std::min(bufferCapacity / 6, bufferCapacity / 2); // Up to 1/6 of buffer
                std::cout << "[AUDIO] Pre-filling buffer: generating " << framesToGenerate << " frames\n";
            } else if (audioBufferFrameCount < bufferCapacity / 2) {
                // Buffer is low - generate medium batch
                framesToGenerate = std::min(bufferCapacity / 12, bufferCapacity / 3);
            } else {
                // Buffer is healthy - maintain normal generation rate
                framesToGenerate = std::min(config.inputBufferSize * 4, bufferCapacity / 8); // 4x normal batch
            }

            // Ensure we don't exceed buffer capacity
            framesToGenerate = std::min(framesToGenerate, bufferCapacity - audioBufferFrameCount);

            // Pre-fill buffer with silence to prevent initial underruns
            if (audioBufferFrameCount == 0 && audioBuffer.size() > 0) {
                int preFillFrames = std::min(1024, static_cast<int>(audioBuffer.size()) / 2);
                for (int i = 0; i < preFillFrames; i++) {
                    audioBuffer[i * 2] = 0.0f;
                    audioBuffer[i * 2 + 1] = 0.0f;
                }
                audioBufferFrameCount = preFillFrames;
                std::cout << "[AUDIO] Pre-filled buffer with " << preFillFrames << " frames of silence\n";
            }

            if (framesToGenerate > 0) {
                // Generate sine wave with optimized batch processing
                std::vector<float> tempBuffer(framesToGenerate * 2);
                generateSineWave(tempBuffer.data(), framesToGenerate, config.sampleRate);

                // Write to circular buffer in one operation
                for (int i = 0; i < framesToGenerate; ++i) {
                    audioBuffer[audioBufferWritePos * 2] = tempBuffer[i * 2];
                    audioBuffer[audioBufferWritePos * 2 + 1] = tempBuffer[i * 2 + 1];

                    audioBufferWritePos = (audioBufferWritePos + 1) % bufferCapacity;

                    if (audioBufferFrameCount < bufferCapacity) {
                        audioBufferFrameCount++;
                    }
                }

                // Signal frame ready
                audioFrameReady = true;
                // Ensure audio thread is always awake when data is needed
            if (audioBufferFrameCount > 0) {
                audioFrameReadyCond.notify_one();
            }
            }

            // Adaptive timing to maintain smooth operation
            auto end = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);

            // Adjust sleep time based on buffer fill level
            double targetFrameTime = 1.0 / 60.0; // Target 60Hz
            double adaptiveSleepTime = targetFrameTime - elapsed.count();

            // If buffer is low, run more frequently to recover
            if (audioBufferFrameCount < bufferCapacity / 3) {
                adaptiveSleepTime *= 0.5; // Run at 120Hz when buffer is low
            } else if (audioBufferFrameCount > bufferCapacity * 3/4) {
                adaptiveSleepTime *= 1.2; // Slow down when buffer is full
            }

            adaptiveSleepTime = std::max(0.0, adaptiveSleepTime);

            std::unique_lock<std::mutex> waitLock(audioMutex);
            if (audioThreadRunning) {
                audioWaitCond.wait_for(waitLock,
                                       std::chrono::duration<double>(adaptiveSleepTime),
                                       [this] { return !audioThreadRunning; });
            }
        }
    }
};

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static bool validateHandle(EngineSimHandle handle) {
    return handle != nullptr;
}

static MockEngineSimContext* getContext(EngineSimHandle handle) {
    return static_cast<MockEngineSimContext*>(handle);
}

static void setDefaultConfig(EngineSimConfig* config) {
    config->sampleRate = 48000;
    config->inputBufferSize = 1024;
    config->audioBufferSize = 96000; // 2 seconds @ 48kHz
    config->simulationFrequency = 10000;
    config->fluidSimulationSteps = 8;
    config->targetSynthesizerLatency = 0.05; // 50ms
    config->volume = 0.5f;
    config->convolutionLevel = 1.0f;
    config->airNoise = 1.0f;
}

// ============================================================================
// LIFECYCLE FUNCTIONS
// ============================================================================

extern "C" EngineSimResult EngineSimCreate(
    const EngineSimConfig* config,
    EngineSimHandle* outHandle)
{
    if (!outHandle) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    // Allocate context
    MockEngineSimContext* ctx = new (std::nothrow) MockEngineSimContext();
    if (!ctx) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    // Use default config if none provided
    if (config) {
        ctx->config = *config;
    } else {
        setDefaultConfig(&ctx->config);
    }

    // Validate configuration
    EngineSimResult validateResult = EngineSimValidateConfig(&ctx->config);
    if (validateResult != ESIM_SUCCESS) {
        delete ctx;
        return validateResult;
    }

    *outHandle = ctx;
    return ESIM_SUCCESS;
}

extern "C" EngineSimResult EngineSimLoadScript(
    EngineSimHandle handle,
    const char* scriptPath,
    const char* assetBasePath)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    MockEngineSimContext* ctx = getContext(handle);

    // Set warmup start time
    ctx->warmupStartTime = std::chrono::steady_clock::now();

    // Initialize engine to idle state
    ctx->currentRPM = 0.0;
    ctx->throttlePosition = 0.0;
    ctx->smoothedThrottle = 0.0;
    ctx->sinePhase = 0.0;
    ctx->lastSineFrequency = 0.0;
    ctx->inWarmupPhase = true;

    return ESIM_SUCCESS;
}

EngineSimResult EngineSimStartAudioThread(
    EngineSimHandle handle)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    MockEngineSimContext* ctx = getContext(handle);

    if (ctx->audioThreadRunning) {
        return ESIM_SUCCESS; // Already running
    }

    ctx->audioThreadRunning = true;
    ctx->audioThread = std::thread(&MockEngineSimContext::audioThreadFunction, ctx);

    return ESIM_SUCCESS;
}

EngineSimResult EngineSimDestroy(
    EngineSimHandle handle)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    MockEngineSimContext* ctx = getContext(handle);
    delete ctx;

    return ESIM_SUCCESS;
}

// ============================================================================
// CONTROL FUNCTIONS
// ============================================================================

EngineSimResult EngineSimSetThrottle(
    EngineSimHandle handle,
    double position)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    if (position < 0.0 || position > 1.0) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    MockEngineSimContext* ctx = getContext(handle);
    ctx->throttlePosition.store(position, std::memory_order_relaxed);

    return ESIM_SUCCESS;
}

EngineSimResult EngineSimUpdate(
    EngineSimHandle handle,
    double deltaTime)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    if (deltaTime <= 0.0 || deltaTime > 1.0) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    MockEngineSimContext* ctx = getContext(handle);

    // Update engine state
    ctx->updateEngineState(deltaTime);

    return ESIM_SUCCESS;
}

// ============================================================================
// AUDIO RENDERING (CRITICAL HOT PATH)
// ============================================================================

EngineSimResult EngineSimRender(
    EngineSimHandle handle,
    float* buffer,
    int32_t frames,
    int32_t* outSamplesWritten)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    if (!buffer || frames <= 0) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    MockEngineSimContext* ctx = getContext(handle);

    // Generate sine wave directly (synchronous mode)
    ctx->generateSineWave(buffer, frames, ctx->config.sampleRate);

    if (outSamplesWritten) {
        *outSamplesWritten = frames;
    }

    return ESIM_SUCCESS;
}

EngineSimResult EngineSimReadAudioBuffer(
    EngineSimHandle handle,
    float* buffer,
    int32_t frames,
    int32_t* outSamplesRead)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    if (!buffer || frames <= 0) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    MockEngineSimContext* ctx = getContext(handle);

    std::lock_guard<std::mutex> lock(ctx->audioMutex);

    // Check if we have enough frames
    int framesAvailable = ctx->audioBufferFrameCount.load();
    int framesToRead = std::min(frames, framesAvailable);

    // Read from circular buffer
    for (int i = 0; i < framesToRead; ++i) {
        int readIndex = (ctx->audioBufferReadPos + i) % (ctx->audioBuffer.size() / 2);
        buffer[i * 2] = ctx->audioBuffer[readIndex * 2];
        buffer[i * 2 + 1] = ctx->audioBuffer[readIndex * 2 + 1];
    }

    // Update read position
    ctx->audioBufferReadPos = (ctx->audioBufferReadPos + framesToRead) % (ctx->audioBuffer.size() / 2);
    ctx->audioBufferFrameCount -= framesToRead;

    // Mark frame as consumed
    if (framesToRead > 0) {
        ctx->audioFrameReady = false;
    }

    if (outSamplesRead) {
        *outSamplesRead = framesToRead;
    }

    // Zero-fill any remaining frames to prevent crackling
    if (framesToRead < frames) {
        const int remainingFrames = frames - framesToRead;
        float* silenceStart = buffer + framesToRead * 2;
        std::memset(silenceStart, 0, remainingFrames * 2 * sizeof(float));
    }

    return ESIM_SUCCESS;
}

// ============================================================================
// DIAGNOSTICS & TELEMETRY
// ============================================================================

EngineSimResult EngineSimGetStats(
    EngineSimHandle handle,
    EngineSimStats* outStats)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    if (!outStats) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    MockEngineSimContext* ctx = getContext(handle);
    *outStats = ctx->stats;

    return ESIM_SUCCESS;
}

const char* EngineSimGetLastError(
    EngineSimHandle handle)
{
    if (!validateHandle(handle)) {
        return "Invalid handle";
    }

    MockEngineSimContext* ctx = getContext(handle);
    std::string error = ctx->getError();

    // Use thread-local storage for allocation-free API
    static thread_local std::string error_buffer;
    error_buffer = error;
    return error_buffer.c_str();
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

const char* EngineSimGetVersion(void)
{
    return "mock-engine-sim/1.0.0";
}

EngineSimResult EngineSimValidateConfig(
    const EngineSimConfig* config)
{
    if (!config) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    // Validate sample rate
    if (config->sampleRate < 8000 || config->sampleRate > 192000) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    // Validate buffer sizes
    if (config->inputBufferSize < 64 || config->inputBufferSize > config->sampleRate) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    if (config->audioBufferSize < config->sampleRate / 2) {
        return ESIM_ERROR_INVALID_PARAMETER; // At least 0.5 seconds
    }

    // Validate simulation frequency
    if (config->simulationFrequency < 1000 || config->simulationFrequency > 100000) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    // Validate fluid steps
    if (config->fluidSimulationSteps < 1 || config->fluidSimulationSteps > 64) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    // Validate latency
    if (config->targetSynthesizerLatency < 0.001 || config->targetSynthesizerLatency > 1.0) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    // Validate audio parameters
    if (config->volume < 0.0f || config->volume > 10.0f) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    if (config->convolutionLevel < 0.0f || config->convolutionLevel > 1.0f) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    return ESIM_SUCCESS;
}

// ============================================================================
// ADDITIONAL CONTROL FUNCTIONS
// ============================================================================

EngineSimResult EngineSimSetSpeedControl(
    EngineSimHandle handle,
    double position)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    if (position < 0.0 || position > 1.0) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    MockEngineSimContext* ctx = getContext(handle);
    ctx->throttlePosition.store(position, std::memory_order_relaxed);

    return ESIM_SUCCESS;
}

EngineSimResult EngineSimSetStarterMotor(
    EngineSimHandle handle,
    int enabled)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    MockEngineSimContext* ctx = getContext(handle);
    ctx->starterMotorEnabled.store(enabled != 0, std::memory_order_relaxed);

    return ESIM_SUCCESS;
}

EngineSimResult EngineSimSetIgnition(
    EngineSimHandle handle,
    int enabled)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    MockEngineSimContext* ctx = getContext(handle);
    ctx->ignitionEnabled.store(enabled != 0, std::memory_order_relaxed);

    return ESIM_SUCCESS;
}

EngineSimResult EngineSimShiftGear(
    EngineSimHandle handle,
    int gear)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    MockEngineSimContext* ctx = getContext(handle);
    ctx->currentGear.store(gear, std::memory_order_relaxed);

    return ESIM_SUCCESS;
}

EngineSimResult EngineSimSetClutch(
    EngineSimHandle handle,
    double pressure)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    if (pressure < 0.0 || pressure > 1.0) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    MockEngineSimContext* ctx = getContext(handle);
    ctx->clutchPressure.store(pressure, std::memory_order_relaxed);

    return ESIM_SUCCESS;
}

EngineSimResult EngineSimSetDyno(
    EngineSimHandle handle,
    int enabled)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    MockEngineSimContext* ctx = getContext(handle);
    ctx->dynoEnabled.store(enabled != 0, std::memory_order_relaxed);

    return ESIM_SUCCESS;
}

EngineSimResult EngineSimSetDynoHold(
    EngineSimHandle handle,
    int enabled,
    double speed)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    MockEngineSimContext* ctx = getContext(handle);
    ctx->dynoHoldEnabled.store(enabled != 0, std::memory_order_relaxed);
    ctx->dynoSpeed.store(speed, std::memory_order_relaxed);

    return ESIM_SUCCESS;
}

EngineSimResult EngineSimLoadImpulseResponse(
    EngineSimHandle handle,
    int exhaustIndex,
    const int16_t* impulseData,
    int sampleCount,
    float volume)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    if (!impulseData || sampleCount <= 0) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    if (volume < 0.0f || volume > 10.0f) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    MockEngineSimContext* ctx = getContext(handle);

    // In mock implementation, we just validate parameters but don't actually load impulse responses
    // The sine wave generation doesn't use convolution
    return ESIM_SUCCESS;
}

EngineSimResult EngineSimWaitForAudioFrame(
    EngineSimHandle handle)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    MockEngineSimContext* ctx = getContext(handle);

    // Wait for audio thread to complete current frame
    std::unique_lock<std::mutex> lock(ctx->audioMutex);
    ctx->audioFrameReadyCond.wait(lock, [ctx] { return ctx->audioFrameReady || !ctx->audioThreadRunning; });

    return ESIM_SUCCESS;
}
} // extern "C"