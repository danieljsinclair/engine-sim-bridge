#include "simulator/engine_sim_bridge.h"
#include "simulator/ScriptLoadHelpers.h"
#include "common/ILogging.h"

// Core engine-sim includes
#include "simulator.h"
#include "engine.h"
#include "vehicle.h"
#include "transmission.h"
#include "synthesizer.h"
#include "units.h"
#include "impulse_response.h"
#include "exhaust_system.h"

// DR_WAV_IMPLEMENTATION: Must be defined in exactly one TU before including wav_loader.h
#define DR_WAV_IMPLEMENTATION
#include "common/wav_loader.h"

// Scripting includes (only when piranha is enabled)
#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
#include "../scripting/include/compiler.h"
#endif

#include <string>
#include <cstring>
#include <cmath>
#include <mutex>
#include <atomic>
#include <filesystem>

namespace ANSIColors {
    const std::string GREEN = "\x1b[32m";
    const std::string YELLOW = "\x1b[33m";
    const std::string RED = "\x1b[31m";
    const std::string CYAN = "\x1b[36m";
    const std::string RESET = "\x1b[0m";
}

// ============================================================================
// INTERNAL IMPLEMENTATION STRUCTURES
// ============================================================================

struct EngineSimContext {
    // Core components - SINGLE simulator pointer (polymorphic)
    // Can be either PistonEngineSimulator or SineWaveSimulator based on config
    Simulator* simulator;

    // Configuration
    EngineSimConfig config;

    // State
    std::atomic<double> throttlePosition;
    std::string lastError;
    std::mutex errorMutex;

    // Audio conversion buffer (reused, allocation-free after init)
    int16_t* audioConversionBuffer;
    size_t conversionBufferSize;

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
    // Scripting
    es_script::Compiler* compiler;
#endif
    // Engine state pointers (available with or without scripting)
    Engine* engine;
    Vehicle* vehicle;
    Transmission* transmission;

    // Statistics
    EngineSimStats stats;

    // DI: Injected logging interface
    // Default logger is always available (owned by context)
    // Caller can provide custom logger via EngineSimSetLogging
    ConsoleLogger defaultLogger;
    ILogging* logger;

    EngineSimContext()
        : simulator(nullptr)
        , throttlePosition(0.0)
        , audioConversionBuffer(nullptr)
        , conversionBufferSize(0)
#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
        , compiler(nullptr)
#endif
        , engine(nullptr)
        , vehicle(nullptr)
        , transmission(nullptr)
        , logger(&defaultLogger)
    {
        std::memset(&config, 0, sizeof(config));
        std::memset(&stats, 0, sizeof(stats));
    }

    ~EngineSimContext() {
        if (audioConversionBuffer) {
            delete[] audioConversionBuffer;
            audioConversionBuffer = nullptr;
        }

        if (simulator) {
            simulator->endAudioRenderingThread();
            simulator->destroy();
            delete simulator;
            simulator = nullptr;
        }

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
        if (compiler) {
            compiler->destroy();
            delete compiler;
            compiler = nullptr;
        }
#endif
    }

    void setError(const std::string& msg) {
        std::lock_guard<std::mutex> lock(errorMutex);
        lastError = msg;
    }

    std::string getError() const {
        std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(errorMutex));
        return lastError;
    }
};

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

static bool validateHandle(EngineSimHandle handle) {
    return handle != nullptr;
}

static EngineSimContext* getContext(EngineSimHandle handle) {
    return static_cast<EngineSimContext*>(handle);
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
// HELPER FUNCTIONS FOR SCRIPT LOADING (SRP)
// Delegated to ScriptLoadHelpers.h (DRY)
// ============================================================================

/**
 * Normalize script path (C-string overload for C API).
 * Delegates to ScriptLoadHelpers::normalizeScriptPath.
 */
static std::string normalizeScriptPath(const char* scriptPath) {
    if (!scriptPath || strlen(scriptPath) == 0) {
        return "";
    }
    return ScriptLoadHelpers::normalizeScriptPath(std::string(scriptPath));
}

/**
 * Resolve asset base path (C-string overload for C API).
 * Delegates to ScriptLoadHelpers::resolveAssetBasePath.
 */
static std::string resolveAssetBasePath(const std::string& scriptPath, const char* assetBasePath) {
    std::string assetBase;
    if (assetBasePath != nullptr && strlen(assetBasePath) > 0) {
        assetBase = std::string(assetBasePath);
    }
    return ScriptLoadHelpers::resolveAssetBasePath(scriptPath, assetBase);
}

// Delegated to shared helpers (DRY)
static Vehicle* createDefaultVehicle() {
    return ScriptLoadHelpers::createDefaultVehicle();
}

static Transmission* createDefaultTransmission() {
    return ScriptLoadHelpers::createDefaultTransmission();
}

// ============================================================================
// LIFECYCLE FUNCTIONS
// ============================================================================

EngineSimResult EngineSimCreate(
    const EngineSimConfig* config,
    Simulator* simulatorInstance,
    EngineSimHandle* outHandle)
{
    if (!outHandle || !simulatorInstance) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    // Allocate context
    EngineSimContext* ctx = new EngineSimContext();

    // Use provided simulator instance (injected by caller)
    ctx->simulator = simulatorInstance;

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

    // Initialize simulator with common parameters
    Simulator::Parameters simParams;
    simParams.systemType = Simulator::SystemType::NsvOptimized;

    ctx->simulator->initialize(simParams);
    ctx->simulator->setSimulationFrequency(ctx->config.simulationFrequency);
    ctx->simulator->setTargetSynthesizerLatency(ctx->config.targetSynthesizerLatency);
    ctx->simulator->setFluidSimulationSteps(ctx->config.fluidSimulationSteps);

    // Allocate audio conversion buffer (stereo)
    ctx->conversionBufferSize = 4096 * 2; // Max frames * 2 channels
    ctx->audioConversionBuffer = new int16_t[ctx->conversionBufferSize];

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
    // Create compiler for script loading (will be used by LoadScript)
    ctx->compiler = new es_script::Compiler();
    ctx->compiler->initialize();
#endif

    *outHandle = ctx;
    return ESIM_SUCCESS;
}

/// ==============================================================================
/// SCRIPT LOADING
/// ==============================================================================
EngineSimResult EngineSimLoadScript(
    EngineSimHandle handle,
    const char* scriptPath,
    const char* assetBasePath)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    EngineSimContext* ctx = getContext(handle);

    // POLYMORPHIC CHECK: If simulator has already initialized its engine
    // (e.g., SineWaveSimulator creates dummy objects in initialize()),
    // just populate context pointers without loading a script.
    // This respects Open/Closed and Liskov - no type checking, just behavior.
    if (ctx->simulator->getEngine() != nullptr) {
        ctx->engine = ctx->simulator->getEngine();
        ctx->vehicle = ctx->simulator->getVehicle();
        ctx->transmission = ctx->simulator->getTransmission();
        ctx->throttlePosition.store(0.0, std::memory_order_relaxed);
        return ESIM_SUCCESS;
    }

    // Normalize script path (SRP: path handling extracted)
    std::string scriptPathStr = normalizeScriptPath(scriptPath);
    if (scriptPathStr.empty()) {
        ctx->setError("Script path required for engine simulation");
        return ESIM_ERROR_INVALID_PARAMETER;
    }

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
    // Compile the script
    if (!ctx->compiler->compile(scriptPathStr.c_str())) {
        ctx->setError("Failed to compile script: " + scriptPathStr);
        return ESIM_ERROR_SCRIPT_COMPILATION;
    }

    // Execute the compiled script
    es_script::Compiler::Output output = ctx->compiler->execute();

    // Create defaults for missing components (SRP: object creation extracted)
    Vehicle* vehicle = output.vehicle ? output.vehicle : createDefaultVehicle();
    Transmission* transmission = output.transmission ? output.transmission : createDefaultTransmission();

    // Store references
    ctx->engine = output.engine;
    ctx->vehicle = vehicle;
    ctx->transmission = transmission;

    if (!ctx->engine) {
        ctx->setError("Script did not create an engine");
        return ESIM_ERROR_LOAD_FAILED;
    }

    // Load the simulation (virtual dispatch - works for any Simulator subclass)
    ctx->simulator->loadSimulation(ctx->engine, ctx->vehicle, ctx->transmission);

    // Resolve asset path and load impulse responses (SRP: path handling extracted)
    std::string resolvedAssetPath = resolveAssetBasePath(scriptPathStr, assetBasePath);
    if (!ScriptLoadHelpers::loadImpulseResponses(ctx->simulator, ctx->engine, resolvedAssetPath, ctx->logger)) {
        ctx->setError("Failed to load impulse responses");
        return ESIM_ERROR_LOAD_FAILED;
    }

    return ESIM_SUCCESS;
#else
    (void)scriptPathStr;  // Suppress unused warning
    (void)assetBasePath;  // Suppress unused warning
    ctx->setError("Script loading not available (Piranha support disabled)");
    return ESIM_ERROR_SCRIPT_COMPILATION;
#endif
}

/// ==============================================================================
/// AUDIO THREAD CONTROL
/// ==============================================================================
EngineSimResult EngineSimStartAudioThread(
    EngineSimHandle handle)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    EngineSimContext* ctx = getContext(handle);

    if (!ctx->engine) {
        ctx->setError("No engine loaded. Ensure script path was provided to EngineSimCreate() before EngineSimLoadImpulseResponse().");
        return ESIM_ERROR_NOT_INITIALIZED;
    }

    // Drain the synthesizer's pre-fill buffer (96000 samples of silence)
    // This must be done BEFORE starting the audio thread
    // The audio thread waits for buffer.size() < 2000, but pre-fill is 96000
    int16_t drainBuffer[4096];
    int totalDrained = 0;
    int samplesDrained;
    while ((samplesDrained = ctx->simulator->readAudioOutput(4096, drainBuffer)) > 0) {
        totalDrained += samplesDrained;
    }
    ctx->logger->debug(LogMask::BUFFER, "Drained %d pre-fill samples", totalDrained);
    ctx->simulator->startAudioRenderingThread();

    return ESIM_SUCCESS;
}

/// ==============================================================================
/// LIFECYCLE CONTINUED
/// ==============================================================================
EngineSimResult EngineSimDestroy(
    EngineSimHandle handle)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    EngineSimContext* ctx = getContext(handle);
    delete ctx;

    return ESIM_SUCCESS;
}

EngineSimResult EngineSimSetLogging(
    EngineSimHandle handle,
    ILogging* logger)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    EngineSimContext* ctx = getContext(handle);
    // If null logger is provided, keep using the default ConsoleLogger
    // Otherwise, use the provided logger (caller retains ownership)
    if (logger) {
        ctx->logger = logger;
    }
    // If logger is null, ctx->logger already points to defaultLogger

    return ESIM_SUCCESS;
}

// ==============================================================================
/// CONTROL FUNCTIONS
/// ==============================================================================

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

    EngineSimContext* ctx = getContext(handle);
    ctx->throttlePosition.store(position, std::memory_order_relaxed);

    // Use the Governor abstraction for proper closed-loop feedback
    // This ensures the Governor's safety features (full throttle at low RPM) are active
    if (ctx->engine) {
        ctx->engine->setSpeedControl(position);
    }

    return ESIM_SUCCESS;
}

/// =======================================================================
/// SIMULATION UPDATE
/// =======================================================================
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

    EngineSimContext* ctx = getContext(handle);

    if (!ctx->engine) {
        ctx->setError("No engine loaded");
        return ESIM_ERROR_NOT_INITIALIZED;
    }

    // Run simulation frame
    ctx->simulator->startFrame(deltaTime);

    while (ctx->simulator->simulateStep()) {
        // Process all simulation steps for this frame
    }

    ctx->simulator->endFrame();

    // Update statistics
    double throttle = ctx->throttlePosition.load(std::memory_order_relaxed);
    if (ctx->engine) {
        ctx->stats.currentRPM = units::toRpm(ctx->engine->getSpeed());
        ctx->stats.currentLoad = throttle;
        ctx->stats.exhaustFlow = ctx->simulator->getTotalExhaustFlow();
        ctx->stats.processingTimeMs = ctx->simulator->getAverageProcessingTime() * 1000.0;
    }

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

    EngineSimContext* ctx = getContext(handle);

    // Check buffer size
    size_t requiredSize = frames * 2; // Stereo
    if (requiredSize > ctx->conversionBufferSize) {
        ctx->setError("Render buffer size exceeds internal buffer");
        return ESIM_ERROR_AUDIO_BUFFER;
    }

    // CRITICAL: For synchronous rendering (no audio thread), we must call renderAudio()
    // to generate audio samples before reading them. The audio thread normally does this.
    //
    // However, renderAudio() can only be called ONCE per simulation step because:
    // 1. It sets m_processed = true after each call (synthesizer.cpp:241)
    // 2. Subsequent calls BLOCK waiting for m_processed to be reset to false
    // 3. m_processed is only reset by endInputBlock() at the end of simulateStep()
    //
    // Therefore, we can ONLY call renderAudio() once per EngineSimRender() call.
    // If there aren't enough samples, we must handle the underrun gracefully.

    // Call renderAudio() ONCE to generate samples from the latest simulation step
    ctx->simulator->synthesizer().renderAudio();

    // Read audio from synthesizer (int16 format)
    // IMPORTANT: readAudioOutput returns MONO samples (1 sample per frame)
    int samplesRead = ctx->simulator->readAudioOutput(
        frames,
        ctx->audioConversionBuffer
    );

    static int renderCount = 0;
    static int lastUnderrunReport = 0;
    if (samplesRead < frames) {
        // Report underruns every 60 frames (1 second) to avoid spam
        if (renderCount - lastUnderrunReport > 60) {
            ctx->logger->warning(LogMask::DIAGNOSTICS, "RENDER UNDERRUN at frame %d: requested=%d got=%d missing=%d",
                                renderCount, frames, samplesRead, frames - samplesRead);
            lastUnderrunReport = renderCount;
        }
    }
    if (renderCount < 5 || samplesRead == 0) {
        ctx->logger->debug(LogMask::AUDIO, "RENDER: samplesRead=%d frames=%d", samplesRead, frames);
    }
    renderCount++;

    // Convert mono int16 to stereo float32 using shared utility (DRY)
    EngineSimAudio::convertInt16ToStereoFloat(ctx->audioConversionBuffer, buffer, samplesRead);


    // CRITICAL: Zero-fill any remaining frames to prevent crackling from uninitialized memory
    // If we have a buffer underrun (samplesRead < frames), the rest of the buffer
    // must be explicitly zeroed - otherwise the output has garbage data = static/crackling
    if (samplesRead < frames) {
        const int remainingFrames = frames - samplesRead;
        float* silenceStart = buffer + samplesRead * 2;  // Stereo offset
        EngineSimAudio::fillSilence(silenceStart, remainingFrames);
    }

    if (outSamplesWritten) {
        *outSamplesWritten = samplesRead;
    }

    return ESIM_SUCCESS;
}

/// ==============================================================================
/// SYNCHRONOUS RENDERING (NO AUDIO THREAD)
/// ==============================================================================
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

    EngineSimContext* ctx = getContext(handle);

    if (!ctx->engine) {
        // No engine loaded - output silence
        EngineSimAudio::fillSilence(buffer, frames);
        if (outSamplesRead) {
            *outSamplesRead = 0;
        }
        return ESIM_SUCCESS;
    }

    // Check buffer size
    size_t requiredSize = frames * 2; // Stereo
    if (requiredSize > ctx->conversionBufferSize) {
        ctx->setError("Read buffer size exceeds internal buffer");
        return ESIM_ERROR_AUDIO_BUFFER;
    }

    // IMPORTANT: This function does NOT call renderAudio()
    // It only reads from the audio buffer that the audio thread is filling
    // This matches the GUI pattern (engine_sim_application.cpp line 274)

    // Read audio from synthesizer (int16 format)
    // IMPORTANT: readAudioOutput returns MONO samples (1 sample per frame)
    int samplesRead = ctx->simulator->readAudioOutput(
        frames,
        ctx->audioConversionBuffer
    );

    // Convert mono int16 to stereo float32 using shared utility (DRY)
    EngineSimAudio::convertInt16ToStereoFloat(ctx->audioConversionBuffer, buffer, samplesRead);

    // CRITICAL: Zero-fill any remaining frames to prevent crackling from uninitialized memory
    // If we have a buffer underrun (samplesRead < frames), the rest of the buffer
    // must be explicitly zeroed - otherwise the callback receives garbage data = static/crackling
    if (samplesRead < frames) {
        const int remainingFrames = frames - samplesRead;
        float* silenceStart = buffer + samplesRead * 2;  // Stereo offset
        EngineSimAudio::fillSilence(silenceStart, remainingFrames);
    }

    if (outSamplesRead) {
        *outSamplesRead = samplesRead;
    }

    return ESIM_SUCCESS;
}

/// ==============================================================================
/// RENDER ON DEMAND (FOR SYNCHRONOUS RENDERING WITHOUT AUDIO THREAD)
/// ==============================================================================  
EngineSimResult EngineSimRenderOnDemand(
    EngineSimHandle handle,
    float* buffer,
    int32_t frames,
    int32_t* outFramesWritten)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    if (!buffer || frames <= 0) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    EngineSimContext* ctx = getContext(handle);

    if (!ctx->engine) {
        EngineSimAudio::fillSilence(buffer, frames);
        if (outFramesWritten) *outFramesWritten = 0;
        return ESIM_SUCCESS;
    }

    if (frames * 2 > static_cast<int>(ctx->conversionBufferSize)) {
        ctx->setError("Render buffer size exceeds internal buffer");
        return ESIM_ERROR_AUDIO_BUFFER;
    }

    // Run simulation for this audio frame
    const double dt = static_cast<double>(frames) / ctx->config.sampleRate;
    ctx->simulator->startFrame(dt);
    
    int simSteps = static_cast<int>(ctx->config.simulationFrequency * dt);
    if (simSteps < 1) simSteps = 1;
    
    for (int i = 0; i < simSteps; ++i) {
        ctx->simulator->simulateStep();
    }
    
    ctx->simulator->endFrame();

    // Update statistics (same as EngineSimUpdate, needed because sync-pull
    // mode skips Update() and relies solely on RenderOnDemand)
    double throttle = ctx->throttlePosition.load(std::memory_order_relaxed);
    ctx->stats.currentRPM = units::toRpm(ctx->engine->getSpeed());
    ctx->stats.currentLoad = throttle;
    ctx->stats.exhaustFlow = ctx->simulator->getTotalExhaustFlow();
    ctx->stats.processingTimeMs = ctx->simulator->getAverageProcessingTime() * 1000.0;

    // Render and read audio
    ctx->simulator->synthesizer().renderAudioOnDemand();

    int samplesRead = ctx->simulator->readAudioOutput(
        frames,
        ctx->audioConversionBuffer
    );

    // Convert mono int16 to stereo float using shared utility with clipping (DRY)
    EngineSimAudio::convertInt16ToStereoFloatClipped(ctx->audioConversionBuffer, buffer, samplesRead);

    // Calculate actual frames with non-zero data BEFORE zeroing
    // This is critical: we must NOT count zero-filled frames as valid data
    int actualFramesWithData = 0;
    if (samplesRead > 0) {
        const int16_t* src = ctx->audioConversionBuffer;
        for (int i = 0; i < samplesRead; ++i) {
            if (src[i] != 0) {
                actualFramesWithData = i + 1;
            }
        }
    }

    // Zero-fill remainder
    if (samplesRead < frames) {
        if (samplesRead == 0) {
            // Log first underrun (happens occasionally at startup)
            ctx->logger->debug(LogMask::DIAGNOSTICS, "RenderOnDemand underrun: requested=%d got=%d", frames, samplesRead);
        }
        EngineSimAudio::fillSilence(buffer + samplesRead * 2, frames - samplesRead);
    }

    if (outFramesWritten) {
        *outFramesWritten = samplesRead;
    }

    return ESIM_SUCCESS;
}

/// ============================================================================
/// DIAGNOSTICS & TELEMETRY
/// ============================================================================

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

    EngineSimContext* ctx = getContext(handle);
    *outStats = ctx->stats;

    return ESIM_SUCCESS;
}

const char* EngineSimGetLastError(
    EngineSimHandle handle)
{
    if (!validateHandle(handle)) {
        return "Invalid handle";
    }

    EngineSimContext* ctx = getContext(handle);
    std::string error = ctx->getError();

    // Use thread-local storage for allocation-free API
    // Note: Not safe to call from multiple threads for same handle
    static thread_local std::string error_buffer;
    error_buffer = error;
    return error_buffer.c_str();
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

const char* EngineSimGetVersion(void)
{
    return "engine-sim-bridge/1.0.0";
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
    // Allow inputBufferSize up to sampleRate (GUI uses 44100, which is > 8192)
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

    EngineSimContext* ctx = getContext(handle);
    ctx->throttlePosition.store(position, std::memory_order_relaxed);

    // Use the Governor abstraction for proper closed-loop feedback
    // This ensures the Governor's safety features (full throttle at low RPM) are active
    if (ctx->engine) {
        ctx->engine->setSpeedControl(position);
    }

    return ESIM_SUCCESS;
}

// Additional controls for testing and diagnostics
EngineSimResult EngineSimSetStarterMotor(
    EngineSimHandle handle,
    int enabled)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    EngineSimContext* ctx = getContext(handle);

    ctx->simulator->m_starterMotor.m_enabled = (enabled != 0);
    return ESIM_SUCCESS;
}

// Additional controls for testing and diagnostics
EngineSimResult EngineSimSetIgnition(
    EngineSimHandle handle,
    int enabled)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    EngineSimContext* ctx = getContext(handle);

    if (ctx->simulator->getEngine()) {
        ctx->simulator->getEngine()->getIgnitionModule()->m_enabled = (enabled != 0);
        return ESIM_SUCCESS;
    }

    return ESIM_ERROR_NOT_INITIALIZED;
}

// Additional controls for testing and diagnostics
EngineSimResult EngineSimShiftGear(
    EngineSimHandle handle,
    int gear)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    EngineSimContext* ctx = getContext(handle);

    if (ctx->simulator->getTransmission()) {
        ctx->simulator->getTransmission()->changeGear(gear);
        return ESIM_SUCCESS;
    }

    return ESIM_ERROR_NOT_INITIALIZED;
}

// Additional controls for testing and diagnostics
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

    EngineSimContext* ctx = getContext(handle);

    if (ctx->simulator->getTransmission()) {
        ctx->simulator->getTransmission()->setClutchPressure(pressure);
        return ESIM_SUCCESS;
    }

    return ESIM_ERROR_NOT_INITIALIZED;
}

// Additional controls for testing and diagnostics
EngineSimResult EngineSimSetDyno(
    EngineSimHandle handle,
    int enabled)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    EngineSimContext* ctx = getContext(handle);

    ctx->simulator->m_dyno.m_enabled = (enabled != 0);
    return ESIM_SUCCESS;
}

// Additional controls for testing and diagnostics
EngineSimResult EngineSimSetDynoHold(
    EngineSimHandle handle,
    int enabled,
    double speed)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    EngineSimContext* ctx = getContext(handle);

    ctx->simulator->m_dyno.m_hold = (enabled != 0);
    ctx->simulator->m_dyno.m_rotationSpeed = speed;
    return ESIM_SUCCESS;
}

/// ============================================================================
/// @brief  Load custom impulse response data for a specific exhaust system index.
/// @param handle 
/// @param exhaustIndex 
/// @param impulseData 
/// @param sampleCount 
/// @param volume 
/// @return 
/// ============================================================================
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

    EngineSimContext* ctx = getContext(handle);

    if (!ctx->engine) {
        ctx->setError("No engine loaded. Ensure script path was provided to EngineSimCreate() before EngineSimLoadImpulseResponse().");
        return ESIM_ERROR_NOT_INITIALIZED;
    }

    // Check if we have an exhaust system at this index
    if (exhaustIndex < 0 || exhaustIndex >= ctx->engine->getExhaustSystemCount()) {
        ctx->setError("Invalid exhaust index");
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    // Load the impulse response into the synthesizer
    ctx->simulator->synthesizer().initializeImpulseResponse(
        impulseData,
        static_cast<unsigned int>(sampleCount),
        volume,
        exhaustIndex
    );

    return ESIM_SUCCESS;
}
