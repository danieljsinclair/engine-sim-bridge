#include "../include/engine_sim_bridge.h"

// Core engine-sim includes
#include "../include/piston_engine_simulator.h"
#include "../include/engine.h"
#include "../include/vehicle.h"
#include "../include/transmission.h"
#include "../include/synthesizer.h"
#include "../include/units.h"
#include "../include/impulse_response.h"
#include "../include/exhaust_system.h"
#include "../include/wav_loader.h"

// Scripting includes
#include "../scripting/include/compiler.h"

#include <string>
#include <cstring>
#include <cmath>
#include <mutex>
#include <atomic>

// ============================================================================
// INTERNAL IMPLEMENTATION STRUCTURES
// ============================================================================

struct EngineSimContext {
    // Core components
    PistonEngineSimulator* simulator;
    Engine* engine;
    Vehicle* vehicle;
    Transmission* transmission;

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

    // Statistics
    EngineSimStats stats;

    EngineSimContext()
        : simulator(nullptr)
        , engine(nullptr)
        , vehicle(nullptr)
        , transmission(nullptr)
        , throttlePosition(0.0)
        , audioConversionBuffer(nullptr)
        , conversionBufferSize(0)
#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
        , compiler(nullptr)
#endif
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

        // Note: Engine, Vehicle, Transmission are owned by simulator
        // Don't delete them explicitly

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
// IMPULSE RESPONSE LOADING
// ============================================================================

/**
 * Load impulse responses for all exhaust systems in the engine.
 * Delegates to WavLoader module for actual WAV file parsing (SRP).
 */
static bool loadImpulseResponses(
    EngineSimContext* ctx,
    const std::string& assetBasePath)
{
    if (!ctx->engine) {
        return false;
    }

    const int exhaustCount = ctx->engine->getExhaustSystemCount();
    std::cerr << "DEBUG BRIDGE: Engine has " << exhaustCount << " exhaust systems\n";
    for (int i = 0; i < exhaustCount; ++i) {
        ExhaustSystem* exhaust = ctx->engine->getExhaustSystem(i);
        if (!exhaust) continue;

        ImpulseResponse* impulse = exhaust->getImpulseResponse();
        if (!impulse) continue;

        std::string filename = impulse->getFilename();
        std::cerr << "DEBUG BRIDGE: IR filename='" << filename << "'\n";
        if (filename.empty()) {
            continue;  // No impulse response specified
        }

        // Construct full path
        std::string fullPath;
        if (filename[0] == '/' || (filename.length() > 1 && filename[1] == ':')) {
            // Absolute path
            fullPath = filename;
        } else {
            // Relative path - combine with asset base path
            fullPath = assetBasePath + "/" + filename;
            std::cerr << "DEBUG BRIDGE: Loading impulse: fullPath=" << fullPath << "\n";
        }

        // Load WAV file using WavLoader module
        WavLoader::Result wavResult = WavLoader::load(fullPath);
        if (!wavResult.valid) {
            ctx->setError("Failed to load impulse response: " + fullPath);
            // Continue with other exhaust systems rather than failing completely
            continue;
        }

        // Initialize synthesizer with impulse response
        ctx->simulator->synthesizer().initializeImpulseResponse(
            wavResult.getData(),
            static_cast<unsigned int>(wavResult.getSampleCount()),
            static_cast<float>(impulse->getVolume()),
            i
        );
    }

    return true;
}

// ============================================================================
// LIFECYCLE FUNCTIONS
// ============================================================================

EngineSimResult EngineSimCreate(
    const EngineSimConfig* config,
    EngineSimHandle* outHandle)
{
    if (!outHandle) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    // Allocate context
    EngineSimContext* ctx = new (std::nothrow) EngineSimContext();
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

    // Create simulator
    ctx->simulator = new (std::nothrow) PistonEngineSimulator();
    if (!ctx->simulator) {
        ctx->setError("Failed to allocate PistonEngineSimulator");
        delete ctx;
        return ESIM_ERROR_INVALID_HANDLE;
    }

    // Initialize simulator parameters
    Simulator::Parameters simParams;
    simParams.systemType = Simulator::SystemType::NsvOptimized;

    ctx->simulator->initialize(simParams);
    ctx->simulator->setSimulationFrequency(ctx->config.simulationFrequency);
    ctx->simulator->setFluidSimulationSteps(ctx->config.fluidSimulationSteps);
    ctx->simulator->setTargetSynthesizerLatency(ctx->config.targetSynthesizerLatency);

    // Configure synthesizer
    Synthesizer::Parameters synthParams;
    synthParams.inputChannelCount = 2; // Stereo
    synthParams.inputBufferSize = ctx->config.inputBufferSize;
    synthParams.audioBufferSize = ctx->config.audioBufferSize;
    synthParams.inputSampleRate = static_cast<float>(ctx->config.simulationFrequency);
    synthParams.audioSampleRate = static_cast<float>(ctx->config.sampleRate);

    synthParams.initialAudioParameters.volume = ctx->config.volume;
    synthParams.initialAudioParameters.convolution = ctx->config.convolutionLevel;
    synthParams.initialAudioParameters.airNoise = ctx->config.airNoise;

    ctx->simulator->synthesizer().initialize(synthParams);

    // Allocate audio conversion buffer (stereo)
    ctx->conversionBufferSize = 4096 * 2; // Max frames * 2 channels
    ctx->audioConversionBuffer = new (std::nothrow) int16_t[ctx->conversionBufferSize];
    if (!ctx->audioConversionBuffer) {
        ctx->setError("Failed to allocate audio conversion buffer");
        delete ctx;
        return ESIM_ERROR_AUDIO_BUFFER;
    }

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
    // Create compiler for script loading
    ctx->compiler = new (std::nothrow) es_script::Compiler();
    if (!ctx->compiler) {
        ctx->setError("Failed to allocate script compiler");
        delete ctx;
        return ESIM_ERROR_SCRIPT_COMPILATION;
    }

    ctx->compiler->initialize();
#endif

    *outHandle = ctx;
    return ESIM_SUCCESS;
}

#ifdef ATG_ENGINE_SIM_PIRANHA_ENABLED
EngineSimResult EngineSimLoadScript(
    EngineSimHandle handle,
    const char* scriptPath,
    const char* assetBasePath)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    if (!scriptPath || strlen(scriptPath) == 0) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    EngineSimContext* ctx = getContext(handle);

    if (!ctx->compiler) {
        ctx->setError("Compiler not initialized. Piranha support not available.");
        return ESIM_ERROR_SCRIPT_COMPILATION;
    }

    // Compile the script
    if (!ctx->compiler->compile(scriptPath)) {
        ctx->setError("Failed to compile script: " + std::string(scriptPath));
        return ESIM_ERROR_SCRIPT_COMPILATION;
    }

    // Execute the compiled script
    es_script::Compiler::Output output = ctx->compiler->execute();

    // Create default vehicle if none specified
    Vehicle* vehicle = output.vehicle;
    if (vehicle == nullptr) {
        Vehicle::Parameters vehParams;
        vehParams.mass = units::mass(1597, units::kg);
        vehParams.diffRatio = 3.42;
        vehParams.tireRadius = units::distance(10, units::inch);
        vehParams.dragCoefficient = 0.25;
        vehParams.crossSectionArea = units::distance(6.0, units::foot) * units::distance(6.0, units::foot);
        vehParams.rollingResistance = 2000.0;
        vehicle = new Vehicle;
        vehicle->initialize(vehParams);
    }

    // Create default transmission if none specified
    Transmission* transmission = output.transmission;
    if (transmission == nullptr) {
        const double gearRatios[] = { 2.97, 2.07, 1.43, 1.00, 0.84, 0.56 };
        Transmission::Parameters tParams;
        tParams.GearCount = 6;
        tParams.GearRatios = gearRatios;
        tParams.MaxClutchTorque = units::torque(1000.0, units::ft_lb);
        transmission = new Transmission;
        transmission->initialize(tParams);
    }

    // Store references
    ctx->engine = output.engine;
    ctx->vehicle = vehicle;
    ctx->transmission = transmission;

    // Load the simulation
    if (ctx->engine) {
        ctx->simulator->loadSimulation(ctx->engine, ctx->vehicle, ctx->transmission);
    } else {
        ctx->setError("Script did not create an engine");
        return ESIM_ERROR_LOAD_FAILED;
    }

    // Determine asset base path
    std::string resolvedAssetPath;
    if (assetBasePath != nullptr && strlen(assetBasePath) > 0) {
        resolvedAssetPath = assetBasePath;
        std::cerr << "DEBUG BRIDGE: Using provided assetBasePath: " << resolvedAssetPath << "\n";
    } else {
        // Default: derive from script path (go up to assets/ directory)
        std::string scriptPathStr(scriptPath);
        size_t assetsPos = scriptPathStr.find("/assets/");
        if (assetsPos != std::string::npos) {
            // Extract path up to and including /assets/
            resolvedAssetPath = scriptPathStr.substr(0, assetsPos + 8); // +8 for "/assets/"
        } else {
            // Fallback: use script's directory
            size_t lastSlash = scriptPathStr.find_last_of('/');
            if (lastSlash != std::string::npos) {
                resolvedAssetPath = scriptPathStr.substr(0, lastSlash);
            } else {
                resolvedAssetPath = ".";  // Current directory
            }
        }
    }

    // Load impulse responses
    loadImpulseResponses(ctx, resolvedAssetPath);

    return ESIM_SUCCESS;
}
#endif

EngineSimResult EngineSimStartAudioThread(
    EngineSimHandle handle)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    EngineSimContext* ctx = getContext(handle);

    if (!ctx->engine) {
        ctx->setError("No engine loaded. Call EngineSimLoadScript first.");
        return ESIM_ERROR_NOT_INITIALIZED;
    }

    ctx->simulator->startAudioRenderingThread();

    return ESIM_SUCCESS;
}

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

    EngineSimContext* ctx = getContext(handle);
    ctx->throttlePosition.store(position, std::memory_order_relaxed);

    // Use the Governor abstraction for proper closed-loop feedback
    // This ensures the Governor's safety features (full throttle at low RPM) are active
    if (ctx->engine) {
        ctx->engine->setSpeedControl(position);
    }

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

    if (!ctx->engine) {
        // No engine loaded - output silence
        std::memset(buffer, 0, frames * 2 * sizeof(float));
        if (outSamplesWritten) {
            *outSamplesWritten = 0;
        }
        return ESIM_SUCCESS;
    }

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
            std::cerr << "DEBUG BRIDGE RENDER: UNDERRUN at frame " << renderCount
                      << ": requested=" << frames << " got=" << samplesRead
                      << " missing=" << (frames - samplesRead) << "\n";
            lastUnderrunReport = renderCount;
        }
    }
    if (renderCount < 5 || samplesRead == 0) {
        std::cerr << "DEBUG BRIDGE RENDER: samplesRead=" << samplesRead << " frames=" << frames << "\n";
    }
    renderCount++;

    // Convert mono int16 to stereo float32 [-1.0, 1.0]
    // This is the CRITICAL PATH - must be FAST and allocation-free
    // readAudioOutput returns MONO samples, so we duplicate each sample to L and R channels
    constexpr float scale = 1.0f / 32768.0f;

    for (int i = 0; i < samplesRead; ++i) {
        const float sample = static_cast<float>(ctx->audioConversionBuffer[i]) * scale;
        buffer[i * 2] = sample;     // Left channel
        buffer[i * 2 + 1] = sample; // Right channel
    }

    if (outSamplesWritten) {
        *outSamplesWritten = samplesRead;
    }

    return ESIM_SUCCESS;
}

// ============================================================================
// DIAGNOSTICS & TELEMETRY
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
    if (config->inputBufferSize < 64 || config->inputBufferSize > 8192) {
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

EngineSimResult EngineSimSetStarterMotor(
    EngineSimHandle handle,
    int enabled)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    EngineSimContext* ctx = getContext(handle);

    if (ctx->simulator) {
        ctx->simulator->m_starterMotor.m_enabled = (enabled != 0);
        return ESIM_SUCCESS;
    }

    return ESIM_ERROR_NOT_INITIALIZED;
}

EngineSimResult EngineSimSetIgnition(
    EngineSimHandle handle,
    int enabled)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    EngineSimContext* ctx = getContext(handle);

    if (ctx->simulator && ctx->simulator->getEngine()) {
        ctx->simulator->getEngine()->getIgnitionModule()->m_enabled = (enabled != 0);
        return ESIM_SUCCESS;
    }

    return ESIM_ERROR_NOT_INITIALIZED;
}

EngineSimResult EngineSimShiftGear(
    EngineSimHandle handle,
    int gear)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    EngineSimContext* ctx = getContext(handle);

    if (ctx->simulator && ctx->simulator->getTransmission()) {
        ctx->simulator->getTransmission()->changeGear(gear);
        return ESIM_SUCCESS;
    }

    return ESIM_ERROR_NOT_INITIALIZED;
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

    EngineSimContext* ctx = getContext(handle);

    if (ctx->simulator && ctx->simulator->getTransmission()) {
        ctx->simulator->getTransmission()->setClutchPressure(pressure);
        return ESIM_SUCCESS;
    }

    return ESIM_ERROR_NOT_INITIALIZED;
}

EngineSimResult EngineSimSetDyno(
    EngineSimHandle handle,
    int enabled)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    EngineSimContext* ctx = getContext(handle);

    if (ctx->simulator) {
        ctx->simulator->m_dyno.m_enabled = (enabled != 0);
        return ESIM_SUCCESS;
    }

    return ESIM_ERROR_NOT_INITIALIZED;
}

EngineSimResult EngineSimSetDynoHold(
    EngineSimHandle handle,
    int enabled,
    double speed)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    EngineSimContext* ctx = getContext(handle);

    if (ctx->simulator) {
        ctx->simulator->m_dyno.m_hold = (enabled != 0);
        ctx->simulator->m_dyno.m_rotationSpeed = speed;
        return ESIM_SUCCESS;
    }

    return ESIM_ERROR_NOT_INITIALIZED;
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

    EngineSimContext* ctx = getContext(handle);

    if (!ctx->engine) {
        ctx->setError("No engine loaded. Call EngineSimLoadScript first.");
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
