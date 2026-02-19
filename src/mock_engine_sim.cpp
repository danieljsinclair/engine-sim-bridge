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

// ============================================================================
// RING BUFFER - Matches engine-sim's RingBuffer<T> semantics
// ============================================================================

template<typename T>
class MockRingBuffer {
public:
    MockRingBuffer() : m_data(nullptr), m_capacity(0), m_readPos(0), m_writePos(0), m_size(0) {}
    ~MockRingBuffer() { destroy(); }

    void initialize(int capacity) {
        destroy();
        m_capacity = capacity;
        m_data = new T[capacity]();
        m_readPos = 0;
        m_writePos = 0;
        m_size = 0;
    }

    void destroy() {
        delete[] m_data;
        m_data = nullptr;
        m_capacity = 0;
        m_readPos = 0;
        m_writePos = 0;
        m_size = 0;
    }

    void write(T value) {
        if (m_size < m_capacity) {
            m_data[m_writePos] = value;
            m_writePos = (m_writePos + 1) % m_capacity;
            ++m_size;
        }
    }

    // Read n items into dest without removing them (matches engine-sim's read())
    void read(int n, T* dest) const {
        int pos = m_readPos;
        for (int i = 0; i < n && i < m_size; ++i) {
            dest[i] = m_data[pos];
            pos = (pos + 1) % m_capacity;
        }
    }

    // Read and remove n items (matches engine-sim's readAndRemove())
    void readAndRemove(int n, T* dest) {
        int count = std::min(n, m_size);
        for (int i = 0; i < count; ++i) {
            dest[i] = m_data[m_readPos];
            m_readPos = (m_readPos + 1) % m_capacity;
        }
        m_size -= count;
    }

    // Remove n items from the beginning (matches engine-sim's removeBeginning())
    void removeBeginning(int n) {
        int count = std::min(n, m_size);
        m_readPos = (m_readPos + count) % m_capacity;
        m_size -= count;
    }

    int size() const { return m_size; }
    int capacity() const { return m_capacity; }

    size_t writeIndex() const { return m_writePos; }

private:
    T* m_data;
    int m_capacity;
    int m_readPos;
    int m_writePos;
    int m_size;
};

// ============================================================================
// MOCK SYNTHESIZER - Replicates engine-sim Synthesizer threading pattern
//
// Real engine-sim flow:
//   Main thread: startFrame() → simulateStep() loop [writeToSynthesizer()
//     → writeInput()] → endFrame() [endInputBlock() → cv0.notify_one()]
//   Audio thread: cv0.wait() → read input → process → write to audioBuffer
//   CLI: readAudioOutput() reads from audioBuffer under lock
//
// Mock replicates this EXACTLY, but generates sine waves instead of
// processing engine exhaust data through convolution filters.
// ============================================================================

class MockSynthesizer {
public:
    MockSynthesizer()
        : m_run(true)
        , m_thread(nullptr)
        , m_inputSamplesRead(0)
        , m_processed(true)
        , m_inputSampleRate(10000.0)
        , m_audioSampleRate(44100.0)
        , m_inputBufferSize(1024)
        , m_audioBufferSize(2000)
        , m_inputChannelCount(1)
        , m_targetBufferLevel(2000)
        , m_latency(0)
        , m_resampleAccumulator(0.0)
        , m_sinePhase(0.0)
    {}

    ~MockSynthesizer() {
        if (m_thread != nullptr) {
            endAudioRenderingThread();
        }
    }

    void initialize(int inputBufferSize, int audioBufferSize, int inputChannelCount,
                    double inputSampleRate, double audioSampleRate) {
        m_inputBufferSize = inputBufferSize;
        m_audioBufferSize = audioBufferSize;
        m_inputChannelCount = inputChannelCount;
        m_inputSampleRate = inputSampleRate;
        m_audioSampleRate = audioSampleRate;
        m_resampleAccumulator = 0.0;
        m_processed = true;
        m_inputSamplesRead = 0;

        // Dynamic target buffer level based on exhaust system count
        // Matches real Synthesizer's buffer sizing logic
        // Base target: 20000 samples * exhaust system count (increased from 10000 to handle CLI consumption rate)
        // CLI reads 735 samples per iteration at 60Hz, so we need enough buffer to avoid oscillation
        // Increased to 20000 to prevent the buffer from oscillating between full and empty
        m_targetBufferLevel = 20000 * m_inputChannelCount;
        std::cout << "[MOCK SYNTH] inputChannelCount: " << m_inputChannelCount << std::endl;

        // Multi-exhaust engines need extra buffer for computational load
        // This matches real Synthesizer logic
        // Adds 20000 samples per additional exhaust system beyond the first (increased from 10000)
        if (m_inputChannelCount > 1) {
            int extraBuffer = (m_inputChannelCount - 1) * 20000;
            m_targetBufferLevel += extraBuffer;
            std::cout << "[MOCK SYNTH] Multi-exhaust engine detected - added " << extraBuffer
                      << " samples (total: " << m_targetBufferLevel << ")\n";
        }

        // Input channel ring buffer - matches real synthesizer
        m_inputChannel.initialize(inputBufferSize);

        // Audio output buffer - matches real synthesizer (int16 mono)
        m_audioBuffer.initialize(audioBufferSize);

        // Transfer buffer for reading input data
        m_transferBuffer.resize(inputBufferSize, 0.0f);

        // Pre-fill audio buffer with a SMALL amount to prevent initial underruns.
        // CRITICAL: Must be LESS than targetBufferLevel so audio thread can wake up.
        // The wait condition is: m_audioBuffer.size() < targetBufferLevel
        // If we pre-fill to targetBufferLevel, the audio thread will deadlock.
        const int preFillAmount = std::min(500, m_audioBufferSize);  // Small pre-fill
        for (int i = 0; i < preFillAmount; ++i) {
            m_audioBuffer.write(0);
        }
    }

    void startAudioRenderingThread() {
        m_run = true;
        m_thread = new std::thread(&MockSynthesizer::audioRenderingThread, this);
    }

    void endAudioRenderingThread() {
        if (m_thread != nullptr) {
            m_run = false;
            endInputBlock();  // Wake up the audio thread so it can exit

            m_thread->join();
            delete m_thread;
            m_thread = nullptr;
        }
    }

    // Called from main thread during simulateStep() - feeds one sample per step
    // Replicates Synthesizer::writeInput() with proper upsampling from sim rate to audio rate
    void writeInput(float sample) {
        // Resample from simulation rate (10kHz) to audio rate (44.1kHz)
        // Each simulation step should produce ~4.41 audio-rate samples on average
        // Use fractional accumulator for jitter-free sample rate conversion
        m_resampleAccumulator += m_audioSampleRate / m_inputSampleRate;

        // Write one audio-rate sample for each whole unit accumulated
        int samplesToWrite = static_cast<int>(m_resampleAccumulator);
        m_resampleAccumulator -= samplesToWrite;

        for (int i = 0; i < samplesToWrite; ++i) {
            m_inputChannel.write(sample);
        }
    }

    // Called from main thread at end of frame - signals audio thread
    // Replicates Synthesizer::endInputBlock()
    void endInputBlock() {
        std::unique_lock<std::mutex> lk(m_lock0);  // FIX: Use m_lock0, not m_inputLock (matches real synthesizer)

        // Remove samples that the audio thread already processed
        m_inputChannel.removeBeginning(m_inputSamplesRead);

        if (m_inputChannel.size() > 0) {
            m_latency = m_inputChannel.size();
        }

        m_inputSamplesRead = 0;
        m_processed = false;

        lk.unlock();
        m_cv0.notify_one();
    }

    // Replicates Synthesizer::waitProcessed()
    void waitProcessed() {
        std::unique_lock<std::mutex> lk(m_lock0);
        m_cv0.wait(lk, [this] { return m_processed; });
    }

    // Read audio output - called from CLI thread
    // Replicates Synthesizer::readAudioOutput()
    int readAudioOutput(int samples, int16_t* buffer) {
        std::lock_guard<std::mutex> lock(m_lock0);

        const int available = m_audioBuffer.size();
        if (available >= samples) {
            m_audioBuffer.readAndRemove(samples, buffer);
        } else {
            m_audioBuffer.readAndRemove(available, buffer);
            std::memset(
                buffer + available,
                0,
                sizeof(int16_t) * (static_cast<size_t>(samples) - available));
        }

        return std::min(samples, available);
    }

    void setInputSampleRate(double rate) {
        if (rate != m_inputSampleRate) {
            std::lock_guard<std::mutex> lock(m_lock0);
            m_inputSampleRate = rate;
        }
    }

    double getLatency() const {
        return static_cast<double>(m_latency) / m_audioSampleRate;
    }

    // Synchronous audio render: process all pending input without cv0/audio thread.
    // Used for WAV-only export mode where no audio thread is running.
    void renderAudioSync() {
        std::lock_guard<std::mutex> lock(m_lock0);

        const int available = m_inputChannel.size();
        if (available <= 0) return;

        const int n = std::min(available, static_cast<int>(m_transferBuffer.size()));
        // CRITICAL: use readAndRemove (not read) to drain the input buffer.
        // read() is non-destructive and leaves data in the ring buffer,
        // causing it to fill to capacity and corrupt at wrap boundaries.
        m_inputChannel.readAndRemove(n, m_transferBuffer.data());

        for (int i = 0; i < n; ++i) {
            float sample = m_transferBuffer[i];
            sample = std::max(-1.0f, std::min(1.0f, sample));
            int16_t intSample = static_cast<int16_t>(sample * 32767.0f);
            m_audioBuffer.write(intSample);
        }

        m_processed = true;
    }

    void destroy() {
        m_audioBuffer.destroy();
        m_inputChannel.destroy();
        m_transferBuffer.clear();
    }

private:
    // Audio rendering thread - EXACT replica of Synthesizer::audioRenderingThread()
    void audioRenderingThread() {
        while (m_run) {
            renderAudio();
        }
    }

    // EXACT replica of Synthesizer::renderAudio() cv0.wait() pattern
    void renderAudio() {
        std::unique_lock<std::mutex> lk0(m_lock0);

        // This is the CRITICAL cv0.wait() pattern from synthesizer.cpp:239-244
        // Use dynamic target buffer level for multi-exhaust engines
        // The larger m_audioBufferSize (96000) is the ring buffer capacity, not the target level.
        const int targetBufferLevel = m_targetBufferLevel;
        m_cv0.wait(lk0, [this, targetBufferLevel] {
            const bool inputAvailable =
                m_inputChannel.size() > 0
                && m_audioBuffer.size() < targetBufferLevel;
            return !m_run || (inputAvailable && !m_processed);
        });

        if (!m_run) return;

        // Calculate how many samples to process (matches real synthesizer)
        const int n = std::min(
            std::max(0, targetBufferLevel - m_audioBuffer.size()),
            m_inputChannel.size());

        // Read from input channel into transfer buffer
        m_inputChannel.read(n, m_transferBuffer.data());

        m_inputSamplesRead = n;
        m_processed = true;

        lk0.unlock();

        // Write processed samples to audio buffer
        // In real engine-sim, this is where convolution/filtering happens
        // For mock, we just pass through the sine wave samples
        for (int i = 0; i < n; ++i) {
            // Convert float [-1.0, 1.0] to int16 [-32768, 32767]
            float sample = m_transferBuffer[i];
            sample = std::max(-1.0f, std::min(1.0f, sample));
            int16_t intSample = static_cast<int16_t>(sample * 32767.0f);
            m_audioBuffer.write(intSample);
        }

        m_cv0.notify_one();
    }

    // Threading primitives - EXACT match to real Synthesizer
    std::mutex m_lock0;              // Main synchronization mutex
    std::mutex m_inputLock;          // Input block mutex
    std::condition_variable m_cv0;   // THE cv0 condition variable
    bool m_processed;                // Frame processed flag
    bool m_run;                      // Thread run flag
    std::thread* m_thread;

    // Buffers - match real Synthesizer types
    MockRingBuffer<float> m_inputChannel;       // Input samples from simulation
    MockRingBuffer<int16_t> m_audioBuffer;      // Output audio buffer (mono int16)
    std::vector<float> m_transferBuffer;        // Transfer buffer for reading

    int m_inputSamplesRead;
    int m_latency;

    // Sample rate conversion
    double m_inputSampleRate;
    double m_audioSampleRate;
    int m_inputBufferSize;
    int m_audioBufferSize;
    int m_inputChannelCount;  // Number of exhaust systems (for dynamic buffer sizing)
    int m_targetBufferLevel;    // Dynamic target based on exhaust system count
    double m_resampleAccumulator;    // Fractional accumulator for sim→audio resampling

    // Sine wave phase (used by the simulation step, not the audio thread)
    double m_sinePhase;

public:
    // Expose phase for simulation step to use
    double& sinePhase() { return m_sinePhase; }
};

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

    // RPM controller parameters
    double kp;
    double ki;
    double integral;
    double rpmControllerLastError;
    double lastThrottle;
    bool firstUpdate;

    // Starter motor parameters
    const double minSustainedRPM = 550.0;
    const double starterCutoffRPM = 600.0;

    // Synthesizer - replicates real engine-sim's Synthesizer
    MockSynthesizer synthesizer;

    // Simulation state (matches Simulator)
    int m_simulationSteps;
    int m_currentIteration;
    double m_simulationSpeed;
    double m_physicsProcessingTime;
    std::chrono::steady_clock::time_point m_simulationStart;

    // Sine wave state (used during simulateStep)
    double sinePhase;

    // Audio conversion buffer (int16 → float, matches real bridge)
    std::vector<int16_t> audioConversionBuffer;

    // Error handling
    std::string lastErrorMsg;
    std::mutex errorMutex;

    // Statistics
    EngineSimStats stats;

    // Warmup phase
    const double warmupDuration = 2.0;
    bool inWarmupPhase;
    std::chrono::steady_clock::time_point warmupStartTime;

    MockEngineSimContext()
        : throttlePosition(0.0)
        , targetRPM(0.0)
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
        , throttleSmoothingRatio(0.8)
        , kp(0.002)
        , ki(0.0001)
        , integral(0.0)
        , rpmControllerLastError(0.0)
        , lastThrottle(0.0)
        , firstUpdate(true)
        , m_simulationSteps(0)
        , m_currentIteration(0)
        , m_simulationSpeed(1.0)
        , m_physicsProcessingTime(0.0)
        , sinePhase(0.0)
        , inWarmupPhase(false)
        , warmupStartTime(std::chrono::steady_clock::now()) {

        std::memset(&config, 0, sizeof(config));
        std::memset(&stats, 0, sizeof(stats));
    }

    ~MockEngineSimContext() {
        synthesizer.endAudioRenderingThread();
        synthesizer.destroy();
    }

    void initializeSynthesizer() {
        // Mock synthesizer uses 1 channel (single exhaust system)
        // This matches 4-cylinder engines typically
        const int inputChannelCount = 1;

        synthesizer.initialize(
            config.inputBufferSize,
            config.audioBufferSize,
            inputChannelCount,
            config.simulationFrequency,  // Input sample rate = simulation frequency
            config.sampleRate            // Audio sample rate from config
        );

        // Allocate audio conversion buffer (matches real bridge pattern)
        audioConversionBuffer.resize(config.sampleRate, 0);
    }

    void setError(const std::string& msg) {
        std::lock_guard<std::mutex> lock(errorMutex);
        lastErrorMsg = msg;
    }

    std::string getError() const {
        std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(errorMutex));
        return lastErrorMsg;
    }

    // ========================================================================
    // THROTTLE SMOOTHING - Adaptive smoothing based on throttle level
    // ========================================================================
    double applyThrottleSmoothing(double targetThrottle, double deltaTime) {
        double smoothingRatio;
        if (targetThrottle < 0.1) {
            smoothingRatio = 0.9;
        } else if (targetThrottle < 0.3) {
            smoothingRatio = 0.8;
        } else {
            smoothingRatio = 0.7;
        }

        throttleSmoothingRatio = throttleSmoothingRatio * 0.95 + smoothingRatio * 0.05;

        double newThrottle = targetThrottle * throttleSmoothingRatio +
                           smoothedThrottle * (1.0 - throttleSmoothingRatio);

        smoothedThrottle = newThrottle;
        return newThrottle;
    }

    // ========================================================================
    // ENGINE STATE UPDATE - RPM physics model
    // ========================================================================
    void updateEngineState(double deltaTime) {
        if (ignitionEnabled == 0) {
            currentRPM = std::max(0.0, currentRPM.load() - deltaTime * 500.0);
            targetRPM = 0.0;
            return;
        }

        if (currentRPM < 100.0 && ignitionEnabled != 0) {
            currentRPM = 100.0;
        }

        double targetThrottle = throttlePosition.load(std::memory_order_relaxed);

        // Handle warmup phase
        if (inWarmupPhase) {
            auto currentTime = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
                currentTime - warmupStartTime
            ).count();

            if (elapsed >= warmupDuration || currentRPM > minSustainedRPM) {
                inWarmupPhase = false;
                targetThrottle = std::max(targetThrottle, 0.2);
            } else {
                double throttleProgress = elapsed / warmupDuration;
                targetThrottle = throttleProgress * 0.3;
                targetThrottle = applyThrottleSmoothing(targetThrottle, deltaTime);
                throttlePosition.store(targetThrottle, std::memory_order_relaxed);
            }
        }

        double smoothed = applyThrottleSmoothing(targetThrottle, deltaTime);

        double targetRPMFromThrottle;
        double extTargetRPM = targetRPM.load(std::memory_order_relaxed);
        if (extTargetRPM > 0) {
            targetRPMFromThrottle = extTargetRPM;
            smoothed = 0.05;
        } else {
            targetRPMFromThrottle = 800.0 + smoothed * 5200.0;

            if (!inWarmupPhase && currentRPM < 800.0) {
                targetRPMFromThrottle = std::max(targetRPMFromThrottle, 1000.0);
            }

            if (ignitionEnabled != 0 && currentRPM < 1500.0) {
                smoothed = std::max(smoothed, 0.15);
            }
        }

        // Starter motor effect
        if (starterMotorEnabled != 0 && currentRPM < starterCutoffRPM) {
            targetRPMFromThrottle = std::min(targetRPMFromThrottle, 1000.0);
            targetRPMFromThrottle += deltaTime * 500.0;
        }

        // Smooth RPM transitions
        double rpmChangeRate = 5000.0;
        double curRPM = currentRPM.load();
        if (targetRPMFromThrottle > curRPM) {
            curRPM = std::min(targetRPMFromThrottle,
                             curRPM + deltaTime * rpmChangeRate);
        } else {
            curRPM = std::max(targetRPMFromThrottle,
                             curRPM - deltaTime * rpmChangeRate * 0.3);
        }

        if (curRPM < 0.0) curRPM = 0.0;
        currentRPM.store(curRPM, std::memory_order_relaxed);

        // Update statistics
        currentLoad = smoothed;
        stats.currentRPM = curRPM;
        stats.currentLoad = smoothed;
        stats.exhaustFlow = smoothed * 0.1;
        stats.manifoldPressure = 101325 * (1.0 + smoothed * 0.3);
        stats.processingTimeMs = 0.5;

        // Auto-disable starter motor
        if (starterMotorEnabled != 0 && curRPM > starterCutoffRPM) {
            starterMotorEnabled = 0;
        }
    }

    // ========================================================================
    // SIMULATION CYCLE - Replicates Simulator::startFrame/simulateStep/endFrame
    // ========================================================================

    // Matches Simulator::startFrame(dt)
    void startFrame(double dt) {
        m_simulationStart = std::chrono::steady_clock::now();
        m_currentIteration = 0;

        synthesizer.setInputSampleRate(config.simulationFrequency * m_simulationSpeed);

        const double timestep = 1.0 / config.simulationFrequency;
        m_simulationSteps = static_cast<int>(std::round((dt * m_simulationSpeed) / timestep));

        // Latency-based step adjustment (matches real simulator)
        const double targetLatency = config.targetSynthesizerLatency;
        if (synthesizer.getLatency() < targetLatency) {
            m_simulationSteps = static_cast<int>((m_simulationSteps + 1) * 1.1);
        } else if (synthesizer.getLatency() > targetLatency) {
            m_simulationSteps = static_cast<int>((m_simulationSteps - 1) * 0.9);
            if (m_simulationSteps < 0) {
                m_simulationSteps = 0;
            }
        }
    }

    // Matches Simulator::simulateStep()
    bool simulateStep() {
        if (m_currentIteration >= m_simulationSteps) {
            // Frame complete - update timing
            auto s1 = std::chrono::steady_clock::now();
            const long long lastFrame =
                std::chrono::duration_cast<std::chrono::microseconds>(s1 - m_simulationStart).count();
            m_physicsProcessingTime = m_physicsProcessingTime * 0.98 + 0.02 * lastFrame;
            return false;
        }

        const double timestep = 1.0 / config.simulationFrequency;

        // Update engine state at simulation rate
        updateEngineState(timestep);

        // Generate sine wave sample and write to synthesizer
        // This is the mock equivalent of writeToSynthesizer()
        writeToSynthesizer();

        ++m_currentIteration;
        return true;
    }

    // Matches PistonEngineSimulator::writeToSynthesizer()
    void writeToSynthesizer() {
        double curRPM = currentRPM.load(std::memory_order_relaxed);

        if (ignitionEnabled == 0 || curRPM < 1.0) {
            // No ignition or stalled - write silence
            synthesizer.writeInput(0.0f);
            return;
        }

        // Map RPM to frequency: 600 RPM → 100Hz, 6000 RPM → 1000Hz
        double safeRPM = std::max(curRPM, 100.0);
        double frequency = (safeRPM / 600.0) * 100.0;

        // Phase increment at simulation rate
        double phaseIncrement = 2.0 * M_PI * frequency / config.simulationFrequency;

        // Generate phase-continuous sine sample
        float sample = static_cast<float>(std::sin(sinePhase)) * config.volume;

        // Advance phase
        sinePhase += phaseIncrement;
        if (sinePhase > 2.0 * M_PI) {
            sinePhase -= 2.0 * M_PI;
        }

        // Write to synthesizer (matches real writeToSynthesizer → writeInput flow)
        synthesizer.writeInput(sample);
    }

    // Matches Simulator::endFrame()
    void endFrame() {
        synthesizer.endInputBlock();
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
    config->audioBufferSize = 96000;
    config->simulationFrequency = 10000;
    config->fluidSimulationSteps = 8;
    config->targetSynthesizerLatency = 0.05;
    config->volume = 0.5f;
    config->convolutionLevel = 1.0f;
    config->airNoise = 1.0f;
}

// ============================================================================
// C API FUNCTIONS - extern "C" linkage
// Prefixed with Mock_ to allow runtime dispatch alongside real implementation
// ============================================================================
extern "C" {

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

    MockEngineSimContext* ctx = new (std::nothrow) MockEngineSimContext();
    if (!ctx) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    if (config) {
        ctx->config = *config;
    } else {
        setDefaultConfig(&ctx->config);
    }

    EngineSimResult validateResult = EngineSimValidateConfig(&ctx->config);
    if (validateResult != ESIM_SUCCESS) {
        delete ctx;
        return validateResult;
    }

    // Initialize synthesizer with config
    ctx->initializeSynthesizer();

    *outHandle = ctx;
    return ESIM_SUCCESS;
}

EngineSimResult EngineSimLoadScript(
    EngineSimHandle handle,
    const char* scriptPath,
    const char* assetBasePath)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    MockEngineSimContext* ctx = getContext(handle);

    ctx->warmupStartTime = std::chrono::steady_clock::now();
    ctx->currentRPM = 0.0;
    ctx->throttlePosition = 0.0;
    ctx->smoothedThrottle = 0.0;
    ctx->sinePhase = 0.0;
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
    ctx->synthesizer.startAudioRenderingThread();

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

    // EXACT replica of real bridge EngineSimUpdate():
    //   startFrame(dt) → while(simulateStep()) {} → endFrame()
    ctx->startFrame(deltaTime);

    while (ctx->simulateStep()) {
        // Process all simulation steps for this frame
    }

    ctx->endFrame();

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

    // Synchronous mode: process any pending input first, then read output.
    // This is the key difference from EngineSimReadAudioBuffer - we don't
    // rely on the audio thread to process input; we do it inline.
    ctx->synthesizer.renderAudioSync();

    int32_t samplesRead = ctx->synthesizer.readAudioOutput(
        frames, ctx->audioConversionBuffer.data());

    // Convert int16 → float stereo (matches real bridge pattern exactly)
    for (int32_t i = 0; i < samplesRead; ++i) {
        float sample = static_cast<float>(ctx->audioConversionBuffer[i]) / 32768.0f;
        buffer[i * 2] = sample;
        buffer[i * 2 + 1] = sample;
    }

    if (outSamplesWritten) {
        *outSamplesWritten = samplesRead;
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

    // Read from synthesizer's audio buffer (matches real bridge EXACTLY)
    // Real bridge: ctx->simulator->readAudioOutput(frames, ctx->audioConversionBuffer)
    int32_t samplesRead = ctx->synthesizer.readAudioOutput(
        frames, ctx->audioConversionBuffer.data());

    // Convert int16 → float stereo (matches real bridge pattern)
    for (int32_t i = 0; i < samplesRead; ++i) {
        float sample = static_cast<float>(ctx->audioConversionBuffer[i]) / 32768.0f;
        buffer[i * 2] = sample;
        buffer[i * 2 + 1] = sample;
    }

    if (outSamplesRead) {
        *outSamplesRead = samplesRead;
    }

    // Zero-fill remaining frames to prevent crackling
    if (samplesRead < frames) {
        int32_t remaining = frames - samplesRead;
        std::memset(buffer + samplesRead * 2, 0, remaining * 2 * sizeof(float));
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

    static thread_local std::string error_buffer;
    error_buffer = error;
    return error_buffer.c_str();
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

const char* EngineSimGetVersion(void)
{
    return "mock-engine-sim/2.0.0";
}

EngineSimResult EngineSimValidateConfig(
    const EngineSimConfig* config)
{
    if (!config) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    if (config->sampleRate < 8000 || config->sampleRate > 192000) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    if (config->inputBufferSize < 64 || config->inputBufferSize > config->sampleRate) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    if (config->audioBufferSize < config->sampleRate / 2) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    if (config->simulationFrequency < 1000 || config->simulationFrequency > 100000) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    if (config->fluidSimulationSteps < 1 || config->fluidSimulationSteps > 64) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    if (config->targetSynthesizerLatency < 0.001 || config->targetSynthesizerLatency > 1.0) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

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

    if (enabled && !ctx->ignitionEnabled.load(std::memory_order_relaxed) && !ctx->inWarmupPhase) {
        ctx->inWarmupPhase = true;
        ctx->warmupStartTime = std::chrono::steady_clock::now();
    }

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

    // Mock doesn't use impulse responses - just validate
    return ESIM_SUCCESS;
}

EngineSimResult EngineSimSetRPM(
    EngineSimHandle handle,
    double rpm)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    if (rpm < 0.0 || rpm > 20000.0) {
        return ESIM_ERROR_INVALID_PARAMETER;
    }

    MockEngineSimContext* ctx = getContext(handle);
    ctx->targetRPM.store(rpm, std::memory_order_relaxed);

    return ESIM_SUCCESS;
}

EngineSimResult EngineSimWaitForAudioFrame(
    EngineSimHandle handle)
{
    if (!validateHandle(handle)) {
        return ESIM_ERROR_INVALID_HANDLE;
    }

    MockEngineSimContext* ctx = getContext(handle);
    ctx->synthesizer.waitProcessed();

    return ESIM_SUCCESS;
}
} // extern "C"
