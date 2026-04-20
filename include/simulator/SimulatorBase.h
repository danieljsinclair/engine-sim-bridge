// SimulatorBase.h - Common base for ISimulator implementations
// DRY: Extracts shared logger/telemetry storage and telemetry pushing logic.
// DI: Accepts logger + telemetry via create(), defaults to ConsoleLogger and
//     NullTelemetryWriter if nullptr is passed. Members are never null after create().

#ifndef SIMULATOR_BASE_H
#define SIMULATOR_BASE_H

#include "simulator/ISimulator.h"
#include "common/ILogging.h"
#include "telemetry/ITelemetryProvider.h"
#include "simulator/engine_sim_bridge.h"
#include "engine-sim/include/simulator.h"
#include <cmath>
#include <memory>
#include <string>
#include <vector>

// Minimal no-op telemetry writer - used as default when no writer is injected
class NullTelemetryWriter : public telemetry::ITelemetryWriter {
public:
    void writeEngineState(const telemetry::EngineStateTelemetry&) override {}
    void writeFramePerformance(const telemetry::FramePerformanceTelemetry&) override {}
    void writeAudioDiagnostics(const telemetry::AudioDiagnosticsTelemetry&) override {}
    void writeAudioTiming(const telemetry::AudioTimingTelemetry&) override {}
    void writeVehicleInputs(const telemetry::VehicleInputsTelemetry&) override {}
    void writeSimulatorMetrics(const telemetry::SimulatorMetricsTelemetry&) override {}
    void reset() override {}
    const char* getName() const override { return "NullTelemetryWriter"; }
};

class SimulatorBase : public ISimulator {
public:
    ~SimulatorBase() override = default;

    const char* getName() const override { return name_.c_str(); }

protected:
    // Call from subclass create() to initialize dependencies.
    // If logger is nullptr, defaults to ConsoleLogger.
    // If telemetryWriter is nullptr, defaults to NullTelemetryWriter.
    void initDependencies(ILogging* logger, telemetry::ITelemetryWriter* telemetryWriter) {
        if (logger) {
            logger_ = logger;
        } else {
            defaultLogger_ = std::make_unique<ConsoleLogger>();
            logger_ = defaultLogger_.get();
        }

        if (telemetryWriter) {
            telemetryWriter_ = telemetryWriter;
        } else {
            defaultTelemetryWriter_ = std::make_unique<NullTelemetryWriter>();
            telemetryWriter_ = defaultTelemetryWriter_.get();
        }
    }

    // Push engine state + frame performance to telemetry.
    // Call after each update() or render cycle.
    void pushTelemetry(const EngineSimStats& stats) {
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

    // Derive a display name from a script path (filename without extension).
    // Used by getName() until a real engine name is available.
    void setNameFromScript(const std::string& scriptPath) {
        if (scriptPath.empty()) {
            name_ = "Unnamed";
            return;
        }
        // Extract filename
        auto lastSlash = scriptPath.find_last_of("/\\");
        std::string filename = (lastSlash != std::string::npos)
            ? scriptPath.substr(lastSlash + 1)
            : scriptPath;
        // Strip extension
        auto lastDot = filename.find_last_of('.');
        name_ = (lastDot != std::string::npos)
            ? filename.substr(0, lastDot)
            : filename;
    }

    // Audio conversion buffer (RAII, used for int16 -> stereo float conversion)
    // Subclasses should use getAudioConversionBuffer() and ensureBufferSize()
    std::vector<int16_t> m_audioConversionBuffer;

    // Ensure the conversion buffer is at least the requested size
    // Returns pointer to the buffer data
    int16_t* ensureAudioConversionBufferSize(size_t requiredSize) {
        if (requiredSize > m_audioConversionBuffer.size()) {
            m_audioConversionBuffer.resize(requiredSize);
        }
        return m_audioConversionBuffer.data();
    }

    // Get the current conversion buffer (read-only)
    const int16_t* getAudioConversionBuffer() const {
        return m_audioConversionBuffer.data();
    }

    // Get the current conversion buffer (mutable)
    int16_t* getAudioConversionBuffer() {
        return m_audioConversionBuffer.data();
    }

    // Get the current conversion buffer size
    size_t getAudioConversionBufferSize() const {
        return m_audioConversionBuffer.size();
    }

    // Never null after initDependencies()
    ILogging* logger_ = nullptr;
    telemetry::ITelemetryWriter* telemetryWriter_ = nullptr;
    std::string name_;

    // Shared audio configuration (set from EngineSimConfig in create())
    int sampleRate_ = 48000;
    int simulationFrequency_ = 10000;

    // Fixed-step simulation advance, bypassing the latency regulator.
    // The regulator in startFrame() boosts step count by 10% when input latency is low,
    // causing audio to accumulate over time. This bypasses it with a deterministic step count.
    // Use ceil=true for update() (Threaded mode needs slight over-production to prevent crackles).
    // Use ceil=false for renderOnDemand() (SyncPull retry loop handles the deficit).
    static void advanceFixedSteps(Simulator* sim, int simulationFrequency, double dt, bool ceil) {
        sim->startFrame(dt);
        const int simSteps = ceil
            ? static_cast<int>(std::ceil(simulationFrequency * dt))
            : static_cast<int>(simulationFrequency * dt);
        for (int i = 0; i < simSteps; ++i) {
            sim->simulateStep();
        }
        sim->endFrame();
    }

    // Drain the Synthesizer's internal audio buffer.
    // Must be called before startAudioRenderingThread() to remove stale pre-fill.
    // Matches old C API EngineSimStartAudioThread behavior.
    static void drainSynthesizerBuffer(Simulator* sim) {
        int16_t drainBuffer[4096];
        while (sim->readAudioOutput(4096, drainBuffer) > 0) {
            // Drain all pre-fill
        }
    }

private:
    // Owned defaults (only created if nullptr was passed for the respective dependency)
    std::unique_ptr<ConsoleLogger> defaultLogger_;
    std::unique_ptr<NullTelemetryWriter> defaultTelemetryWriter_;
};

#endif // SIMULATOR_BASE_H
