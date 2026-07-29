// Diagnostics.h - Audio performance and timing diagnostics
// SRP: Single responsibility - manages only diagnostic metrics
// OCP: New diagnostic types can be added without modifying existing code
// DIP: High-level modules depend on this abstraction
// Phase F: Moved to engine-sim-bridge submodule

#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include <atomic>
#include <chrono>

/**
 * Diagnostics - Audio performance and timing diagnostics
 *
 * Responsibilities:
 * - Track rendering timing metrics
 * - Monitor frame budget usage
 * - Track headroom for buffer health
 * - Provide performance metrics for monitoring
 * - Thread-safe metric collection
 *
 * SRP: Only manages diagnostics, not audio or buffer state
 */
struct Diagnostics {
    /**
     * Time measurement type
     */
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = std::chrono::time_point<Clock>;
    using Duration = std::chrono::microseconds;

    /**
     * Initialize with default values
     */
    Diagnostics()
        : lastRenderMs(0.0)
        , lastHeadroomMs(0.0)
        , lastBudgetPct(0.0)
        , lastFrameBudgetPct(0.0)
        , totalFramesRendered(0)
        , lastFramesRequested(0)
        , lastFramesRendered(0)
        , sampleRate_(0)
        , callbackCount_(0)
        , lastCallbackRateHz(0.0)
        , generatingRateFps(0.0)
        , previousGeneratingRateFps(0.0)
    {}

    /**
     * Last render time in milliseconds
     * - Measured in audio callback
     * - Used for real-time performance monitoring
     */
    std::atomic<double> lastRenderMs;

    /**
     * Last headroom time in milliseconds
     * - Time between render request and buffer depletion
     * - Should be around 16ms for healthy cursor-chasing
     * - Values < 10ms indicate potential underrun
     * - Values > 20ms indicate buffer overfill
     */
    std::atomic<double> lastHeadroomMs;

    /**
     * Last render time budget percentage used
     * - renderTime / 16ms budget per callback
     * - Values > 80% indicate performance concern
     * - Used for system load monitoring
     */
    std::atomic<double> lastBudgetPct;

    /**
     * Last frame count requested vs available
     * - framesNeeded / framesAvailable (when > 1.0)
     * - Used to detect frame starvation
     */
    std::atomic<double> lastFrameBudgetPct;

    /**
     * Total number of frames rendered
     * - Monitored for throughput calculation
     * - Reset on re-initialization
     */
    std::atomic<int64_t> totalFramesRendered;

    std::atomic<int> lastFramesRequested{0};
    std::atomic<int> lastFramesRendered{0};

    // Sample rate for accurate budget calculation
    int sampleRate_;

    // Callback throughput tracking
    std::atomic<int64_t> callbackCount_{0};
    std::atomic<double> lastCallbackRateHz{0.0};
    std::atomic<double> generatingRateFps{0.0};
    std::atomic<double> previousGeneratingRateFps{0.0};

    void setSampleRate(int sampleRate) {
        sampleRate_ = sampleRate;
    }

    /**
     * Compute callback interval in ms from framesRequested and sampleRate
     */
    double callbackIntervalMs(int framesRequested) const {
        if (sampleRate_ <= 0) return 16.0;  // fallback
        return (static_cast<double>(framesRequested) / sampleRate_) * 1000.0;
    }

    /**
     * Record render time
     * @param renderTimeMs Time taken for this render in milliseconds
     * @param framesRendered Number of frames rendered in this call
     * @param framesRequested Number of frames requested
     */
    void recordRender(double renderTimeMs, int framesRendered, int framesRequested) {
        lastRenderMs.store(renderTimeMs);

        // Calculate budget from actual callback interval, not hardcoded 16ms
        double budgetMs = callbackIntervalMs(framesRequested);
        lastHeadroomMs.store(budgetMs - renderTimeMs);

        double budgetPct = (budgetMs > 0.0) ? (renderTimeMs / budgetMs) * 100.0 : 0.0;
        lastBudgetPct.store(budgetPct);

        // Calculate frame budget
        double frameBudgetPct = 0.0;
        if (renderTimeMs > 0.0 && sampleRate_ > 0) {
            frameBudgetPct = static_cast<double>(framesRendered) / (renderTimeMs * sampleRate_ / 1000.0) * 100.0;
        }
        lastFrameBudgetPct.store(frameBudgetPct);

        lastFramesRequested.store(framesRequested);
        lastFramesRendered.store(framesRendered);

        totalFramesRendered.fetch_add(framesRendered);

        // Track callback throughput
        callbackCount_.fetch_add(1);
    }

    /**
     * Update throughput rates. Called periodically (e.g., once per second).
     * @param elapsedSeconds Time since last update
     */
    void updateThroughput(double elapsedSeconds) {
        if (elapsedSeconds <= 0.0) return;

        int64_t callbacks = callbackCount_.exchange(0);
        int64_t frames = totalFramesRendered.exchange(0);

        double callbackHz = static_cast<double>(callbacks) / elapsedSeconds;
        double genFps = static_cast<double>(frames) / elapsedSeconds;

        previousGeneratingRateFps.store(generatingRateFps.load());
        lastCallbackRateHz.store(callbackHz);
        generatingRateFps.store(genFps);
    }

    /**
     * Reset all diagnostic counters
     * Useful for cleanup or test scenarios
     */
    void reset() {
        lastRenderMs.store(0.0);
        lastHeadroomMs.store(0.0);
        lastBudgetPct.store(0.0);
        lastFrameBudgetPct.store(0.0);
        totalFramesRendered.store(0);
        lastFramesRequested.store(0);
        lastFramesRendered.store(0);
        callbackCount_.store(0);
        lastCallbackRateHz.store(0.0);
        generatingRateFps.store(0.0);
        previousGeneratingRateFps.store(0.0);
    }

    /**
     * Get current diagnostic snapshot
     * @return Copy of current diagnostic state
     */
    struct Snapshot {
        double lastRenderMs;
        double lastHeadroomMs;
        double lastBudgetPct;
        double lastFrameBudgetPct;
        int64_t totalFramesRendered;
        int lastFramesRequested;
        int lastFramesRendered;
        double callbackRateHz;
        double generatingRateFps;
        double trendPct;

        Snapshot()
            : lastRenderMs(0.0)
            , lastHeadroomMs(0.0)
            , lastBudgetPct(0.0)
            , lastFrameBudgetPct(0.0)
            , totalFramesRendered(0)
            , lastFramesRequested(0)
            , lastFramesRendered(0)
            , callbackRateHz(0.0)
            , generatingRateFps(0.0)
            , trendPct(0.0)
        {}
    };

    /**
     * Get current diagnostic snapshot
     * Thread-safe read of all diagnostic metrics
     * @return Snapshot of current diagnostic state
     */
    Snapshot getSnapshot() const {
        Snapshot snapshot;
        snapshot.lastRenderMs = lastRenderMs.load();
        snapshot.lastHeadroomMs = lastHeadroomMs.load();
        snapshot.lastBudgetPct = lastBudgetPct.load();
        snapshot.lastFrameBudgetPct = lastFrameBudgetPct.load();
        snapshot.totalFramesRendered = totalFramesRendered.load();
        snapshot.lastFramesRequested = lastFramesRequested.load();
        snapshot.lastFramesRendered = lastFramesRendered.load();
        snapshot.callbackRateHz = lastCallbackRateHz.load();
        snapshot.generatingRateFps = generatingRateFps.load();
        double prev = previousGeneratingRateFps.load();
        double curr = generatingRateFps.load();
        snapshot.trendPct = (prev > 0.0) ? ((curr - prev) / prev) * 100.0 : 0.0;
        return snapshot;
    }
};

#endif // DIAGNOSTICS_H
