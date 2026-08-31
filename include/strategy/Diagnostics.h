// Diagnostics.h - Audio performance and timing diagnostics
// SRP: Single responsibility - manages only diagnostic metrics
// OCP: New diagnostic types can be added without modifying existing code
// DIP: High-level modules depend on this abstraction
// Phase F: Moved to engine-sim-bridge submodule

#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include <atomic>
#include <chrono>
#include <cstdint>

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

    // ---- Audio-ring health (the gap the sync-pull knock slipped through) --
    // Cumulative counters mirrored from the simulator's synthesizer via
    // recordRingHealth() each callback; the windowed production/consumption
    // ratio and the sustained-overproduction streak are computed on the same
    // 1-second cadence as updateThroughput().
    std::atomic<uint64_t> ringLapCount{0};
    std::atomic<uint64_t> seamDiscontinuityCount{0};
    std::atomic<uint64_t> ringFramesWritten{0};
    std::atomic<uint64_t> ringFramesConsumed{0};
    std::atomic<double> lastProdConsRatio{0.0};
    std::atomic<int> sustainedOverproductionWindows_{0};

    // A window ratio above this counts as overproduction. ~5% headroom over
    // unity absorbs startup transients and legitimate catch-up bursts; the
    // sync-pull knock ran at 1.44x.
    static constexpr double kOverproductionRatio = 1.05;

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

        updateRingHealthWindow(elapsedSeconds);
    }

    /**
     * Mirror the simulator's cumulative audio-ring counters. Called each
     * audio callback (four relaxed atomic stores - negligible on the
     * realtime path).
     */
    void recordRingHealth(uint64_t framesWritten, uint64_t framesConsumed,
                          uint64_t laps, uint64_t seams) {
        ringFramesWritten.store(framesWritten, std::memory_order_relaxed);
        ringFramesConsumed.store(framesConsumed, std::memory_order_relaxed);
        ringLapCount.store(laps, std::memory_order_relaxed);
        seamDiscontinuityCount.store(seams, std::memory_order_relaxed);
    }

    /**
     * Compute the windowed production/consumption ratio from the cumulative
     * ring counters and advance the sustained-overproduction streak.
     * Called on the same cadence as updateThroughput (once per second from
     * the audio callback thread).
     */
    void updateRingHealthWindow(double elapsedSeconds) {
        (void)elapsedSeconds;  // the window is the delta between calls

        const uint64_t written = ringFramesWritten.load(std::memory_order_relaxed);
        const uint64_t consumed = ringFramesConsumed.load(std::memory_order_relaxed);
        const uint64_t writtenDelta = (written > prevRingFramesWritten_) ? written - prevRingFramesWritten_ : 0;
        const uint64_t consumedDelta = (consumed > prevRingFramesConsumed_) ? consumed - prevRingFramesConsumed_ : 0;
        prevRingFramesWritten_ = written;
        prevRingFramesConsumed_ = consumed;

        // No consumption this window (e.g. pre-roll before playback starts):
        // no ratio to judge - neither alarm nor reset.
        if (consumedDelta == 0) {
            if (writtenDelta == 0) return;
            lastProdConsRatio.store(0.0);
            return;
        }

        const double ratio = static_cast<double>(writtenDelta) / static_cast<double>(consumedDelta);
        lastProdConsRatio.store(ratio);

        if (ratio > kOverproductionRatio) {
            sustainedOverproductionWindows_.fetch_add(1);
        }
        else {
            sustainedOverproductionWindows_.store(0);
        }
    }

    // Window state for updateRingHealthWindow (audio-callback-thread only).
    uint64_t prevRingFramesWritten_ = 0;
    uint64_t prevRingFramesConsumed_ = 0;

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
        ringLapCount.store(0);
        seamDiscontinuityCount.store(0);
        ringFramesWritten.store(0);
        ringFramesConsumed.store(0);
        lastProdConsRatio.store(0.0);
        sustainedOverproductionWindows_.store(0);
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
        // Audio-ring health
        uint64_t ringLapCount;
        uint64_t seamDiscontinuityCount;
        double prodConsRatio;
        int sustainedOverproductionWindows;

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
            , ringLapCount(0)
            , seamDiscontinuityCount(0)
            , prodConsRatio(0.0)
            , sustainedOverproductionWindows(0)
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
        snapshot.ringLapCount = ringLapCount.load();
        snapshot.seamDiscontinuityCount = seamDiscontinuityCount.load();
        snapshot.prodConsRatio = lastProdConsRatio.load();
        snapshot.sustainedOverproductionWindows = sustainedOverproductionWindows_.load();
        return snapshot;
    }
};

#endif // DIAGNOSTICS_H
