// ILoopClock.h - Clock abstraction for simulation loop pacing
// Enables unit testing of SimulationLoop by removing real-time sleep

#ifndef I_LOOP_CLOCK_H
#define I_LOOP_CLOCK_H

#include <chrono>
#include <thread>

// ============================================================================
// ILoopClock - Clock interface for loop pacing
// ============================================================================

class ILoopClock {
public:
    virtual ~ILoopClock() = default;

    // Wait until the next tick (real-time pacing for steady clock)
    virtual void waitUntilNextTick() = 0;

    // Re-anchor the pacing schedule to NOW. Call after a period of unpaced
    // stepping (e.g. the fast-forwarded warm-start prefix on a file trace):
    // without resync the schedule is stale in the past and the next paced
    // loop sprints to catch up. Default no-op: clocks without a schedule
    // (FakeLoopClock) need nothing.
    virtual void resync() {}
};

// ============================================================================
// SteadyClockLoopClock - Real-time pacing using steady_clock
// Wraps the LoopTimer pacing logic (steady_clock + sleep_until)
// ============================================================================

class SteadyClockLoopClock : public ILoopClock {
public:
    explicit SteadyClockLoopClock(double intervalSeconds)
        : nextWakeTime_(std::chrono::steady_clock::now())
        , intervalUs_(static_cast<long long>(intervalSeconds * 1000000)) {}

    void waitUntilNextTick() override {
        nextWakeTime_ += intervalUs_;
        std::this_thread::sleep_until(nextWakeTime_);
    }

    void resync() override {
        nextWakeTime_ = std::chrono::steady_clock::now();
    }

private:
    std::chrono::steady_clock::time_point nextWakeTime_;
    std::chrono::microseconds intervalUs_;
};

// ============================================================================
// FakeLoopClock - Test double with no-op pacing
// Used in unit tests to avoid real-time delays
// ============================================================================

class FakeLoopClock : public ILoopClock {
public:
    void waitUntilNextTick() override {
        // No-op for test purposes
    }
};

#endif // I_LOOP_CLOCK_H
