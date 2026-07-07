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
