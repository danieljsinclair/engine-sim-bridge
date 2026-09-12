// ThreadSchedulerTest.cpp - pins the ThreadScheduler destructor contract:
// still-pending tasks are DISCARDED at destruction, never executed. A
// destructor must not run client callbacks (see ThreadScheduler::~ThreadScheduler
// for why drain-run was removed — re-entry plus a post-spawned worker thread
// that would outlive the scheduler).

#include <io/PosixTransports.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <gtest/gtest.h>

using input::ThreadScheduler;

namespace {

TEST(ThreadSchedulerTest, DestructorDiscardsPendingTasks) {
    std::atomic<bool> ran{false};
    {
        ThreadScheduler scheduler;
        scheduler.post(60000, [&ran]() { ran = true; });
        // Task pending ~60 s beyond the teardown window; scope exit destroys
        // the scheduler, which must drop it without running it.
    }
    EXPECT_FALSE(ran.load());
}

// Control for the pin above: the same post() path DOES deliver a task when
// the scheduler is given time. Without it, DestructorDiscardsPendingTasks
// could pass vacuously (e.g. if post stopped scheduling at all).
TEST(ThreadSchedulerTest, PostedTaskRunsWhenGivenTime) {
    std::atomic<bool> ran{false};
    {
        ThreadScheduler scheduler;
        scheduler.post(0, [&ran]() { ran = true; });
        for (int i = 0; i < 200 && !ran.load(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    EXPECT_TRUE(ran.load());
}

}  // namespace
