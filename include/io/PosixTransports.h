// PosixTransports.h - macOS/Linux impls of the ITransport seams
//
// Concrete ITcpTransport / IUdpListener / IScheduler for desktop platforms.
// Threading contract (io/ITransport.h): sinks fire on the impl's private I/O
// threads; a clean owner-initiated close() suppresses the disconnected sink
// (only a genuine link drop or failed connect fires it), so the client's own
// close-before-reconnect never schedules a spurious retry.

#ifndef IO_POSIX_TRANSPORTS_H
#define IO_POSIX_TRANSPORTS_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "io/ITransport.h"

namespace input {

/// Blocking connect in a private thread; after connect the socket goes
/// non-blocking and a poll() loop delivers onData. close() from any thread
/// shuts the socket down so the loop wakes and the thread joins.
class PosixTcpTransport : public ITcpTransport {
public:
    PosixTcpTransport() = default;
    ~PosixTcpTransport() override;

    void connect(const std::string& host, uint16_t port) override;
    void send(const std::string& bytes) override;
    void close() override;

private:
    void runReceiveLoop(int fd);

    std::thread worker_;
    std::atomic<bool> closed_{false};
    std::atomic<int> fd_{-1};
};

/// SO_REUSEADDR UDP bind + recvfrom loop on a private thread; the sender
/// address arrives as a dotted quad. The receive uses a 200 ms SO_RCVTIMEO
/// tick so close() never has to race (or rely on waking) a blocked recvfrom.
class PosixUdpListener : public IUdpListener {
public:
    PosixUdpListener() = default;
    ~PosixUdpListener() override;

    bool listen(uint16_t port) override;
    void close() override;

private:
    std::thread worker_;
    std::atomic<bool> closed_{false};
    int fd_ = -1;
};

/// One worker thread with a timed task queue ordered by deadline. post()
/// returns a monotonically increasing token; cancel() drops a pending task.
/// The destructor drains remaining tasks and joins the worker.
class ThreadScheduler : public IScheduler {
public:
    ThreadScheduler() = default;
    ~ThreadScheduler() override;

    uint64_t post(int delayMs, std::function<void()> fn) override;
    void cancel(uint64_t token) override;

private:
    struct Task {
        uint64_t token;
        std::chrono::steady_clock::time_point deadline;
        std::function<void()> fn;
    };

    void runLoop();

    std::mutex mutex_;
    std::condition_variable cv_;
    std::vector<Task> queue_;  // small n: sorted by (deadline, token) on insert
    std::deque<uint64_t> cancelled_;
    uint64_t nextToken_ = 1;
    bool stopped_ = false;
    std::thread worker_;
};

} // namespace input

#endif // IO_POSIX_TRANSPORTS_H
