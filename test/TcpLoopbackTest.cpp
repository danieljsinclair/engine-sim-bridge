// TcpLoopbackTest.cpp
//
// One end-to-end test over the REAL POSIX impls (PosixTcpTransport,
// PosixUdpListener, ThreadScheduler): the client connects to a local server,
// receives a CSV header + data row, loses the link when the server closes the
// connection, and reconnects through its real ThreadScheduler backoff.
// Deterministic: every wait is a predicate poll with a generous deadline,
// never a sleep-as-sync.

#include <gtest/gtest.h>

#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "io/PosixTransports.h"
#include "io/TcpTelemetryClient.h"

using input::PosixTcpTransport;
using input::TcpLinkState;
using input::TcpTelemetryClient;
using input::TcpTelemetryConfig;
using input::TcpTelemetryFrame;
using input::ThreadScheduler;

namespace {

/// Local TCP server: listens on 127.0.0.1 with an ephemeral port and accepts
/// connections one at a time on a private thread. The server never closes
/// accepted fds itself — the test drives the link lifetime (send over the
/// accepted fd, close it to simulate the server-side drop); the destructor
/// reaps anything left open after joining the acceptor.
class LocalTcpServer {
public:
    LocalTcpServer() {
        listener_ = ::socket(AF_INET, SOCK_STREAM, 0);
        int reuse = 1;
        ::setsockopt(listener_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = 0;  // ephemeral
        EXPECT_EQ(::bind(listener_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)), 0);
        EXPECT_EQ(::listen(listener_, 4), 0);
        socklen_t len = sizeof(addr);
        EXPECT_EQ(::getsockname(listener_, reinterpret_cast<sockaddr*>(&addr), &len), 0);
        port_ = ntohs(addr.sin_port);
    }

    ~LocalTcpServer() {
        stopping_ = true;  // the accept loop ticks out of poll() within 200 ms
        if (worker_.joinable()) {
            worker_.join();
        }
        ::close(listener_);
        std::lock_guard<std::mutex> lock(mutex_);
        for (int fd : accepted_) {
            ::close(fd);
        }
    }

    uint16_t port() const { return port_; }

    /// The most recently accepted fd (for the test to send over / close).
    int lastAccepted() {
        std::lock_guard<std::mutex> lock(mutex_);
        return accepted_.empty() ? -1 : accepted_.back();
    }

    /// Accept the next connection on the private thread and drain the client's
    /// AUTH line, then run `session(fd)`. The fd stays open afterwards.
    void serveOne(const std::function<void(int fd)>& session) {
        if (worker_.joinable()) {
            worker_.join();  // reap the previous acceptor before respawning
        }
        worker_ = std::thread([this, session]() {
            while (!stopping_) {
                pollfd pfd{};
                pfd.fd = listener_;
                pfd.events = POLLIN;
                if (::poll(&pfd, 1, 200) <= 0) {
                    continue;  // tick: re-check stopping_
                }
                sockaddr_in peer{};
                socklen_t len = sizeof(peer);
                const int fd =
                    ::accept(listener_, reinterpret_cast<sockaddr*>(&peer), &len);
                if (fd < 0) {
                    return;
                }
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    accepted_.push_back(fd);
                }
                acceptedCount_.fetch_add(1);
                acceptedCv_.notify_all();

                // Drain the client's AUTH line (the client sends it
                // immediately after connect succeeds, before its receive
                // loop starts, so it is already in flight here).
                char auth[128] = {0};
                ::recv(fd, auth, sizeof(auth) - 1, 0);
                session(fd);
                return;
            }
        });
    }

    /// Wait until `expected` connections have been accepted, or time out.
    bool awaitAccepted(int expected, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        return acceptedCv_.wait_for(lock, timeout, [&]() {
            return acceptedCount_.load() >= expected;
        });
    }

private:
    int listener_ = -1;
    uint16_t port_ = 0;
    std::atomic<bool> stopping_{false};
    std::atomic<int> acceptedCount_{0};
    std::mutex mutex_;
    std::condition_variable acceptedCv_;
    std::vector<int> accepted_;
    std::thread worker_;
};

/// Poll a predicate with a deadline, waking in small ticks (no sleeps-as-sync:
/// the predicate is the synchronisation point, the tick is only granularity).
template <typename Predicate>
bool awaitTrue(Predicate predicate, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return predicate();
}

constexpr auto kTimeout = std::chrono::seconds(5);

} // namespace

TEST(TcpLoopbackTest, ClientConnectsReceivesFrameAndReconnectsAfterServerClose) {
    LocalTcpServer server;
    PosixTcpTransport tcp;
    ThreadScheduler scheduler;

    TcpTelemetryConfig config;
    config.host = "127.0.0.1";
    config.port = server.port();

    std::mutex frameMutex;
    std::optional<TcpTelemetryFrame> frame;
    TcpTelemetryClient client(&tcp, nullptr, &scheduler, config);
    client.onFrame = [&](const TcpTelemetryFrame& f) {
        std::lock_guard<std::mutex> lock(frameMutex);
        frame = f;
    };

    client.start();

    // 1. The real transport connects; the server accepts and reads the AUTH.
    server.serveOne([](int) {});
    ASSERT_TRUE(server.awaitAccepted(1, kTimeout)) << "no first connection";
    ASSERT_TRUE(
        awaitTrue([&]() { return client.status().state == TcpLinkState::Connected; },
                  kTimeout))
        << "client never reached Connected on the first link";
    ASSERT_EQ(client.status().port, server.port());
    ASSERT_EQ(client.status().host, "127.0.0.1");

    // 2. Server sends the live-schema header and one data row over the open
    //    link; the client must decode and emit a frame with the row's values.
    const int fd1 = server.lastAccepted();
    ASSERT_GE(fd1, 0);
    const std::string payload =
        "timestamp_ms,speed_kmh,throttle_percent,brake_percent,acceleration_g\r\n"
        "1000,60.0,40.0,0.0,0.3\r\n";
    ASSERT_EQ(static_cast<size_t>(::send(fd1, payload.data(), payload.size(), 0)),
              payload.size());
    ASSERT_TRUE(awaitTrue(
        [&]() {
            std::lock_guard<std::mutex> lock(frameMutex);
            return frame.has_value();
        },
        kTimeout))
        << "client never emitted the expected frame";
    {
        std::lock_guard<std::mutex> lock(frameMutex);
        EXPECT_DOUBLE_EQ(frame->timestampMs, 1000.0);
        EXPECT_DOUBLE_EQ(frame->speedKmh.value(), 60.0);
        EXPECT_DOUBLE_EQ(frame->throttle.value(), 40.0);
        EXPECT_DOUBLE_EQ(frame->brake.value(), 0.0);
        EXPECT_DOUBLE_EQ(frame->accelerationG.value(), 0.3);
    }

    // 3. Server-side close drops the real link: the client must leave
    //    Connected and reconnect via its real ThreadScheduler (250 ms base
    //    backoff), producing a SECOND accepted connection.
    ::close(fd1);
    server.serveOne([](int) {});  // acceptor for the reconnect
    awaitTrue([&]() { return client.status().state != TcpLinkState::Connected; },
              kTimeout);
    EXPECT_TRUE(server.awaitAccepted(2, kTimeout))
        << "client did not reconnect after the server closed the link";

    client.stop();
    EXPECT_EQ(client.status().state, TcpLinkState::Disconnected);
    EXPECT_FALSE(client.isConnected());
}
