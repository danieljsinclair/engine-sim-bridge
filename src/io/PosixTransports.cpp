// PosixTransports.cpp - macOS/Linux impls of the ITransport seams
// (see include/io/PosixTransports.h)

#include "io/PosixTransports.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <utility>

namespace input {

// ---- PosixTcpTransport -----------------------------------------------------

PosixTcpTransport::~PosixTcpTransport() {
    close();
    if (worker_.joinable()) {
        worker_.join();
    }
}

void PosixTcpTransport::connect(const std::string& host, uint16_t port) {
    // A previous worker (if any) exited via close(): reap it before resetting
    // state, or the move-assign below would terminate on a joinable thread.
    if (worker_.joinable()) {
        worker_.join();
    }
    closed_ = false;
    worker_ = std::thread([this, host, port]() {
        // Resolve, then blocking connect (private thread keeps the caller
        // non-blocking without the state machine of a half-open socket).
        addrinfo hints{};
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        addrinfo* list = nullptr;
        if (::getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &list) != 0 ||
            list == nullptr) {
            if (onDisconnected) {
                onDisconnected();
            }
            return;
        }

        int fd = ::socket(list->ai_family, list->ai_socktype, list->ai_protocol);
        if (fd < 0 || ::connect(fd, list->ai_addr, list->ai_addrlen) != 0) {
            if (fd >= 0) {
                ::close(fd);
            }
            ::freeaddrinfo(list);
            if (onDisconnected) {
                onDisconnected();
            }
            return;
        }
        ::freeaddrinfo(list);
        ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
        fd_ = fd;

        if (onConnected) {
            onConnected();
        }
        runReceiveLoop(fd);
    });
}

void PosixTcpTransport::runReceiveLoop(int fd) {
    while (!closed_) {
        pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;
        const int ready = ::poll(&pfd, 1, 200);
        if (closed_) {
            break;
        }
        if (ready <= 0) {
            continue;  // timeout or EINTR: re-check closed_, poll again
        }
        if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            break;
        }
        if ((pfd.revents & POLLIN) == 0) {
            continue;
        }
        std::array<char, 4096> buf;
        const ssize_t n = ::recv(fd, buf.data(), std::size(buf), 0);
        if (n <= 0) {
            break;  // 0 = orderly remote close, <0 = error (ECONNRESET etc.)
        }
        if (onData) {
            onData(std::string(buf.data(), static_cast<std::size_t>(n)));
        }
    }
    // Exactly one closer wins: close() already shut the fd down (owner clean
    // close) or we do (genuine drop). Never double-close the descriptor.
    int expected = fd;
    if (fd_.compare_exchange_strong(expected, -1)) {
        ::close(fd);
    }
    // Clean owner close() suppresses the sink; a genuine drop reports it so
    // the client can schedule its reconnect.
    if (!closed_ && onDisconnected) {
        onDisconnected();
    }
}

void PosixTcpTransport::send(const std::string& bytes) {
    const int fd = fd_;
    if (fd >= 0) {
        ::send(fd, bytes.data(), bytes.size(), 0);
    }
}

void PosixTcpTransport::close() {
    const bool alreadyClosed = closed_.exchange(true);
    if (alreadyClosed) {
        return;
    }
    const int fd = fd_.exchange(-1);
    if (fd >= 0) {
        ::shutdown(fd, SHUT_RDWR);  // wakes a concurrent poll()/recv()
        ::close(fd);
    }
}

// ---- PosixUdpListener ------------------------------------------------------

PosixUdpListener::~PosixUdpListener() {
    close();
    if (worker_.joinable()) {
        worker_.join();
    }
}

bool PosixUdpListener::listen(uint16_t port) {
    if (worker_.joinable()) {
        worker_.join();  // reap a previous receiver (see PosixTcpTransport)
    }
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        return false;
    }
    int reuse = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    // Bounded blocking receive: recvfrom() returns every tick so the loop can
    // re-check closed_ — close() never needs to race a blocked receive.
    timeval recvTimeout{0, 200 * 1000};
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &recvTimeout, sizeof(recvTimeout));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        ::close(fd);
        return false;
    }
    fd_ = fd;
    closed_ = false;
    worker_ = std::thread([this, fd]() {
        while (!closed_) {
            std::array<char, 2048> buf;
            sockaddr_in from{};
            socklen_t fromLen = sizeof(from);
            const ssize_t n = ::recvfrom(fd, buf.data(), std::size(buf), 0,
                                         reinterpret_cast<sockaddr*>(&from), &fromLen);
            if (closed_ || n <= 0) {
                break;
            }
            std::array<char, INET_ADDRSTRLEN> dotted{};
            if (!onPacket) {
                continue;
            }
            if (::inet_ntop(AF_INET, &from.sin_addr, dotted.data(), std::size(dotted)) != nullptr) {
                onPacket(std::string(buf.data(), static_cast<std::size_t>(n)), dotted.data());
            }
        }
    });
    return true;
}

void PosixUdpListener::close() {
    const bool alreadyClosed = closed_.exchange(true);
    if (alreadyClosed) {
        return;
    }
    const int fd = fd_;
    fd_ = -1;
    if (fd >= 0) {
        // The receiver's SO_RCVTIMEO ticks it out of recvfrom() within 200 ms;
        // it then sees closed_ and exits. Closing the fd here is safe because
        // no receive can be parked on it longer than one tick.
        ::close(fd);
    }
}

// ---- ThreadScheduler -------------------------------------------------------

ThreadScheduler::~ThreadScheduler() {
    {
        std::scoped_lock lock(mutex_);
        stopped_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
    // Drain: run whatever is still pending (uncancelled), in deadline order.
    std::vector<Task> drain;
    {
        std::scoped_lock lock(mutex_);
        drain.swap(queue_);
    }
    for (Task& task : drain) {
        if (std::find(cancelled_.begin(), cancelled_.end(), task.token) == cancelled_.end()) {
            task.fn();
        }
    }
}

uint64_t ThreadScheduler::post(int delayMs, std::function<void()> fn) {
    const uint64_t token = nextToken_++;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(delayMs);
    std::scoped_lock lock(mutex_);
    Task task{token, deadline, std::move(fn)};
    auto it = std::lower_bound(queue_.begin(), queue_.end(), task,
                               [](const Task& a, const Task& b) {
                                   return a.deadline < b.deadline;
                               });
    queue_.insert(it, std::move(task));
    // Spawned under the lock so concurrent first posters cannot both see a
    // non-joinable worker_ and double-spawn.
    if (!worker_.joinable()) {
        worker_ = std::thread([this]() { runLoop(); });
    }
    cv_.notify_all();
    return token;
}

void ThreadScheduler::cancel(uint64_t token) {
    std::scoped_lock lock(mutex_);
    cancelled_.push_back(token);
}

void ThreadScheduler::runLoop() {
    std::unique_lock lock(mutex_);
    while (true) {
        if (stopped_) {
            return;  // the destructor drains whatever is left
        }
        if (queue_.empty()) {
            cv_.wait(lock, [this]() { return stopped_ || !queue_.empty(); });
            continue;
        }
        // Sleep until the earliest deadline; a stop or a newly scheduled
        // earlier task notifies us out sooner (the loop then re-aims).
        cv_.wait_until(lock, queue_.front().deadline, [this]() {
            return stopped_ || queue_.empty() ||
                   queue_.front().deadline <= std::chrono::steady_clock::now();
        });
        if (queue_.empty() ||
            queue_.front().deadline > std::chrono::steady_clock::now()) {
            continue;  // woken for a stop or a different front — re-evaluate
        }
        Task task = queue_.front();
        queue_.erase(queue_.begin());
        lock.unlock();
        if (std::find(cancelled_.begin(), cancelled_.end(), task.token) ==
            cancelled_.end()) {
            task.fn();
        }
        lock.lock();
    }
}

} // namespace input
