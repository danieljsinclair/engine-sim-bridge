// ITransport.h - Tiny evented-IO seams for the TCP telemetry client
//
// Three interfaces, no factory zoo (SRP/DIP): the TcpTelemetryClient is the
// protocol state machine and depends only on these seams, so unit tests drive
// it with scripted in-memory fakes and the loopback test exercises the real
// POSIX impls end-to-end on 127.0.0.1.
//
// Threading contract: the CLIENT is single-threaded-evented — all event sinks
// (onConnected / onData / onDisconnected / onPacket) are invoked on the thread
// the impl chooses; a real deployment pairs the POSIX impls with the
// ThreadScheduler so events land on one worker thread. Scripted test fakes
// fire the sinks synchronously from the test thread.

#ifndef IO_ITRANSPORT_H
#define IO_ITRANSPORT_H

#include <cstdint>
#include <functional>
#include <string>

namespace input {

/// One TCP connection. The client installs the sinks BEFORE connect() and the
/// impl fires exactly one of onConnected / onDisconnected per connect attempt
/// (plus onDisconnected whenever an established link drops or close() is
/// called due to error). onData delivers raw stream bytes in arrival order.
class ITcpTransport {
public:
    virtual ~ITcpTransport() = default;

    virtual void connect(const std::string& host, uint16_t port) = 0;
    virtual void send(const std::string& bytes) = 0;
    /// Close the link. Must be safe to call from any thread and must make a
    /// concurrent blocking receive return (impl detail: poll + shutdown).
    virtual void close() = 0;

    std::function<void()> onConnected;
    std::function<void(const std::string& bytes)> onData;
    std::function<void()> onDisconnected;
};

/// UDP discovery listener. One packet per onPacket, with the sender's address
/// as a best-effort string (dotted quad for IPv4).
class IUdpListener {
public:
    virtual ~IUdpListener() = default;

    /// Bind and start receiving. Returns false if the port could not be bound
    /// (discovery is best-effort: the client keeps its sticky reconnect).
    virtual bool listen(uint16_t port) = 0;
    virtual void close() = 0;

    std::function<void(const std::string& bytes, const std::string& senderHost)> onPacket;
};

/// Delayed/deferred task execution for the reconnect backoff. Tokens allow
/// cancelling a pending task (stop(), or a discovery win preempting a retry).
class IScheduler {
public:
    virtual ~IScheduler() = default;
    virtual uint64_t post(int delayMs, std::function<void()> fn) = 0;
    virtual void cancel(uint64_t token) = 0;
};

} // namespace input

#endif // IO_ITRANSPORT_H
