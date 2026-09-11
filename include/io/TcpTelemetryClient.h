// TcpTelemetryClient.h - Sticky/magnetic TCP telemetry stream client
//
// Port of the app's TcpTelemetrySource.swift protocol logic onto the
// ITransport seams (io/ITransport.h): connect to a vehicle-sim host on TCP,
// send the AUTH handshake, receive the CSV telemetry stream, parse each line
// via TcpCsvFrameParser and emit TcpTelemetryFrames.
//
// Sticky / magnetic reconnect (behavior-identical to the Swift source):
//   1. Aggressive-then-escalating retry of the KNOWN host (base 250ms for the
//      first 4 attempts, then exponential capped at 8000ms — reconnectBackoffMs).
//   2. UDP discovery hunt for a broadcaster whose packet starts with the magic
//      "VSIM" + version byte 1; a valid packet from a DIFFERENT host re-aims
//      the connection there.
// Deliberate delta over the Swift source: both paths funnel through
// connectTo(), which CANCELS the pending reconnect timer, so a discovery win
// is never disturbed by a stale retry (the Swift version could re-fire an
// already-scheduled retry at the old host after a re-aim).
//
// Threading: single-threaded-evented. The client does no locking; pair the
// POSIX impls with a ThreadScheduler so all sinks fire on one worker thread.
// Test fakes fire them synchronously.

#ifndef IO_TCP_TELEMETRY_CLIENT_H
#define IO_TCP_TELEMETRY_CLIENT_H

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>

#include "io/ITransport.h"
#include "io/TcpCsvFrameParser.h"

namespace input {

/// Reconnect backoff in ms for a zero-based attempt index. The first 4
/// attempts hold the aggressive base; attempt 4+ escalates exponentially, the
/// shift exponent clamped at 24 so the integer math cannot overflow no matter
/// how large `attempt` gets, and the result capped at `max`.
/// Pure function (Swift TcpTelemetrySource.backoffDelay port).
int reconnectBackoffMs(int attempt, int base = 250, int max = 8000);

struct TcpTelemetryConfig {
    std::string host;                       // initial sticky target
    uint16_t port = 3333;
    std::string authToken = "vehicle-sim-2026";
    int baseRetryDelayMs = 250;
    int maxRetryDelayMs = 8000;
    uint16_t discoveryPort = 3335;
};

enum class TcpLinkState { Disconnected, Connecting, Connected, Hunting };

struct TcpLinkStatus {
    TcpLinkState state = TcpLinkState::Disconnected;
    std::string host;
    uint16_t port = 0;
};

class TcpTelemetryClient {
public:
    /// Non-owning pointers to the three seams (the owner outlives the client).
    TcpTelemetryClient(ITcpTransport* tcp, IUdpListener* discovery,
                       IScheduler* scheduler, TcpTelemetryConfig config);

    /// (Re)start: reset parse state, begin the discovery hunt, connect to the
    /// sticky target.
    void start();
    /// Stop everything: cancel pending reconnects, close the link and the
    /// discovery listener, report Disconnected. Safe before start().
    void stop();

    bool isRunning() const;
    bool isConnected() const;
    const TcpLinkStatus& status() const;

    std::function<void(const TcpTelemetryFrame&)> onFrame;
    std::function<void(const TcpLinkStatus&)> onStatus;

    // ---- Direct-test surface (Swift exposed the same logic as `internal`) --
    /// Append raw received bytes and drain complete lines.
    void feedBuffer(std::string_view bytes);
    /// Parse one line: probe as a candidate header until one locks, then
    /// decode data rows and emit frames (banners/keepalives ignored).
    void handleLine(std::string_view line);
    /// True iff bytes start with the vehicle-sim discovery magic "VSIM"
    /// (0x56 0x53 0x49 0x4D) followed by protocol version 1.
    static bool isVehicleSimDiscovery(const std::string& bytes);
    /// Magnetic-switch decision: returns the host to re-aim at when `bytes`
    /// is a valid discovery packet from a host different from the sticky
    /// target and the client is running; empty string = no switch.
    std::string processDiscoveryPacket(const std::string& bytes,
                                       const std::string& senderHost) const;

private:
    void connectTo(const std::string& host);
    void scheduleReconnect(const std::string& host);
    void handleDisconnected(const std::string& host);
    void startDiscovery();
    void stopDiscovery();
    void notifyStatus(TcpLinkState state, std::string host, uint16_t port);

    ITcpTransport* tcp_;
    IUdpListener* discovery_;
    IScheduler* scheduler_;
    TcpTelemetryConfig config_;

    TcpLinkStatus status_;
    bool running_ = false;
    bool connected_ = false;
    std::string stickyHost_;   // the current target (discovery may re-aim it)
    std::string buffer_;       // receive buffer pending a newline
    std::optional<TcpCsvColumnMap> columnMap_;
    int retryCount_ = 0;
    uint64_t reconnectToken_ = 0;
};

} // namespace input

#endif // IO_TCP_TELEMETRY_CLIENT_H
