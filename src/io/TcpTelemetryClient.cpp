// TcpTelemetryClient.cpp - Sticky/magnetic TCP telemetry stream client
// (protocol logic port of TcpTelemetrySource.swift; see TcpTelemetryClient.h)

#include "io/TcpTelemetryClient.h"

#include <algorithm>
#include <array>

namespace input {

int reconnectBackoffMs(int attempt, int base, int max) {
    if (attempt < 4) {
        return base;
    }
    // Shift in 64 bits: base 250 at the clamped exponent 24 overflows a 32-bit
    // signed int (the Swift original's Int was 64-bit and safe). The 24-clamp
    // alone only protects bases <= 127; the widened shift protects all bases.
    const int exponent = std::min(attempt - 3, 24);
    const int64_t escalated = static_cast<int64_t>(base) << exponent;
    return static_cast<int>(std::min(escalated, static_cast<int64_t>(max)));
}

TcpTelemetryClient::TcpTelemetryClient(ITcpTransport* tcp, IUdpListener* discovery,
                                       IScheduler* scheduler, TcpTelemetryConfig config)
    : tcp_(tcp),
      discovery_(discovery),
      scheduler_(scheduler),
      config_(std::move(config)),
      stickyHost_(config_.host) {}

void TcpTelemetryClient::start() {
    running_ = true;
    retryCount_ = 0;
    columnMap_.reset();
    buffer_.clear();
    startDiscovery();
    connectTo(stickyHost_);
}

void TcpTelemetryClient::stop() {
    running_ = false;
    scheduler_->cancel(reconnectToken_);
    reconnectToken_ = 0;
    tcp_->close();
    stopDiscovery();
    connected_ = false;
    notifyStatus(TcpLinkState::Disconnected, "", 0);
}

bool TcpTelemetryClient::isRunning() const { return running_; }

bool TcpTelemetryClient::isConnected() const { return connected_; }

const TcpLinkStatus& TcpTelemetryClient::status() const { return status_; }

void TcpTelemetryClient::connectTo(const std::string& host) {
    if (!running_) {
        return;
    }
    // Tear down any prior link / pending retry before re-aiming. Cancelling
    // here is the deliberate delta over the Swift source: a discovery win is
    // never disturbed by a stale scheduled retry at the old host.
    scheduler_->cancel(reconnectToken_);
    reconnectToken_ = 0;
    tcp_->close();
    buffer_.clear();
    columnMap_.reset();

    notifyStatus(TcpLinkState::Connecting, host, config_.port);

    // Wire the event sinks. Sinks capture nothing that outlives the client:
    // stop() closes the transport first, and the owner destroys client and
    // transport together.
    tcp_->onConnected = [this]() {
        stickyHost_ = status_.host;  // a successful connect pins the target
        retryCount_ = 0;
        connected_ = true;
        notifyStatus(TcpLinkState::Connected, status_.host, config_.port);
        // AUTH handshake per the firmware's TCP contract. A host that does not
        // require auth ignores the line; the parser skips any non-CSV reply
        // (e.g. "OK") until the real CSV header arrives.
        tcp_->send("AUTH " + config_.authToken + "\r\n");
    };
    tcp_->onData = [this](const std::string& bytes) { feedBuffer(bytes); };
    tcp_->onDisconnected = [this]() { handleDisconnected(status_.host); };

    // notifyStatus(Connecting, ...) already snapshotted the re-aimed host into
    // status_; the sinks read it from there.
    tcp_->connect(host, config_.port);
}

void TcpTelemetryClient::handleDisconnected(const std::string& host) {
    connected_ = false;
    if (!running_) {
        return;
    }
    scheduleReconnect(host);
}

void TcpTelemetryClient::feedBuffer(const std::string& bytes) {
    buffer_ += bytes;
    // Drain complete lines (split on \n; tolerate \r\n; trim; skip blanks).
    std::size_t start = 0;
    while (true) {
        const std::size_t newline = buffer_.find('\n', start);
        if (newline == std::string::npos) {
            break;
        }
        const std::string line = buffer_.substr(start, newline - start);
        start = newline + 1;
        handleLine(line);
    }
    buffer_.erase(0, start);
}

void TcpTelemetryClient::handleLine(const std::string& raw) {
    // Trim ASCII whitespace/newlines (Swift: .whitespacesAndNewlines).
    static const char* kWs = " \t\r\n\v\f";
    const auto begin = raw.find_first_not_of(kWs);
    if (begin == std::string::npos) {
        return;  // blank line
    }
    const auto end = raw.find_last_not_of(kWs);
    const std::string line = raw.substr(begin, end - begin + 1);

    // Until a header locks in, probe each line as a candidate header (this
    // tolerates auth banners / non-CSV prelude lines).
    if (!columnMap_) {
        columnMap_ = TcpCsvFrameParser::tryHeader(line);
        return;
    }
    const std::optional<TcpTelemetryFrame> frame =
        TcpCsvFrameParser::parseRow(line, columnMap_.value());
    if (!frame) {
        return;  // banner / keepalive / malformed row — ignore
    }
    if (onFrame) {
        onFrame(frame.value());
    }
}

void TcpTelemetryClient::scheduleReconnect(const std::string& host) {
    scheduler_->cancel(reconnectToken_);
    const int attempt = retryCount_++;
    const int delayMs = reconnectBackoffMs(attempt, config_.baseRetryDelayMs,
                                           config_.maxRetryDelayMs);
    reconnectToken_ = scheduler_->post(delayMs, [this, host]() {
        if (running_) {
            connectTo(host);
        }
    });
}

void TcpTelemetryClient::startDiscovery() {
    if (!discovery_) {
        return;
    }
    discovery_->onPacket =
        [this](const std::string& bytes, const std::string& senderHost) {
            const std::string newHost = processDiscoveryPacket(bytes, senderHost);
            if (!newHost.empty()) {
                connectTo(newHost);
            }
        };
    // Best-effort: a bound failure keeps the sticky reconnect working.
    discovery_->listen(config_.discoveryPort);
}

void TcpTelemetryClient::stopDiscovery() {
    if (discovery_) {
        discovery_->close();
    }
}

bool TcpTelemetryClient::isVehicleSimDiscovery(const std::string& bytes) {
    static const std::array<unsigned char, 4> kMagic = {0x56, 0x53, 0x49, 0x4D};  // "VSIM"
    if (bytes.size() < 5) {
        return false;
    }
    for (std::size_t i = 0; i < std::size(kMagic); ++i) {
        if (static_cast<unsigned char>(bytes[i]) !=
            static_cast<unsigned char>(kMagic[i])) {
            return false;
        }
    }
    return static_cast<unsigned char>(bytes[4]) == 1;
}

std::string TcpTelemetryClient::processDiscoveryPacket(const std::string& bytes,
                                                       const std::string& senderHost) const {
    if (!isVehicleSimDiscovery(bytes) || !running_) {
        return "";
    }
    if (senderHost.empty() || senderHost == stickyHost_) {
        return "";
    }
    return senderHost;
}

void TcpTelemetryClient::notifyStatus(TcpLinkState state, std::string host, uint16_t port) {
    status_.state = state;
    status_.host = std::move(host);
    status_.port = port;
    if (onStatus) {
        onStatus(status_);
    }
}

} // namespace input
