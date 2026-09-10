// TcpTelemetryClientTests.cpp
//
// Port of EngineSimApp/Tests/TcpTelemetrySourceTests.swift onto
// TcpTelemetryClient. No real sockets: ScriptedTcpTransport /
// ScriptedUdpListener / ManualScheduler record calls and fire the client's
// sinks synchronously from the test thread (io/ITransport.h contract).

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "io/TcpTelemetryClient.h"

using input::ITcpTransport;
using input::IScheduler;
using input::IUdpListener;
using input::TcpLinkState;
using input::TcpTelemetryClient;
using input::TcpTelemetryConfig;
using input::TcpTelemetryFrame;

namespace {

// ---- Scripted fakes ---------------------------------------------------------

/// Fires onConnected synchronously on connect(), records sent bytes.
class ScriptedTcpTransport : public ITcpTransport {
public:
    void connect(const std::string& host, uint16_t port) override {
        connectedHost = host;
        connectedPort = port;
        connectCalls++;
        if (shouldConnect && onConnected) {
            onConnected();
        }
    }
    void send(const std::string& bytes) override { sent.push_back(bytes); }
    void close() override { closeCalls++; }

    /// Deliver received stream bytes to the client.
    void injectData(const std::string& bytes) {
        if (onData) {
            onData(bytes);
        }
    }
    /// Simulate the remote end dropping the link.
    void injectDisconnected() {
        if (onDisconnected) {
            onDisconnected();
        }
    }

    std::vector<std::string> sent;
    std::string connectedHost;
    uint16_t connectedPort = 0;
    int connectCalls = 0;
    int closeCalls = 0;
    bool shouldConnect = true;
};

/// Records listen/close; deliverPacket() fires onPacket synchronously.
class ScriptedUdpListener : public IUdpListener {
public:
    bool listen(uint16_t port) override {
        listenCalls++;
        listenedPort = port;
        return true;
    }
    void close() override { closeCalls++; }
    void deliverPacket(const std::string& bytes, const std::string& sender) {
        if (onPacket) {
            onPacket(bytes, sender);
        }
    }

    int listenCalls = 0;
    int closeCalls = 0;
    uint16_t listenedPort = 0;
};

/// Records posts and cancels; firePost() executes a pending task inline.
class ManualScheduler : public IScheduler {
public:
    uint64_t post(int delayMs, std::function<void()> fn) override {
        posts.emplace_back(nextToken, delayMs);
        tasks.emplace_back(nextToken, std::move(fn));
        return nextToken++;
    }
    void cancel(uint64_t token) override { cancelled.push_back(token); }

    /// Execute the task posted under `token`, if still pending.
    bool firePost(uint64_t token) {
        for (std::size_t i = 0; i < tasks.size(); ++i) {
            if (tasks[i].first == token) {
                std::function<void()> fn = tasks[i].second;
                tasks.erase(tasks.begin() + static_cast<long>(i));
                fn();
                return true;
            }
        }
        return false;
    }

    std::vector<std::pair<uint64_t, int>> posts;  // (token, delayMs)
    std::vector<uint64_t> cancelled;
    std::vector<std::pair<uint64_t, std::function<void()>>> tasks;
    uint64_t nextToken = 1;
};

/// Client wired to the scripted seams with the Swift tests' sticky target.
class TcpTelemetryClientTests : public ::testing::Test {
protected:
    void SetUp() override {
        config_.host = "192.168.1.100";
        client_ = std::make_unique<TcpTelemetryClient>(&tcp_, &udp_, &scheduler_, config_);
    }

    TcpTelemetryConfig config_;
    ScriptedTcpTransport tcp_;
    ScriptedUdpListener udp_;
    ManualScheduler scheduler_;
    std::unique_ptr<TcpTelemetryClient> client_;
};

const std::string kValidDiscovery = std::string({0x56, 0x53, 0x49, 0x4D, 0x01});

} // namespace

// ---- feedBuffer / handleLine (Swift happy path) ------------------------------

TEST_F(TcpTelemetryClientTests, MultiLineFeedEmitsFramesInOrder) {
    std::vector<TcpTelemetryFrame> frames;
    client_->onFrame = [&frames](const TcpTelemetryFrame& f) { frames.push_back(f); };

    const std::string csv =
        "timestamp_ms,speed_kmh,throttle_percent,brake_percent,acceleration_g\n"
        "1000,60.0,40.0,0.0,0.3\n"
        "2000,80.0,55.0,10.0,0.2\n"
        "3000,100.0,70.0,20.0,0.1\n";
    client_->feedBuffer(csv);

    ASSERT_EQ(frames.size(), 3u);
    EXPECT_DOUBLE_EQ(frames[0].timestampMs, 1000.0);
    EXPECT_DOUBLE_EQ(frames[0].speedKmh.value(), 60.0);
    EXPECT_DOUBLE_EQ(frames[0].throttle.value(), 40.0);
    EXPECT_DOUBLE_EQ(frames[1].timestampMs, 2000.0);
    EXPECT_DOUBLE_EQ(frames[1].speedKmh.value(), 80.0);
    EXPECT_DOUBLE_EQ(frames[2].timestampMs, 3000.0);
}

TEST_F(TcpTelemetryClientTests, CrlfTolerated) {
    std::vector<TcpTelemetryFrame> frames;
    client_->onFrame = [&frames](const TcpTelemetryFrame& f) { frames.push_back(f); };

    client_->feedBuffer("timestamp_ms,speed_kmh\r\n1000,60.0\r\n");

    ASSERT_EQ(frames.size(), 1u);
    EXPECT_DOUBLE_EQ(frames[0].timestampMs, 1000.0);
    EXPECT_DOUBLE_EQ(frames[0].speedKmh.value(), 60.0);
}

TEST_F(TcpTelemetryClientTests, BlankLinesSkipped) {
    std::vector<TcpTelemetryFrame> frames;
    client_->onFrame = [&frames](const TcpTelemetryFrame& f) { frames.push_back(f); };

    client_->feedBuffer("timestamp_ms,speed_kmh\n\n1000,60.0\n\n2000,80.0\n\n");

    ASSERT_EQ(frames.size(), 2u);
    EXPECT_DOUBLE_EQ(frames[0].timestampMs, 1000.0);
    EXPECT_DOUBLE_EQ(frames[1].timestampMs, 2000.0);
}

TEST_F(TcpTelemetryClientTests, PartialTrailingLineStaysBufferedUntilNewline) {
    std::vector<TcpTelemetryFrame> frames;
    client_->onFrame = [&frames](const TcpTelemetryFrame& f) { frames.push_back(f); };

    client_->feedBuffer("timestamp_ms,speed_kmh\n");
    client_->feedBuffer("1000,60.0");  // no newline yet
    EXPECT_TRUE(frames.empty());

    client_->feedBuffer("\n");
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_DOUBLE_EQ(frames[0].timestampMs, 1000.0);
    EXPECT_DOUBLE_EQ(frames[0].speedKmh.value(), 60.0);
}

TEST_F(TcpTelemetryClientTests, AuthBannerOkIgnoredBeforeHeader) {
    std::vector<TcpTelemetryFrame> frames;
    client_->onFrame = [&frames](const TcpTelemetryFrame& f) { frames.push_back(f); };

    client_->feedBuffer("OK\r\ntimestamp_ms,speed_kmh\r\n1000,60.0\r\n");

    ASSERT_EQ(frames.size(), 1u);
    EXPECT_DOUBLE_EQ(frames[0].timestampMs, 1000.0);
    EXPECT_DOUBLE_EQ(frames[0].speedKmh.value(), 60.0);
}

// ---- AUTH handshake ----------------------------------------------------------

TEST_F(TcpTelemetryClientTests, StartSendsAuthLineAfterOnConnected) {
    client_->start();

    // The scripted transport fires onConnected inside connect(); the client
    // must send the exact auth line as the first bytes on the link.
    ASSERT_EQ(tcp_.sent.size(), 1u);
    EXPECT_EQ(tcp_.sent[0], "AUTH vehicle-sim-2026\r\n");
    EXPECT_EQ(tcp_.connectedHost, "192.168.1.100");
    EXPECT_EQ(tcp_.connectedPort, 3333);  // config default port
    EXPECT_TRUE(client_->isConnected());
}

// ---- Discovery validation (Swift isVehicleSimDiscovery) -----------------------

TEST_F(TcpTelemetryClientTests, ValidVsimDiscoveryAccepted) {
    EXPECT_TRUE(TcpTelemetryClient::isVehicleSimDiscovery(kValidDiscovery));
}

TEST_F(TcpTelemetryClientTests, ShortPacketRejected) {
    // Only the 4 magic bytes; protocol version byte is required.
    const std::string shortPacket = std::string({0x56, 0x53, 0x49, 0x4D});
    EXPECT_FALSE(TcpTelemetryClient::isVehicleSimDiscovery(shortPacket));
}

TEST_F(TcpTelemetryClientTests, WrongMagicRejected) {
    const std::string wrongMagic = std::string({0x56, 0x53, 0x49, 0x4E, 0x01});  // 'N'
    EXPECT_FALSE(TcpTelemetryClient::isVehicleSimDiscovery(wrongMagic));
}

TEST_F(TcpTelemetryClientTests, WrongVersionRejected) {
    const std::string wrongVersion = std::string({0x56, 0x53, 0x49, 0x4D, 0x02});
    EXPECT_FALSE(TcpTelemetryClient::isVehicleSimDiscovery(wrongVersion));
}

// ---- processDiscoveryPacket ---------------------------------------------------

TEST_F(TcpTelemetryClientTests, ReAimsAtDifferentHostWhenRunning) {
    client_->start();  // enters running state
    EXPECT_EQ(client_->processDiscoveryPacket(kValidDiscovery, "192.168.1.200"),
              "192.168.1.200");
}

TEST_F(TcpTelemetryClientTests, NoReAimWhenSenderIsStickyTarget) {
    client_->start();
    EXPECT_TRUE(client_->processDiscoveryPacket(kValidDiscovery, "192.168.1.100").empty());
}

TEST_F(TcpTelemetryClientTests, NoReAimForInvalidPacket) {
    const std::string wrongMagic = std::string({0x56, 0x53, 0x49, 0x4E, 0x01});
    EXPECT_TRUE(client_->processDiscoveryPacket(wrongMagic, "192.168.1.200").empty());
}

// ---- Reconnect backoff (pure function) -----------------------------------------

TEST_F(TcpTelemetryClientTests, BackoffFirstFourAttemptsAreBase) {
    EXPECT_EQ(input::reconnectBackoffMs(0), 250);
    EXPECT_EQ(input::reconnectBackoffMs(1), 250);
    EXPECT_EQ(input::reconnectBackoffMs(2), 250);
    EXPECT_EQ(input::reconnectBackoffMs(3), 250);
}

TEST_F(TcpTelemetryClientTests, BackoffEscalatesExponentially) {
    EXPECT_EQ(input::reconnectBackoffMs(4), 500);   // 250 << 1
    EXPECT_EQ(input::reconnectBackoffMs(5), 1000);  // 250 << 2
    EXPECT_EQ(input::reconnectBackoffMs(6), 2000);  // 250 << 3
}

TEST_F(TcpTelemetryClientTests, BackoffCapsAtMax) {
    EXPECT_EQ(input::reconnectBackoffMs(20), 8000);  // 250 << 17 capped
}

TEST_F(TcpTelemetryClientTests, BackoffExponentClampedAt24) {
    // The clamp branch (exponent pinned at 24) with a base whose shift stays
    // inside int range: 1 << 24 overflows nothing and caps at max.
    EXPECT_EQ(input::reconnectBackoffMs(27, 1, 8000), 8000);
    EXPECT_EQ(input::reconnectBackoffMs(1000, 1, 8000), 8000);
    // The production shape (base 250) at the clamped exponent overflows a
    // 32-bit signed shift (250 << 24 > INT32_MAX) — the widened 64-bit shift
    // must cap cleanly at max instead of UB/negative.
    EXPECT_EQ(input::reconnectBackoffMs(27, 250, 8000), 8000);
    EXPECT_EQ(input::reconnectBackoffMs(1000, 250, 8000), 8000);
}

// ---- Start/stop contract --------------------------------------------------------

TEST_F(TcpTelemetryClientTests, StopBeforeStartDoesNotCrashAndIsConnectedFalse) {
    client_->stop();
    EXPECT_FALSE(client_->isConnected());
    EXPECT_FALSE(client_->isRunning());
}

TEST_F(TcpTelemetryClientTests, StopCancelsPendingReconnect) {
    client_->start();
    tcp_.injectDisconnected();  // link drops while running -> reconnect posted
    ASSERT_EQ(scheduler_.posts.size(), 1u);
    EXPECT_EQ(scheduler_.posts[0].second, 250);  // first retry holds the base

    client_->stop();
    // Intent: the pending reconnect token is cancelled. (The contract also
    // issues placeholder cancel(0) calls on connectTo/scheduleReconnect, so
    // the cancelled list may legitimately hold more than one entry.)
    const auto& cancelled = scheduler_.cancelled;
    EXPECT_NE(std::find(cancelled.begin(), cancelled.end(),
                        scheduler_.posts[0].first),
              cancelled.end());
    EXPECT_FALSE(client_->isConnected());
}

TEST_F(TcpTelemetryClientTests, DiscoveryWinCancelsPendingReconnect) {
    // Deliberate delta over the Swift source: connectTo() (discovery win)
    // cancels a pending retry so a stale re-fire cannot race the re-aim.
    // (Placeholder cancel(0) calls are expected too — see StopCancels.)
    client_->start();
    tcp_.injectDisconnected();
    ASSERT_EQ(scheduler_.posts.size(), 1u);

    udp_.deliverPacket(kValidDiscovery, "192.168.1.200");

    ASSERT_EQ(tcp_.connectCalls, 2);  // initial + re-aim
    EXPECT_EQ(tcp_.connectedHost, "192.168.1.200");
    const auto& cancelled = scheduler_.cancelled;
    EXPECT_NE(std::find(cancelled.begin(), cancelled.end(),
                        scheduler_.posts[0].first),
              cancelled.end());
}

TEST_F(TcpTelemetryClientTests, StatusMirrorsLifecycle) {
    std::vector<TcpLinkState> states;
    client_->onStatus = [&states](const input::TcpLinkStatus& s) { states.push_back(s.state); };

    client_->start();
    EXPECT_EQ(client_->status().state, TcpLinkState::Connected);
    // A drop clears the connected flag; the state machine emits its next
    // status when the retry fires connectTo (Connecting) or on stop.
    tcp_.injectDisconnected();
    EXPECT_FALSE(client_->isConnected());
    client_->stop();
    EXPECT_EQ(client_->status().state, TcpLinkState::Disconnected);

    ASSERT_FALSE(states.empty());
    EXPECT_EQ(states.front(), TcpLinkState::Connecting);
    EXPECT_EQ(states.back(), TcpLinkState::Disconnected);
}
