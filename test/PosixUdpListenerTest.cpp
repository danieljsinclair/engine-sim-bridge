// PosixUdpListenerTest.cpp
//
// Direct loopback tests for PosixUdpListener over real 127.0.0.1 sockets.
// The TCP loopback suite never enables discovery, so nothing else drives the
// UDP receive loop; these pins are its only coverage. They pin the public
// contract — listen() binds, a datagram is delivered with its payload and
// dotted-quad sender, close()/destructor reap the receiver, and listen() can
// be reused after a reap — so any internal refactor of the loop must keep
// them green.

#include <gtest/gtest.h>

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <future>
#include <string>
#include <utility>

#include "io/PosixTransports.h"

using input::PosixUdpListener;

namespace {

/// A free 127.0.0.1 UDP port for this run: bind a probe socket to port 0,
/// read the OS-assigned port, close the probe. IUdpListener exposes no
/// bound-port getter, so the test must choose the port itself. On any probe
/// failure this returns 0, and listen(0) then fails the test downstream
/// instead of the helper aborting (EXPECT_, not ASSERT_: non-void return).
uint16_t freeLoopbackPort() {
    constexpr uint16_t kUnbound = 0;
    const int probe = ::socket(AF_INET, SOCK_DGRAM, 0);
    EXPECT_NE(probe, -1);
    if (probe == -1) {
        return kUnbound;
    }
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    socklen_t len = sizeof(addr);
    EXPECT_EQ(::bind(probe, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)), 0);
    EXPECT_EQ(::getsockname(probe, reinterpret_cast<sockaddr*>(&addr), &len), 0);
    if (addr.sin_port != 0) {
        ::close(probe);
        return ntohs(addr.sin_port);
    }
    ::close(probe);
    return kUnbound;
}

/// Sends one datagram from an independent socket to 127.0.0.1:port.
void sendDatagram(uint16_t port, const std::string& bytes) {
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    ASSERT_NE(fd, -1);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port);
    const ssize_t sent =
        ::sendto(fd, bytes.data(), bytes.size(), 0,
                 reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    EXPECT_EQ(sent, static_cast<ssize_t>(bytes.size()));
    ::close(fd);
}

} // namespace

TEST(PosixUdpListenerTest, ReceiveLoopDeliversPayloadAndDottedQuadSender) {
    PosixUdpListener listener;
    std::promise<std::pair<std::string, std::string>> received;
    listener.onPacket =
        [&received](const std::string& bytes, const std::string& senderHost) {
            received.set_value({bytes, senderHost});
        };

    const uint16_t port = freeLoopbackPort();
    ASSERT_TRUE(listener.listen(port));
    sendDatagram(port, "V-SIM-DISCOVER 1");

    // Deadline wait on a real event (never sleep-as-sync): the receive tick
    // is 200 ms, so a healthy loop delivers far inside this budget.
    std::future<std::pair<std::string, std::string>> delivered = received.get_future();
    ASSERT_EQ(delivered.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    const auto packet = delivered.get();
    EXPECT_EQ(packet.first, "V-SIM-DISCOVER 1");
    EXPECT_EQ(packet.second, "127.0.0.1");
}

TEST(PosixUdpListenerTest, CloseReapsReceiverAndListenCanBeReused) {
    PosixUdpListener listener;
    ASSERT_TRUE(listener.listen(freeLoopbackPort()));

    // close() signals the loop; the receiver ticks out of its 200 ms
    // SO_RCVTIMEO and exits. The reap itself happens on the next join, so a
    // fresh listen() succeeding is the observable reap contract.
    listener.close();
    EXPECT_TRUE(listener.listen(freeLoopbackPort()));
    listener.close();
}

TEST(PosixUdpListenerTest, DestructorWhileListeningJoinsCleanly) {
    {
        PosixUdpListener listener;
        ASSERT_TRUE(listener.listen(freeLoopbackPort()));
        // Destructor while the receiver ticks: the inline close + join must
        // complete without racing a parked recvfrom.
    }
    SUCCEED();
}
