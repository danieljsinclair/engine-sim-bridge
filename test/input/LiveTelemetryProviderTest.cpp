// LiveTelemetryProviderTest.cpp - Contract tests for LiveTelemetryProvider
//
// LiveTelemetryProvider.cpp is a dead-stripped file (120 LOC, 0% coverage) that
// wraps a VirtualIceInputProvider (the "twin") behind a thread-safe signal inbox.
// These tests assert the OBSERVABLE contract:
//   - Initialize() success makes the provider connected after a signal is submitted
//   - Initialize() called again returns false (already-initialized guard)
//   - OnUpdateSimulation before init returns empty input with a "not initialized" error
//   - submitSignal round-trips through getCurrentSignal
//   - submitSignal(timed) stamps the signal's timestamp
//   - Shutdown() disconnects the provider
//   - setGearSelector/setIgnition/provideFeedback delegate to the twin (no-crash
//     with a real twin, observed via IsConnected remaining true)
//
// The inner twin-failure branch (twin provider's Initialize fails) is NOT covered:
// VirtualIceInputProvider::Initialize only fails on an exception, which cannot be
// triggered from the public profile seam without hacking production code. Per the
// spec we accept the happy path + idempotency + not-init guard as the core.

#include "input/LiveTelemetryProvider.h"
#include "twin/IceVehicleProfile.h"
#include "simulator/EngineSimTypes.h"

#include <gtest/gtest.h>

namespace {

class LiveTelemetryProviderTest : public ::testing::Test {
protected:
    void SetUp() override {
        profile_ = twin::IceVehicleProfile::zf8hp45();
        provider_ = std::make_unique<input::LiveTelemetryProvider>(profile_);
    }

    twin::IceVehicleProfile profile_;
    std::unique_ptr<input::LiveTelemetryProvider> provider_;
};

// Initialize() success path: provider becomes connected only after a signal is
// submitted (signalReceived_ gate).
TEST_F(LiveTelemetryProviderTest, InitializeConnectsAfterSignalSubmitted) {
    ASSERT_TRUE(provider_->Initialize());

    // Not connected until a signal arrives.
    EXPECT_FALSE(provider_->IsConnected());

    input::UpstreamSignal signal;
    signal.isValid = true;
    signal.throttleFraction = 0.5;
    provider_->submitSignal(signal);

    EXPECT_TRUE(provider_->IsConnected());
}

// Initialize() again returns false — already-initialized guard, with an error set.
TEST_F(LiveTelemetryProviderTest, InitializeTwiceReturnsFalse) {
    ASSERT_TRUE(provider_->Initialize());
    EXPECT_FALSE(provider_->Initialize());
    EXPECT_FALSE(provider_->GetLastError().empty());
}

// OnUpdateSimulation before init returns empty input and records a not-initialized
// error (intent, not exact message).
TEST_F(LiveTelemetryProviderTest, OnUpdateBeforeInitReturnsEmptyWithError) {
    EXPECT_FALSE(provider_->IsConnected());

    input::EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_EQ(input.throttle, 0.0);
    EXPECT_EQ(input.gearAbsolute, -1);

    // Intent: error communicates the provider is not initialized.
    EXPECT_FALSE(provider_->GetLastError().empty());
    EXPECT_NE(provider_->GetLastError().find("not initialized"), std::string::npos);
}

// submitSignal + getCurrentSignal round-trip the latest signal (atomic store/load).
TEST_F(LiveTelemetryProviderTest, SubmitAndGetCurrentSignalRoundTrip) {
    ASSERT_TRUE(provider_->Initialize());

    input::UpstreamSignal signal;
    signal.throttleFraction = 0.77;
    signal.speedKmh = 50.0;
    signal.isValid = true;
    provider_->submitSignal(signal);

    input::UpstreamSignal got = provider_->getCurrentSignal();
    EXPECT_DOUBLE_EQ(got.throttleFraction, 0.77);
    EXPECT_DOUBLE_EQ(got.speedKmh, 50.0);
    EXPECT_TRUE(got.isValid);
}

// submitSignal(timed) stamps the signal's timestampUtcMs.
TEST_F(LiveTelemetryProviderTest, SubmitWithTimestampSetsTimestamp) {
    ASSERT_TRUE(provider_->Initialize());

    input::UpstreamSignal signal;
    signal.isValid = true;
    provider_->submitSignal(signal, 1700000000123ULL);

    EXPECT_EQ(provider_->getCurrentSignal().timestampUtcMs, 1700000000123ULL);
}

// Shutdown() disconnects the provider.
TEST_F(LiveTelemetryProviderTest, ShutdownDisconnects) {
    ASSERT_TRUE(provider_->Initialize());
    provider_->submitSignal(input::UpstreamSignal{});

    ASSERT_TRUE(provider_->IsConnected());
    provider_->Shutdown();
    EXPECT_FALSE(provider_->IsConnected());
}

// setGearSelector / setIgnition / provideFeedback delegate to the twin without
// crashing (observed: provider stays connected after the delegated calls).
TEST_F(LiveTelemetryProviderTest, DelegatesForwardToTwin) {
    ASSERT_TRUE(provider_->Initialize());
    provider_->submitSignal(input::UpstreamSignal{});

    // These must not throw and must not break the connection.
    EXPECT_NO_THROW({
        provider_->setGearSelector(2);
        provider_->setIgnition(true);
        EngineSimStats stats;
        provider_->provideFeedback(stats);
    });

    EXPECT_TRUE(provider_->IsConnected());
}

}  // namespace
