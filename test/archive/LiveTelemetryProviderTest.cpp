// LiveTelemetryProviderTest.cpp
//
// Tests for LiveTelemetryProvider — reads CSV from an input stream,
// step-holds the last sample, and sets IsConnected=false on EOF.
//
// Uses std::istringstream injection for deterministic testing.

#include <gtest/gtest.h>
#include <input/LiveTelemetryProvider.h>

#include <memory>
#include <sstream>
#include <string>

using namespace input;

// ============================================================================
// Test Fixture
// ============================================================================

class LiveTelemetryProviderTest : public ::testing::Test {
protected:
    void makeProvider(const std::string& csv, bool autoStart = true) {
        stream_ = std::make_unique<std::istringstream>(csv);
        provider_ = std::make_unique<LiveTelemetryProvider>(*stream_, autoStart);
    }

    std::unique_ptr<std::istringstream> stream_;
    std::unique_ptr<LiveTelemetryProvider> provider_;
};

// ============================================================================
// Lifecycle
// ============================================================================

TEST_F(LiveTelemetryProviderTest, Initialize_SetsConnectedTrue) {
    makeProvider("time_s,throttle_pct\n0.0,0.0\n");
    EXPECT_TRUE(provider_->Initialize());
    EXPECT_TRUE(provider_->IsConnected());
}

TEST_F(LiveTelemetryProviderTest, Shutdown_SetsConnectedFalse) {
    makeProvider("time_s,throttle_pct\n0.0,0.0\n");
    ASSERT_TRUE(provider_->Initialize());
    provider_->Shutdown();
    EXPECT_FALSE(provider_->IsConnected());
}

TEST_F(LiveTelemetryProviderTest, ProviderName_IsLiveTelemetry) {
    makeProvider("time_s,throttle_pct\n0.0,0.0\n");
    EXPECT_EQ(provider_->GetProviderName(), "LiveTelemetry");
}

// ============================================================================
// CSV header parsing: first line is consumed on first OnUpdateSimulation call.
// ============================================================================

TEST_F(LiveTelemetryProviderTest, OnUpdateSimulation_ParsesHeader) {
    makeProvider(
        "time_s,throttle_pct,road_speed_kmh,gear,clutch_pct\n"
        "0.0,50.0,10.0,1,100.0\n");

    ASSERT_TRUE(provider_->Initialize());

    // First call consumes the header line (no data sample yet).
    EngineInput input1 = provider_->OnUpdateSimulation(0.016);
    EXPECT_TRUE(provider_->IsConnected());
    // No sample received yet, so throttle defaults to 0.
    EXPECT_DOUBLE_EQ(input1.throttle, 0.0);

    // Second call consumes the first data row.
    EngineInput input2 = provider_->OnUpdateSimulation(0.016);
    EXPECT_TRUE(provider_->IsConnected());
    EXPECT_DOUBLE_EQ(input2.throttle, 0.5);  // 50% -> 0.5
}

// ============================================================================
// Step-hold: between row arrivals, the last sample is held
// ============================================================================

TEST_F(LiveTelemetryProviderTest, StepHold_HoldsLastSampleBetweenRows) {
    makeProvider(
        "time_s,throttle_pct,road_speed_kmh,gear,clutch_pct\n"
        "0.0,80.0,20.0,2,100.0\n");

    ASSERT_TRUE(provider_->Initialize());

    // First update: consumes the header.
    provider_->OnUpdateSimulation(0.016);

    // Second update: reads the first data row.
    EngineInput input1 = provider_->OnUpdateSimulation(0.016);
    EXPECT_DOUBLE_EQ(input1.throttle, 0.8);
    EXPECT_DOUBLE_EQ(input1.roadSpeedKmh, 20.0);

    // Third update: no new data available, should hold the last sample.
    EngineInput input2 = provider_->OnUpdateSimulation(0.016);
    EXPECT_DOUBLE_EQ(input2.throttle, 0.8);
    EXPECT_DOUBLE_EQ(input2.roadSpeedKmh, 20.0);

    // Fourth update: still holding.
    EngineInput input3 = provider_->OnUpdateSimulation(0.016);
    EXPECT_DOUBLE_EQ(input3.throttle, 0.8);
}

// ============================================================================
// EOF: sets IsConnected false without crash
// ============================================================================

TEST_F(LiveTelemetryProviderTest, Eof_SetsConnectedFalse) {
    makeProvider(
        "time_s,throttle_pct,road_speed_kmh,gear,clutch_pct\n"
        "0.0,50.0,10.0,1,100.0\n");

    ASSERT_TRUE(provider_->Initialize());

    // Consume header.
    provider_->OnUpdateSimulation(0.016);
    EXPECT_TRUE(provider_->IsConnected());

    // Consume the data row.
    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_TRUE(provider_->IsConnected());
    EXPECT_DOUBLE_EQ(input.throttle, 0.5);

    // Next call should hit EOF (stream has no more data).
    provider_->OnUpdateSimulation(0.016);
    EXPECT_FALSE(provider_->IsConnected());

    // Should not crash on subsequent calls; last sample still held.
    EngineInput input3 = provider_->OnUpdateSimulation(0.016);
    EXPECT_FALSE(provider_->IsConnected());
    EXPECT_DOUBLE_EQ(input3.throttle, 0.5);
}

// ============================================================================
// Auto-start: fires starterButton on the first frame
// ============================================================================

TEST_F(LiveTelemetryProviderTest, AutoStart_FiresStarterOnFirstFrame) {
    makeProvider(
        "time_s,throttle_pct,road_speed_kmh,gear,clutch_pct\n"
        "0.0,0.0,0.0,0,100.0\n");

    ASSERT_TRUE(provider_->Initialize());

    // First frame: header consumed, starter fires.
    EngineInput input1 = provider_->OnUpdateSimulation(0.016);
    EXPECT_TRUE(input1.starterButton);

    // Second frame: data row consumed, starter off.
    EngineInput input2 = provider_->OnUpdateSimulation(0.016);
    EXPECT_FALSE(input2.starterButton);
}

// ============================================================================
// Multiple rows: each new row updates the held sample
// ============================================================================

TEST_F(LiveTelemetryProviderTest, MultipleRows_UpdatesHeldSample) {
    makeProvider(
        "time_s,throttle_pct,road_speed_kmh,gear,clutch_pct\n"
        "0.0,20.0,5.0,1,100.0\n"
        "0.016,60.0,30.0,2,100.0\n");

    ASSERT_TRUE(provider_->Initialize());

    // First update: consumes header.
    provider_->OnUpdateSimulation(0.016);

    // Second update: reads first data row.
    EngineInput input1 = provider_->OnUpdateSimulation(0.016);
    EXPECT_DOUBLE_EQ(input1.throttle, 0.2);
    EXPECT_DOUBLE_EQ(input1.roadSpeedKmh, 5.0);

    // Third update: reads second data row.
    EngineInput input2 = provider_->OnUpdateSimulation(0.016);
    EXPECT_DOUBLE_EQ(input2.throttle, 0.6);
    EXPECT_DOUBLE_EQ(input2.roadSpeedKmh, 30.0);
}

// ============================================================================
// Default input before any data: zero throttle, default values
// ============================================================================

TEST_F(LiveTelemetryProviderTest, DefaultInput_BeforeAnyData) {
    makeProvider(
        "time_s,throttle_pct,road_speed_kmh,gear,clutch_pct\n");

    ASSERT_TRUE(provider_->Initialize());

    // Header is parsed but no data rows yet.
    EngineInput input = provider_->OnUpdateSimulation(0.016);
    // Before any sample is received, throttle should be 0.
    EXPECT_DOUBLE_EQ(input.throttle, 0.0);
    // Starter should fire on first frame (autoStart=true).
    EXPECT_TRUE(input.starterButton);
}

// ============================================================================
// Throttle normalisation: 0..100 -> 0..1
// ============================================================================

TEST_F(LiveTelemetryProviderTest, ThrottleNormalisation_0to100_to_0to1) {
    makeProvider(
        "time_s,throttle_pct,road_speed_kmh,gear,clutch_pct\n"
        "0.0,0.0,0.0,0,100.0\n"
        "0.016,100.0,0.0,0,100.0\n"
        "0.032,50.0,0.0,0,100.0\n");

    ASSERT_TRUE(provider_->Initialize());

    // Consume header.
    provider_->OnUpdateSimulation(0.016);

    EngineInput i0 = provider_->OnUpdateSimulation(0.016);
    EXPECT_DOUBLE_EQ(i0.throttle, 0.0);

    EngineInput i1 = provider_->OnUpdateSimulation(0.016);
    EXPECT_DOUBLE_EQ(i1.throttle, 1.0);

    EngineInput i2 = provider_->OnUpdateSimulation(0.016);
    EXPECT_DOUBLE_EQ(i2.throttle, 0.5);
}
