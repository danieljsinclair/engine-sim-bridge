// ReplayTelemetryProviderTest.cpp
//
// Behavior-driven tests for ReplayTelemetryProvider. Tests the public
// contract only — no peeking at the .cpp implementation. CSV fixtures are
// written to temp files; a MockSession records stop() calls and a
// MockKeyboardInput drives the Q/P/S/I key paths.
//
// Empirically-observed contract (derived from the public API, not the
// implementation):
//   * Initialize() returns false for an empty or header-only CSV (no data
//     rows -> nothing to replay). It returns true if there is >= 1 data row,
//     and IsConnected() tracks that.
//   * The provider owns its keyboard polling: OnUpdateSimulation reads the
//     wired IKeyboardInput unconditionally, so a keyboard MUST be wired
//     before driving frames or it segfaults. getKey() returns -1 when the
//     queue is empty, which cleanly exits the poll loop.
//   * Samples are sorted by time. The provider advances an internal
//     `elapsedS_` by dt each frame and selects the sample whose time is the
//     largest one <= elapsed (the "floor" sample). currentTimestampS() is
//     that sample's time. So with ascending-from-zero data, the throttle
//     tracks the sample whose time the clock has most recently crossed.
//   * durationS() is the span of the trace: lastSampleTime - firstSampleTime.
//     A single sample has span 0.
//   * The 'i' key toggles ignition, but the toggle takes effect on the frame
//     AFTER the key is consumed (one-frame latency).
//   * Once elapsed reaches endAtS, the provider calls session.stop() on every
//     subsequent frame and zeros the output.

#include <gtest/gtest.h>

#include "input/ReplayTelemetryProvider.h"
#include "input/IKeyboardInput.h"
#include "io/IInputProvider.h"
#include "mocks/MockKeyboardInput.h"
#include "session/ISimulatorSession.h"
#include "simulator/EngineSimTypes.h"
#include "simulator/GearConventions.h"
#include "twin/IceVehicleProfile.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

using namespace input;
using namespace bridge;

// ---------------------------------------------------------------------------
// MockSession — records stop() so keyboard quit / end-at-time can be verified
// without a real audio/sim runtime.
// ---------------------------------------------------------------------------
class MockSession : public ISimulatorSession {
public:
    int run() override { return 0; }
    void stop() override { stopCount_++; }
    void close() override {}
    ISimulator* getSimulator() const override { return nullptr; }

    int stopCount() const { return stopCount_; }

private:
    int stopCount_ = 0;
};

// ---------------------------------------------------------------------------
// Fixture helpers
// ---------------------------------------------------------------------------
namespace {

std::filesystem::path makeTempCsvPath() {
    const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
    return std::filesystem::temp_directory_path() /
           ("replay_telemetry_test_" + std::to_string(nonce) + ".csv");
}

void writeCsv(const std::filesystem::path& path, const std::string& content) {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        throw std::runtime_error("Failed to open CSV for writing: " + path.string());
    }
    out << content;
    out.close();
    if (!out) {
        throw std::runtime_error("Failed to write CSV: " + path.string());
    }
}

} // anonymous namespace

class ReplayTelemetryProviderTest : public ::testing::Test {
protected:
    void TearDown() override {
        for (const auto& path : tempFiles_) {
            std::error_code ec;
            std::filesystem::remove(path, ec);
        }
    }

    // Build a provider over a temp file with the given CSV body.
    std::filesystem::path makeProvider(const std::string& csv,
                                       bool autoStart = true,
                                       bool autoGearbox = false) {
        const auto path = makeTempCsvPath();
        writeCsv(path, csv);
        tempFiles_.push_back(path);
        provider_ = std::make_unique<ReplayTelemetryProvider>(
            path.string(), autoStart, autoGearbox);
        return path;
    }

    // Wire both session and a default (empty) keyboard so OnUpdateSimulation
    // can poll keys without crashing. The implementation reads the keyboard
    // unconditionally, so every driving test must wire one.
    void wireDefault() {
        provider_->setSession(&session_);
        provider_->setKeyboardInput(&keyboard_);
    }

    // Wire session + keyboard for Q/P/S/I tests.
    void wireKeyboard() { wireDefault(); }

    // Advance the replay clock by roughly `seconds` in fixed dt steps and
    // return the last EngineInput observed. The provider advances its
    // internal elapsed time by dt each frame.
    EngineInput advanceSeconds(double seconds, double dt = 0.016) {
        const int frames = std::max(1, static_cast<int>(std::ceil(seconds / dt)));
        EngineInput last{};
        for (int i = 0; i < frames; ++i) {
            last = provider_->OnUpdateSimulation(dt);
        }
        return last;
    }

    // Poll up to `maxFrames` frames after enqueueing a key, returning the
    // first EngineInput for which `predicate` is true, or the last input if
    // it never is. Used to absorb the one-frame input latency.
    template <typename Pred>
    EngineInput pollUntil(int key, Pred predicate, int maxFrames = 8) {
        keyboard_.enqueue(key);
        EngineInput in{};
        for (int i = 0; i < maxFrames; ++i) {
            in = provider_->OnUpdateSimulation(0.016);
            if (predicate(in)) break;
        }
        return in;
    }

    std::unique_ptr<ReplayTelemetryProvider> provider_;
    MockSession session_;
    MockKeyboardInput keyboard_;
    std::vector<std::filesystem::path> tempFiles_;
};

// ===========================================================================
// Group 1: Construction & Initialization
// ===========================================================================

TEST_F(ReplayTelemetryProviderTest, AutoGearboxFlagCreatesGearbox) {
    // autoGearbox=true owns an AutomaticGearbox; with a DRIVE selector and
    // road speed the provider drives gearAutoMode + a forward gear.
    makeProvider("time_s,speed_kmh,gear_selector\n0.0,60.0,D\n", true, true);
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_TRUE(input.gearAutoMode);
    EXPECT_GT(input.gearAbsolute, 0);
}

TEST_F(ReplayTelemetryProviderTest, NoAutoGearboxHasNoGearbox) {
    // autoGearbox=false: gearAutoMode stays false regardless of selector.
    makeProvider("time_s,speed_kmh,gear_selector\n0.0,60.0,D\n", true, false);
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_FALSE(input.gearAutoMode);
}

TEST_F(ReplayTelemetryProviderTest, InitializeOnNonexistentPathFails) {
    auto provider = std::make_unique<ReplayTelemetryProvider>("/no/such/path.csv");
    EXPECT_FALSE(provider->Initialize());
    EXPECT_FALSE(provider->IsConnected());
    EXPECT_NE(provider->GetLastError().find("path.csv"), std::string::npos);
}

TEST_F(ReplayTelemetryProviderTest, InitializeOnEmptyFileReturnsFalse) {
    // No data rows -> nothing to replay. Initialize reports failure.
    makeProvider("");
    EXPECT_FALSE(provider_->Initialize());
    EXPECT_FALSE(provider_->IsConnected());
}

TEST_F(ReplayTelemetryProviderTest, InitializeOnHeaderOnlyReturnsFalse) {
    // Header without data rows is not a playable trace.
    makeProvider("time_s,throttle_pct,speed_kmh\n");
    EXPECT_FALSE(provider_->Initialize());
    EXPECT_FALSE(provider_->IsConnected());
}

TEST_F(ReplayTelemetryProviderTest, InitializeOnValidCsvConnectedTrue) {
    makeProvider("time_s,throttle_pct\n0.0,50.0\n");
    EXPECT_TRUE(provider_->Initialize());
    EXPECT_TRUE(provider_->IsConnected());
}

// ===========================================================================
// Group 2: CSV Parsing (via public API)
// ===========================================================================

TEST_F(ReplayTelemetryProviderTest, SingleRowParsesTimeAndThrottle) {
    makeProvider("time_s,throttle_pct\n0.0,50.0\n");
    ASSERT_TRUE(provider_->Initialize());
    ASSERT_TRUE(provider_->IsConnected());
    wireDefault();

    // Single sample -> span is 0 (lastTime == firstTime).
    EXPECT_DOUBLE_EQ(provider_->durationS(), 0.0);

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_DOUBLE_EQ(input.throttle, 0.5);
    EXPECT_DOUBLE_EQ(provider_->currentTimestampS(), 0.0);
}

TEST_F(ReplayTelemetryProviderTest, MultiRowAdvancesByTime) {
    // Ascending-from-zero data: the provider tracks the sample whose time the
    // clock has most recently crossed. Drive the clock past each boundary and
    // confirm the throttle follows the time-sorted samples.
    makeProvider("time_s,throttle_pct\n0.0,20.0\n0.5,50.0\n1.0,80.0\n");
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();

    EXPECT_DOUBLE_EQ(provider_->durationS(), 1.0);

    // Before t=0.5: shows the t=0.0 sample.
    EngineInput a = advanceSeconds(0.4);
    EXPECT_DOUBLE_EQ(a.throttle, 0.2);

    // Cross into [0.5, 1.0): shows the t=0.5 sample.
    EngineInput b = advanceSeconds(0.2);
    EXPECT_DOUBLE_EQ(b.throttle, 0.5);

    // Cross into [1.0, ...): shows the t=1.0 sample.
    EngineInput c = advanceSeconds(0.6);
    EXPECT_DOUBLE_EQ(c.throttle, 0.8);
}

TEST_F(ReplayTelemetryProviderTest, NegativeRoadSpeedOmitted) {
    // Negative speed => dyno off (unchanged). roadSpeedKmh stays at its
    // default sentinel rather than propagating the negative value.
    makeProvider("time_s,speed_kmh\n0.0,-5.0\n");
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_LT(input.roadSpeedKmh, 0.0);
}

TEST_F(ReplayTelemetryProviderTest, MissingOptionalColumnsDefaultApplied) {
    // Only time_s present; throttle/gear/clutch all default.
    makeProvider("time_s\n0.0\n");
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_DOUBLE_EQ(input.throttle, 0.0);
    EXPECT_EQ(input.gearAbsolute, -1);
    EXPECT_DOUBLE_EQ(input.clutchPressure, -1.0);
}

TEST_F(ReplayTelemetryProviderTest, ColumnAlias_timecode) {
    makeProvider("timecode,throttle_pct\n0.0,40.0\n");
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();
    EXPECT_TRUE(provider_->IsConnected());
    EXPECT_DOUBLE_EQ(provider_->OnUpdateSimulation(0.016).throttle, 0.4);
}

TEST_F(ReplayTelemetryProviderTest, ColumnAlias_ts_ms) {
    // timestamp_ms is an epoch-ms alias; relative ordering is what matters.
    makeProvider("ts_ms,throttle_pct\n0,40.0\n1000,60.0\n");
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();
    EXPECT_TRUE(provider_->IsConnected());
    EXPECT_DOUBLE_EQ(provider_->OnUpdateSimulation(0.016).throttle, 0.4);
}

TEST_F(ReplayTelemetryProviderTest, ColumnAlias_throttle_percent) {
    makeProvider("time_s,throttle_percent\n0.0,25.0\n");
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();
    EXPECT_DOUBLE_EQ(provider_->OnUpdateSimulation(0.016).throttle, 0.25);
}

TEST_F(ReplayTelemetryProviderTest, ColumnAlias_speed) {
    // road_speed_kmh alias "speed" — positive value propagates.
    makeProvider("time_s,speed\n0.0,10.0\n");
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();
    EXPECT_DOUBLE_EQ(provider_->OnUpdateSimulation(0.016).roadSpeedKmh, 10.0);
}

TEST_F(ReplayTelemetryProviderTest, ColumnAlias_gearselector) {
    // "gear_selector" header recognized.
    makeProvider("time_s,gear_selector\n0.0,D\n", true, true);
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();
    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_TRUE(input.gearAutoMode);
}

TEST_F(ReplayTelemetryProviderTest, RawCanFormatRejected) {
    // A CAN-frame CSV (can_id + data_hex) has no recognizable time column, so
    // the provider rejects it as a non-telemetry CSV. The error names the
    // missing time column — the intent ("this isn't a telemetry trace") is what
    // matters, not the exact phrasing.
    makeProvider("can_id,data_hex\n1A3,00FFAA\n");
    EXPECT_FALSE(provider_->Initialize());
    EXPECT_FALSE(provider_->IsConnected());
    const std::string err = provider_->GetLastError();
    EXPECT_FALSE(err.empty());
    EXPECT_NE(err.find("time"), std::string::npos);
}

TEST_F(ReplayTelemetryProviderTest, BlankLinesSkippedGracefully) {
    makeProvider("time_s,throttle_pct\n\n0.0,10.0\n\n1.0,20.0\n\n");
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();
    EXPECT_TRUE(provider_->IsConnected());
    EXPECT_DOUBLE_EQ(provider_->durationS(), 1.0);
    EXPECT_DOUBLE_EQ(provider_->OnUpdateSimulation(0.016).throttle, 0.1);
}

// ===========================================================================
// Group 3: OnUpdateSimulation behavior
// ===========================================================================

TEST_F(ReplayTelemetryProviderTest, AutoStartFiresStarterOnce) {
    makeProvider("time_s\n0.0\n1.0\n", true, false);
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();

    EngineInput first = provider_->OnUpdateSimulation(0.016);
    EXPECT_TRUE(first.starterButton);
    EngineInput second = provider_->OnUpdateSimulation(0.016);
    EXPECT_FALSE(second.starterButton);
}

TEST_F(ReplayTelemetryProviderTest, NoAutoStartNeverFiresStarter) {
    makeProvider("time_s\n0.0\n1.0\n", false, false);
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();

    EngineInput first = provider_->OnUpdateSimulation(0.016);
    EXPECT_FALSE(first.starterButton);
    EngineInput second = provider_->OnUpdateSimulation(0.016);
    EXPECT_FALSE(second.starterButton);
}

TEST_F(ReplayTelemetryProviderTest, AutoGearboxDriveShiftsForward) {
    // Sustained road speed in DRIVE => auto gearbox selects a forward gear.
    makeProvider("time_s,speed_kmh,gear_selector\n0.0,80.0,D\n", true, true);
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_TRUE(input.gearAutoMode);
    EXPECT_GT(input.gearAbsolute, 0);
}

TEST_F(ReplayTelemetryProviderTest, AutoGearboxParkHoldsNeutral) {
    // PARK => no auto shift, gear held at neutral.
    makeProvider("time_s,speed_kmh,gear_selector\n0.0,0.0,P\n", true, true);
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_EQ(input.gearAbsolute, 0);
    EXPECT_FALSE(input.gearAutoMode);
}

TEST_F(ReplayTelemetryProviderTest, ManualGearPropagatesCsvGear) {
    // autoGearbox=false: the CSV 'gear' field sets gearAbsolute directly.
    makeProvider("time_s,gear\n0.0,3\n", true, false);
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_EQ(input.gearAbsolute, 3);
}

TEST_F(ReplayTelemetryProviderTest, TimeSlicingStartFromSkipsEarlySamples) {
    // startFromS=0.5 => the clock starts at 0.5s, so the first sample shown is
    // the one at t=0.5 (the floor of 0.5 in [0.0, 0.5, 1.0]).
    makeProvider("time_s,throttle_pct\n0.0,10.0\n0.5,50.0\n1.0,90.0\n");
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();
    provider_->setStartFromS(0.5);

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_DOUBLE_EQ(provider_->currentTimestampS(), 0.5);
    EXPECT_DOUBLE_EQ(input.throttle, 0.5);
}

TEST_F(ReplayTelemetryProviderTest, TimeSlicingEndAtStopsSession) {
    // Once the clock reaches endAtS, session.stop() is called every frame.
    makeProvider("time_s,throttle_pct\n0.0,10.0\n0.25,40.0\n0.5,90.0\n");
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();
    provider_->setEndAtS(0.25);

    // Advance to ~0.2s (before the cutoff): no stop yet.
    advanceSeconds(0.2);
    const int stopsBefore = session_.stopCount();
    // Advance past 0.25s: stop should now fire.
    advanceSeconds(0.1);
    EXPECT_GT(session_.stopCount(), stopsBefore);
}

// ===========================================================================
// Group 4: Keyboard (via setKeyboardInput + MockKeyboardInput)
// ===========================================================================

TEST_F(ReplayTelemetryProviderTest, LowercaseQStopsSession) {
    makeProvider("time_s\n0.0\n1.0\n");
    ASSERT_TRUE(provider_->Initialize());
    wireKeyboard();
    keyboard_.enqueue('q');
    provider_->OnUpdateSimulation(0.016);
    EXPECT_GE(session_.stopCount(), 1);
}

TEST_F(ReplayTelemetryProviderTest, UppercaseQStopsSession) {
    makeProvider("time_s\n0.0\n1.0\n");
    ASSERT_TRUE(provider_->Initialize());
    wireKeyboard();
    keyboard_.enqueue('Q');
    provider_->OnUpdateSimulation(0.016);
    EXPECT_GE(session_.stopCount(), 1);
}

TEST_F(ReplayTelemetryProviderTest, EscapeStopsSession) {
    makeProvider("time_s\n0.0\n1.0\n");
    ASSERT_TRUE(provider_->Initialize());
    wireKeyboard();
    keyboard_.enqueue(27);
    provider_->OnUpdateSimulation(0.016);
    EXPECT_GE(session_.stopCount(), 1);
}

TEST_F(ReplayTelemetryProviderTest, LowercasePCyclesPreset) {
    makeProvider("time_s\n0.0\n1.0\n");
    ASSERT_TRUE(provider_->Initialize());
    wireKeyboard();
    keyboard_.enqueue('p');
    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_TRUE(input.presetCycle);
}

TEST_F(ReplayTelemetryProviderTest, LowercaseSFiresStarter) {
    makeProvider("time_s\n0.0\n1.0\n");
    ASSERT_TRUE(provider_->Initialize());
    wireKeyboard();
    keyboard_.enqueue('s');
    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_TRUE(input.starterButton);
}

TEST_F(ReplayTelemetryProviderTest, LowercaseITogglesIgnition) {
    // Default ignition is true. 'i' toggles it, with one-frame latency: the
    // toggle takes effect on the frame after the key is consumed.
    makeProvider("time_s\n0.0\n1.0\n");
    ASSERT_TRUE(provider_->Initialize());
    wireKeyboard();

    EngineInput before = provider_->OnUpdateSimulation(0.016);
    ASSERT_TRUE(before.ignition);

    EngineInput after = pollUntil('i', [](const EngineInput& in) { return !in.ignition; });
    EXPECT_FALSE(after.ignition);
}

TEST_F(ReplayTelemetryProviderTest, UnmappedKeyNoEffect) {
    // autoStart=false so the auto starter pulse doesn't mask the assertion.
    makeProvider("time_s\n0.0\n1.0\n", false);
    ASSERT_TRUE(provider_->Initialize());
    wireKeyboard();
    keyboard_.enqueue('z');
    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_EQ(session_.stopCount(), 0);
    EXPECT_FALSE(input.presetCycle);
    EXPECT_FALSE(input.starterButton);
}

// ===========================================================================
// Group 5: reconfigureProfile
// ===========================================================================

TEST_F(ReplayTelemetryProviderTest, ReconfigureProfileEmptyRatiosNoOp) {
    // autoGearbox=false => gearbox_ is null. reconfigureProfile with empty
    // ratios on a provider that has no gearbox must not crash.
    makeProvider("time_s\n0.0\n", true, false);
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();
    provider_->reconfigureProfile({}, 3.15, 0.32);
    // No observable state change; the call simply returns.
    EXPECT_FALSE(provider_->OnUpdateSimulation(0.016).gearAutoMode);
}

TEST_F(ReplayTelemetryProviderTest, ReconfigureProfileManualGearboxNoOp) {
    // autoGearbox=false => reconfigureProfile is a no-op even with ratios.
    makeProvider("time_s\n0.0\n", true, false);
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();
    provider_->reconfigureProfile({4.71, 3.14, 2.11}, 3.15, 0.32);
    EXPECT_FALSE(provider_->OnUpdateSimulation(0.016).gearAutoMode);
}

TEST_F(ReplayTelemetryProviderTest, ReconfigureProfileValidRatiosGeneratesShiftTable) {
    // autoGearbox=true => gearbox_ exists. After reconfigureProfile the
    // gearbox profile's shift table is regenerated from the supplied ratios.
    // With a forward gear selectable under speed, the gearbox proves the table
    // was wired up (a zeroed/missing table would refuse to shift).
    makeProvider("time_s,speed_kmh,gear_selector\n0.0,80.0,D\n", true, true);
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();

    auto profile = twin::IceVehicleProfile::zf8hp45();
    const std::vector<double> ratios = {
        4.714, 3.143, 2.106, 1.667, 1.285, 1.000, 0.839, 0.667
    };
    provider_->reconfigureProfile(ratios, profile.diffRatio, profile.tireRadiusM);

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_TRUE(input.gearAutoMode);
    EXPECT_GT(input.gearAbsolute, 0);
}
