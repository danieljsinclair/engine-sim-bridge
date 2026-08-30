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

// --- DRIVE-branch output characterization (protects the auto-gearbox split) ---
// These pin the computed outputs of the DRIVE path that the existing tests
// don't assert: the vehicle-speed target, the disabled dyno floor, and the
// slip-lock clutch coupling. They are the safety net for decomposing
// OnUpdateSimulation, so they assert the public contract, not internal math.

TEST_F(ReplayTelemetryProviderTest, AutoGearboxDrivePinsVehicleSpeedAndFloor) {
    // DRIVE with a positive road speed: the auto-gearbox commands the wheels to
    // the CSV road speed (vehicleSpeedTargetKmh) and disables the downstream
    // dyno floor (engineRpmFloor == 0). The raw road speed also propagates.
    makeProvider("time_s,speed_kmh,gear_selector\n0.0,80.0,D\n", true, true);
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    ASSERT_TRUE(input.gearAutoMode);
    EXPECT_DOUBLE_EQ(input.vehicleSpeedTargetKmh, 80.0);
    EXPECT_DOUBLE_EQ(input.engineRpmFloor, 0.0);
    EXPECT_DOUBLE_EQ(input.roadSpeedKmh, 80.0);
}

TEST_F(ReplayTelemetryProviderTest, AutoGearboxDriveEngagesClutchControl) {
    // DRIVE routes through the SlipLock clutch controller, so clutchPressure is
    // a real 0..1 coupling value — not the -1 "unchanged" sentinel. The exact
    // value depends on slip-lock math + the selected gear, so assert the
    // engagement contract (set + in range), not a specific number.
    makeProvider("time_s,throttle_pct,speed_kmh,gear_selector\n0.0,50.0,30.0,D\n", true, true);
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    ASSERT_TRUE(input.gearAutoMode);
    EXPECT_NE(input.clutchPressure, -1.0);  // DRIVE overrode the sentinel
    EXPECT_GE(input.clutchPressure, 0.0);
    EXPECT_LE(input.clutchPressure, 1.0);
}

TEST_F(ReplayTelemetryProviderTest, AutoGearboxNonDriveReleasesVehicleSpeed) {
    // PARK/NEUTRAL/REVERSE must release any prior vehicle-speed constraint
    // (vehicleSpeedTargetKmh -> -1 sentinel) and force neutral, so the dyno
    // doesn't hold the engine at the last commanded road speed.
    makeProvider("time_s,speed_kmh,gear_selector\n0.0,0.0,P\n", true, true);
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_EQ(input.gearAbsolute, 0);
    EXPECT_FALSE(input.gearAutoMode);
    EXPECT_DOUBLE_EQ(input.vehicleSpeedTargetKmh, -1.0);
}

TEST_F(ReplayTelemetryProviderTest, ManualClutchPctPropagates) {
    // autoGearbox=false: the CSV clutch_pct (0..100) normalises to 0..1 clutch
    // pressure and propagates directly. 50% -> 0.5.
    makeProvider("time_s,clutch_pct\n0.0,50.0\n", true, false);
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_DOUBLE_EQ(input.clutchPressure, 0.5);
}

TEST_F(ReplayTelemetryProviderTest, TimeSlicingStartFromClockStartsAtZero) {
    // startFromS=0.5 is owned by the loop-side warm-start prefix
    // (SimulationLoop::run), NOT by the provider. The provider clock starts at 0
    // and advances by dt each frame -- there is no cold-jump/teleport to
    // startFromS. So the first OnUpdateSimulation samples the t=0 row.
    makeProvider("time_s,throttle_pct\n0.0,10.0\n0.5,50.0\n1.0,90.0\n");
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();
    provider_->setStartFromS(0.5);

    EngineInput input = provider_->OnUpdateSimulation(0.016);
    // Provider clock is at the t=0 sample (no cold-jump): timestamp 0.0,
    // throttle 10% -> 0.10.
    EXPECT_DOUBLE_EQ(provider_->currentTimestampS(), 0.0);
    EXPECT_DOUBLE_EQ(input.throttle, 0.10);
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

// ===========================================================================
// Group N: PIN-coupling compliance (--pin-tau-ms) on the replay path
// ===========================================================================
namespace {

// A road-speed staircase at the measured CAN cadence: a new held level every
// 0.181 s, +0.9 km/h per level (the "piano keys" signal the chase smooths).
std::string staircaseCsv(int levels) {
    std::string csv = "time_s,speed_kmh,gear_selector\n";
    for (int level = 0; level <= levels; ++level) {
        csv += std::to_string(level * 0.181) + "," +
               std::to_string(level * 0.9) + ",D\n";
    }
    return csv;
}

}  // namespace

TEST_F(ReplayTelemetryProviderTest, PinTauSetBeforeInitializeChasesCsvStaircase) {
    // The CLI sets --pin-tau-ms BEFORE Initialize() (the twin provider is
    // created inside Initialize), so the value must be STORED and applied to
    // the twin then. PIN mode + tau=150: the surfaced vehicleSpeedTargetKmh
    // glides between held levels instead of stepping the full 0.9 km/h.
    makeProvider(staircaseCsv(25), true, true);
    provider_->setWheelCouplingMode(twin::WheelCouplingMode::Pin);
    provider_->setPinTauMs(150.0);
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();

    double maxJump = 0.0;
    double prev = -1.0;
    for (int i = 0; i < 300; ++i) {
        const EngineInput in = provider_->OnUpdateSimulation(0.016);
        if (in.vehicleSpeedTargetKmh >= 0.0 && prev >= 0.0) {
            maxJump = std::max(maxJump, std::abs(in.vehicleSpeedTargetKmh - prev));
        }
        if (in.vehicleSpeedTargetKmh >= 0.0) prev = in.vehicleSpeedTargetKmh;
    }
    // Sanity: the pin engaged at all.
    ASSERT_GE(prev, 0.0) << "PIN mode must surface a pin target during the drive";
    EXPECT_LT(maxJump, 0.45)
        << "per-frame pin jumps must stay well under the 0.9 km/h CSV step";
}

TEST_F(ReplayTelemetryProviderTest, PinTauZeroReplayIsRigid) {
    makeProvider(staircaseCsv(8), true, true);
    provider_->setWheelCouplingMode(twin::WheelCouplingMode::Pin);
    provider_->setPinTauMs(0.0);
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();

    // tau=0: every surfaced pin target equals a held CSV level EXACTLY (the
    // rigid pin). Collect the distinct levels seen and verify each matches
    // n*0.9 to the digit (never an in-between chase value).
    bool sawPin = false;
    for (int i = 0; i < 150; ++i) {
        const EngineInput in = provider_->OnUpdateSimulation(0.016);
        if (in.vehicleSpeedTargetKmh < 0.0) continue;
        sawPin = true;
        const double level = in.vehicleSpeedTargetKmh / 0.9;
        EXPECT_NEAR(level, std::round(level), 1e-9)
            << "tau=0 must surface only exact held CSV levels, got "
            << in.vehicleSpeedTargetKmh;
    }
    ASSERT_TRUE(sawPin);
}

// ===========================================================================
// Group 6: Instant --start-from (IArrivalStatePrimer contract)
//
// The owner contract: rows before --start-from are NEVER simulated. The loop
// asks the provider to PRIME the arrival state (twin warm-boot seeded from
// the arrival row + clock anchored on it) and HOLD that row as the constant
// synthetic input while the engine core settles; on release the trace plays
// from the arrival row onward. These tests drive the provider directly.
// ===========================================================================

TEST_F(ReplayTelemetryProviderTest, PrimeAnchorsClockOnArrivalRow) {
    // After primeArrivalState() the replay clock cold-jumps to the FIRST row
    // at/after the offset and the held poll returns that row — no pre-offset
    // row is ever sampled.
    makeProvider("time_s,throttle_pct\n0.0,10.0\n0.5,50.0\n10.0,60.0\n11.0,70.0\n");
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();
    provider_->setStartFromS(10.0);
    provider_->primeArrivalState();

    EXPECT_TRUE(provider_->arrivalHoldActive());
    const EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_DOUBLE_EQ(provider_->currentTimestampS(), 10.0);
    EXPECT_DOUBLE_EQ(input.throttle, 0.60);
}

TEST_F(ReplayTelemetryProviderTest, PrimeHoldsArrivalRowAcrossPolls) {
    // While held, repeated polls keep returning the SAME arrival row: the
    // settle window must see a constant operating point, not advancing rows.
    makeProvider("time_s,throttle_pct\n0.0,10.0\n10.0,60.0\n10.5,65.0\n11.0,70.0\n");
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();
    provider_->setStartFromS(10.0);
    provider_->primeArrivalState();

    for (int i = 0; i < 100; ++i) {  // ~1.6s of settle polls
        const EngineInput input = provider_->OnUpdateSimulation(0.016);
        EXPECT_DOUBLE_EQ(provider_->currentTimestampS(), 10.0) << "poll " << i;
        EXPECT_DOUBLE_EQ(input.throttle, 0.60) << "poll " << i;
    }
}

TEST_F(ReplayTelemetryProviderTest, ArrivalRowIsFirstAtOrAfterOffset) {
    // An offset that falls BETWEEN rows anchors on the LATER row (the first
    // row at/after the offset), never the floor row before it.
    makeProvider("time_s,throttle_pct\n0.0,10.0\n10.0,60.0\n10.4,64.0\n11.0,70.0\n");
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();
    provider_->setStartFromS(10.2);
    provider_->primeArrivalState();

    const EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_DOUBLE_EQ(provider_->currentTimestampS(), 10.4);
    EXPECT_DOUBLE_EQ(input.throttle, 0.64);
}

TEST_F(ReplayTelemetryProviderTest, ReleaseResumesAdvanceFromArrivalRow) {
    // After releaseArrivalHold() the clock resumes advancing from the arrival
    // row and later rows are reached — the hold is not a permanent latch.
    makeProvider("time_s,throttle_pct\n0.0,10.0\n10.0,60.0\n10.3,63.0\n10.6,66.0\n");
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();
    provider_->setStartFromS(10.0);
    provider_->primeArrivalState();
    (void)provider_->OnUpdateSimulation(0.016);  // one held poll

    provider_->releaseArrivalHold();
    EXPECT_FALSE(provider_->arrivalHoldActive());

    // Advance ~0.64s past the arrival row: the clock moves past 10.0 and the
    // 10.6 row (throttle 66%) is reached.
    bool sawLateRow = false;
    for (int i = 0; i < 40; ++i) {
        const EngineInput input = provider_->OnUpdateSimulation(0.016);
        if (provider_->currentTimestampS() >= 10.6 - 1e-9) {
            sawLateRow = true;
            EXPECT_DOUBLE_EQ(input.throttle, 0.66);
        }
        EXPECT_GE(provider_->currentTimestampS(), 10.0 - 1e-9);  // never rewinds
    }
    EXPECT_TRUE(sawLateRow);
}

TEST_F(ReplayTelemetryProviderTest, PrimeWithoutOffsetIsNoOp) {
    // from-0 runs never prime: without an offset, primeArrivalState() must
    // not move the clock or sample anything but the t=0 row.
    makeProvider("time_s,throttle_pct\n0.0,10.0\n1.0,50.0\n");
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();
    provider_->setStartFromS(-1.0);
    provider_->primeArrivalState();

    EXPECT_FALSE(provider_->arrivalHoldActive());
    const EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_DOUBLE_EQ(provider_->currentTimestampS(), 0.0);
    EXPECT_DOUBLE_EQ(input.throttle, 0.10);
}

TEST_F(ReplayTelemetryProviderTest, PrimeSeedsTwinWithArrivalRowValues) {
    // The twin warm-boot is seeded from the ARRIVAL row's operating point
    // (speed/throttle), not the first row's: a DRIVE trace arriving at road
    // speed must expose a vehicle-speed pin at that speed on the FIRST held
    // poll (the twin is RUNNING, wheels pinned).
    makeProvider("time_s,throttle_pct,road_speed_kmh,gear_selector\n"
                 "0.0,5.0,0.0,P\n10.0,40.0,60.0,D\n11.0,40.0,60.0,D\n",
                 /*autoStart=*/true, /*autoGearbox=*/true);
    provider_->setWheelCouplingMode(twin::WheelCouplingMode::Pin);
    ASSERT_TRUE(provider_->Initialize());
    wireDefault();
    provider_->setStartFromS(10.0);
    provider_->primeArrivalState();

    const EngineInput input = provider_->OnUpdateSimulation(0.016);
    EXPECT_DOUBLE_EQ(input.replayTimestampS, 10.0);
    // PIN coupling surfaces the CSV road speed as the vehicle-speed target.
    EXPECT_NEAR(input.vehicleSpeedTargetKmh, 60.0, 1e-6);
}
