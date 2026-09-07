// LiveTelemetryParseCharacterizationTest.cpp - Characterization net for the
// LiveTelemetryProvider CSV parse/read path.
//
// SCOPE: pins the CURRENT observable behaviour of the row parse loops that a
// planned sonar refactor will restructure (nested-break elimination in
// refillRowBuffer / tryReadNextRowLive and declaration moves into if
// init-statements for "trailingJunk" / "parseError"). Every test here is a
// green-phase characterization test against master @ 933ad00: it asserts what
// the code DOES today so the refactor must keep doing it.
//
// Pinned behaviours (through the real public API — stream ctor + Initialize +
// OnUpdateSimulation + getCurrentSignal, no mocks):
//   #vs-start-from hint (tryParseSourceSkipHint):
//     - valid hint with trailing whitespace after the number is ACCEPTED
//     - hint with junk suffix after the number / negative seconds /
//       non-numeric value is REJECTED: treated as absent, the stream recovers
//       within the same tick (the line is stepped past, the header parses, the
//       first row still surfaces) and the display degrades to the LOCAL
//       timecode (no cold-jump)
//     - the FIRST well-formed hint wins; a second hint line is consumed but
//       ignored
//   refillRowBuffer (paced path, liveStream=false):
//     - a malformed row between valid rows is SKIPPED (loop continues; later
//       rows still surface; provider stays connected) — no latched error, no
//       stall
//     - a malformed FIRST row does not anchor the clock: the baseline anchors
//       on the first row that actually parses
//     - a field with trailing junk after a valid number is ACCEPTED with the
//       parsed prefix value (stod partial parse) — time field and value field
//   tryReadNextRowLive (live pipe path, liveStream=true):
//     - malformed rows interleaved with valid rows are skipped; the drain
//       keeps going and the LATEST VALID row surfaces
//     - a malformed row arriving after the last valid row does not clear it
//     - the readiness probe not-ready breaks the drain WITHOUT consuming the
//       stream (row remains unread)
//     - with --start-from, the discard window is measured from the FIRST
//       PARSED row (stream anchor), not the first KEPT row — a wrong anchor
//       would discard the whole tail and surface the wrong row
//
// NOT pinned here (indistinguishable through the provider, no business value):
// blank-line skip vs parseRow-empty-skip — parseRow("") returns false anyway,
// so isBlankLine's early continue is not separately observable at this seam.

#include "input/LiveTelemetryProvider.h"
#include "simulator/EngineSimTypes.h"
#include "simulator/GearConventions.h"

#include <gtest/gtest.h>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

namespace {

constexpr const char* kHeader = "time_s,throttle_pct,road_speed_kmh,gear_selector";

// Paced construction (liveStream=false, no probe): exercises refillRowBuffer.
struct PacedHarness {
    std::istringstream stream;
    std::unique_ptr<input::LiveTelemetryProvider> provider;
    explicit PacedHarness(const std::string& csv)
        : stream(csv) {
        provider = std::make_unique<input::LiveTelemetryProvider>(
            stream, /*autoStart=*/true, /*streamDataReady=*/nullptr,
            /*liveStream=*/false);
    }
};

// Live-pipe construction (liveStream=true, no probe): exercises
// tryReadNextRowLive. This is the unpaced shape CLIMain uses for
// --live-telemetry stdin.
struct LiveHarness {
    std::istringstream stream;
    std::unique_ptr<input::LiveTelemetryProvider> provider;
    explicit LiveHarness(const std::string& csv)
        : stream(csv) {
        provider = std::make_unique<input::LiveTelemetryProvider>(
            stream, /*autoStart=*/true, /*streamDataReady=*/nullptr,
            /*liveStream=*/true);
    }
};

// Live-pipe construction with a controllable readiness probe (the poll(2)
// seam the sync-pull boundary fix injects for real pipes).
struct ProbedLiveHarness {
    std::istringstream stream;
    std::unique_ptr<input::LiveTelemetryProvider> provider;
    explicit ProbedLiveHarness(const std::string& csv, bool& probeReady)
        : stream(csv) {
        provider = std::make_unique<input::LiveTelemetryProvider>(
            stream, /*autoStart=*/true,
            [&probeReady]() { return probeReady; },
            /*liveStream=*/true);
    }
};

// ============================================================================
// #vs-start-from hint (tryParseSourceSkipHint) — the "trailingJunk" site.
// ============================================================================

// A hint whose number carries a NON-WHITESPACE suffix ("40abc") is malformed:
// strtod stops at 'a', the junk check rejects the line, and the hint is
// treated as absent. The stream must RECOVER WITHIN THE SAME TICK (the
// malformed line is stepped past, the header parses on the retry inside
// tryReadNextRow) so the first row still surfaces — and the display clock
// degrades to the LOCAL timecode (no 40 s cold-jump).
TEST(LiveTelemetryParseCharacterization, HintJunkSuffixAfterNumber_IsRejected_DegradesToLocalTimecode) {
    PacedHarness h(std::string("#vs-start-from 40abc\n") + kHeader + "\n"
                   "0.0,50,30,D\n"
                   "0.5,50,31,D\n");
    ASSERT_TRUE(h.provider->Initialize());

    input::EngineInput in = h.provider->OnUpdateSimulation(0.05);

    // The stream recovered: the first real row surfaced.
    const input::UpstreamSignal sig = h.provider->getCurrentSignal();
    ASSERT_TRUE(sig.isValid) << "a malformed hint must not lose the stream";
    EXPECT_DOUBLE_EQ(sig.speedKmh, 30.0);

    // The hint was NOT applied: local timecode (dt only), no 40 s cold-jump.
    EXPECT_NEAR(in.replayTimestampS, 0.05, 1e-9)
        << "malformed hint must degrade to the legacy local timecode";
    EXPECT_TRUE(h.provider->IsConnected());
}

// Negative seconds are malformed (a skip amount cannot be negative): same
// degrade contract — hint absent, local timecode, stream intact.
TEST(LiveTelemetryParseCharacterization, HintNegativeSeconds_IsRejected_DegradesToLocalTimecode) {
    PacedHarness h(std::string("#vs-start-from -5\n") + kHeader + "\n"
                   "0.0,50,30,D\n"
                   "0.5,50,31,D\n");
    ASSERT_TRUE(h.provider->Initialize());

    input::EngineInput in = h.provider->OnUpdateSimulation(0.05);

    const input::UpstreamSignal sig = h.provider->getCurrentSignal();
    ASSERT_TRUE(sig.isValid);
    EXPECT_DOUBLE_EQ(sig.speedKmh, 30.0);
    EXPECT_NEAR(in.replayTimestampS, 0.05, 1e-9)
        << "negative hint must be treated as absent (local timecode)";
}

// A value that does not start with a number ("soon") is malformed: strtod
// converts nothing, the line is rejected, same degrade contract.
TEST(LiveTelemetryParseCharacterization, HintNonNumericValue_IsRejected_DegradesToLocalTimecode) {
    PacedHarness h(std::string("#vs-start-from soon\n") + kHeader + "\n"
                   "0.0,50,30,D\n"
                   "0.5,50,31,D\n");
    ASSERT_TRUE(h.provider->Initialize());

    input::EngineInput in = h.provider->OnUpdateSimulation(0.05);

    const input::UpstreamSignal sig = h.provider->getCurrentSignal();
    ASSERT_TRUE(sig.isValid);
    EXPECT_DOUBLE_EQ(sig.speedKmh, 30.0);
    EXPECT_NEAR(in.replayTimestampS, 0.05, 1e-9);
}

// Contrast: whitespace AFTER the number is tolerated (the trailingJunk scan
// skips isspace). "#vs-start-from 40.000  " is a VALID hint — the display
// cold-jumps by the full 40 s source skip exactly as the tight hint does.
TEST(LiveTelemetryParseCharacterization, HintTrailingWhitespaceAfterNumber_IsAccepted) {
    PacedHarness h(std::string("#vs-start-from 40.000  \n") + kHeader + "\n"
                   "100.0,20,30,D\n"
                   "100.5,20,32,D\n");
    ASSERT_TRUE(h.provider->Initialize());

    input::EngineInput in = h.provider->OnUpdateSimulation(0.05);

    EXPECT_NEAR(in.replayTimestampS, 40.05, 1e-9)
        << "trailing whitespace after the number must not invalidate the hint";
    EXPECT_DOUBLE_EQ(h.provider->getCurrentSignal().speedKmh, 30.0);
}

// First hint wins: a second well-formed hint line is consumed (it matches the
// prefix) but ignored — the recorded skip stays the FIRST one (40 s, not 90).
TEST(LiveTelemetryParseCharacterization, HintFirstWellFormedHintWins) {
    PacedHarness h(std::string("#vs-start-from 40.000\n"
                               "#vs-start-from 90.000\n") + kHeader + "\n"
                   "100.0,20,30,D\n"
                   "100.5,20,32,D\n");
    ASSERT_TRUE(h.provider->Initialize());

    input::EngineInput in = h.provider->OnUpdateSimulation(0.05);

    EXPECT_NEAR(in.replayTimestampS, 40.05, 1e-9)
        << "the first hint (40 s) must win; the second must not overwrite it";
}

// ============================================================================
// refillRowBuffer (paced path) — the parseError call site at ~607 and the
// nested-break loop at ~584.
// ============================================================================

// A malformed row (unparseable time field) between valid rows is SKIPPED:
// the refill loop continues past it (the continue-on-parse-fail branch), the
// later valid row still surfaces on the same frame, and the provider remains
// connected — no latched error state, no stall on the bad row.
TEST(LiveTelemetryParseCharacterization, PacedMalformedRowBetweenValidRows_IsSkipped_StreamContinues) {
    PacedHarness h(std::string(kHeader) + "\n"
                   "0.0,50,30,P\n"
                   "not-a-time,99,999,Q\n"
                   "1.0,60,40,D\n");
    ASSERT_TRUE(h.provider->Initialize());

    h.provider->OnUpdateSimulation(2.0);  // both valid rows in window

    const input::UpstreamSignal sig = h.provider->getCurrentSignal();
    ASSERT_TRUE(sig.isValid) << "the malformed row must not poison the stream";
    EXPECT_EQ(sig.gearSelector, bridge::GearSelector::DRIVE);
    EXPECT_DOUBLE_EQ(sig.speedKmh, 40.0)
        << "the valid row AFTER the malformed one must surface (skip, not stall)";
    EXPECT_TRUE(h.provider->IsConnected())
        << "a malformed row must not disconnect the provider";
}

// A malformed FIRST row must not become the clock anchor: the recording
// baseline anchors on the first row that actually PARSES (t=5 here). With the
// sim clock at 1.0 s the anchored row sits at relT=0 and surfaces on frame 1;
// an anchor at 0 (or a stall on the bad row) would surface nothing.
TEST(LiveTelemetryParseCharacterization, PacedMalformedFirstRow_DoesNotAnchorBaseline) {
    PacedHarness h(std::string(kHeader) + "\n"
                   "garbage,1,2,Q\n"
                   "5.0,20,30,D\n");
    ASSERT_TRUE(h.provider->Initialize());

    h.provider->OnUpdateSimulation(1.0);

    const input::UpstreamSignal sig = h.provider->getCurrentSignal();
    ASSERT_TRUE(sig.isValid)
        << "the loop must step past the malformed first row, not stall on it";
    EXPECT_DOUBLE_EQ(sig.speedKmh, 30.0)
        << "baseline anchors on the first PARSED row (t=5 -> relT 0), so the "
           "row is in window at simElapsedS=1.0";
}

// Trailing junk after a valid number in a VALUE field is ACCEPTED with the
// parsed prefix (stod partial parse): "50abc" decodes throttle 50%. Pinning
// ACTUAL current behaviour — the row is not rejected.
TEST(LiveTelemetryParseCharacterization, PacedTrailingJunkAfterValueNumber_RowAcceptedWithParsedPrefix) {
    PacedHarness h(std::string(kHeader) + "\n"
                   "0.0,50abc,30,D\n");
    ASSERT_TRUE(h.provider->Initialize());

    h.provider->OnUpdateSimulation(0.05);

    const input::UpstreamSignal sig = h.provider->getCurrentSignal();
    ASSERT_TRUE(sig.isValid) << "a row with trailing junk after a number is accepted today";
    EXPECT_DOUBLE_EQ(sig.throttleFraction, 0.5)
        << "the numeric prefix (50) is the decoded value";
    EXPECT_DOUBLE_EQ(sig.speedKmh, 30.0);
}

// Same acceptance on the TIME field: "1.0x" decodes t=1.0 s, the row is
// accepted and paced like a clean t=1.0 row (in window at simElapsedS=2.0).
TEST(LiveTelemetryParseCharacterization, PacedTrailingJunkAfterTimeNumber_RowAccepted) {
    PacedHarness h(std::string(kHeader) + "\n"
                   "1.0x,50,30,D\n");
    ASSERT_TRUE(h.provider->Initialize());

    h.provider->OnUpdateSimulation(2.0);

    const input::UpstreamSignal sig = h.provider->getCurrentSignal();
    ASSERT_TRUE(sig.isValid);
    EXPECT_DOUBLE_EQ(sig.speedKmh, 30.0)
        << "time \"1.0x\" decodes to 1.0 s and the row is paced accordingly";
}

// ============================================================================
// tryReadNextRowLive (live pipe path) — the parseError call site at ~649 and
// the nested-break loop at ~640.
// ============================================================================

// The live drain skips malformed rows and keeps draining: with garbage
// BETWEEN two valid rows, the LATEST VALID row (the one after the garbage)
// surfaces on frame 1.
TEST(LiveTelemetryParseCharacterization, LiveMalformedRowBetweenValid_LatestValidWins) {
    LiveHarness h(std::string(kHeader) + "\n"
                  "2.0,10,5,P\n"
                  "oops,1,2,Q\n"
                  "8.0,100,80,D\n");
    ASSERT_TRUE(h.provider->Initialize());

    h.provider->OnUpdateSimulation(0.05);

    const input::UpstreamSignal sig = h.provider->getCurrentSignal();
    ASSERT_TRUE(sig.isValid);
    EXPECT_EQ(sig.gearSelector, bridge::GearSelector::DRIVE);
    EXPECT_DOUBLE_EQ(sig.speedKmh, 80.0)
        << "the drain must continue past the malformed row to the latest valid one";
}

// A malformed row AFTER the last valid row must not clear the held sample:
// the continue-on-parse-fail leaves `latest` untouched, so the last VALID row
// still surfaces.
TEST(LiveTelemetryParseCharacterization, LiveMalformedRowAfterLastValid_KeepsLastValid) {
    LiveHarness h(std::string(kHeader) + "\n"
                  "8.0,100,80,D\n"
                  "oops,1,2,Q\n");
    ASSERT_TRUE(h.provider->Initialize());

    h.provider->OnUpdateSimulation(0.05);

    const input::UpstreamSignal sig = h.provider->getCurrentSignal();
    ASSERT_TRUE(sig.isValid);
    EXPECT_EQ(sig.gearSelector, bridge::GearSelector::DRIVE);
    EXPECT_DOUBLE_EQ(sig.speedKmh, 80.0)
        << "a trailing malformed row must not evict the last valid sample";
}

// The readiness probe guards the live drain exactly as it guards the paced
// refill: not-ready breaks the drain BEFORE any row read, leaving the stream
// unconsumed (the row must still be readable afterwards). Mirrors the paced
// probe test for the second loop's probe branch.
TEST(LiveTelemetryParseCharacterization, LiveProbeNotReady_BreaksWithoutConsumingStream) {
    bool probeReady = false;  // pipe "empty"
    ProbedLiveHarness h(std::string(kHeader) + "\n"
                        "0.0,50,30,D\n",
                        probeReady);
    ASSERT_TRUE(h.provider->Initialize()) << h.provider->GetLastError();

    h.provider->OnUpdateSimulation(0.05);

    // The data row must remain unread (the probe break fired before getline).
    std::string leftover;
    EXPECT_TRUE(std::getline(h.stream, leftover))
        << "probe not-ready: the live drain must leave the row in the stream";
    EXPECT_EQ(leftover, "0.0,50,30,D");
    EXPECT_TRUE(h.provider->IsConnected())
        << "not-ready is not EOF — the provider stays connected";
}

// With --start-from, the discard window is measured from the FIRST PARSED row
// (the stream anchor, set before the start-from filter), not from the first
// KEPT row. Rows at relT 0 and 5 are discarded; rows at relT 10 and 15 are
// kept; the drain surfaces the LATEST kept row (t=15, N, 60 km/h). Had the
// anchor landed on the first kept row (t=10), the t=15 row would read relT=5
// < 10 and be discarded — surfacing D at 50 km/h instead.
TEST(LiveTelemetryParseCharacterization, LiveStartFrom_WindowMeasuredFromFirstParsedRow) {
    LiveHarness h(std::string(kHeader) + "\n"
                  "0.0,10,5,P\n"
                  "5.0,20,10,N\n"
                  "10.0,30,50,D\n"
                  "15.0,40,60,N\n");
    ASSERT_TRUE(h.provider->Initialize());
    h.provider->setStartFromS(10.0);

    h.provider->OnUpdateSimulation(0.05);

    const input::UpstreamSignal sig = h.provider->getCurrentSignal();
    ASSERT_TRUE(sig.isValid) << "post-window rows must surface";
    EXPECT_EQ(sig.gearSelector, bridge::GearSelector::NEUTRAL)
        << "the anchor is the FIRST PARSED row (t=0): the t=15 row (relT 15) "
           "is the latest kept, not the t=10 row";
    EXPECT_DOUBLE_EQ(sig.speedKmh, 60.0);
}

}  // namespace
