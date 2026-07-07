// InMemoryTelemetryResetTests.cpp - the one honest InMemoryTelemetry contract
//
// Most of InMemoryTelemetry is std::atomic store/load passthrough — a truism to
// test. Per-field write->read and reset-clears-every-field suites only assert
// that two hand-maintained field lists (the struct vs the test) stay in sync,
// which can't catch a divergence and adds no value.
//
// The ONE contract with a real, list-independent failure mode is the
// enginePhase enum<->int conversion across the storage boundary:
//   - writer casts EnginePhase -> int   (InMemoryTelemetry.cpp:65)
//   - reader casts int -> EnginePhase   (InMemoryTelemetry.cpp:111)
//   - reset()  stores literal int 0     (InMemoryTelemetry.cpp:35)
// A future enum reorder, a non-zero Stopped default, or a changed storage type
// would silently alter what readers observe after reset/write, and this test
// catches it without mirroring a second field list. Real InMemoryTelemetry.

#include "telemetry/ITelemetryProvider.h"

#include <gtest/gtest.h>

using namespace telemetry;

// enginePhase survives the int storage round-trip for every enumerator, AND
// reset() yields Stopped (the int-0 enumerator). Pins the bidirectional cast +
// the implicit "reset means Stopped" contract in one assertion set.
TEST(InMemoryTelemetryResetTest, EnginePhaseRoundTripsThroughIntStorage) {
    for (EnginePhase phase : {
            EnginePhase::Stopped, EnginePhase::Cranking, EnginePhase::Rollover,
            EnginePhase::Running, EnginePhase::Stopping,
        }) {
        InMemoryTelemetry telemetry;
        EngineStateTelemetry es;
        es.enginePhase = phase;
        telemetry.writeEngineState(es);
        EXPECT_EQ(telemetry.getEngineState().enginePhase, phase)
            << "enum did not survive int-storage round-trip";
    }

    // reset() stores literal 0, which must read back as Stopped (enumerator 0).
    // Catches a reorder that moves Stopped off the zeroth slot.
    InMemoryTelemetry telemetry;
    telemetry.reset();
    EXPECT_EQ(telemetry.getEngineState().enginePhase, EnginePhase::Stopped);
}
