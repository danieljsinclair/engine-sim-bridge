// SyncPullStrategyContractTests.cpp - Mode-identity contract for sync-pull audio
//
// SyncPullStrategy is the on-demand (pull) audio strategy: it generates audio in
// the render callback rather than pre-filling a buffer. Several of its
// IAudioBuffer methods are documented no-ops for this mode (fillBufferFromEngine,
// prepareBuffer, resetBufferAfterWarmup, updateSimulation, reset) — testing them
// would assert only "callable, returns" with no real contract, i.e. the truism
// the test constraints forbid. We do NOT test those here.
//
// What IS a real contract and was zero-hit per lcov: the strategy's MODE
// IDENTITY — the values by which the audio framework identifies and routes this
// strategy. These tests pin exactly that:
//   - isEnabled() reports the strategy as active (constant true by design).
//   - getModeString() identifies the mode ("SYNC-PULL") for diagnostics/logging.
//
// REFACTOR-FIRST FLAG (not worked around here): swapSimulator() mutates real
// state (the simulator pointer + crossfade countdown), but that state is private
// with no getter — observable only through render(), which requires a fully
// initialized audio pipeline (AudioBufferView + sample rate + a live simulator).
// Pinning swap's contract would need either a production getter added (a
// refactor, out of scope for the test author) or a heavy pipeline fixture.
// Flagged, not forced.

#include "strategy/SyncPullStrategy.h"

#include <gtest/gtest.h>

#include <string>

// isEnabled() declares the sync-pull strategy as an active strategy in the
// framework's routing decision (constant true by design).
TEST(SyncPullStrategyContractTest, IsEnabledReportsActive) {
    SyncPullStrategy strategy;
    EXPECT_TRUE(strategy.isEnabled());
}

// getModeString() identifies this strategy as SYNC-PULL for diagnostics and
// logging. The string is the strategy's identity contract, not free text.
TEST(SyncPullStrategyContractTest, ModeStringIdentifiesAsSyncPull) {
    SyncPullStrategy strategy;
    EXPECT_EQ(strategy.getModeString(), "SYNC-PULL");
}
