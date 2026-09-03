// LogMaskTest.cpp
//
// Locks the CLI console-logging contract (owner 2026-09-03: default output
// must carry no [DEBUG] lines; --verbose re-enables them). The mask contract
// lives here; the SyncPullStrategy startup diagnostics are emitted at DBG
// level, so the default runMask filtering DBG is what keeps them silent.

#include <gtest/gtest.h>

#include "common/ILogging.h"

using namespace LogMask;

TEST(LogMaskTest, DefaultRunMaskExcludesDebugLevel) {
    const uint32_t mask = runMask(false);
    EXPECT_FALSE(mask & DBG);
    EXPECT_TRUE(mask & INFO);
    EXPECT_TRUE(mask & WARN);
    EXPECT_TRUE(mask & ERROR);
    EXPECT_TRUE(mask & ALL_CATS);  // every category stays on
}

TEST(LogMaskTest, VerboseRunMaskIsEverything) {
    EXPECT_EQ(runMask(true), ALL);
}

TEST(LogMaskTest, ConsoleLoggerFiltersDebugAtDefaultRunMask) {
    ConsoleLogger logger;
    logger.setMask(runMask(false));
    // A debug-category message (e.g. SyncPullStrategy startup diagnostics,
    // AUDIO category at DBG level) is filtered at the default mask...
    EXPECT_FALSE(logger.shouldLog(AUDIO | DBG));
    // ...while the same category at INFO passes.
    EXPECT_TRUE(logger.shouldLog(AUDIO | INFO));
}

TEST(LogMaskTest, ConsoleLoggerPassesDebugAtVerboseRunMask) {
    ConsoleLogger logger;
    logger.setMask(runMask(true));
    EXPECT_TRUE(logger.shouldLog(AUDIO | DBG));
    EXPECT_TRUE(logger.shouldLog(SYNC_PULL | DBG));
}
