// ThreadedStrategyOverflowWarningTest.cpp - Overflow-skip logging contract
// TDD: The skip warning is EDGE-gated (first occurrence per run) with a
// SUSTAINED re-warning discriminator, because lead sawtooths through the
// max threshold with the two realtime clocks' beat — level-triggered
// logging machine-guns the log (~35 lines/run measured) with no fault.
// Strategies own their own state -- no BufferContext needed.

#include "strategy/ThreadedStrategy.h"
#include "../mocks/MockSimulator.h"
#include "AudioTestConstants.h"
#include "common/ILogging.h"
#include "telemetry/ITelemetryProvider.h"

#include <gtest/gtest.h>
#include <string>
#include <vector>

using namespace test::constants;

namespace {

// Captures log output by level; the production ConsoleLogger writes to
// stderr, which these tests cannot inspect.
class CapturingLogger final : public ILogging {
public:
    void setMask(uint32_t mask) override { mask_ = mask; }
    uint32_t getMask() const override { return mask_; }

    const std::vector<std::string>& warnings() const { return warnings_; }
    const std::vector<std::string>& debugs() const { return debugs_; }

private:
    uint32_t mask_ = LogMask::ALL;
    std::vector<std::string> warnings_;
    std::vector<std::string> debugs_;
    std::vector<std::string> infos_;

    void _write(uint32_t mask, const std::string& msg) override {
        if (mask & LogMask::WARN) warnings_.push_back(msg);
        else if (mask & LogMask::DBG) debugs_.push_back(msg);
        else infos_.push_back(msg);
    }
};

} // namespace

class ThreadedStrategyOverflowWarningTest : public ::testing::Test {
protected:
    void SetUp() override {
        telemetry_ = std::make_unique<telemetry::InMemoryTelemetry>();
        logger_ = std::make_unique<CapturingLogger>();
        strategy_ = std::make_unique<ThreadedStrategy>(logger_.get(), telemetry_.get());

        AudioBufferConfig config;
        config.channels = STEREO_CHANNELS;
        config.synthLatency = 0.02;  // target lead 20ms; max lead 40ms @ 44100
        ASSERT_TRUE(strategy_->initialize(config, DEFAULT_SAMPLE_RATE));
    }

    // Drives fills with no playback reader, so lead only grows: each call
    // either fills (lead rises toward/over max) or skips (lead already over).
    void pumpFills(int count) {
        MockSimulator simulator;
        for (int i = 0; i < count; ++i) {
            strategy_->fillBufferFromEngine(&simulator, 512);
        }
    }

    std::unique_ptr<CapturingLogger> logger_;
    std::unique_ptr<telemetry::InMemoryTelemetry> telemetry_;
    std::unique_ptr<ThreadedStrategy> strategy_;
};

TEST_F(ThreadedStrategyOverflowWarningTest, FirstSkipWarnsOncePerRun) {
    // Push lead over max (fills), then a short skip burst: exactly ONE
    // "Skipping buffer fill" warning regardless of burst length.
    pumpFills(3);
    pumpFills(6);  // short burst: benign beat chatter

    int skipWarns = 0;
    for (const auto& w : logger_->warnings()) {
        if (w.find("Skipping buffer fill") != std::string::npos) ++skipWarns;
    }
    EXPECT_EQ(skipWarns, 1) << "a short skip burst must log its first occurrence only";
}

TEST_F(ThreadedStrategyOverflowWarningTest, SustainedSkipReWarnsPeriodically) {
    pumpFills(3);
    pumpFills(45);  // 45 consecutive skips once lead is over max

    int skipWarns = 0, sustainedWarns = 0;
    for (const auto& w : logger_->warnings()) {
        if (w.find("SUSTAINED buffer-overflow skip") != std::string::npos) ++sustainedWarns;
        else if (w.find("Skipping buffer fill") != std::string::npos) ++skipWarns;
    }
    EXPECT_EQ(skipWarns, 1) << "first occurrence, once per run";
    EXPECT_EQ(sustainedWarns, 2) << "re-warn at every 20 consecutive skipped updates (skips 20 and 40)";
}

TEST_F(ThreadedStrategyOverflowWarningTest, DemotedSkipsStayVisibleAtDebug) {
    pumpFills(3);
    pumpFills(45);

    int skipDebugs = 0;
    for (const auto& d : logger_->debugs()) {
        if (d.find("Skipping buffer fill") != std::string::npos) ++skipDebugs;
    }
    // 46 skips total (pumpFills(3) ends on the first skip): 1 first-occurrence
    // + 2 sustained (at 20/40) warn; the remaining 43 demote to debug.
    EXPECT_EQ(skipDebugs, 43);
}

TEST_F(ThreadedStrategyOverflowWarningTest, PrepareBufferRearmsFirstOccurrence) {
    pumpFills(3);
    pumpFills(6);
    strategy_->prepareBuffer();  // new run: counters AND the once-per-run gate reset
    pumpFills(3);
    pumpFills(6);

    int skipWarns = 0;
    for (const auto& w : logger_->warnings()) {
        if (w.find("Skipping buffer fill") != std::string::npos) ++skipWarns;
    }
    EXPECT_EQ(skipWarns, 2) << "each run logs its own first occurrence";
}
