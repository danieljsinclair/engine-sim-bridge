// OverlayInputProviderTest.cpp - Contract tests for the keyboard overlay's
// interface delegation. The overlay WRAPS a core provider (replay/live) and
// must be TRANSPARENT to the loop's capability queries: whatever the core
// implements (IReplayTimeline, IArrivalStatePrimer) must be reachable
// THROUGH the wrapper. #66: the overlay forwarded IReplayTimeline but not
// IArrivalStatePrimer, so --replay-telemetry + --interactive +
// --start-from > 0 hit SimulationLoop's fail-fast ASSERT (the cast on the
// WRAPPER returned null) instead of priming the arrival state.

#include "input/OverlayInputProvider.h"
#include "input/IArrivalStatePrimer.h"
#include "input/IReplayTimeline.h"
#include "input/IKeyboardInput.h"
#include "input/EngineInputTarget.h"
#include "io/IInputProvider.h"
#include "simulator/EngineSimTypes.h"

#include <gtest/gtest.h>

#include <memory>
#include <string>

namespace {

using namespace input;

// Silent keyboard: no key ever arrives (getKey exhausted).
class NullKeyboard : public IKeyboardInput {
public:
    int getKey() override { return -1; }
};

// Core with the full file-trace surface: IInputProvider + IReplayTimeline +
// IArrivalStatePrimer (the ReplayTelemetryProvider shape).
class PrimerCore : public IInputProvider,
                   public IReplayTimeline,
                   public IArrivalStatePrimer {
public:
    EngineInput OnUpdateSimulation(double) override { return EngineInput{}; }
    void provideFeedback(const EngineSimStats&) override {}
    bool Initialize() override { return true; }
    void Shutdown() override {}
    bool IsConnected() const override { return true; }
    std::string GetProviderName() const override { return "PrimerCore"; }
    std::string GetLastError() const override { return ""; }

    // IReplayTimeline
    double durationS() const override { return 600.0; }
    void setEndAtS(double) override {}
    double getStartFromS() const override { return 5.0; }

    // IArrivalStatePrimer
    void primeArrivalState() override { ++primeCalls_; }
    void releaseArrivalHold() override { ++releaseCalls_; }

    int primeCalls_ = 0;
    int releaseCalls_ = 0;
};

// Core WITHOUT the primer (the live/keyboard-ish shape). The overlay must be
// null-safe on the primer surface exactly as it already is on the timeline
// surface — wrapping must never turn "core lacks a capability" into a crash.
class PlainCore : public IInputProvider, public IReplayTimeline {
public:
    EngineInput OnUpdateSimulation(double) override { return EngineInput{}; }
    void provideFeedback(const EngineSimStats&) override {}
    bool Initialize() override { return true; }
    void Shutdown() override {}
    bool IsConnected() const override { return true; }
    std::string GetProviderName() const override { return "PlainCore"; }
    std::string GetLastError() const override { return ""; }

    // IReplayTimeline
    double durationS() const override { return 600.0; }
    void setEndAtS(double) override {}
    double getStartFromS() const override { return 5.0; }
};

std::unique_ptr<OverlayInputProvider> makeOverlay(
        std::unique_ptr<IInputProvider> core, IKeyActionTarget* target) {
    return std::make_unique<OverlayInputProvider>(
        std::move(core), std::make_unique<NullKeyboard>(), target);
}

// #66 regression: the overlay itself must satisfy the IArrivalStatePrimer
// query — the exact dynamic_cast SimulationLoop::run() makes on the
// file-trace --start-from path before settling at the arrival point.
TEST(OverlayInputProviderTest, SatisfiesArrivalPrimerQuery) {
    EngineInputTarget target;
    auto overlay = makeOverlay(std::make_unique<PrimerCore>(), &target);

    EXPECT_NE(dynamic_cast<IArrivalStatePrimer*>(overlay.get()), nullptr);
}

// The primer surface delegates to the wrapped core (mirrors the existing
// IReplayTimeline delegation): prime/release land on the core, once each.
TEST(OverlayInputProviderTest, PrimerDelegatesToCore) {
    EngineInputTarget target;
    auto core = std::make_unique<PrimerCore>();
    PrimerCore* coreRaw = core.get();
    auto overlay = makeOverlay(std::move(core), &target);

    overlay->primeArrivalState();
    overlay->releaseArrivalHold();

    EXPECT_EQ(coreRaw->primeCalls_, 1);
    EXPECT_EQ(coreRaw->releaseCalls_, 1);
}

// Null-safety: a core without the primer must not crash the overlay's
// primer surface (the loop would not be on this path for such a core, but
// the wrapper must degrade as gracefully as it does for IReplayTimeline).
TEST(OverlayInputProviderTest, PrimerWithoutCoreImplementationIsNullSafe) {
    EngineInputTarget target;
    auto overlay = makeOverlay(std::make_unique<PlainCore>(), &target);

    EXPECT_NO_THROW({
        overlay->primeArrivalState();
        overlay->releaseArrivalHold();
    });
}

}  // namespace
