// StartStopInputAdapterTest.cpp - tests for the decorator that applies the
// vehicle start/stop decision to EngineInput.
//
// We drive a REAL LiveTelemetryProvider (JSON/network path) via submitSignal so
// the decorator can read getCurrentSignal(). The test asserts:
//   (i)   the inner provider's EngineInput passes through unchanged EXCEPT
//         ignition / starterButton (the only two fields the adapter overwrites).
//   (ii)  a brake-pressed signal produces the crank-then-ignite sequence across
//         dt calls (starter fires once, ignition only after the crank delay).
//   (iii) brake + PARK while running produces ignition == false (stop).
//
// The VehicleStartController itself is already covered by
// VehicleStartControllerTest.cpp; we do NOT re-test the controller here. We test
// the SEAM: that the decorator reflects the controller's decision into EngineInput
// and honours the momentary starter pulse contract.

#include <gtest/gtest.h>

#include "input/StartStopInputAdapter.h"
#include "input/LiveTelemetryProvider.h"
#include "twin/IceVehicleProfile.h"
#include "simulator/GearConventions.h"

#include <memory>
#include <vector>

using namespace input;
using bridge::GearSelector;

namespace {

class StartStopInputAdapterTest : public ::testing::Test {
protected:
    void SetUp() override {
        profile_ = twin::IceVehicleProfile::zf8hp45();
        auto live = std::make_unique<LiveTelemetryProvider>(profile_);
        ASSERT_TRUE(live->Initialize());
        live_ = live.get();
        adapter_ = std::make_unique<StartStopInputAdapter>(std::move(live));
    }

    // Convenience: build an upstream signal with brake + gear set.
    static UpstreamSignal makeSignal(bool brakePressed, GearSelector gear) {
        UpstreamSignal s;
        s.isValid = true;
        s.brakePedalState = brakePressed ? 1 : 0;  // ON=1, else OFF(0)
        s.gearSelector = gear;
        return s;
    }

    twin::IceVehicleProfile profile_;
    LiveTelemetryProvider* live_ = nullptr;  // non-owning; adapter owns it
    std::unique_ptr<StartStopInputAdapter> adapter_;
};

// (i) Pass-through: the inner provider's fields other than ignition/starterButton
// are preserved. A neutral, no-brake signal leaves the engine OFF (no start
// demand), so ignition should stay at the inner provider's default (true) — but
// the adapter does not invent a start. We assert the untouched fields survive and
// that starterButton is the adapter's flattened pulse (false when no demand).
TEST_F(StartStopInputAdapterTest, InnerInputPassesThroughExceptStartStopFields) {
    // Neutral, no brake -> no start demand -> controller leaves ignition false,
    // starter false. The inner provider returns its own throttle/gear; those must
    // survive verbatim.
    live_->submitSignal(makeSignal(/*brake=*/false, GearSelector::NEUTRAL));

    // Capture the inner provider's raw output for comparison.
    EngineInput raw = live_->OnUpdateSimulation(0.016);

    // Reset adapter state by constructing a fresh one reading the same provider.
    auto live2 = std::make_unique<LiveTelemetryProvider>(profile_);
    ASSERT_TRUE(live2->Initialize());
    live2->submitSignal(makeSignal(/*brake=*/false, GearSelector::NEUTRAL));
    StartStopInputAdapter adapter(std::move(live2));

    EngineInput wrapped = adapter.OnUpdateSimulation(0.016);

    // Untouched fields pass through identically.
    EXPECT_EQ(wrapped.throttle, raw.throttle);
    EXPECT_EQ(wrapped.gearAbsolute, raw.gearAbsolute);
    EXPECT_EQ(wrapped.clutchPressure, raw.clutchPressure);
    EXPECT_EQ(wrapped.gearSelector, raw.gearSelector);
    EXPECT_EQ(wrapped.gearAutoMode, raw.gearAutoMode);

    // The only fields the adapter is allowed to set:
    EXPECT_FALSE(wrapped.starterButton);  // no start demand -> no pulse
    EXPECT_FALSE(wrapped.ignition);       // controller says off (no brake, no drive gear)
}

// (ii) Brake-pressed while off: starter engages on the first frame (single pulse),
// ignition only after the 0.5s crank delay. Mirrors VehicleStartControllerTest
// case #1 but observed through the EngineInput seam.
TEST_F(StartStopInputAdapterTest, BrakePressed_CranksThenIgnitesAfterDelay) {
    live_->submitSignal(makeSignal(/*brake=*/true, GearSelector::PARK));

    EngineInput f0 = adapter_->OnUpdateSimulation(0.2);
    EXPECT_TRUE(f0.starterButton);   // rising edge -> pulse fires this frame
    EXPECT_FALSE(f0.ignition);       // ignition still off

    EngineInput f1 = adapter_->OnUpdateSimulation(0.2);  // accum 0.4
    EXPECT_FALSE(f1.starterButton);  // held high -> suppressed (pulse is 1 frame)
    EXPECT_FALSE(f1.ignition);

    EngineInput f2 = adapter_->OnUpdateSimulation(0.1);  // accum 0.5 -> ignite
    EXPECT_TRUE(f2.ignition);
}

// (iii) Brake + PARK while running: ignition goes false (stop). We start via
// brake, ignite, then issue brake+PARK to stop.
TEST_F(StartStopInputAdapterTest, BrakeAndParkWhileRunning_StopsEngine) {
    live_->submitSignal(makeSignal(/*brake=*/true, GearSelector::PARK));
    adapter_->OnUpdateSimulation(0.2);
    adapter_->OnUpdateSimulation(0.2);
    adapter_->OnUpdateSimulation(0.1);  // ignition on

    // Now running. Issue brake + PARK to stop.
    EngineInput stopFrame = adapter_->OnUpdateSimulation(0.1);
    EXPECT_FALSE(stopFrame.ignition);
}

// (iv) Starter pulse contract: a controller that holds starter true across frames
// must NOT emit more than one pulse per rising edge. We verify by checking that
// after the initial pulse, consecutive frames with brake held (crank pending)
// produce starterButton == false until the controller releases and re-engages.
TEST_F(StartStopInputAdapterTest, StarterPulseIsSingleFramePerRisingEdge) {
    live_->submitSignal(makeSignal(/*brake=*/true, GearSelector::PARK));

    EXPECT_TRUE(adapter_->OnUpdateSimulation(0.016).starterButton);  // pulse
    EXPECT_FALSE(adapter_->OnUpdateSimulation(0.016).starterButton); // suppressed
    EXPECT_FALSE(adapter_->OnUpdateSimulation(0.016).starterButton); // still suppressed
}

} // namespace
